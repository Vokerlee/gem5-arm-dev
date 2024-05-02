# Copyright (c) 2024 Roman Glaz <vokerlee@gmail.com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import argparse
import os
import sys

import caches
import cluster
import systems
import freqs

import m5
from m5.objects import *
from m5.options import *
from m5.util import addToPath

m5.util.addToPath("../..")

from common import (
    MemConfig,
    ObjectList,
    SysPaths,
)

from common.cores.arm import (
    HPI
)

default_root_device = "/dev/vda1"


def create_cow_image(name):
    """Helper function to create a Copy-on-Write disk image"""
    image = CowDiskImage()
    image.child.image_file = SysPaths.disk(name)

    return image


def create(args):
    """Create and configure the system object."""

    if args.script and not os.path.isfile(args.script):
        print(f"Error: Bootscript {args.script} does not exist")
        sys.exit(1)

    system = systems.SimpleSystem(
        want_caches=True,
        mem_size=args.mem_size,
        l3_cache=caches.L3Cache,
        l3_clock=freqs.L3_CD,
        l3_voltage=freqs.L3_VD,
        mem_mode="timing",
        workload=ArmFsLinux(object_file=SysPaths.binary(args.kernel)),
        readfile=args.script,
    )

    mem_voltage_domain = VoltageDomain(voltage=freqs.LPDDR5_VD)
    mem_clk_domain = SrcClockDomain(
        clock=freqs.LPDDR5_CD, voltage_domain=mem_voltage_domain, domain_id=0x400
    )

    MemConfig.config_mem(args, system, mem_clk_domain)
    for channel in range(len(system.mem_ctrls)):
        system.mem_ctrls[channel].dram.device_size = "1GB"

    # Add the PCI devices we need for this system. The base system
    # doesn't have any PCI devices by default since they are assumed
    # to be added by the configuration scripts needing them.
    system.pci_devices = [
        # Create a VirtIO block device for the system's boot
        # disk. Attach the disk image using gem5's Copy-on-Write
        # functionality to avoid writing changes to the stored copy of
        # the disk image.
        PciVirtIO(vio=VirtIOBlock(image=create_cow_image(args.disk_image)))
    ]

    # Attach the PCI devices to the system. The helper method in the
    # system assigns a unique PCI bus ID to each of the devices and
    # connects them to the IO bus.
    for dev in system.pci_devices:
        system.attach_pci(dev)

    # Wire up the system's memory system
    system.connect()

    system.HPICluster = cluster.ArmCpuCluster(
        system=system,
        num_cpus=1,
        cpu_type=HPI.HPI,
        cpu_clock=freqs.CPU0_CD,
        cpu_voltage=freqs.CPU0_VD,
        split_l2_cache=False,
        l1i_type=HPI.HPI_ICache,
        l1d_type=HPI.HPI_DCache,
        l2_type=HPI.HPI_L2,
        itlb_size=32,
        dtlb_size=32,
        l2tlb_size=512,
    )

    system.addCaches(need_caches=True, last_cache_level=3)

    cpu_requestor_last = system.HPICluster.addClusterRelatedPMUs(
        ints=([23] * len(system.HPICluster.cpus)), cpu_requestor_init=0
    )
    # system.HPICluster2.addClusterRelatedPMUs(
    #     ints=([23] * len(system.HPICluster2.cpus)),
    #     cpu_requestor_init=cpu_requestor_last,
    # )
    system.addL3RelatedPMUs()

    # Setup gem5's minimal Linux boot loader.
    system.realview.setupBootLoader(system, SysPaths.binary)

    if args.initrd:
        system.workload.initrd_filename = args.initrd

    system.dvfs_handler.domains = [
        system.HPICluster.clk_domain,
        mem_clk_domain,
    ]
    system.dvfs_handler.enable = True
    system.dvfs_handler.transition_latency = "10us"

    # No DTB specified: autogenerate DTB
    system.workload.dtb_filename = os.path.join(
        m5.options.outdir, "system.dtb"
    )
    system.generateDtb(system.workload.dtb_filename)

    # Linux boot command flags
    kernel_cmd = [
        # Tell Linux to use the simulated serial port as a console
        "console=ttyAMA0",
        # Hard-code timing
        "lpj=19988480",
        # Disable address space randomisation to get a consistent
        # memory layout.
        "norandmaps",
        # Tell Linux where to find the root disk image.
        f"root={args.root_device}",
        # Mount the root disk read-write by default.
        "rw",
        # Tell Linux about the amount of physical memory present.
        f"mem={args.mem_size}",
    ]
    system.workload.command_line = " ".join(kernel_cmd)

    return system


def run(args):
    cptdir = m5.options.outdir
    if args.checkpoint:
        print(f"Checkpoint directory: {cptdir}")

    while True:
        event = m5.simulate()
        exit_msg = event.getCause()
        if exit_msg == "checkpoint":
            print("Dropping checkpoint at tick %d" % m5.curTick())
            cpt_dir = os.path.join(m5.options.outdir, "cpt.%d" % m5.curTick())
            m5.checkpoint(os.path.join(cpt_dir))
            print("Checkpoint done.")
        else:
            print(f"{exit_msg} ({event.getCode()}) @ {m5.curTick()}")
            break


def main():
    parser = argparse.ArgumentParser(epilog=__doc__)

    parser.add_argument("--kernel", type=str, help="Linux kernel")
    parser.add_argument(
        "--initrd",
        type=str,
        default=None,
        help="initrd/initramfs file to load",
    )
    parser.add_argument(
        "--disk-image",
        type=str,
        help="Disk to instantiate",
    )
    parser.add_argument(
        "--root-device",
        type=str,
        default=default_root_device,
        help=f"OS device name for root partition (no default)",
    )
    parser.add_argument(
        "--script", type=str, default="", help="Linux bootscript"
    )
    parser.add_argument(
        "--mem-type",
        default="LPDDR5_6400_1x16_BG_BL16",
        choices=ObjectList.mem_list.get_names(),
        help="type of memory to use",
    )
    parser.add_argument(
        "--mem-ranks",
        type=int,
        default=None,
        help="number of memory ranks per channel",
    )
    parser.add_argument(
        "--mem-channels", type=int, default=8, help="number of memory channels"
    )
    parser.add_argument(
        "--mem-size",
        action="store",
        type=str,
        default="8GB",
        help="Specify the physical memory size",
    )

    parser.add_argument("--checkpoint", action="store_true")
    parser.add_argument("--restore", type=str, default=None)

    args = parser.parse_args()

    root = Root(full_system=True)
    root.system = create(args)

    if args.restore is not None:
        m5.instantiate(args.restore)
    else:
        m5.instantiate()

    run(args)


if __name__ == "__m5_main__":
    main()
