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

import caches
import cluster

from m5.defines import buildEnv
from m5.objects import *


class SimpleSESystem(System, cluster.ClusterSystem):
    # Use a fixed cache line size of 64 bytes.
    cache_line_size = 64

    def __init__(self, l3_cache, l3_clock, l3_voltage, **kwargs):
        System.__init__(self, **kwargs)
        cluster.ClusterSystem.__init__(self, l3_cache, l3_clock, l3_voltage, **kwargs)

        self.voltage_domain = VoltageDomain(voltage="3.3V")
        self.clk_domain = SrcClockDomain(
            clock="1GHz", voltage_domain=self.voltage_domain
        )

        # Create the off-chip memory bus.
        self.membus = SystemXBar()

    def connect(self):
        self.system_port = self.membus.cpu_side_ports


class BaseSimpleSystem(ArmSystem, cluster.ClusterSystem):
    cache_line_size = 64

    def __init__(self, mem_size, platform, l3_cache, l3_clock, l3_voltage, **kwargs):
        ArmSystem.__init__(self, **kwargs)
        cluster.ClusterSystem.__init__(self, l3_cache, l3_clock, l3_voltage, **kwargs)

        self.voltage_domain = VoltageDomain(voltage="1.0V")
        self.clk_domain = SrcClockDomain(
            clock="1GHz", voltage_domain=Parent.voltage_domain
        )

        if platform is None:
            self.realview = VExpress_GEM5_V1()
        else:
            self.realview = platform

        if hasattr(self.realview.gic, "cpu_addr"):
            self.gic_cpu_addr = self.realview.gic.cpu_addr

        self.terminal = Terminal()
        self.vncserver = VncServer()

        self.iobus = IOXBar()
        # Device DMA -> MEM.
        self.mem_ranges = self.getMemRanges(int(Addr(mem_size)))

    def getMemRanges(self, mem_size):
        """
        Define system memory ranges. This depends on the physical
        memory map provided by the realview platform and by the memory
        size provided by the user (mem_size argument).
        The method is iterating over all platform ranges until they cover
        the entire user's memory requirements.
        """
        mem_ranges = []
        for mem_range in self.realview._mem_regions:
            size_in_range = min(mem_size, mem_range.size())

            mem_ranges.append(
                AddrRange(start=mem_range.start, size=size_in_range)
            )

            mem_size -= size_in_range
            if mem_size == 0:
                return mem_ranges

        raise ValueError("memory size too big for platform capabilities")


class SimpleSystem(BaseSimpleSystem):
    """
    Meant to be used with the classic memory model
    """

    def __init__(
        self,
        want_caches,
        mem_size,
        l3_cache,
        l3_clock,
        l3_voltage,
        platform=None,
        **kwargs
    ):
        super().__init__(mem_size, platform, l3_cache, l3_clock, l3_voltage, **kwargs)

        self.membus = caches.MemBus()
        # CPUs->PIO
        self.iobridge = Bridge(delay="50ns")

        self._caches = want_caches
        if self._caches:
            self.iocache = caches.IOCache(addr_ranges=self.mem_ranges)
        else:
            self.dmabridge = Bridge(delay="50ns", ranges=self.mem_ranges)

    def connect(self):
        self.iobridge.mem_side_port = self.iobus.cpu_side_ports
        self.iobridge.cpu_side_port = self.membus.mem_side_ports

        if self._caches:
            self.iocache.mem_side = self.membus.cpu_side_ports
            self.iocache.cpu_side = self.iobus.mem_side_ports
        else:
            self.dmabridge.mem_side_port = self.membus.cpu_side_ports
            self.dmabridge.cpu_side_port = self.iobus.mem_side_ports

        if hasattr(self.realview.gic, "cpu_addr"):
            self.gic_cpu_addr = self.realview.gic.cpu_addr
        self.realview.attachOnChipIO(self.membus, self.iobridge)
        self.realview.attachIO(self.iobus)
        self.system_port = self.membus.cpu_side_ports

    def attach_pci(self, dev):
        self.realview.attachPciDevice(dev, self.iobus)
