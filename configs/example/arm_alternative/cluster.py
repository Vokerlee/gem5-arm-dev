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
import numpy as np

from m5.defines import buildEnv
from m5.objects import *


class ArmCpuCluster(CpuCluster):
    def __init__(
        self,
        system,
        num_cpus,
        cpu_type,
        l1i_type=None,
        l1d_type=None,
        l2_type=None,
        split_l2_cache=False,
        itlb_size=0,
        dtlb_size=0,
        l2tlb_size=0,
        cpu_clock="1GHz",
        cpu_voltage="1V",
    ):
        super().__init__()
        self._cpu_type = cpu_type
        assert num_cpus > 0

        self._l1i_type = l1i_type
        self._l1d_type = l1d_type
        self._l2_type = l2_type

        self._have_l1 = False
        self._have_l2 = False

        # Typically, out-of-order clusters have separate L2-caches
        self._split_l2_cache = split_l2_cache

        self._itlb_size = itlb_size
        self._dtlb_size = dtlb_size
        self._l2tlb_size = l2tlb_size

        self.voltage_domain = VoltageDomain(voltage=cpu_voltage)
        self.clk_domain = SrcClockDomain(
            clock=cpu_clock,
            voltage_domain=self.voltage_domain,
            domain_id=system.getUniqueDomainId(),
        )

        self.generate_cpus(cpu_type, num_cpus)

        for cpu in self.cpus:
            cpu.mmu.itb.size = self._itlb_size
            cpu.mmu.dtb.size = self._dtlb_size

            # Actually we don't need it, as we don't need virtualization
            cpu.mmu.stage2_itb.size = self._itlb_size
            cpu.mmu.stage2_dtb.size = self._dtlb_size

            cpu.mmu.l2_shared.size = self._l2tlb_size
            cpu.mmu.l2_shared.partial_levels = ["L1", "L2"]

        system.addCpuCluster(self)

    def addL1(self):
        for cpu in self.cpus:
            l1i = None if self._l1i_type is None else self._l1i_type()
            l1d = None if self._l1d_type is None else self._l1d_type()

            cpu.addPrivateSplitL1Caches(l1i, l1d)

            self._have_l1 = True

    def addL2(self, clk_domain, l2_bus_width=64):
        if self._l2_type is None:
            return

        if self._split_l2_cache:
            for i, cpu in enumerate(self.cpus):
                cpu.addL2Cache(
                    self._l2_type(),
                    L2XBar(width=l2_bus_width, clk_domain=clk_domain),
                )
        else:
            self.toL2Bus = L2XBar(width=l2_bus_width, clk_domain=clk_domain)
            self.l2 = self._l2_type()
            self.toL2Bus.mem_side_ports = self.l2.cpu_side

            for cpu in self.cpus:
                cpu.connectCachedPorts(self.toL2Bus.cpu_side_ports)

        self._have_l2 = True

    def connectMemSide(self, bus):
        try:
            if self._split_l2_cache:
                for i, cpu in enumerate(self.cpus):
                    cpu.connectCachedPorts(bus.cpu_side_ports)
            else:
                self.l2.mem_side = bus.cpu_side_ports

        except AttributeError:
            for cpu in self.cpus:
                cpu.connectCachedPorts(bus.cpu_side_ports)

    def addClusterRelatedPMUs(
        self,
        ints,
        events=[],
        cpu_requestor_init=0,
        exit_sim_on_control=False,
        exit_sim_on_interrupt=False,
    ):
        assert len(ints) == len(self.cpus)

        cpu_requestor = cpu_requestor_init
        for cpu, pint in zip(self.cpus, ints):
            int_class = ArmPPI if pint < 32 else ArmSPI

            for isa in cpu.isa:
                isa.pmu = ArmPMU(interrupt=int_class(num=pint))
                isa.pmu.exitOnPMUControl = exit_sim_on_control
                isa.pmu.exitOnPMUInterrupt = exit_sim_on_interrupt

                # L1-cache related PMU
                if self._have_l1:
                    isa.pmu.addEvent(
                        ProbeEvent(
                            isa.pmu,
                            0x14,
                            cpu.icache,
                            "MissEvents" + str(cpu_requestor),
                            "HitEvents" + str(cpu_requestor),
                        )
                    )  # 0x14: L1I_CACHE
                    isa.pmu.addEvent(
                        ProbeEvent(
                            isa.pmu,
                            0x01,
                            cpu.icache,
                            "RefillEvents" + str(cpu_requestor),
                        )
                    )  # 0x01: L1I_CACHE_REFILL

                    isa.pmu.addEvent(
                        ProbeEvent(
                            isa.pmu,
                            0x04,
                            cpu.dcache,
                            "MissEvents" + str(cpu_requestor + 1),
                            "HitEvents" + str(cpu_requestor + 1),
                        )
                    )  # 0x04: L1D_CACHE
                    isa.pmu.addEvent(
                        ProbeEvent(
                            isa.pmu,
                            0x03,
                            cpu.dcache,
                            "RefillEvents" + str(cpu_requestor + 1),
                        )
                    )  # 0x03: L1D_CACHE_REFILL

                    # 0x15: L1D_CACHE_WB

                    # Fake event: cache misses are counted instead
                    isa.pmu.addEvent(
                        ProbeEvent(
                            isa.pmu,
                            0x1F,
                            cpu.dcache,
                            "MissEvents" + str(cpu_requestor + 1),
                        )
                    )  # 0x1F: L1D_CACHE_ALLOCATE

                # L2-cache related PMU
                if self._have_l2:
                    if self._split_l2_cache:
                        l2 = cpu.l2cache
                    else:  # L2 cache is shared between cpus in a cluster
                        l2 = self.l2

                    isa.pmu.addEvent(
                        ProbeEvent(
                            isa.pmu,
                            0x16,
                            l2,
                            "MissEvents" + str(cpu_requestor),
                            "MissEvents" + str(cpu_requestor + 1),
                            "HitEvents" + str(cpu_requestor),
                            "HitEvents" + str(cpu_requestor + 1),
                        )
                    )  # 0x16: L2D_CACHE

                    isa.pmu.addEvent(
                        ProbeEvent(
                            isa.pmu,
                            0x17,
                            l2,
                            "RefillEvents" + str(cpu_requestor),
                            "RefillEvents" + str(cpu_requestor + 1),
                        )
                    )  # 0x17: L2D_CACHE_REFILL

                    # Fake event: cache misses are counted instead
                    isa.pmu.addEvent(
                        ProbeEvent(
                            isa.pmu,
                            0x20,
                            l2,
                            "MissEvents" + str(cpu_requestor),
                            "MissEvents" + str(cpu_requestor + 1),
                        )
                    )  # 0x20: L2D_CACHE_ALLOCATE

                # 0x18: L2D_CACHE_WB

                # 0x27: L2I_CACHE
                # 0x28: L2I_CACHE_REFILL

                # Other memory-related PMU
                isa.pmu.addEvent(
                    ProbeEvent(isa.pmu, 0x02, cpu.mmu.itb, "Refills")
                )  # 0x02: L1I_TLB_REFILL,
                isa.pmu.addEvent(
                    ProbeEvent(isa.pmu, 0x05, cpu.mmu.dtb, "Refills")
                )  # 0x05: L1D_TLB_REFILL
                isa.pmu.addEvent(
                    ProbeEvent(isa.pmu, 0x06, cpu, "RetiredLoads")
                )  # 0x06: LD_RETIRED
                isa.pmu.addEvent(
                    ProbeEvent(isa.pmu, 0x07, cpu, "RetiredStores")
                )  # 0x07: ST_RETIRED
                isa.pmu.addEvent(
                    ProbeEvent(
                        isa.pmu, 0x13, cpu, "RetiredLoads", "RetiredStores"
                    )
                )  # 0x13: MEM_ACCESS

                # 0x25: L1D_TLB
                # 0x26: L1I_TLB
                # 0x2D: L2D_TLB_REFILL
                # 0x2E: L2I_TLB_REFILL
                # 0x2F: L2D_TLB
                # 0x30: L2I_TLB

                # CPU-branch related PMU
                brpred = getattr(
                    cpu, "branchPred", None
                )  # BaseCPU doesn't have branch predictor
                if brpred is not None:
                    isa.pmu.addEvent(
                        ProbeEvent(isa.pmu, 0x10, brpred, "Misses")
                    )  # 0x10: BR_MIS_PRED
                    isa.pmu.addEvent(
                        ProbeEvent(isa.pmu, 0x12, brpred, "Branches")
                    )  # 0x12: BR_PRED

                isa.pmu.addEvent(
                    ProbeEvent(isa.pmu, 0x21, cpu, "RetiredBranches")
                )  # 0x21: BR_RETIRED

                # General PMU
                isa.pmu.addEvent(
                    SoftwareIncrement(isa.pmu, 0x00)
                )  # 0x00: SW_INCR
                isa.pmu.addEvent(
                    ProbeEvent(isa.pmu, 0x11, cpu, "ActiveCycles")
                )  # 0x11: CPU_CYCLES
                isa.pmu.addEvent(
                    ProbeEvent(isa.pmu, 0x08, cpu, "RetiredInsts")
                )  # 0x08: INST_RETIRED

                for ev in events:
                    isa.pmu.addEvent(ev)

                cpu_requestor += 2  # 1 for data and 1 for instruction

        return cpu_requestor


class ClusterSystem:
    def __init__(self, l3_cache, l3_clock, l3_voltage, **kwargs):
        self._clusters = []
        self._n_clock_domain_id = 0

        self._l3_clock = l3_clock
        self._l3_voltage = l3_voltage

        self._l3_cache = l3_cache
        self._caches_added = False

    def numCpuClusters(self):
        return len(self._clusters)

    def getUniqueDomainId(self):
        self._n_clock_domain_id += 1
        return self._n_clock_domain_id - 1

    def addCpuCluster(self, cpu_cluster):
        self._clusters.append(cpu_cluster)

    def addCaches(self, need_caches, last_cache_level):
        if not need_caches:
            for cluster in self._clusters:
                cluster.connectMemSide(self.membus)
            return

        cluster_mem_bus = self.membus
        assert last_cache_level >= 1 and last_cache_level <= 3

        for cluster in self._clusters:
            cluster.addL1()

        if last_cache_level > 1:
            for cluster in self._clusters:
                cluster.addL2(cluster.clk_domain)

        if last_cache_level > 2:
            l3_voltage = VoltageDomain(voltage=self._l3_voltage)
            l3_clock = SrcClockDomain(
                clock=self._l3_clock,
                voltage_domain=l3_voltage,
                domain_id=0x401
            )

            self.l3 = self._l3_cache(clk_domain=l3_clock)
            self.toL3Bus = L2XBar(width=64)
            self.toL3Bus.mem_side_ports = self.l3.cpu_side
            self.l3.mem_side = self.membus.cpu_side_ports

            cluster_mem_bus = self.toL3Bus
        elif self._l3_cache != None:
            return False

        for cluster in self._clusters:
            cluster.connectMemSide(cluster_mem_bus)

        self._caches_added = True
        return True

    def addL3RelatedPMUs(self, clusters=None):
        cpu_requestor = 0

        if clusters == None:
            clusters = self._clusters

        if self._caches_added == True and self._l3_cache != None:
            for cluster in clusters:
                for cpu in cluster.cpus:
                    for isa in cpu.isa:
                        if isa.pmu != None:
                            isa.pmu.addEvent(
                                ProbeEvent(
                                    isa.pmu,
                                    0x2B,
                                    self.l3,
                                    "MissEvents" + str(cpu_requestor),
                                    "HitEvents" + str(cpu_requestor),
                                    "MissEvents" + str(cpu_requestor + 1),
                                    "HitEvents" + str(cpu_requestor + 1),
                                )
                            )  # 0x2B: L3D_CACHE
                            isa.pmu.addEvent(
                                ProbeEvent(
                                    isa.pmu,
                                    0x2A,
                                    self.l3,
                                    "RefillEvents" + str(cpu_requestor),
                                    "RefillEvents" + str(cpu_requestor + 1),
                                )
                            )  # 0x2A: L3D_CACHE_REFILL

                            # Fake event: cache misses are counted instead
                            isa.pmu.addEvent(
                                ProbeEvent(
                                    isa.pmu,
                                    0x29,
                                    self.l3,
                                    "MissEvents" + str(cpu_requestor),
                                    "MissEvents" + str(cpu_requestor + 1),
                                )
                            )  # 0x29: L3D_CACHE_ALLOCATE

                            # 0x2C: L3D_CACHE_WB

                            cpu_requestor += (
                                2  # 1 for data and 1 for instruction
                            )