from nmigen import *
from nmigen.test.utils import *
from nmigen.asserts import *
from nmigen.utils import log2_int

from nmigen_soc import wishbone

from ..periph.sdram import LiteDRAMNativePort, WritebackCache


class WritebackCacheSpec(Elaboratable):
    def __init__(self, dut):
        self.dut = dut

    def elaborate(self, platform):
        m = Module()
        m.submodules.dut = dut = self.dut

        # Wishbone interface

        with m.If(Initial() | ResetSignal()):
            m.d.comb += [
                Assume(~dut.intr_bus.cyc),
                Assume(~dut.intr_bus.stb),
                Assert(~dut.intr_bus.ack),
            ]

        with m.If(~dut.intr_bus.cyc & ~Past(dut.intr_bus.cyc)):
            m.d.comb += Assert(~dut.intr_bus.ack)

        with m.If(Past(dut.intr_bus.cyc) & Past(dut.intr_bus.stb)):
            m.d.comb += Assume(dut.intr_bus.bte == Past(dut.intr_bus.bte))
            with m.If(~Past(dut.intr_bus.ack)):
                m.d.comb += [
                    Assume(dut.intr_bus.adr   == Past(dut.intr_bus.adr)),
                    Assume(dut.intr_bus.we    == Past(dut.intr_bus.we)),
                    Assume(dut.intr_bus.dat_w == Past(dut.intr_bus.dat_w)),
                    Assume(dut.intr_bus.sel   == Past(dut.intr_bus.sel)),
                    Assume(dut.intr_bus.cti   == Past(dut.intr_bus.cti)),
                ]
            with m.Elif(Past(dut.intr_bus.cti) == wishbone.CycleType.INCR_BURST):
               intr_adr_past = Past(dut.intr_bus.adr)
               with m.Switch(dut.intr_bus.bte):
                   with m.Case(wishbone.BurstTypeExt.LINEAR):
                       m.d.comb += Assume(dut.intr_bus.adr == intr_adr_past + 1)
                   with m.Case(wishbone.BurstTypeExt.WRAP_4):
                       m.d.comb += Assume(dut.intr_bus.adr[:2] == intr_adr_past[:2] + 1)
                       m.d.comb += Assume(dut.intr_bus.adr[2:] == intr_adr_past[2:])
                   with m.Case(wishbone.BurstTypeExt.WRAP_8):
                       m.d.comb += Assume(dut.intr_bus.adr[:3] == intr_adr_past[:3] + 1)
                       m.d.comb += Assume(dut.intr_bus.adr[3:] == intr_adr_past[3:])
                   with m.Case(wishbone.BurstTypeExt.WRAP_16):
                       m.d.comb += Assume(dut.intr_bus.adr[:4] == intr_adr_past[:4] + 1)
                       m.d.comb += Assume(dut.intr_bus.adr[4:] == intr_adr_past[4:])
            with m.Else():
                m.d.comb += Assume(~dut.intr_bus.cyc)

        dram_addr = AnyConst(dut.dram_port.addr_width)
        dram_data = Signal(dut.dram_port.data_width)

        with m.If(Initial() | ResetSignal()):
            m.d.comb += Assume(dram_data == 0)

        ratio    = dut.dram_port.data_width // dut.intr_bus.data_width
        nb_lines = (dut.size * dut.intr_bus.granularity) // dut.dram_port.data_width
        intr_adr = Record([
            ("offset", range(ratio)),
            ("line",   range(nb_lines)),
            ("tag",    len(dut.intr_bus.adr) - log2_int(nb_lines) - log2_int(ratio)),
        ])
        m.d.comb += [
            Assert(len(intr_adr) == len(dut.intr_bus.adr)),
            intr_adr.eq(dut.intr_bus.adr),
        ]

        # Any read at `dram_addr` must be coherent with `dram_data`.
        with m.If(dut.intr_bus.cyc & dut.intr_bus.stb & dut.intr_bus.ack
                  & (Cat(intr_adr.line, intr_adr.tag) == dram_addr)):
            m.d.comb += dram_we.eq(1)
            with m.Switch(intr_adr.offset):
                for offset in range(ratio):
                    dram_data_word = dram_data.word_select(offset, dut.intr_bus.data_width)
                    with m.Case(offset):
                        m.d.comb += Assert(dut.intr_bus.dat_r == dram_data_word)
                        for i, intr_sel in enumerate(dut.intr_bus.sel):
                            intr_bits = dut.intr_bus.dat_w.word_select(i, dut.intr_bus.granularity)
                            dram_bits = dram_data_word    .word_select(i, dut.intr_bus.granularity)
                            with m.If(dut.intr_bus.we & intr_sel):
                                m.d.sync += dram_bits.eq(intr_bits)

        # DRAM interface

        dram_cmd_stb  = Signal()
        dram_cmd_addr = Signal.like(dut.dram_port.cmd.addr)
        dram_cmd_we   = Signal()
        dram_w_stb    = Signal()
        dram_w_data   = Signal.like(dut.dram_port.w.data)
        dram_r_stb    = Signal()
        dram_r_data   = Signal.like(dut.dram_port.r.data)

        with m.If(Initial() | ResetSignal()):
            m.d.comb += [
                Assume(~dram_cmd_stb),
                Assume(~dram_w_stb),
                Assume(~dram_r_stb),
            ]

        with m.If(dut.dram_port.cmd.valid):
            m.d.comb += Assert(~dram_cmd_stb)
            with m.If(dut.dram_port.cmd.ready):
                m.d.sync += [
                    dram_cmd_stb .eq(1),
                    dram_cmd_addr.eq(dut.dram_port.cmd.addr),
                    dram_cmd_we  .eq(dut.dram_port.cmd.we),
                ]

        with m.If(dut.dram_port.w.valid):
            m.d.comb += [
                Assert(~dram_w_stb),
                Assert(~dram_r_stb),
            ]
            with m.If(dut.dram_port.w.ready):
                m.d.sync += [
                    dram_w_stb .eq(1),
                    dram_w_data.eq(dut.dram_port.w.data),
                ]

        with m.If(dut.dram_port.r.ready):
            m.d.comb += [
                Assert(~dram_w_stb),
                Assert(~dram_r_stb),
            ]
            with m.If(dut.dram_port.r.valid):
                m.d.comb += Assume(dram_cmd_stb)
                with m.If(dram_cmd_addr == dram_addr):
                    m.d.comb += Assume(dut.dram_port.r.data == dram_data)
                m.d.sync += dram_r_stb.eq(1)

        m.d.comb += [
            Assert(dram_r_stb.implies(~dram_w_stb)),
            Assert(dram_w_stb.implies(~dram_r_stb)),
        ]

        with m.If(dram_cmd_stb & dram_w_stb):
            m.d.comb += [
                Assert(~dram_r_stb),
                Assert(dram_cmd_we),
            ]
            with m.If(dram_cmd_addr == dram_addr):
                m.d.comb += Assert(dram_w_data == dram_data)
            m.d.sync += [
                dram_cmd_stb.eq(0),
                dram_w_stb.eq(0),
            ]

        with m.If(dram_cmd_stb & dram_r_stb):
            m.d.comb += [
                Assert(~dram_w_stb),
                Assert(~dram_cmd_we),
            ]
            m.d.sync += [
                dram_cmd_stb.eq(0),
                dram_r_stb.eq(0),
            ]

        dram_cmd_wait = Signal(range(3))
        dram_w_wait   = Signal(range(3))
        dram_r_wait   = Signal(range(3))

        with m.If(dut.dram_port.cmd.valid & ~dut.dram_port.cmd.ready):
            m.d.sync += dram_cmd_wait.eq(dram_cmd_wait + 1)
        with m.Else():
            m.d.sync += dram_cmd_wait.eq(0)

        with m.If(dut.dram_port.w.valid & ~dut.dram_port.w.ready):
            m.d.sync += dram_w_wait.eq(dram_w_wait + 1)
        with m.Else():
            m.d.sync += dram_w_wait.eq(0)

        with m.If(dut.dram_port.r.ready & ~dut.dram_port.r.valid):
            m.d.sync += dram_r_wait.eq(dram_r_wait + 1)
        with m.Else():
            m.d.sync += dram_r_wait.eq(0)

        m.d.comb += [
            Assume(dram_cmd_wait < 2),
            Assume(dram_r_wait   < 2),
            Assume(dram_w_wait   < 2),
        ]

        return m


class WritebackCacheTestCase(FHDLTestCase):
    def check(self, dut):
        self.assertFormal(WritebackCacheSpec(dut), mode="hybrid", depth=13)

    def test_simple(self):
        dram_port = LiteDRAMNativePort(addr_width=23, data_width=32)
        self.check(WritebackCache(size=8, dram_port=dram_port, data_width=16, granularity=8))
