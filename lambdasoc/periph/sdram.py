from nmigen import *
from nmigen.asserts import *
from nmigen.utils import log2_int

from nmigen_soc import wishbone

from . import Peripheral


__all__ = ["LiteDRAMNativePort", "WritebackCache", "SDRAMPeripheral"]


class LiteDRAMNativePort(Record):
    def __init__(self, addr_width, data_width, name=None, src_loc_at=0):
        if not isinstance(addr_width, int) or addr_width <= 0:
            raise ValueError("Address width must be a positive integer, not {!r}"
                             .format(addr_width))
        if not isinstance(data_width, int) or data_width <= 0 or data_width & data_width - 1:
            raise ValueError("Data width must be a positive power of two integer, not {!r}"
                             .format(data_width))

        self.addr_width  = addr_width
        self.data_width  = data_width

        super().__init__([
            ("cmd", [
                ("valid", 1),
                ("ready", 1),
                ("last",  1),
                ("we",    1),
                ("addr",  addr_width),
            ]),
            ("w", [
                ("valid", 1),
                ("ready", 1),
                ("data",  data_width),
                ("we",    data_width // 8),
            ]),
            ("r", [
                ("valid", 1),
                ("ready", 1),
                ("data",  data_width),
            ]),
        ], name=name, src_loc_at=1 + src_loc_at)


class WritebackCache(Elaboratable):
    def __init__(self, dram_port, *, size, data_width, granularity=None):
        if not isinstance(dram_port, LiteDRAMNativePort):
            raise TypeError("DRAM port must be an instance of LiteDRAMNativePort, not {!r}"
                            .format(dram_port))
        if not isinstance(size, int) or size <= 0 or size & size - 1:
            raise ValueError("Cache size must be a positive power of two integer, not {!r}"
                             .format(size))
        if not isinstance(data_width, int) or data_width <= 0 or data_width & data_width - 1:
            raise ValueError("Data width must be a positive power of two integer, not {!r}"
                             .format(data_width))
        if dram_port.data_width % data_width != 0:
            raise ValueError("DRAM port data width {} must be a multiple of data width {}"
                             .format(dram_port.data_width, data_width))

        self.size      = size
        self.dram_port = dram_port
        self.intr_bus  = wishbone.Interface(
            addr_width=dram_port.addr_width + log2_int(dram_port.data_width // data_width),
            data_width=data_width,
            granularity=granularity,
            features={"cti", "bte"},
        )
        # TODO: memory map

        ratio    = self.dram_port.data_width // self.intr_bus.data_width
        nb_lines = (self.size * self.intr_bus.granularity) // self.dram_port.data_width

    def elaborate(self, platform):
        m = Module()

        ratio    = self.dram_port.data_width // self.intr_bus.data_width
        nb_lines = (self.size * self.intr_bus.granularity) // self.dram_port.data_width

        intr_adr = Record([
            ("offset", log2_int(ratio)),
            ("line",   log2_int(nb_lines)),
            ("tag",    len(self.intr_bus.adr) - log2_int(nb_lines) - log2_int(ratio)),
        ])
        m.d.comb += intr_adr.eq(self.intr_bus.adr),

        intr_adr_next = Record.like(intr_adr)

        with m.Switch(self.intr_bus.bte):
            with m.Case(wishbone.BurstTypeExt.LINEAR):
                m.d.comb += intr_adr_next.eq(intr_adr + 1)
            with m.Case(wishbone.BurstTypeExt.WRAP_4):
                m.d.comb += intr_adr_next[:2].eq(intr_adr[:2] + 1)
                m.d.comb += intr_adr_next[2:].eq(intr_adr[2:])
            with m.Case(wishbone.BurstTypeExt.WRAP_8):
                m.d.comb += intr_adr_next[:3].eq(intr_adr[:3] + 1)
                m.d.comb += intr_adr_next[3:].eq(intr_adr[3:])
            with m.Case(wishbone.BurstTypeExt.WRAP_16):
                m.d.comb += intr_adr_next[:4].eq(intr_adr[:4] + 1)
                m.d.comb += intr_adr_next[4:].eq(intr_adr[4:])

        tag_rp_data = Record([
            ("tag",   intr_adr.tag.shape()),
            ("dirty", 1),
        ])
        tag_wp_data = Record.like(tag_rp_data)

        tag_mem = Memory(width=len(tag_rp_data), depth=nb_lines)
        m.submodules.tag_rp = tag_rp = tag_mem.read_port(transparent=False)
        m.submodules.tag_wp = tag_wp = tag_mem.write_port()
        tag_rp.en.reset = 0

        m.d.comb += [
            tag_rp_data.eq(tag_rp.data),
            tag_wp.data.eq(tag_wp_data),
        ]

        dat_mem = Memory(width=self.dram_port.data_width, depth=nb_lines)
        m.submodules.dat_rp = dat_rp = dat_mem.read_port(transparent=False)
        m.submodules.dat_wp = dat_wp = dat_mem.write_port(granularity=self.intr_bus.granularity)
        dat_rp.en.reset = 0

        intr_bus_r = Record.like(self.intr_bus)
        intr_adr_r = Record.like(intr_adr)
        m.d.comb += intr_adr_r.eq(intr_bus_r.adr)

        with m.FSM() as fsm:
            with m.State("CHECK"):
                m.d.sync += [
                    intr_bus_r.cyc.eq(self.intr_bus.cyc),
                    intr_bus_r.stb.eq(self.intr_bus.stb),
                    intr_bus_r.adr.eq(self.intr_bus.adr),
                ]
                # Tag/Data memory read
                with m.If(self.intr_bus.cyc & self.intr_bus.stb):
                    with m.If(self.intr_bus.ack & (self.intr_bus.cti == wishbone.CycleType.INCR_BURST)):
                        m.d.comb += [
                            tag_rp.addr.eq(intr_adr_next.line),
                            dat_rp.addr.eq(intr_adr_next.line),
                        ]
                    with m.Else():
                        m.d.comb += [
                            tag_rp.addr.eq(intr_adr.line),
                            dat_rp.addr.eq(intr_adr.line),
                        ]
                    with m.If(~intr_bus_r.cyc | ~intr_bus_r.stb | self.intr_bus.ack):
                        m.d.comb += [
                            tag_rp.en.eq(1),
                            dat_rp.en.eq(1),
                        ]
                m.d.comb += [
                    self.intr_bus.dat_r.eq(
                        dat_rp.data.word_select(intr_adr.offset, len(self.intr_bus.dat_r))
                    ),
                ]
                # Tag/Data memory write
                m.d.comb += [
                    tag_wp.addr      .eq(intr_adr.line),
                    tag_wp_data.tag  .eq(intr_adr.tag),
                    tag_wp_data.dirty.eq(1),
                    dat_wp.addr      .eq(intr_adr.line),
                    dat_wp.data      .eq(Repl(self.intr_bus.dat_w, ratio)),
                ]
                with m.If(self.intr_bus.cyc & self.intr_bus.stb):
                    with m.If(intr_adr.tag == tag_rp_data.tag):
                        m.d.comb += self.intr_bus.ack.eq(intr_bus_r.cyc & intr_bus_r.stb)
                        with m.If(self.intr_bus.we & self.intr_bus.ack):
                            m.d.comb += [
                                tag_wp.en.eq(1),
                                dat_wp.en.word_select(intr_adr.offset, len(self.intr_bus.sel)).eq(self.intr_bus.sel),
                            ]
                    with m.Elif(intr_bus_r.cyc & intr_bus_r.stb):
                        m.d.sync += [
                            intr_bus_r.cyc.eq(0),
                            intr_bus_r.stb.eq(0),
                        ]
                        with m.If(tag_rp_data.dirty):
                            m.next = "EVICT"
                        with m.Else():
                            m.next = "REFILL"

            with m.State("EVICT"):
                evict_done = Record([("cmd", 1), ("w", 1)])
                with m.If(evict_done.all()):
                    m.d.sync += evict_done.eq(0)
                    m.next = "REFILL"
                # Command
                m.d.comb += [
                    self.dram_port.cmd.valid.eq(~evict_done.cmd),
                    self.dram_port.cmd.last .eq(0),
                    self.dram_port.cmd.addr .eq(Cat(intr_adr_r.line, tag_rp_data.tag)),
                    self.dram_port.cmd.we   .eq(1),
                ]
                with m.If(self.dram_port.cmd.valid & self.dram_port.cmd.ready):
                    m.d.sync += evict_done.cmd.eq(1)
                # Write
                m.d.comb += [
                    self.dram_port.w.valid.eq(~evict_done.w),
                    self.dram_port.w.we   .eq(Repl(Const(1), self.dram_port.data_width // 8)),
                    self.dram_port.w.data .eq(dat_rp.data),
                ]
                with m.If(self.dram_port.w.valid & self.dram_port.w.ready):
                    m.d.sync += evict_done.w.eq(1)

            with m.State("REFILL"):
                refill_done = Record([("cmd", 1), ("r", 1)])
                with m.If(refill_done.all()):
                    m.d.sync += refill_done.eq(0)
                    m.next = "CHECK"
                # Command
                m.d.comb += [
                    self.dram_port.cmd.valid.eq(~refill_done.cmd),
                    self.dram_port.cmd.last .eq(1),
                    self.dram_port.cmd.addr .eq(Cat(intr_adr_r.line, intr_adr_r.tag)),
                    self.dram_port.cmd.we   .eq(0),
                ]
                with m.If(self.dram_port.cmd.valid & self.dram_port.cmd.ready):
                    m.d.sync += refill_done.cmd.eq(1)
                # Read
                m.d.comb += [
                    self.dram_port.r.ready.eq(~refill_done.r),
                    tag_wp.addr      .eq(intr_adr_r.line),
                    tag_wp.en        .eq((self.dram_port.r.valid & self.dram_port.r.ready)),
                    tag_wp_data.tag  .eq(intr_adr_r.tag),
                    tag_wp_data.dirty.eq(0),
                    dat_wp.addr      .eq(intr_adr_r.line),
                    dat_wp.en        .eq(Repl((self.dram_port.r.valid & self.dram_port.r.ready), len(dat_wp.en))),
                    dat_wp.data      .eq(self.dram_port.r.data),
                ]
                with m.If(self.dram_port.r.valid & self.dram_port.r.ready):
                    m.d.sync += refill_done.r.eq(1)

        if platform == "formal":
            with m.If(Initial()):
                m.d.comb += [
                    Assume(fsm.ongoing("CHECK")),
                    Assume(~intr_bus_r.cyc),
                    Assume(~evict_done.any()),
                    Assume(~refill_done.any()),
                ]

        return m


# TODO:
# - memory map
# - validate pins
# - don't hardcode widths
class SDRAMPeripheral(Peripheral, Elaboratable):
    def __init__(self, *, pins=None):
        super().__init__()
        self._data_bus = self.window(addr_width=27, data_width=32, granularity=8,
                                     features={"cti", "bte"},
                                     addr=0x20000000) # FIXME should be 0x0 ??
        self._ctrl_bus = self.window(addr_width=14, data_width=32, granularity=8,
                                     features={"cti", "bte"},
                                     addr=0x40000000) # FIXME should be 0x20000000

        dram_port      = LiteDRAMNativePort(addr_width=25, data_width=128)
        self._cache    = WritebackCache(dram_port, size=8192, data_width=32, granularity=8)
        self._bridge   = self.bridge(data_width=32, granularity=8)
        self.bus       = self._bridge.bus

        self._pins = pins
        self.ddr3  = Record([
            ("clk",     [("p", 1), ("n", 1)]),
            ("clk_en",  1),
            ("we",      1),
            ("cs",      1),
            ("ras",     1),
            ("cas",     1),
            ("a",      15),
            ("ba",      3),
            ("dqs",     [("p", 2), ("n", 2)]),
            ("dq",     16),
            ("dm",      2),
            ("odt",     1),
        ])

    def elaborate(self, platform):
        m = Module()

        if self._pins is not None:
            m.d.comb += [
                self._pins.a     .eq(self.ddr3.a),
                self._pins.ba    .eq(self.ddr3.ba),
                self._pins.ras   .eq(self.ddr3.ras),
                self._pins.cas   .eq(self.ddr3.cas),
                self._pins.we    .eq(self.ddr3.we),
                #self._pins.cs  .eq(self.ddr3.cs),
                self._pins.dm    .eq(self.ddr3.dm),
                self._pins.odt   .eq(self.ddr3.odt),
                self._pins.clk.p .eq(self.ddr3.clk.p),
                #self._pins.clk.n .eq(self.ddr3.clk.n),
                self._pins.clk_en.eq(self.ddr3.clk_en),

                self.ddr3.dq   .eq(self._pins.dq),
                self.ddr3.dqs.p.eq(self._pins.dqs.p),
                #self.ddr3.dqs.n.eq(self._pins.dqs.n),
            ]

        m.submodules.bridge = self._bridge

        m.submodules.cache = self._cache
        m.d.comb += [
            self._cache.intr_bus.adr  .eq(self._data_bus.adr),
            self._cache.intr_bus.cyc  .eq(self._data_bus.cyc),
            self._cache.intr_bus.stb  .eq(self._data_bus.stb),
            self._cache.intr_bus.sel  .eq(self._data_bus.sel),
            self._cache.intr_bus.we   .eq(self._data_bus.we),
            self._cache.intr_bus.dat_w.eq(self._data_bus.dat_w),
            self._cache.intr_bus.cti  .eq(self._data_bus.cti),
            self._cache.intr_bus.bte  .eq(self._data_bus.bte),
            self._data_bus.ack  .eq(self._cache.intr_bus.ack),
            self._data_bus.dat_r.eq(self._cache.intr_bus.dat_r),
        ]

        m.domains.sync = ClockDomain("sync")

        m.submodules.litedram_core = Instance("litedram_core",
            i_clk=platform.request("clk100", 0).i,
            i_rst=platform.request("rst", 0).i,

            o_user_clk = ClockSignal("sync"),
            o_user_rst = ResetSignal("sync"),

            i_wb_ctrl_adr   = self._ctrl_bus.adr,
            i_wb_ctrl_dat_w = self._ctrl_bus.dat_w,
            o_wb_ctrl_dat_r = self._ctrl_bus.dat_r,
            i_wb_ctrl_sel   = self._ctrl_bus.sel,
            i_wb_ctrl_cyc   = self._ctrl_bus.cyc,
            i_wb_ctrl_stb   = self._ctrl_bus.stb,
            o_wb_ctrl_ack   = self._ctrl_bus.ack,
            i_wb_ctrl_we    = self._ctrl_bus.we,
            i_wb_ctrl_cti   = self._ctrl_bus.cti,
            i_wb_ctrl_bte   = self._ctrl_bus.bte,

            i_user_port_native_0_cmd_valid   = self._cache.dram_port.cmd.valid,
            o_user_port_native_0_cmd_ready   = self._cache.dram_port.cmd.ready,
            i_user_port_native_0_cmd_we      = self._cache.dram_port.cmd.we,
            i_user_port_native_0_cmd_addr    = self._cache.dram_port.cmd.addr,
            i_user_port_native_0_wdata_valid = self._cache.dram_port.w.valid,
            o_user_port_native_0_wdata_ready = self._cache.dram_port.w.ready,
            i_user_port_native_0_wdata_we    = self._cache.dram_port.w.we,
            i_user_port_native_0_wdata_data  = self._cache.dram_port.w.data,
            o_user_port_native_0_rdata_valid = self._cache.dram_port.r.valid,
            i_user_port_native_0_rdata_ready = self._cache.dram_port.r.ready,
            o_user_port_native_0_rdata_data  = self._cache.dram_port.r.data,

            o_ddram_a       = self.ddr3.a,
            o_ddram_ba      = self.ddr3.ba,
            o_ddram_ras_n   = self.ddr3.ras,
            o_ddram_cas_n   = self.ddr3.cas,
            o_ddram_we_n    = self.ddr3.we,
            o_ddram_cs_n    = self.ddr3.cs,
            o_ddram_dm      = self.ddr3.dm,
            i_ddram_dq      = self.ddr3.dq,
            i_ddram_dqs_p   = self.ddr3.dqs.p,
            i_ddram_dqs_n   = self.ddr3.dqs.n,
            o_ddram_clk_p   = self.ddr3.clk.p,
            o_ddram_cke     = self.ddr3.clk_en,
            o_ddram_odt     = self.ddr3.odt,
        )

        return m
