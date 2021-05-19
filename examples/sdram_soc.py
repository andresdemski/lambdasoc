import argparse

from nmigen import *
from nmigen_soc import wishbone

from lambdasoc.cpu.minerva import MinervaCPU
from lambdasoc.periph.intc import GenericInterruptController
from lambdasoc.periph.serial import AsyncSerialPeripheral
from lambdasoc.periph.sram import SRAMPeripheral
from lambdasoc.periph.timer import TimerPeripheral
from lambdasoc.periph.sdram import SDRAMPeripheral
from lambdasoc.soc.cpu import CPUSoC


__all__ = ["SRAMSoC"]


class SDRAMSoC(CPUSoC, Elaboratable):
    def __init__(self, *, reset_addr, clk_freq,
                 rom_addr, rom_size,
                 ram_addr, ram_size,
                 uart_addr, uart_divisor, uart_pins,
                 timer_addr, timer_width,
                 sdram_addr, sdram_pins):
        self._arbiter = wishbone.Arbiter(addr_width=30, data_width=32, granularity=8,
                                         features={"cti", "bte"})
        self._decoder = wishbone.Decoder(addr_width=30, data_width=32, granularity=8,
                                         features={"cti", "bte"})

        self.cpu = MinervaCPU(
            reset_address=reset_addr,
            with_icache=True, icache_nlines=128, icache_nwords=4, icache_nways=1,
                              icache_base=0x10000000, icache_limit=0x40000000,
            with_dcache=True, dcache_nlines=128, dcache_nwords=4, dcache_nways=1,
                              dcache_base=0x10000000, dcache_limit=0x40000000,
            with_muldiv=True,
        )
        self._arbiter.add(self.cpu.ibus)
        self._arbiter.add(self.cpu.dbus)

        self.rom = SRAMPeripheral(size=rom_size, writable=False)
        self._decoder.add(self.rom.bus, addr=rom_addr)

        self.ram = SRAMPeripheral(size=ram_size)
        self._decoder.add(self.ram.bus, addr=ram_addr)

        self.uart = AsyncSerialPeripheral(divisor=uart_divisor, pins=uart_pins)
        self._decoder.add(self.uart.bus, addr=uart_addr)

        self.timer = TimerPeripheral(width=timer_width)
        self._decoder.add(self.timer.bus, addr=timer_addr)

        self._sdram = SDRAMPeripheral(pins=sdram_pins)
        self._decoder.add(self._sdram.bus, addr=sdram_addr)

        self.intc = GenericInterruptController(width=len(self.cpu.ip))
        self.intc.add_irq(self.timer.irq, 0)
        self.intc.add_irq(self.uart .irq, 1)

        self.memory_map = self._decoder.bus.memory_map

        self.clk_freq = clk_freq

    def elaborate(self, platform):
        m = Module()

        m.submodules.arbiter = self._arbiter
        m.submodules.cpu     = self.cpu

        m.submodules.decoder = self._decoder
        m.submodules.rom     = self.rom
        m.submodules.ram     = self.ram
        m.submodules.uart    = self.uart
        m.submodules.timer   = self.timer
        m.submodules.intc    = self.intc
        m.submodules.sdram   = self._sdram

        m.d.comb += [
            self._arbiter.bus.connect(self._decoder.bus),
            self.cpu.ip.eq(self.intc.ip),
        ]

        return m


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--baudrate", type=int,
            default=9600,
            help="UART baudrate (default: 9600)")
    args = parser.parse_args()

    from nmigen_boards.ecpix5 import ECPIX585Platform
    platform = ECPIX585Platform()

    clk_freq = int(70e6)
    sdram_pins = platform.request("ddr3", 0, dir={
        pin: "-" for pin in ("clk", "clk_en", "we", "ras", "cas", "a", "ba", "dqs", "dq", "dm", "odt")
    })

    soc = SDRAMSoC(
         reset_addr=0x10000000, clk_freq=clk_freq,
          uart_addr=0x00005000, uart_divisor=int(clk_freq // args.baudrate),
                                uart_pins=platform.request("uart", 0),
         timer_addr=0x00006000, timer_width=32,

           rom_addr=0x10000000, rom_size=0x4000,
           ram_addr=0x10004000, ram_size=0x1000,
         sdram_addr=0x20000000, sdram_pins=sdram_pins,
    )

    soc.build(do_build=True, do_init=True)

    with open("litedram_core_70MHz_native.v", "r") as f:
        platform.add_file("litedram_core_70MHz_native.v", f)
    platform.build(soc, do_program=True)
