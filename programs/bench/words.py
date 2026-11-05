"""Every program assembles with sim/asm.py, to the word count section 11.2 of docs/ISA.md lists for it,
and fits the memory depth its bench runs it at."""
from report import assemble, check, src

# path: (words, memory depth the bench uses)
WORDS = {
    'basic/uart_tx.s': (15, 128), 'basic/uart_rx.s': (23, 128), 'basic/spi.s': (20, 128),
    'basic/i2c_write.s': (42, 128), 'basic/ws2812.s': (19, 128), 'basic/onewire.s': (34, 128),
    'basic/memhog.s': (8, 512),
    'standard/uart_tx.s': (15, 128), 'standard/uart_rx.s': (22, 128), 'standard/spi.s': (22, 128),
    'standard/i2c.s': (76, 128), 'standard/i2c_write.s': (42, 128), 'standard/i2c_target.s': (62, 512),
    'standard/ws2812.s': (17, 128), 'standard/onewire.s': (28, 128), 'standard/jtag.s': (51, 128),
    'standard/cec.s': (50, 128), 'standard/crc16.s': (14, 128), 'standard/crc8.s': (12, 128),
    'standard/crc32.s': (26, 128), 'standard/hdlc_tx.s': (48, 128), 'standard/manch_rx.s': (13, 128),
    'standard/reflex.s': (15, 128), 'standard/memstress.s': (17, 128),
    'trimmed/uart_tx.s': (14, 128), 'trimmed/uart_rx.s': (20, 128), 'trimmed/i2c.s': (72, 128),
    'extra/spi_target.s': (17, 128), 'extra/parallel.s': (7, 128), 'extra/swd.s': (64, 128),
    'extra/i2c.s': (76, 128), 'extra/lin.s': (52, 128), 'extra/nec_tx.s': (41, 128),
    'advanced/manch_rx.s': (28, 128), 'advanced/dmx_tx.s': (29, 128), 'advanced/dmx_rx.s': (52, 128),
    'advanced/ps2_dev.s': (114, 128), 'advanced/mdio_sta.s': (47, 128), 'advanced/mdio_phy.s': (60, 128),
    'advanced/jtag.s': (37, 128), 'advanced/swd.s': (128, 128), 'advanced/i2c_tgt.s': (78, 128),
    'advanced/i2c_fp.s': (87, 128), 'advanced/usb_tx.s': (43, 128), 'advanced/usb_rx.s': (69, 128),
    'advanced/usb_dev.s': (194, 512), 'advanced/can.s': (166, 512),
}
PROGRAMS = tuple(WORDS)


def main(log):
    log.line('--- every program: word count')
    for rel, (want, depth) in WORDS.items():
        try:
            img = assemble(src(rel), depth=depth)
            got, err = img.size, ''
        except Exception as e:      # an assembler error is a failed check, not a crash
            got, err = None, str(e)
        check('%s assembles to %d words, the count in section 11.2, and fits %d words' % (rel, want, depth),
              got == want, err or '%d words' % got, progs=(rel,))
