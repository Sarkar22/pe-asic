# SPDX-License-Identifier: Apache-2.0
"""Run the bring-up against a recorded transcript, checking that every transaction sends the recorded bytes.

    python3 host/test_host.py --transcript bringup.txt    # record one, from the simulator
    micropython host/replay.py bringup.txt                # MicroPython's unix port must send the same bytes

The transcript has one line per call the library makes on its transport: "x OUT REPLY" in hex for a transaction,
"p US" for a pause, "r UO UIO" for a read of the pads. It runs under CPython too.
"""
import sys
import binascii

try:
    sys.dont_write_bytecode = True      # CPython: leave no cache beside the sources
except AttributeError:
    pass                                # MicroPython
HERE = sys.argv[0].rsplit('/', 1)[0] if '/' in sys.argv[0] else '.'
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import pehost   # noqa: E402
import bringup  # noqa: E402


class Mismatch(Exception):
    pass


class Replay:
    def __init__(self, lines):
        self.calls = [line.split() for line in lines if line.strip()]
        self.i = 0
        self.count = {'x': 0, 'p': 0, 'r': 0}

    def _next(self, kind):
        if self.i >= len(self.calls):
            raise Mismatch('call %d (%s) is past the end of the transcript' % (self.i + 1, kind))
        c = self.calls[self.i]
        self.i += 1
        if c[0] != kind:
            raise Mismatch('call %d is %s, the transcript has %s' % (self.i, kind, ' '.join(c)))
        self.count[kind] += 1
        return c

    def xfer(self, out):
        c = self._next('x')
        sent = binascii.hexlify(bytes(out)).decode()
        if sent != c[1]:
            raise Mismatch('transaction %d sent %s, the transcript has %s' % (self.count['x'], sent, c[1]))
        return binascii.unhexlify(c[2])

    def pause_us(self, us):
        c = self._next('p')
        if str(us) != c[1]:
            raise Mismatch('pause %d is %s us, the transcript has %s' % (self.count['p'], us, c[1]))

    def probe(self):
        c = self._next('r')
        return int(c[1]), int(c[2])


def main(path, image=None):
    with open(path) as fh:
        rp = Replay(fh.read().split('\n'))
    with open(image or HERE + '/bringup.img') as fh:
        img = pehost.read_image(fh)
    lines = []
    rep = bringup.run(pehost.Host(rp), img, probe=rp.probe, log=lines.append)
    if rp.i != len(rp.calls):
        raise Mismatch('%d calls of the transcript were never made' % (len(rp.calls) - rp.i))
    print('replay: %d transactions, %d pauses, %d pad reads, all as recorded; bring-up %d passed, %d failed' % (
        rp.count['x'], rp.count['p'], rp.count['r'], len(rep.rows) - len(rep.failed()), len(rep.failed())))
    return 1 if rep.failed() else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
