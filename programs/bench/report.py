"""Check results, the paths of the programs, and the shared rig used by most benches.

Every check is counted for the program or programs named by the last call to group().
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PROGRAMS = os.path.dirname(HERE)                        # programs/
sys.path.insert(0, os.path.normpath(os.path.join(PROGRAMS, '..', 'sim')))
sys.dont_write_bytecode = True

from asm import assemble                                # noqa: E402,F401
from sim import Chip                                    # noqa: E402,F401

DETAIL = 300                                            # longest detail printed after a check


def path(rel):
    """rel is a path under programs/, such as 'basic/uart_tx.s'."""
    return os.path.join(PROGRAMS, rel)


def src(rel):
    with open(path(rel)) as fh:
        return fh.read()


class Log:
    def __init__(self):
        self.rows = []                                  # (programs, ok, name)
        self.progs = ()
        self.t_start = time.time()

    def group(self, *progs, title=''):
        self.progs = progs
        print('--- %s%s' % (', '.join(progs), ': ' + title if title else ''), flush=True)

    def check(self, name, ok, detail='', progs=None):
        """progs, if given, names the programs of this one check instead of the group."""
        ok = bool(ok)
        progs = tuple(progs) if progs else self.progs
        if not progs:
            raise RuntimeError('check before group(): ' + name)
        self.rows.append((progs, ok, name))
        d = '' if detail is None or detail == '' else str(detail)
        if len(d) > DETAIL:
            d = d[:DETAIL] + ' ...'
        print('%s  %s%s' % ('PASS' if ok else 'FAIL', name, '  ' + d if d else ''), flush=True)
        return ok

    def line(self, s):
        print(s, flush=True)

    def info(self, s):
        print('INFO  ' + s, flush=True)


LOG = Log()


def check(name, ok, detail='', progs=None):
    return LOG.check(name, ok, detail, progs)
