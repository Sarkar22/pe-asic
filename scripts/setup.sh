#!/usr/bin/env bash
# Installs the tool versions the online checks use, then runs the program checks and the RTL tests.
#
#   scripts/setup.sh                     install, then test: about 15 min and 3 GB the first time
#   scripts/setup.sh --no-test           install only
#   scripts/setup.sh --install-iverilog  also install Icarus Verilog 13.0 with sudo (Debian, Ubuntu).
#                                        Gate level needs 13.0; the RTL tests also run on 12.0
#   scripts/setup.sh --harden            also pull the LibreLane container (6.6 GB) for a local build
#
# Everything goes into $TTSETUP (default ~/ttsetup) and tt/ in this repo. Nothing else on the
# machine changes, except the system iverilog package with --install-iverilog. Run it again at
# any time: a step that is already done is checked and skipped.
#
# Needs Linux x86-64, git, curl, make, and Python 3.11 or 3.12 with venv. PYTHON=python3.12
# picks the interpreter.
set -euo pipefail

# The versions behind the online build in docs/STATUS.md (tt-gds-action tag ihp-cmos5l)
TT_TOOLS_SHA=d66cf179e7bc4d296362ab7e2e3b344dc3c4f665   # tt-support-tools, branch ihp-sg13cmos5l
PDK_SHA=2bbec755dc67ca3db0261c3d6163e15735d66710        # IHP-Open-PDK
LIBRELANE=3.1.0.dev3
LIBRELANE_DIGEST=sha256:d109140b8f17fc54f4fca998beb8124f4949404ec52e339eebd2250854a18b5a
COCOTB=2.0.1                                            # as in test/requirements.txt
IVERILOG_DEB=https://github.com/TinyTapeout/iverilog/releases/download/v13.0/iverilog_13.0-1_amd64.deb
IVERILOG_DEB_SHA256=888bca6b06289ea8b55368d43db38dc88932938319ab40d45a763ce5114e8505

TEST=1 HARDEN=0 INSTALL_IVERILOG=0
for arg in "$@"; do
  case $arg in
    --no-test) TEST=0 ;;
    --harden) HARDEN=1 ;;
    --install-iverilog) INSTALL_IVERILOG=1 ;;
    -h | --help) sed -n '2,/^set /s/^# \{0,1\}//p' "$0"; exit 0 ;;
    *) echo "setup: unknown option $arg (see --help)" >&2; exit 2 ;;
  esac
done

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
TTSETUP=${TTSETUP:-$HOME/ttsetup}
VENV=$TTSETUP/venv
PDK_ROOT=$TTSETUP/pdk
cd "$REPO"

step() { printf '\n== %s\n' "$*"; }
die() { printf 'setup: %s\n' "$*" >&2; exit 1; }

# A shallow checkout of one pinned commit. Skipped if the folder is already at it.
pin_checkout() { # folder url sha [branch]
  local dir=$1 url=$2 sha=$3 branch=${4:-}
  if [ ! -d "$dir/.git" ]; then
    [ -z "$(ls -A "$dir" 2>/dev/null)" ] || die "$dir exists but isn't a git checkout. Move it aside and run again"
    mkdir -p "$dir"
    git -C "$dir" init -q
  fi
  if [ "$(git -C "$dir" rev-parse -q --verify HEAD 2>/dev/null)" = "$sha" ]; then
    echo "already at the pin"
    return
  fi
  git -C "$dir" fetch --depth 1 "$url" "$sha"
  if [ -n "$branch" ]; then
    git -C "$dir" checkout -q -B "$branch" FETCH_HEAD
  else
    git -C "$dir" checkout -q FETCH_HEAD
  fi
}

step "Checks"
[ "$(uname -s)/$(uname -m)" = Linux/x86_64 ] || echo "warning: tested on Linux x86-64 only, not $(uname -s) $(uname -m)"
for tool in git curl make; do
  command -v "$tool" >/dev/null || die "$tool not found"
done
PY=
for p in ${PYTHON:-} python3.12 python3.11 python3; do
  command -v "$p" >/dev/null 2>&1 || continue
  case $("$p" -c 'import sys; print("%d.%d" % sys.version_info[:2])') in
    3.11 | 3.12) PY=$(command -v "$p"); break ;;
  esac
done
[ -n "$PY" ] || die "needs Python 3.11 or 3.12: the pinned numpy has no wheels for 3.13. Set PYTHON to one"
echo "Python: $PY ($("$PY" -V 2>&1))"

step "TT tools: tt-support-tools ${TT_TOOLS_SHA:0:8} in tt/"
pin_checkout tt https://github.com/TinyTapeout/tt-support-tools "$TT_TOOLS_SHA" ihp-sg13cmos5l
if git rev-parse --git-dir >/dev/null 2>&1; then
  exclude=$(git rev-parse --git-path info/exclude)
  mkdir -p "$(dirname "$exclude")"
  grep -qx 'tt/' "$exclude" 2>/dev/null || echo 'tt/' >>"$exclude"
fi

step "Python packages in $VENV"
venv_ok() {
  [ "$("$VENV/bin/cocotb-config" --version 2>/dev/null)" = "$COCOTB" ] &&
    [ "$("$VENV/bin/python" -m pip show librelane 2>/dev/null | sed -n 's/^Version: //p')" = "$LIBRELANE" ] &&
    "$VENV/bin/python" -m pip check >/dev/null 2>&1
}
if venv_ok; then
  echo "already set up"
else
  # The marker keeps this script from installing into a venv it didn't make
  if [ ! -e "$VENV" ]; then
    mkdir -p "$VENV"
    touch "$VENV/.pe-asic"
  fi
  [ -e "$VENV/.pe-asic" ] || die "$VENV exists and isn't from this script. Move it aside, or set TTSETUP to another folder"
  [ -x "$VENV/bin/pip" ] || "$PY" -m venv "$VENV" || die "python -m venv failed. On Debian or Ubuntu: sudo apt install python3-venv"
  "$VENV/bin/python" -m pip install -q -r tt/requirements.txt
  "$VENV/bin/python" -m pip install -q "librelane==$LIBRELANE"
  "$VENV/bin/python" -m pip install -q -r test/requirements.txt
  "$VENV/bin/python" -m pip check
  venv_ok || die "the packages in $VENV don't match the pins"
fi

step "IHP PDK ${PDK_SHA:0:8} in $PDK_ROOT (about 360 MB to fetch)"
pin_checkout "$PDK_ROOT" https://github.com/IHP-GmbH/IHP-Open-PDK.git "$PDK_SHA"
echo "IHP-Open-PDK $PDK_SHA" >"$PDK_ROOT/ihp-sg13cmos5l/SOURCES"

step "Icarus Verilog"
iverilog_version() { iverilog -V 2>/dev/null | sed -n '1s/^Icarus Verilog version \([0-9.]*\).*/\1/p'; }
iv=$(iverilog_version || true)
if [ "$INSTALL_IVERILOG" = 1 ] && [ "$iv" != 13.0 ]; then
  command -v apt-get >/dev/null || die "--install-iverilog needs apt (Debian, Ubuntu). Elsewhere, build tag v13_0 of Icarus from source"
  deb=$(mktemp -d)/iverilog_13.0-1_amd64.deb
  curl -fsSL -o "$deb" "$IVERILOG_DEB"
  echo "$IVERILOG_DEB_SHA256  $deb" | sha256sum -c --quiet || die "checksum mismatch on $IVERILOG_DEB"
  sudo apt-get install -y "$deb"
  rm -f "$deb"
  iv=$(iverilog_version || true)
  echo "Installed. To go back to Ubuntu 24.04's 12.0: sudo apt install --allow-downgrades iverilog=12.0-2build2"
fi
case $iv in
  13.0) echo "13.0: runs the RTL and the gate-level tests" ;;
  12.*) echo "$iv: runs the RTL tests. Gate level needs 13.0: run again with --install-iverilog" ;;
  "") die "iverilog not found. Run again with --install-iverilog, or install your system's package" ;;
  *) echo "$iv: not tried here. The online checks use 12.0 for RTL and 13.0 for gate level" ;;
esac

if [ "$HARDEN" = 1 ]; then
  step "LibreLane container $LIBRELANE"
  command -v docker >/dev/null || die "--harden needs Docker, usable without sudo"
  image=ghcr.io/librelane/librelane:$LIBRELANE
  docker image inspect "$image" >/dev/null 2>&1 || docker pull "$image"
  case " $(docker image inspect --format '{{join .RepoDigests " "}}' "$image") " in
    *"@$LIBRELANE_DIGEST "*) echo "the pinned build" ;;
    *) echo "warning: $image isn't the pinned build $LIBRELANE_DIGEST" ;;
  esac
fi

step "Shell setup in $TTSETUP/env.sh"
cat >"$TTSETUP/env.sh" <<EOF
# Run '. $TTSETUP/env.sh' in each new shell, before the tests or a build.
unset PYTHONPATH  # a PYTHONPATH set by other tools can hide the venv's packages
export PDK_ROOT="$PDK_ROOT" PDK=ihp-sg13cmos5l
. "$VENV/bin/activate"
EOF
echo "written"

if [ "$TEST" = 1 ]; then
  step "Every program on the simulator (about 2 min)"
  "$VENV/bin/python" programs/check.py
  step "RTL tests at the pins (about 3 min)"
  (
    set +u
    . "$TTSETUP/env.sh"
    cd test
    make -B
    python -m cocotb_tools.check_results results.xml
  )
fi

cat <<EOF

Done. In each new shell, first:
  . $TTSETUP/env.sh
Then, from $REPO:
  python3 programs/check.py           every program on the simulator
  (cd test && make -B)                RTL tests at the pins
A local build, after setup.sh --harden (about 20 min, 3 GB of memory):
  ./tt/tt_tool.py --create-user-config --ihp && ./tt/tt_tool.py --harden --ihp
  ./tt/tt_tool.py --create-tt-submission --ihp
  cp tt_submission/tt_um_sarkar22_pe.v test/gate_level_netlist.v
  (cd test && make -B GATES=yes)      gate-level tests on that netlist, with Icarus 13.0
EOF
