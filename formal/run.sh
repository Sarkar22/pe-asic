#!/usr/bin/env bash
# Runs the proofs in this folder and prints one line per job: result and seconds.
#
# Usage: formal/run.sh [job ...]        with no job, every job in JOBS below
#   A job is <name>:<task> for <name>.sby, or <name>.eqy for an equivalence check.
#
# Tools: sby, eqy and yosys with the slang frontend, and the bitwuzla solver. eqy runs its proofs with make
# and awk; where those are missing, bin/make and bin/awk stand in. Either the tools are on PATH
# (an OSS CAD Suite), or set FORMAL_IMAGE to a container image that has them and FORMAL_DOCKER to the
# docker command (default: docker); each job then runs in its own container.
#   FORMAL_WORK   folder for the solvers' work folders (default: formal/work); it can grow large
#   PDK_ROOT       for the equivalence check: the PDK, for the standard cells' liberty
#   FORMAL_NETLIST for the equivalence check: the gate netlist to compare (tt_um_sarkar22_pe.nl.v)
#   FORMAL_JOBS    parallel proofs in the equivalence check (default 8)
#
# SPDX-License-Identifier: Apache-2.0
set -uo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(dirname "$HERE")
WORK=${FORMAL_WORK:-$HERE/work}
mkdir -p "$WORK"
WORK=$(cd "$WORK" && pwd)

JOBS_ALL="pads:prove pads:cover isolation:prove isolation:cover shift_timing:prove shift_timing:cover
          reset:gen reset:flops_prove reset:flops_cover reset:pads_prove covers_deep:cover chip.eqy"

# Runs one command, in the image if one is set. bitwuzla may sit outside PATH in the image.
tool() {
  if [ -n "${FORMAL_IMAGE:-}" ]; then
    local pdk=()
    [ -n "${PDK_ROOT:-}" ] && pdk=(-v "$PDK_ROOT":"$PDK_ROOT":ro -e PDK_ROOT="$PDK_ROOT")
    [ -n "${FORMAL_NETLIST:-}" ] && pdk+=(-v "$(dirname "$FORMAL_NETLIST")":"$(dirname "$FORMAL_NETLIST")":ro)
    ${FORMAL_DOCKER:-docker} run --rm -u "$(id -u):$(id -g)" -v "$ROOT":"$ROOT":ro -v "$WORK":"$WORK" \
      "${pdk[@]}" -e WORKDIR="${WORKDIR:-}" -w "$HERE" "$FORMAL_IMAGE" bash -c \
      'command -v bitwuzla > /dev/null || for d in /nix/store/*-bitwuzla-*/bin; do PATH=$PATH:$d; done
       command -v make > /dev/null && command -v awk > /dev/null || PATH=$PATH:'"$HERE"'/bin; exec "$@"' \
      bash "$@"
  else
    command -v make > /dev/null && command -v awk > /dev/null || PATH=$PATH:$HERE/bin
    (cd "$HERE" && "$@")
  fi
}

run_job() {
  local job=$1 start rc status dir
  start=$(date +%s)
  case "$job" in
    *.eqy)
      # The .eqy file names @ROOT@, @PDK_ROOT@ and @NETLIST@; the copy in the work folder has them filled in.
      dir=$WORK/${job%.eqy}_eqy
      if [ -z "${FORMAL_NETLIST:-}" ] || [ -z "${PDK_ROOT:-}" ]; then
        echo "$job needs FORMAL_NETLIST and PDK_ROOT" > "$dir.log"; rc=1
      else
        sed -e "s|@ROOT@|$ROOT|g" -e "s|@PDK_ROOT@|$PDK_ROOT|g" -e "s|@NETLIST@|$FORMAL_NETLIST|g" \
          "$HERE/$job" > "$WORK/$job"
        tool eqy -f -j "${FORMAL_JOBS:-8}" -d "$dir" "$WORK/$job" > "$dir.log" 2>&1
        rc=$?
      fi ;;
    reset:gen)
      # Regenerates reset_flops.sv and probes_reset.ys from the RTL and checks they match the ones here.
      dir=$WORK/reset_gen
      rm -rf "$dir"; mkdir -p "$dir"
      cp "$ROOT"/src/*.sv "$ROOT/verif/cells_rtl.v" "$HERE/sram_free.v" "$HERE/read_chip.ys" "$HERE/reset_flops.ys" "$dir"
      (cd "$dir" && WORKDIR=$dir tool bash -c 'cd "$WORKDIR" && yosys -q reset_flops.ys && python3 -I '"$HERE"'/gen_reset.py chip.json .') \
        > "$dir.log" 2>&1
      rc=$?
      if [ $rc -eq 0 ] && cmp -s "$dir/reset_flops.sv" "$HERE/reset_flops.sv" && cmp -s "$dir/probes_reset.ys" "$HERE/probes_reset.ys"; then
        echo PASS > "$dir/status"
      elif [ $rc -eq 0 ]; then
        echo "STALE: reset_flops.sv or probes_reset.ys differ from $dir; copy them here" | tee -a "$dir.log" > "$dir/status"; rc=1
      else
        echo FAIL > "$dir/status"
      fi ;;
    *:*)
      dir=$WORK/${job%%:*}_${job##*:}
      tool sby -f -d "$dir" "$HERE/${job%%:*}.sby" "${job##*:}" > "$dir.log" 2>&1
      rc=$? ;;
    *) echo "unknown job $job"; return 1 ;;
  esac
  status=$(cat "$dir/status" 2>/dev/null | head -1)
  # eqy leaves no status file; its log ends with DONE (<status>, rc=<n>).
  [ -n "$status" ] || status=$(grep -o "DONE ([A-Z]*" "$dir.log" 2>/dev/null | tail -1 | cut -c7-)
  printf '%-28s %-10s %5d s   rc %d\n' "$job" "${status:-ERROR}" "$(( $(date +%s) - start ))" "$rc"
}

for job in ${@:-$JOBS_ALL}; do
  run_job "$job"
done
