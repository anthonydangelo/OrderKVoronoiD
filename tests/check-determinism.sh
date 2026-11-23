#!/usr/bin/env bash
# Checks that the program gives the same output every time for the same input.
#
# Usage (from the repo root):
#   tests/check-determinism.sh <binary> [runs]
# Runs every case in tests/cases.txt that asks for a diagram (not the err-, bad-
# and help cases) <runs> times (default 50), then <runs> more times with address-
# space randomisation turned off (setarch -R), and checks that all the outputs are
# identical, apart from the elapsedTime line. Memory addresses change from run to
# run, so code whose output depends on them (e.g. on the order of objects in memory)
# would show up here. Exits with 0 if every case gave identical output.
# Example:
#   tests/check-determinism.sh ./OrderKVoronoiD

set -u

if [ $# -lt 1 ]; then
    sed -n '2,/^[^#]/{/^#/p}' "$0" | sed 's/^# \{0,1\}//'
    exit 2
fi
bin="$1"
runs="${2:-50}"
if [ ! -x "$bin" ]; then
    echo "Not an executable: $bin" >&2
    exit 2
fi
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
arch="$(uname -m)"

status=0
checked=0
while IFS='|' read -r name desc args; do
    case "$name" in ''|'#'*|err-*|bad-*|help) continue ;; esac
    read -r -a argv <<< "$args"
    hashes="$(
        for ((i = 0; i < runs; i++)); do
            "$bin" --no-mutex "${argv[@]}" 2> /dev/null | grep -v '"elapsedTime"' | md5sum
        done
        for ((i = 0; i < runs; i++)); do
            setarch "$arch" -R "$bin" --no-mutex "${argv[@]}" 2> /dev/null | grep -v '"elapsedTime"' | md5sum
        done
    )"
    distinct="$(sort -u <<< "$hashes" | wc -l)"
    checked=$((checked + 1))
    if [ "$distinct" -eq 1 ]; then
        printf '%-16s identical (%d runs)\n' "$name" $((2 * runs))
    else
        printf '%-16s DIFFERENT: %d distinct outputs in %d runs\n' "$name" "$distinct" $((2 * runs))
        status=1
    fi
done < "$SCRIPT_DIR/cases.txt"
echo "$checked case(s) checked."
exit "$status"
