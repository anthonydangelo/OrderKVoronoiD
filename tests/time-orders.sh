#!/usr/bin/env bash
# Measures run time and peak memory for one random input at several orders.
#
# Usage (from the repo root):
#   tests/time-orders.sh <binary> <n> [k ...]
# Generates n random sites with integer coordinates in [-500, 500] (seed SEED,
# default 1), then runs the binary with --no-mutex for each order k given (default:
# every order from 1 to n-1), and prints the wall-clock time and peak memory (GNU
# time; "sudo apt install time" if /usr/bin/time is missing). The output JSON is
# discarded. Set CASE_TIMEOUT (a duration for timeout(1), e.g. 10m) to stop any run
# that takes too long.
# As tests/run-all.sh does in OneSteinerTreeProblem, it also logs memory activity with
# vmstat every 5 seconds while it runs, and at the end reports the largest swap use
# and the smallest free memory seen, so a run that swapped is visible. Peak memory
# from GNU time counts RAM only, so it understates a run that swaps.
# Everything printed is also saved in tests/out/time-orders-n<n>-<date-time>.log,
# and the vmstat log in tests/out/vmstat-time-orders-n<n>-<date-time>.txt.
# Examples:
#   tests/time-orders.sh ./OrderKVoronoiD 10
#   CASE_TIMEOUT=10m tests/time-orders.sh ./OrderKVoronoiD 24 4 8 12

set -u

TIME_BIN="${TIME_BIN:-/usr/bin/time}"    # GNU time; override for testing

if [ $# -lt 2 ]; then
    sed -n '2,/^[^#]/{/^#/p}' "$0" | sed 's/^# \{0,1\}//'
    exit 2
fi

bin="$1"
n="$2"
shift 2
if [ ! -x "$bin" ]; then
    echo "Not an executable: $bin" >&2
    exit 2
fi
if [ ! -x "$TIME_BIN" ]; then
    echo "$TIME_BIN (GNU time) not found: sudo apt install time" >&2
    exit 2
fi
if [ $# -gt 0 ]; then
    orders=("$@")
else
    orders=($(seq 1 $((n - 1))))
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$SCRIPT_DIR/out"
stamp="$(date '+%Y%m%d-%H%M%S')"
run_log="$SCRIPT_DIR/out/time-orders-n$n-$stamp.log"
vmstat_log="$SCRIPT_DIR/out/vmstat-time-orders-n$n-$stamp.txt"
exec > >(tee -a "$run_log") 2>&1      # from here on, print to the screen and the log
vmstat -t 5 > "$vmstat_log" &
vmstat_pid=$!
disown "$vmstat_pid"    # no "Terminated" job message when it is stopped at the end
timefile="$(mktemp)"
trap 'kill "$vmstat_pid" 2> /dev/null; rm -f "$timefile"' EXIT

points="$(awk -v n="$n" -v seed="${SEED:-1}" 'BEGIN {
    srand(seed); printf "[";
    for (i = 0; i < n; i++) {
        printf "%s[%d,%d]", (i ? "," : ""), int(rand() * 1001) - 500, int(rand() * 1001) - 500
    }
    printf "]" }')"
echo "sites (n = $n, seed ${SEED:-1}): $points"
echo "binary: $bin; started $(date '+%Y-%m-%d %H:%M:%S')"

for k in "${orders[@]}"; do
    cmd=("$bin")
    [ -n "${CASE_TIMEOUT:-}" ] && cmd=(timeout "$CASE_TIMEOUT" "$bin")
    "$TIME_BIN" -f "%e %M" -o "$timefile" "${cmd[@]}" --no-mutex -i "$points" -o "$k" > /dev/null 2>&1
    code=$?
    read -r secs kb < <(tail -n 1 "$timefile")
    note=""
    [ "$code" -eq 124 ] && note="  TIMEOUT after $CASE_TIMEOUT"
    printf 'n=%-4s k=%-4s exit=%-3s %8s s  peak %6.0f MB  (finished %s)%s\n' \
        "$n" "$k" "$code" "$secs" "$((kb / 1024))" "$(date '+%H:%M:%S')" "$note"
done
sleep 1
kill "$vmstat_pid" 2> /dev/null
sleep 0.5    # let vmstat exit before its log is read
# vmstat columns: r b swpd free buff cache si so ... (memory in KB); skip the header lines
# (vmstat repeats them every 20 samples).
awk 'NR > 2 && $3 ~ /^[0-9]+$/ {
        if (first == "") { first = $3 }
        if ($3 > maxswap) { maxswap = $3 }
        if (minfree == "" || $4 < minfree) { minfree = $4 }
        # skip si/so of the first sample: they are averages since boot
        if (!seen_first++) { next }
        if ($7 + $8 > 0) { swapping++ }
     }
     END {
        if (first == "") { print "vmstat: no samples (the run was shorter than 5 seconds)"; exit }
        printf "vmstat: swap used %d MB at the start, %d MB at most; free RAM at least %d MB; %d sample(s) with swapping (si/so)\n", first / 1024, maxswap / 1024, minfree / 1024, swapping
     }' "$vmstat_log"
echo "this log:   $run_log"
echo "vmstat log: $vmstat_log"
