#!/usr/bin/env bash
# Runs the inputs in tests/cases.txt and compares runs.
#
# Usage (from the repo root):
#   tests/run-cases.sh run <binary> <label> [case ...]
#   tests/run-cases.sh compare <label1> <label2>
#   tests/run-cases.sh sanitizers <label>
# "run" runs every case (or only the named ones) with --no-mutex, and writes
# <case>.json (stdout), <case>.err (stderr) and <case>.exit (exit code) into
# tests/out/<label>/. Existing outputs are replaced.
# "compare" compares two runs case by case: exit code, stdout (ignoring the
# elapsedTime line) and stderr. Set IGNORE_STDERR=1 to skip stderr, e.g. to compare
# a sanitizer run with a release run. Exits with 0 if every case matches.
# "sanitizers" lists the sanitizer reports in a run of the asan build: UBSan reports
# by location and count, and the cases with an ASan/LSan report.
# Example:
#   tests/run-cases.sh run ./OrderKVoronoiD before
#   (change the code, rebuild)
#   tests/run-cases.sh run ./OrderKVoronoiD after
#   tests/run-cases.sh compare before after

set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CASES="$SCRIPT_DIR/cases.txt"
OUT_ROOT="$SCRIPT_DIR/out"

usage() {
    sed -n '2,/^[^#]/{/^#/p}' "$0" | sed 's/^# \{0,1\}//'
    exit 2
}

run_cases() {
    local bin="$1" label="$2"
    shift 2
    local -a only=("$@")
    local dir="$OUT_ROOT/$label" name desc args code start end
    if [ ! -x "$bin" ]; then
        echo "Not an executable: $bin" >&2
        exit 2
    fi
    mkdir -p "$dir"
    while IFS='|' read -r name desc args; do
        case "$name" in ''|'#'*) continue ;; esac
        if [ ${#only[@]} -gt 0 ]; then
            local wanted=0 c
            for c in "${only[@]}"; do [ "$c" = "$name" ] && wanted=1; done
            [ "$wanted" -eq 1 ] || continue
        fi
        local -a argv
        read -r -a argv <<< "$args"
        start=$(date +%s.%N)
        "$bin" --no-mutex "${argv[@]}" > "$dir/$name.json" 2> "$dir/$name.err" < /dev/null
        code=$?
        end=$(date +%s.%N)
        echo "$code" > "$dir/$name.exit"
        printf '%-16s exit=%-3s %6ss  %s\n' "$name" "$code" \
            "$(awk -v s="$start" -v e="$end" 'BEGIN { printf "%.2f", e - s }')" "$desc"
    done < "$CASES"
    echo "Outputs written to $dir"
}

compare_runs() {
    local a="$OUT_ROOT/$1" b="$OUT_ROOT/$2" name desc args status
    local same=0 different=0 missing=0
    while IFS='|' read -r name desc args; do
        case "$name" in ''|'#'*) continue ;; esac
        if [ ! -f "$a/$name.exit" ] || [ ! -f "$b/$name.exit" ]; then
            status="MISSING"
        elif ! cmp -s "$a/$name.exit" "$b/$name.exit"; then
            status="DIFFERENT exit code: $(cat "$a/$name.exit") vs $(cat "$b/$name.exit")"
        elif ! diff -q -I '"elapsedTime"' "$a/$name.json" "$b/$name.json" > /dev/null; then
            status="DIFFERENT stdout"
        elif [ "${IGNORE_STDERR:-0}" != "1" ] && ! cmp -s "$a/$name.err" "$b/$name.err"; then
            status="DIFFERENT stderr"
        else
            status="SAME"
        fi
        case "$status" in
            SAME) same=$((same + 1)) ;;
            MISSING) missing=$((missing + 1)) ;;
            *) different=$((different + 1)) ;;
        esac
        printf '%-16s %s\n' "$name" "$status"
    done < "$CASES"
    echo "$same same, $different different, $missing missing."
    [ "$different" -eq 0 ] && [ "$missing" -eq 0 ]
}

sanitizer_reports() {
    local dir="$OUT_ROOT/$1"
    echo "UBSan reports (count, location):"
    grep -h 'runtime error:' "$dir"/*.err | cut -d: -f1-3 | sort | uniq -c
    echo "Cases with an ASan/LSan report:"
    grep -l -E 'ERROR: (Address|Leak)Sanitizer' "$dir"/*.err
    return 0
}

case "${1:-}" in
    run)
        [ $# -ge 3 ] || usage
        shift
        run_cases "$@" ;;
    compare)
        [ $# -eq 3 ] || usage
        compare_runs "$2" "$3" ;;
    sanitizers)
        [ $# -eq 2 ] || usage
        sanitizer_reports "$2" ;;
    *)
        usage ;;
esac
