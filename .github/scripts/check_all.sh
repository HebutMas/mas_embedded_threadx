#!/usr/bin/env bash
#   - 编译: 只要有一个配置编过就算通过.
#   - cppcheck / clang-tidy: 任何一个编过的配置不干净就拒绝合并, 并把问题原文打出来.
set -uo pipefail

# 工具链
missing=()
for tool in cmake ninja arm-none-eabi-gcc python3 cppcheck clang-tidy; do
    command -v "$tool" >/dev/null || missing+=("$tool")
done
if ((${#missing[@]})); then
    echo "缺少工具: ${missing[*]}"
    exit 2
fi

BOARDS=(damiao_h7 dji_c)
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

# apps/config.cmake
CFG="$ROOT/apps/config.cmake"
cp "$CFG" "$CFG.bak"
trap 'mv -f "$CFG.bak" "$CFG"' EXIT

# $GITHUB_STEP_SUMMARY 不存在时(本地跑)丢弃
summary=""
[[ -n "${GITHUB_STEP_SUMMARY:-}" ]] && summary="$GITHUB_STEP_SUMMARY"

build_failures=()
cppcheck_failures=()
clang_tidy_failures=()
built=0

announce() { # $1=level 标题, $2=正文文件
    echo "::error title=$1::${2//$'\n'/ }"
    if [[ -n "$summary" ]]; then
        {
            echo
            echo "### $1"
            echo '```text'
            cat "$2" 2>/dev/null
            echo '```'
        } >>"$summary"
    fi
    echo "----- $1 -----"
    sed -n '1,60p' "$2" 2>/dev/null
}

for board in "${BOARDS[@]}"; do
    for robot_dir in apps/*; do
        robot=$(basename "$robot_dir")
        [[ -f "$robot_dir/robot.cmake" ]] || continue
        [[ "$robot" == templates ]] && continue # 模板含 <robot>_def.h 占位符, 本身编不过

        for app_board_dir in "$robot_dir"/*_board; do
            [[ -d "$app_board_dir" ]] || continue
            board_type=$(basename "$app_board_dir" _board)
            label="$board/$robot/$board_type"
            build_dir="build/$board/$robot/$board_type"
            log_dir="$build_dir/logs"
            rm -rf "$build_dir"
            mkdir -p "$log_dir"

            sed -i -E "s|^set\(ROBOT .*|set(ROBOT \"$robot\")|; s|^set\(BOARD .*|set(BOARD \"$board_type\")|" "$CFG"

            echo "== $label =="
            if ! cmake -S "board/$board" -B "$build_dir" \
                -G Ninja \
                -DCMAKE_BUILD_TYPE=Debug \
                -DMAS_REQUIRE_CPPCHECK=ON \
                -DCMAKE_C_COMPILER_LAUNCHER=ccache \
                -DCMAKE_TOOLCHAIN_FILE="$ROOT/board/$board/cmake/gcc-arm-none-eabi.cmake" \
                >"$log_dir/configure.log" 2>&1; then
                build_failures+=("$label (configure)")
                announce "Configure failed: $label" "$log_dir/configure.log"
                continue
            fi

            if ! cmake --build "$build_dir" -j "$(nproc)" >"$log_dir/build.log" 2>&1; then
                build_failures+=("$label (build)")
                announce "Build failed: $label" "$log_dir/build.log"
                continue
            fi
            built=$((built + 1))

            # ---- cppcheck ----
            if ! cmake --build "$build_dir" --target cppcheck-log >"$log_dir/cppcheck-command.log" 2>&1; then
                cppcheck_failures+=("$label")
                announce "Cppcheck failed: $label" "$build_dir/cppcheck/cppcheck.log"
            fi

            # clang-tidy
            # 只查本配置编译数据库里出现的文件: 
            ct_dir="$log_dir/clang-tidy"
            mkdir -p "$ct_dir"
            python3 - "$build_dir/compile_commands.json" >"$ct_dir/files.txt" <<'PY'
import json, sys
db = json.load(open(sys.argv[1]))
scope = ("/board/bsp/", "/modules/", "/apps/", "/utils/")
files = sorted({e["file"] for e in db if any(s in e["file"] for s in scope)})
print("\n".join(files))
PY
            # 每个文件单独落盘, 再按序合并
            xargs -P "$(nproc)" -I{} sh -c '
                out="$2/$(printf "%s" "$1" | tr "/" "_").log"
                clang-tidy -p "$0" "$1" >"$out" 2>&1 || printf "%s\n" "$1" >>"$2/failed.txt"
            ' "$build_dir" {} "$ct_dir" <"$ct_dir/files.txt"
            if [[ -s "$ct_dir/failed.txt" ]]; then
                clang_tidy_failures+=("$label")
                grep -hE 'warning:|error:' "$ct_dir"/*.log >"$log_dir/clang-tidy.log" 2>/dev/null
                announce "Clang-tidy failed: $label" "$log_dir/clang-tidy.log"
            fi
        done
    done
done

{
    echo '## CI report'
    echo "- Compiled configurations: $built"
    echo "- Build failures (checks skipped): ${#build_failures[@]}"
    echo "- Cppcheck failures: ${#cppcheck_failures[@]}"
    echo "- Clang-tidy failures: ${#clang_tidy_failures[@]}"
    for kind in build cppcheck clang-tidy; do
        case $kind in
            build) list=("${build_failures[@]}") ;;
            cppcheck) list=("${cppcheck_failures[@]}") ;;
            clang-tidy) list=("${clang_tidy_failures[@]}") ;;
        esac
        ((${#list[@]})) || continue
        echo
        echo "### $kind failures"
        printf '%s\n' "${list[@]}"
    done
} | tee -a ${summary:-/dev/null}

status=0
if ((built == 0)); then
    echo 'No configuration compiled successfully; merge is rejected.'
    status=1
fi
if ((${#cppcheck_failures[@]})); then
    echo 'Cppcheck failed; merge is rejected.'
    status=1
fi
if ((${#clang_tidy_failures[@]})); then
    echo 'Clang-tidy failed; merge is rejected.'
    status=1
fi
[[ $status -eq 0 ]] && echo "All $built compiled configurations passed cppcheck and clang-tidy."
exit $status
