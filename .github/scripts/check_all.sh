#!/usr/bin/env bash
#   - 编译: 只要有一个配置编过就算通过.
#   - cppcheck: 任何一个编过的配置不干净就拒绝合并, 并把问题原文打出来.
set -uo pipefail

# 工具链
missing=()
for tool in cmake ninja arm-none-eabi-gcc python3 cppcheck; do
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
trap 'mv -f "$CFG.bak" "$CFG" 2>/dev/null || true' EXIT INT TERM

# $GITHUB_STEP_SUMMARY 不存在时(本地跑)丢弃
summary=""
[[ -n "${GITHUB_STEP_SUMMARY:-}" ]] && summary="$GITHUB_STEP_SUMMARY"

build_failures=()
cppcheck_failures=()
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
        done
    done
done

{
    echo '## CI report'
    echo "- Compiled configurations: $built"
    echo "- Build failures (checks skipped): ${#build_failures[@]}"
    echo "- Cppcheck failures: ${#cppcheck_failures[@]}"
    for kind in build cppcheck; do
        case $kind in
            build) list=("${build_failures[@]}") ;;
            cppcheck) list=("${cppcheck_failures[@]}") ;;
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
[[ $status -eq 0 ]] && echo "All $built compiled configurations passed cppcheck."
exit $status
