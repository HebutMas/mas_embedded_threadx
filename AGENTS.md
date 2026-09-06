# Agent Notes

## Build

- This repository has no root CMake project; configure from a board source directory (`board/dji_c`, `board/damiao_h7`, or `board/f103_c8`) and use the ARM bare-metal GCC toolchain.
- Required host tools are `cmake`, `ninja`, and `arm-none-eabi-gcc`; `ccache` is used automatically when installed. `cppcheck` is required only when explicitly configured with `-DMAS_REQUIRE_CPPCHECK=ON`.
- A focused build is:
  `cmake -S board/dji_c -B build/dji_c/Debug --preset Debug -DROBOT=infantry3 -DBOARD=single`
  followed by `cmake --build build/dji_c/Debug -j "$(nproc)"`.
- `ROBOT` and `BOARD` are CMake cache variables. Existing build directories retain old values, so override them with `-D...` or use a new/remove the build directory when switching configurations.
- Configuration generates `robot_def.h` and `module_config.h` under the build directory; do not edit generated headers. The firmware ELF is `build/<board>/<config>/base.elf`.
- The root build has no host unit-test or `ctest` workflow. The meaningful local checks are a target-board build and `cmake --build <build-dir> --target cppcheck-log` (when `cppcheck` is installed); the report is under `<build-dir>/cppcheck/cppcheck.log`.

## Layout

- `cmake/board_common.cmake` is the shared build entrypoint. Board `CMakeLists.txt` files select the ThreadX architecture, while `apps/config.cmake` selects robot/board-role settings and `apps/<robot>/robot.cmake` selects modules and hardware parameters.
- Application configurations are discovered by CI from `apps/*/robot.cmake` and each matching `*_board` directory. Keep module source registration in `modules/CMakeLists.txt` and module defaults in `modules/module_config.cmake`.
- `build/` is ignored generated output. IDE build tasks also copy the active `compile_commands.json` to `build/` and the ELF to `build/<board>.elf` for clangd/debugging.

## Change Boundaries

- When adding a module, change only the files required for that module and its explicit registration/configuration; do not modify unrelated modules, BSP code, application code, or formatting incidentally.
- Keep layer ownership intact: an application/module change must not alter a lower layer merely to accommodate an upper-layer design. Change a lower layer only when it is necessary for the required behavior, and verify its other callers.
- Keep the patch and affected surface as small as possible; do not add speculative abstractions, compatibility code, or scaffolding.

## Checks And Workflow

- Format changed C/C++ files with the repository `.clang-format` (`clang-format -i <files>`). Cppcheck/clang-tidy settings are in `.clang-tidy`; CI's `cppcheck-log` covers only `board/bsp`, `modules`, `apps`, and `utils`.
- CI builds every discovered robot/board-role configuration for `damiao_h7` and `dji_c`, then runs cppcheck for merge requests targeting `dev`. It does not validate `f103_c8`.
- `main` accepts merges only from this repository's `dev` branch. Pushing to `dev` triggers the automated merge into `dev-systemview`.
- `.gitattributes` marks `threadx/**` and `utils/**` as `merge=ours` for the `dev` to `dev-systemview` merge. Before doing that merge locally, register the driver with `git config merge.ours.driver true`.
- Flashing is hardware-dependent and supports only `damiao_h7` and `dji_c`; use `.vscode/flash_interactive.sh <board> <probe>` after building, with probe `stlink`, `daplink`, or `jlink`.

## Ponytail

- First ask whether the change is needed; otherwise reuse existing code, then prefer standard-library/native/dependency facilities before adding new code.
- Prefer deletion and the smallest working diff. Do not add an abstraction with one implementation, a factory for one product, or configuration for an unchanging value.
- Understand the real flow before simplifying. For bug fixes, inspect all callers and fix the shared root cause rather than patching only the reported path.
- Do not simplify away validation, error handling, security, hardware calibration, or the smallest meaningful verification check.
- Mark deliberate known-ceiling shortcuts with a `ponytail:` comment that names the limitation and upgrade path.

`CMSIS-DSP/AGENTS.md` applies to work inside `CMSIS-DSP`.
