# Agent Notes

## Build

- This repository has no root CMake project; configure from a board source directory (`board/dji_c`, `board/damiao_h7`, `board/f103_c8`, or `board/105_rc`) and use the ARM bare-metal GCC toolchain.
- Required host tools are `cmake`, `ninja`, and `arm-none-eabi-gcc`; `ccache` is used automatically when installed. `cppcheck` is required only when explicitly configured with `-DMAS_REQUIRE_CPPCHECK=ON`.
- A focused build is:
  `cmake -S board/dji_c -B build/dji_c/Debug --preset Debug`
  followed by `cmake --build build/dji_c/Debug -j "$(nproc)"`.
- `ROBOT` and `BOARD` are selected in `apps/config.cmake`, which is the single source of truth. They are normal CMake variables, so they **shadow** any `-DROBOT=`/`-DBOARD=` cache entry: passing those flags silently keeps building the file's current value. Edit `apps/config.cmake` to switch, and restore it afterwards.
- Configuration generates `robot_def.h` and `module_config.h` under the build directory; do not edit generated headers. The firmware ELF is `build/<board>/<config>/base.elf`.
- The root build has no host unit-test or `ctest` workflow. The meaningful local checks are a target-board build and `cmake --build <build-dir> --target cppcheck-log` (when `cppcheck` is installed); the report is under `<build-dir>/cppcheck/cppcheck.log`.

## Layout

- `cmake/board_common.cmake` is the shared build entrypoint. Board `CMakeLists.txt` files select the ThreadX architecture, while `apps/config.cmake` selects robot/board-role settings and `apps/<robot>/robot.cmake` selects modules and hardware parameters.
- Application configurations are discovered by CI from `apps/*/robot.cmake` and each matching `*_board` directory. Keep module source registration in `modules/CMakeLists.txt` and module defaults in `modules/module_config.cmake`.
- Third-party sources live under `3rdparty/`, trimmed to only what is compiled: `3rdparty/CMSIS-DSP` (target `CMSISDSP`) and `3rdparty/CherryUSB` (device stack + CDC ACM + DWC2 port, registered by `3rdparty/CherryUSB/cherryusb.cmake`). Do not restore upstream files without a caller.
- `build/` is ignored generated output. IDE build tasks also copy the active `compile_commands.json` to `build/` and the ELF to `build/<board>.elf` for clangd/debugging.

## Change Boundaries

- When adding a module, change only the files required for that module and its explicit registration/configuration; do not modify unrelated modules, BSP code, application code, or formatting incidentally.
- Keep layer ownership intact: an application/module change must not alter a lower layer merely to accommodate an upper-layer design. Change a lower layer only when it is necessary for the required behavior, and verify its other callers.
- Keep the patch and affected surface as small as possible; do not add speculative abstractions, compatibility code, or scaffolding.
- Update the owning module's `*.MD` with any API or convention change; the docs are the only place the init convention and the receive design are written down.

## Checks And Workflow

- Format changed C/C++ files with the repository `.clang-format` (`clang-format -i <files>`). Cppcheck settings are in `.clang-tidy`; CI's `cppcheck-log` covers only `board/bsp`, `modules`, `apps`, and `utils`.
- CI runs `.github/scripts/check_all.sh` (the toolchain and packages come from the workflow, not the script; run the script locally to reproduce a full CI pass). It builds every discovered robot/board-role configuration for `damiao_h7` and `dji_c` — `apps/templates` is skipped because it is not buildable — then runs cppcheck. It does not validate `f103_c8` or `105_rc`.
- CI rejects the merge when zero configurations build, or when any configuration fails cppcheck. A single configuration failing to build is reported but tolerated.
- `main` accepts merges only from this repository's `dev` branch. Pushing to `dev` triggers the automated merge into `dev-systemview`.
- `.gitattributes` marks `threadx/**` and `utils/**` as `merge=ours` for the `dev` to `dev-systemview` merge. Before doing that merge locally, register the driver with `git config merge.ours.driver true`.
- Flashing is hardware-dependent and supports only `damiao_h7` and `dji_c`; use `.vscode/flash_interactive.sh <board> <probe>` after building, with probe `stlink`, `daplink`, or `jlink`.

## Ponytail

- First ask whether the change is needed; otherwise reuse existing code, then prefer standard-library/native/dependency facilities before adding new code.
- Prefer deletion and the smallest working diff. Do not add an abstraction with one implementation, a factory for one product, or configuration for an unchanging value.
- Understand the real flow before simplifying. For bug fixes, inspect all callers and fix the shared root cause rather than patching only the reported path.
- Do not simplify away validation, error handling, security, hardware calibration, or the smallest meaningful verification check.
- Mark deliberate known-ceiling shortcuts with a `ponytail:` comment that names the limitation and upgrade path.
