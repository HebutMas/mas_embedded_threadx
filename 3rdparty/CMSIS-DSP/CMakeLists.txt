# CMSIS-DSP（裁剪版）：仅保留本仓库实际链接的内容
#   FastMath: arm_sin_f32 / arm_cos_f32
#   Matrix:   arm_mat_init/add/sub/mult/trans/inverse_f32
# 公共表 sinTable/cosTable 来自 CommonTables/arm_common_tables.c。
# 上游：https://github.com/ARM-software/CMSIS-DSP （Apache-2.0，见 LICENSE）

set(_dsp_src ${CMAKE_CURRENT_SOURCE_DIR}/Source)

add_library(CMSISDSP STATIC
    ${_dsp_src}/CommonTables/arm_common_tables.c
    ${_dsp_src}/FastMathFunctions/arm_cos_f32.c
    ${_dsp_src}/FastMathFunctions/arm_sin_f32.c
    ${_dsp_src}/MatrixFunctions/arm_mat_init_f32.c
    ${_dsp_src}/MatrixFunctions/arm_mat_add_f32.c
    ${_dsp_src}/MatrixFunctions/arm_mat_sub_f32.c
    ${_dsp_src}/MatrixFunctions/arm_mat_mult_f32.c
    ${_dsp_src}/MatrixFunctions/arm_mat_trans_f32.c
    ${_dsp_src}/MatrixFunctions/arm_mat_inverse_f32.c
)

target_include_directories(CMSISDSP PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Include)
target_include_directories(CMSISDSP PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/PrivateInclude)

target_compile_definitions(CMSISDSP PRIVATE ARM_MATH_LOOPUNROLL DISABLEFLOAT16)
