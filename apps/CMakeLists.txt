# app 层构建：根据 ROBOT + BOARD 收集业务代码
# 业务代码目录结构: apps/<ROBOT>/<BOARD>_board/

set(_robot_base_dir ${CMAKE_CURRENT_SOURCE_DIR}/${ROBOT})
set(_robot_board_dir ${_robot_base_dir}/${BOARD}_board)

# 收集该机器人板型目录下所有 .c 源文件
set(_robot_sources "")
if(EXISTS ${_robot_board_dir})
    file(GLOB_RECURSE _robot_sources CONFIGURE_DEPENDS ${_robot_board_dir}/*.c)
endif()

# 收集所有子目录作为头文件包含路径
# 把 apps/<robot>/ 根目录加入，方便板型间共享 <robot>_def.h 等头文件
set(_robot_includes "")
if(EXISTS ${_robot_base_dir})
    list(APPEND _robot_includes ${_robot_base_dir})
endif()
if(EXISTS ${_robot_board_dir})
    file(GLOB_RECURSE _robot_headers CONFIGURE_DEPENDS ${_robot_board_dir}/*.h)
    list(APPEND _robot_includes ${_robot_board_dir})
    foreach(_h ${_robot_headers})
        get_filename_component(_dir ${_h} DIRECTORY)
        list(APPEND _robot_includes ${_dir})
    endforeach()
    list(REMOVE_DUPLICATES _robot_includes)
endif()

message(STATUS "APP robot dir:  ${_robot_board_dir}")

add_library(app OBJECT
    app_init.c
    ${_robot_sources}
)

target_compile_options(app PRIVATE -O2 -include module_config.h)

target_include_directories(app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${_robot_includes}
    ${_generated_dir}
)

target_link_libraries(app PUBLIC stm32cubemx azrtos::threadx utils bsp CMSISDSP modules)