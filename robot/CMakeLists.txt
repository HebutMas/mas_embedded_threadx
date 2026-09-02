add_library(robot OBJECT
    robot_init.c
)

target_compile_options(robot PRIVATE -O2)

target_include_directories(robot PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

if(TARGET CMSISDSP)
    target_link_libraries(robot PUBLIC stm32cubemx azrtos::threadx utils bsp CMSISDSP modules app)
else()
    target_link_libraries(robot PUBLIC stm32cubemx azrtos::threadx utils bsp modules app)
endif()
