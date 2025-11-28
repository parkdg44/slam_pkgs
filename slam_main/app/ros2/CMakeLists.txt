file(GLOB_RECURSE APP_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

set(APP_TARGET ${PROJECT_NAME}_ros2)

add_executable(${APP_TARGET} ${APP_SRCS})
target_link_libraries(${APP_TARGET} ${TARGET_LINK_LIBS})
ament_target_dependencies(${APP_TARGET} ${ROS2_LIBS})

install(TARGETS ${APP_TARGET} DESTINATION lib/${PROJECT_NAME})
