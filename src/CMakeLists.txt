##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# Targets
##############################################################################
add_executable(is_kobuki_node common/common.cpp common/cell.cpp common/mega_cell.cpp
  common/map.cpp stc/stc_navigation.cpp controller/kobuki_controller.cpp main.cpp)
add_executable(is_kobuki_service_node common/common.cpp common/cell.cpp common/mega_cell.cpp
  common/map.cpp common/robot.cpp service/map_service.cpp map_service_main.cpp)

add_dependencies(is_kobuki_node kobuki_msgs_gencpp is_kobuki)
add_dependencies(is_kobuki_service_node kobuki_msgs_gencpp is_kobuki)

target_link_libraries(is_kobuki_node ${catkin_LIBRARIES})
target_link_libraries(is_kobuki_service_node ${catkin_LIBRARIES})

install(TARGETS is_kobuki_node is_kobuki_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS is_kobuki_service_node is_kobuki_service_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

