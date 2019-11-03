add_executable(build_graph build_graph.cc)
if (USE_C++17)
    target_link_libraries(build_graph crisp_robot planar_robot drone_robot)
else (USE_C++17)
    target_link_libraries(build_graph crisp_robot planar_robot)
endif()

add_executable(search_graph search_graph.cc)
target_link_libraries(search_graph search)