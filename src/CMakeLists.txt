# visibility set
add_library(vis_set visibility_set.cc)

# inspection graph
add_library(graph inspection_graph.cc)
if (USE_C++17)
    target_link_libraries(graph vis_set ompl rrg drone)
else (USE_C++17)
    target_link_libraries(graph vis_set ompl rrg)
endif()

# IO
add_library(io io_utils.cc)

# search
add_library(search graph_search.cc traceback_map.cc node.cc)
target_link_libraries(search graph)

# Planar robot related
add_library(planar_env planar_environment.cc)
add_library(planar_robot planar_robot.cc planar_planner.cc)
target_link_libraries(planar_robot planar_env graph)

# CRISP robot related.
add_library(ct_anatomy ct_anatomy.cc)
add_library(crisp_robot crisp_robot.cc crisp_planner.cc)
target_link_libraries(crisp_robot crisp_kinematics ct_anatomy graph)

# Drone robot realted.
if (USE_C++17)
    add_library(bridge_env bridge_environment.cc)
    add_library(drone_robot drone_robot.cc drone_planner.cc)
    target_link_libraries(drone_robot bridge_env graph io)
endif()


add_subdirectory(ompl)
