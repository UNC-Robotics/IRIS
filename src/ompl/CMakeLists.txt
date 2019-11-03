add_library(ompl crisp_state_space.cc crisp_directed_control_sampler.cc control_rrg.cc control_rrt.cc control_rrtstar.cc)
target_link_libraries(ompl crisp_robot ${OMPL_LIBRARIES})

add_library(rrg rrg.cc)
target_link_libraries(rrg ${OMPL_LIBRARIES})

if (USE_C++17)
    add_library(drone drone_state_space.cc)
    target_link_libraries(drone drone_robot ${OMPL_LIBRARIES})
endif()
