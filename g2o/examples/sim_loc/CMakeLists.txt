add_executable(sim_demo 
  sim_demo.cpp
)




include_directories(${CSPARSE_INCLUDE_DIR})

target_link_libraries(sim_demo core types_sim3 ${OPENGL_LIBRARIES} solver_csparse)


