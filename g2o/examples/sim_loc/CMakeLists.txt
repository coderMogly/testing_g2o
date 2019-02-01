find_package(PCL 1.2 REQUIRED)


add_executable(sim_demo 
  sim_demo.cpp
)

add_executable(test 
  test.cpp
)



include_directories(${CSPARSE_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(sim_demo core types_sim3 ${OPENGL_LIBRARIES} ${PCL_LIBRARIES} solver_csparse)
target_link_libraries(test core types_sim3 ${OPENGL_LIBRARIES} ${PCL_LIBRARIES} solver_csparse)


