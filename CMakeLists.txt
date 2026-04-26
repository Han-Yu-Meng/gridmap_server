project(gridmap_server)

find_package(OpenCV   REQUIRED)
find_package(yaml-cpp REQUIRED)

fins_add_node(${PROJECT_NAME}
  node.cpp
)

fins_link_ros_dependencies(${PROJECT_NAME}
  nav_msgs
)

target_include_directories(${PROJECT_NAME} PRIVATE 
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  ${OpenCV_LIBS}
  yaml-cpp
)