cmake_minimum_required(VERSION 3.0.2)
project(drone_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY meshes urdf launch rviz doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
