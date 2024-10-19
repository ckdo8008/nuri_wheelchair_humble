cmake_minimum_required(VERSION 3.5)
project(nuri_humble_navigation2)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch map param rviz
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()