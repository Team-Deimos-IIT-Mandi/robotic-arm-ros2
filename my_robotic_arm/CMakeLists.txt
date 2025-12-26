cmake_minimum_required(VERSION 3.8)
project(my_robotic_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

# 1. Install the URDF folder 
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# 2. Install the Config folder 
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 3. Install the Launch folder 
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# 4. Install the Meshes folder
install(DIRECTORY meshes
 DESTINATION share/${PROJECT_NAME})

ament_package()