cmake_minimum_required(VERSION 3.8)
project(olive_networking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)



# Thank's Gemini :
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Install udev in shared repository
install(FILES
  rules/99-olive-networking.rules
  DESTINATION share/${PROJECT_NAME}/udev
)

# Make the script executable and put it in the good place
install(PROGRAMS
  scripts/olive_network_configuration.sh
  DESTINATION lib/${PROJECT_NAME}/scripts
)

# install config repo 
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
