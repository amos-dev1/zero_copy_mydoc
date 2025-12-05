cmake_minimum_required(VERSION 3.8)
project(my_zero_copy_pkg_40k)

# Abilita warning del compilatore per codice più sicuro
# Enable compiler warnings for safer code
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# DIPENDENZE / DEPENDENCIES
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ============================================================================
# GENERAZIONE MESSAGGI CUSTOM / CUSTOM MESSAGE GENERATION
# ============================================================================
# IT: Messaggio con array di 40.000 interi
# EN: Message with 40,000 integer array
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FixedArray40k.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ============================================================================
# ESEGUIBILE PUBLISHER LOANED (ZERO-COPY) - 40K
# ============================================================================
add_executable(loaned_publisher src/loaned_publisher.cpp)
target_include_directories(loaned_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(loaned_publisher PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(loaned_publisher "rclcpp")
target_link_libraries(loaned_publisher "${cpp_typesupport_target}")

# ============================================================================
# ESEGUIBILE SUBSCRIBER LOANED (ZERO-COPY) - 40K
# ============================================================================
add_executable(loaned_subscriber src/loaned_subscriber.cpp)
target_include_directories(loaned_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(loaned_subscriber PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(loaned_subscriber "rclcpp")
target_link_libraries(loaned_subscriber "${cpp_typesupport_target}")

# ============================================================================
# ESEGUIBILE PUBLISHER STANDARD - 40K
# ============================================================================
add_executable(standard_publisher src/standard_publisher.cpp)
target_include_directories(standard_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(standard_publisher PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(standard_publisher "rclcpp" "std_msgs")

# ============================================================================
# ESEGUIBILE SUBSCRIBER STANDARD - 40K
# ============================================================================
add_executable(standard_subscriber src/standard_subscriber.cpp)
target_include_directories(standard_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(standard_subscriber PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(standard_subscriber "rclcpp" "std_msgs")

# ============================================================================
# INSTALLAZIONE / INSTALLATION
# ============================================================================
install(TARGETS 
  loaned_publisher
  loaned_subscriber
  standard_publisher
  standard_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
)

# ============================================================================
# TESTING
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
