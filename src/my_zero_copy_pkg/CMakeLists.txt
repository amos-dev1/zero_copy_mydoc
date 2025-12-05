cmake_minimum_required(VERSION 3.8)
project(my_zero_copy_pkg)

# Abilita warning del compilatore per codice più sicuro
# Enable compiler warnings for safer code
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# DIPENDENZE / DEPENDENCIES
# ============================================================================
# IT: Trova i pacchetti necessari per il progetto
# EN: Find required packages for the project
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ============================================================================
# GENERAZIONE MESSAGGI CUSTOM / CUSTOM MESSAGE GENERATION
# ============================================================================
# IT: Genera le interfacce C++ dal file .msg per il messaggio personalizzato
# EN: Generate C++ interfaces from .msg file for the custom message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FixedArray40k.msg"
  # NO DEPENDENCIES - Pure POD message, no std_msgs/Header
  # NESSUNA DIPENDENZA - Messaggio POD puro, senza std_msgs/Header
)

# IT: Ottieni il target per linkare il messaggio generato
# EN: Get the target to link the generated message
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ============================================================================
# ESEGUIBILE PUBLISHER LOANED (ZERO-COPY)
# ============================================================================
# IT: Publisher con zero-copy usando borrow_loaned_message()
# EN: Publisher with zero-copy using borrow_loaned_message()
add_executable(loaned_publisher src/loaned_publisher.cpp)
target_include_directories(loaned_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(loaned_publisher PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(loaned_publisher "rclcpp")
target_link_libraries(loaned_publisher "${cpp_typesupport_target}")

# ============================================================================
# ESEGUIBILE SUBSCRIBER LOANED (ZERO-COPY)
# ============================================================================
# IT: Subscriber per messaggi zero-copy
# EN: Subscriber for zero-copy messages
add_executable(loaned_subscriber src/loaned_subscriber.cpp)
target_include_directories(loaned_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(loaned_subscriber PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(loaned_subscriber "rclcpp")
target_link_libraries(loaned_subscriber "${cpp_typesupport_target}")

# ============================================================================
# ESEGUIBILE PUBLISHER STANDARD (NO ZERO-COPY)
# ============================================================================
# IT: Publisher standard con std_msgs, senza zero-copy
# EN: Standard publisher with std_msgs, no zero-copy
add_executable(standard_publisher src/standard_publisher.cpp)
target_include_directories(standard_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(standard_publisher PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(standard_publisher "rclcpp" "std_msgs")

# ============================================================================
# ESEGUIBILE SUBSCRIBER STANDARD (NO ZERO-COPY)
# ============================================================================
# IT: Subscriber standard con std_msgs, senza zero-copy
# EN: Standard subscriber with std_msgs, no zero-copy
add_executable(standard_subscriber src/standard_subscriber.cpp)
target_include_directories(standard_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(standard_subscriber PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(standard_subscriber "rclcpp" "std_msgs")

# ============================================================================
# INSTALLAZIONE / INSTALLATION
# ============================================================================
# IT: Installa tutti gli eseguibili
# EN: Install all executables
install(TARGETS 
  loaned_publisher
  loaned_subscriber
  standard_publisher
  standard_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# IT: Installa la configurazione FastDDS
# EN: Install FastDDS configuration
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# IT: Installa la documentazione
# EN: Install documentation
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

# IT: Finalizza il pacchetto ament
# EN: Finalize the ament package
ament_package()
