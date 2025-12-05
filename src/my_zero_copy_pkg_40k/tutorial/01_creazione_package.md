# 1. Creazione del Package ROS2

Questo capitolo spiega come creare il package ROS2 per la comunicazione zero-copy.

---

## Passo 1: Creare la struttura base

Naviga nel workspace ROS2 e crea il package:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_zero_copy_pkg_40k
```

Questo comando crea la seguente struttura:

```
my_zero_copy_pkg_40k/
├── CMakeLists.txt
├── package.xml
├── include/
└── src/
```

---

## Passo 2: Aggiungere cartelle necessarie

Crea le cartelle per messaggi, configurazione e documentazione:

```bash
cd my_zero_copy_pkg_40k
mkdir -p msg config docs tutorial
```

Struttura finale:

```
my_zero_copy_pkg_40k/
├── CMakeLists.txt
├── package.xml
├── include/
├── src/
├── msg/              # Messaggi custom
├── config/           # Configurazione FastDDS
├── docs/             # Documentazione
└── tutorial/         # Questo tutorial
```

---

## Passo 3: Modificare package.xml

Il file `package.xml` definisce le dipendenze del package.

```xml
<?xml version="1.0" encoding="UTF-8"?>
<package format="3">
  <name>my_zero_copy_pkg_40k</name>
  <version>1.0.0</version>
  <description>
    Package ROS2 per comunicazione Zero-Copy con array di 40.000 interi.
  </description>
  <maintainer email="tuo@email.com">TuoNome</maintainer>
  <license>MIT</license>

  <!-- Strumenti di build -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <!-- Dipendenze runtime -->
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <!-- Generazione messaggi -->
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <!-- Test -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Spiegazione delle dipendenze

| Dipendenza | Scopo |
|------------|-------|
| `ament_cmake` | Sistema di build per C++ |
| `rosidl_default_generators` | Genera codice C++ dai file .msg |
| `rclcpp` | Libreria client ROS2 per C++ |
| `std_msgs` | Messaggi standard (per publisher standard) |
| `rosidl_default_runtime` | Runtime per messaggi generati |

---

## Passo 4: Modificare CMakeLists.txt

Il file `CMakeLists.txt` configura la compilazione:

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_zero_copy_pkg_40k)

# Abilita warning del compilatore
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# DIPENDENZE
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ============================================================================
# GENERAZIONE MESSAGGI CUSTOM
# ============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FixedArray40k.msg"
)

# Ottieni il target per linkare i messaggi generati
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ============================================================================
# ESEGUIBILI
# ============================================================================

# Publisher Loaned (Zero-Copy)
add_executable(loaned_publisher src/loaned_publisher.cpp)
target_compile_features(loaned_publisher PUBLIC cxx_std_17)
ament_target_dependencies(loaned_publisher "rclcpp")
target_link_libraries(loaned_publisher "${cpp_typesupport_target}")

# Subscriber Loaned (Zero-Copy)
add_executable(loaned_subscriber src/loaned_subscriber.cpp)
target_compile_features(loaned_subscriber PUBLIC cxx_std_17)
ament_target_dependencies(loaned_subscriber "rclcpp")
target_link_libraries(loaned_subscriber "${cpp_typesupport_target}")

# Publisher Standard
add_executable(standard_publisher src/standard_publisher.cpp)
target_compile_features(standard_publisher PUBLIC cxx_std_17)
ament_target_dependencies(standard_publisher "rclcpp" "std_msgs")

# Subscriber Standard
add_executable(standard_subscriber src/standard_subscriber.cpp)
target_compile_features(standard_subscriber PUBLIC cxx_std_17)
ament_target_dependencies(standard_subscriber "rclcpp" "std_msgs")

# ============================================================================
# INSTALLAZIONE
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

install(DIRECTORY tutorial/
  DESTINATION share/${PROJECT_NAME}/tutorial
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
```

### Punti chiave del CMakeLists.txt

1. **rosidl_generate_interfaces**: Genera codice C++ dal messaggio `.msg`
2. **rosidl_get_typesupport_target**: Ottiene il target per linkare i messaggi
3. **target_link_libraries**: Collega i messaggi generati agli eseguibili
4. **install(DIRECTORY config/)**: Installa i file di configurazione FastDDS

---

## Passo 5: Verifica struttura

Dopo aver creato tutti i file, la struttura deve essere:

```
my_zero_copy_pkg_40k/
├── CMakeLists.txt        ✓
├── package.xml           ✓
├── msg/
│   └── FixedArray40k.msg (prossimo capitolo)
├── config/
│   └── fastdds_setup.xml (capitolo 3)
├── src/
│   ├── loaned_publisher.cpp   (capitolo 6)
│   ├── loaned_subscriber.cpp  (capitolo 7)
│   ├── standard_publisher.cpp (capitolo 4)
│   └── standard_subscriber.cpp (capitolo 5)
├── docs/
└── tutorial/
```

---

## Prossimo Passo

Vai al capitolo [2. Messaggio Custom POD](02_messaggio_custom.md) per creare il messaggio.
