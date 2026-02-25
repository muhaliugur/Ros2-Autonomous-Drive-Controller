cmake_minimum_required(VERSION 3.8)
project(ovat_komut)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Gerekli kütüphaneleri bul (Dependencies)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 2. Çalıştırılabilir dosyayı (Executable) oluştur
# src/ovat_komut_node.cpp dosyasını derle ve adını ovat_komut_node yap
add_executable(ovat_komut_node src/ovat_komut_node.cpp)

# 3. Kütüphaneleri executable'a bağla
ament_target_dependencies(ovat_komut_node
  rclcpp
  std_msgs
)

# 4. Derlenen dosyayı ROS2'nin install klasörüne kopyala (Kurulum)
install(TARGETS
  ovat_komut_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
