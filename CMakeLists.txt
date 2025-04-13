cmake_minimum_required(VERSION 3.8)
project(xmsg VERSION 2.1.0)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${protobuf_INCLUDE_DIRS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_msgs REQUIRED)


add_library(GenericMessage SHARED
    src/genericmessage.cpp
    src/jsongenericmessage.cpp
    src/protogenericmessage.cpp
    src/rosgenericmessage.cpp

    include/xmsg/genericmessage.h
    include/xmsg/jsongenericmessage.h
    include/xmsg/protogenericmessage.h
    include/xmsg/rosgenericmessage.h

    include/xmsg/Configuration/genericmessageconfigurator.h
    include/xmsg/Configuration/jsongenericconfigurator.h

    src/Configuration/genericmessageconfigurator.cpp
    src/Configuration/jsongenericconfigurator.cpp
)

add_library(XMSG::GenericMessage ALIAS GenericMessage )
set_property(TARGET GenericMessage PROPERTY
             IMPORTED_LOCATION lib)

target_include_directories(GenericMessage PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>)

target_link_libraries(GenericMessage
  protobuf
  nlohmann_json
)

ament_target_dependencies(GenericMessage
  rclcpp
  std_msgs
)

ament_export_include_directories(include)
ament_export_libraries(GenericMessage)


install(TARGETS GenericMessage
    EXPORT "GenericMessageTargets"
    DESTINATION lib
    INCLUDES DESTINATION include)

install(DIRECTORY include/ DESTINATION include)


## Put export set to right place
install(EXPORT "GenericMessageTargets"
  FILE "GenericMessageTargets.cmake"
  NAMESPACE "XMSG::"
  DESTINATION "lib/cmake/GenericMessage"
)

include(CMakePackageConfigHelpers)

ament_package()
