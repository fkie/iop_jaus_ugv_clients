cmake_minimum_required(VERSION 3.5)
project(fkie_iop_client_illumination)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-ignored-qualifiers)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(fkie_iop_ocu_slavelib REQUIRED)
find_package(std_msgs REQUIRED)

###############
## setup IOP ##
###############
iop_init()
iop_export_service(
  urn_jaus_jss_ugv_IlluminationServiceClient
)
## Specify additional locations of header files
iop_code_generator(
  IDLS
    urn.jaus.jss.core/AccessControlClient.xml
    urn.jaus.jss.core/EventsClient.xml
    urn.jaus.jss.core/Transport.xml
    urn.jaus.jss.ugv/illuminationClient.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_ugv_IlluminationServiceClient/IlluminationServiceClient_ReceiveFSM.h
    src/urn_jaus_jss_ugv_IlluminationServiceClient/IlluminationServiceClient_ReceiveFSM.cpp
  EXTERN_SERVICES
    urn_jaus_jss_core_AccessControlClient
    urn_jaus_jss_core_EventsClient
    urn_jaus_jss_core_Transport
  GENERATED_SOURCES cppfiles
)

###########
## Build ##
###########
set(dependencies
  diagnostic_msgs
  fkie_iop_ocu_slavelib
  std_msgs
)

include_directories(
  include
  ${IOP_COMPONENT_INCLUDE_DIRS}
  ${fkie_iop_ocu_slavelib_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/IlluminatorClient.cpp
  src/IlluminatorClientList.cpp
  ${cppfiles}
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${diagnostic_msgs_TARGETS}
  ${std_msgs_TARGETS}
  fkie_iop_ocu_slavelib::fkie_iop_ocu_slavelib
)

#############
## Install ##
#############

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  PATTERN "*.old" EXCLUDE
  PATTERN "*.gen" EXCLUDE
)

install(
  DIRECTORY ${IOP_INSTALL_INCLUDE_DIRS}
  DESTINATION include/${PROJECT_NAME}
  PATTERN "*.old" EXCLUDE
  PATTERN "*.gen" EXCLUDE
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_export_include_directories(include ${IOP_COMPONENT_INCLUDE_DIRS} ${fkie_iop_ocu_slavelib_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(fkie_iop_component plugin_iop.xml)

ament_package()
