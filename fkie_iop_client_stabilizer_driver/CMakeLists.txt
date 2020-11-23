cmake_minimum_required(VERSION 3.0.2)
project(fkie_iop_client_stabilizer_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             fkie_iop_component
             fkie_iop_ocu_slavelib
             roscpp
             sensor_msgs
             std_msgs)
find_package(Boost REQUIRED COMPONENTS system thread)
iop_init(COMPONENT_ID 0)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        fkie_iop_component
        fkie_iop_ocu_slavelib
        roscpp
        sensor_msgs
        std_msgs
    DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
iop_code_generator(
  IDLS
    urn.jaus.jss.core-v1.0/AccessControlClient.xml
    urn.jaus.jss.core-v1.0/EventsClient.xml
    urn.jaus.jss.core-v1.0/Transport.xml
    urn.jaus.jss.core-v1.0/ManagementClient.xml
    urn.jaus.jss.ugv/stabilizerClient.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_ugv_StabilizerDriverClient/StabilizerDriverClient_ReceiveFSM.h
    src/urn_jaus_jss_ugv_StabilizerDriverClient/StabilizerDriverClient_ReceiveFSM.cpp
    src/main.cpp
  EXTERN_SERVICES
    urn_jaus_jss_core_AccessControlClient
    urn_jaus_jss_core_EventsClient
    urn_jaus_jss_core_Transport
    urn_jaus_jss_core_ManagementClient
  GENERATED_SOURCES cppfiles
)
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_library(${PROJECT_NAME}
            src/StabilizerDriverClientPlugin.cpp
            ${cppfiles}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
   FILES ./plugin_iop.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

