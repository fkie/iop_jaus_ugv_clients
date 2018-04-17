cmake_minimum_required(VERSION 2.8.3)
project(iop_client_illumination_fkie)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             iop_ocu_slavelib_fkie
             roscpp
             tf)
iop_init(COMPONENT_ID 0)
iop_export_service(urn_jaus_jss_ugv_IlluminationServiceClient)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include/public
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        iop_ocu_slavelib_fkie
)

###########
## Build ##
###########

## Specify additional locations of header files
iop_code_generator(
  IDLS
    urn.jaus.jss.core-v1.1/AccessControlClient.xml
    urn.jaus.jss.core-v1.1/EventsClient.xml
    urn.jaus.jss.core-v1.1/Transport.xml
    urn.jaus.jss.ugv/illuminationClient.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_ugv_IlluminationServiceClient/IlluminationServiceClient_ReceiveFSM.h
    src/urn_jaus_jss_ugv_IlluminationServiceClient/IlluminationServiceClient_ReceiveFSM.cpp
    src/main.cpp
  EXTERN_SERVICES
    urn_jaus_jss_core_AccessControlClient
    urn_jaus_jss_core_EventsClient
    urn_jaus_jss_core_Transport
  GENERATED_SOURCES cppfiles
)
include_directories(${catkin_INCLUDE_DIRS}
                    include/public
)

## Declare a cpp executable
add_library(${PROJECT_NAME}
            src/IlluminationClientPlugin.cpp
            src/IlluminatorClient.cpp
            src/IlluminatorClientList.cpp
            ${cppfiles}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
)

install(
  DIRECTORY ${IOP_INSTALL_INCLUDE_DIRS} DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN "*.old" EXCLUDE
  PATTERN "*.gen" EXCLUDE
)

install(
  DIRECTORY include/public/${PROJECT_NAME} DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN "*.old" EXCLUDE
  PATTERN "*.gen" EXCLUDE
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

