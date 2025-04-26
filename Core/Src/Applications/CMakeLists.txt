add_library(CHASSIS_APP STATIC "Chassis_App.c" "Chassis_App.h")
target_include_directories(CHASSIS_APP PUBLIC ${PROJECT_SOURCE_DIR}/Core/Inc
                                              ${PROJECT_SOURCE_DIR}/Core/Src/Algorithms/control
                                              ${PROJECT_SOURCE_DIR}/Core/Src/Algorithms/maths
                                              ${PROJECT_SOURCE_DIR}/Core/Src/Device
                                              ${PROJECT_SOURCE_DIR}/Core/Src/User_Software)
target_link_libraries(CHASSIS_APP PUBLIC ARM_MATH PUBLIC MATHS)