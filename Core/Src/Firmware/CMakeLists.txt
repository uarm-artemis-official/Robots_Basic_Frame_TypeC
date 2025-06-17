add_subdirectory(Device)

add_library(DWT STATIC dwt.c)
target_include_directories(DWT PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(DWT PUBLIC COMMON_INCLUDES)
add_library(FW::DWT ALIAS DWT)

add_library(ERROR_HANDLER STATIC error_handler.c)
target_include_directories(ERROR_HANDLER PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ERROR_HANDLER 
    PUBLIC 
        FW::DEVICE::SSD1306_DRIVER
        DWT)
add_library(FW::ERROR_HANDLER ALIAS ERROR_HANDLER)

add_library(PACK_HANDLER STATIC pack_handler.c)
target_include_directories(PACK_HANDLER 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(PACK_HANDLER 
    PUBLIC 
        COMMON_INCLUDES
        SUBSYSTEMS::MESSAGE_CENTER)
add_library(FW::PACK_HANDLER ALIAS PACK_HANDLER)

add_library(QUEUE_M STATIC queue_m.c)
target_include_directories(QUEUE_M PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(QUEUE_M
    PUBLIC 
        COMMON_INCLUDES)
add_library(FW::QUEUE_M ALIAS QUEUE_M)

add_library(REF_DATA STATIC referee_data.c)
target_include_directories(REF_DATA PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(REF_DATA
    PUBLIC 
        COMMON_INCLUDES)
add_library(FW::REF_DATA ALIAS REF_DATA)