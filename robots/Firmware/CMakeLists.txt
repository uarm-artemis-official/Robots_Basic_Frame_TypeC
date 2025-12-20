# TODO: Rename to Middleware and make middleware for essential components.
# Maybe define a interface for easier testing?

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
        MODULES::MESSAGE_CENTER)
add_library(FW::PACK_HANDLER ALIAS PACK_HANDLER)