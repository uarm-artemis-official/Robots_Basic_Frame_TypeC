if (${CMAKE_BUILD_TYPE} STREQUAL "GTest")
    add_executable(MATHS_TEST maths_test.cpp maths.c)
    target_include_directories(MATHS_TEST 
        PRIVATE 
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${PROJECT_SOURCE_DIR}/Core/Inc)
    target_link_libraries(MATHS_TEST PRIVATE GTest::gtest_main)
    target_compile_definitions(MATHS_TEST 
        PRIVATE
            GTEST)

    include(GoogleTest)
    gtest_discover_tests(MATHS_TEST)
else()
    add_library(MATHS_LIBRARIES STATIC)
    target_sources(MATHS_LIBRARIES PUBLIC maths.c kalman_filters.c)
    target_link_libraries(MATHS_LIBRARIES PUBLIC ARM_MATH COMMON_INCLUDES)
    target_include_directories(MATHS_LIBRARIES PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
endif()