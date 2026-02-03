# CMakeLists.txt for C++ Testing Framework - Simplified Three Test System
cmake_minimum_required(VERSION 3.16)

# Enable testing
enable_testing()

# Find and configure Catch2
find_package(Catch2 3 QUIET)
if(NOT Catch2_FOUND)
    # Add Catch2 as subdirectory if not found system-wide
    # Catch2 находится в корне проекта в папке libs
    set(CATCH2_PATH "${CMAKE_SOURCE_DIR}/libs/Catch2")
    if(EXISTS "${CATCH2_PATH}")
        add_subdirectory(${CATCH2_PATH} ${CMAKE_BINARY_DIR}/catch2)
        list(APPEND CMAKE_MODULE_PATH ${CATCH2_PATH}/extras)
        set(Catch2_FOUND TRUE)
        message(STATUS "Using Catch2 from: ${CATCH2_PATH}")
    else()
        message(FATAL_ERROR "Catch2 not found at ${CATCH2_PATH}")
    endif()
endif()

# Include Catch2 CMake functions only if Catch2 is available
if(Catch2_FOUND)
    include(CTest)
    # Try to include Catch, but don't fail if not found
    include(Catch OPTIONAL)
    if(NOT COMMAND catch_discover_tests)
        message(WARNING "catch_discover_tests not available, using add_test instead")
        # Define a simple replacement function
        function(catch_discover_tests TEST_NAME)
            add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
        endfunction()
    endif()
endif()

# Set compiler flags for testing
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add coverage flags for Debug builds
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
    endif()
endif()

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include/core)
include_directories(${CMAKE_SOURCE_DIR}/libs/eigen-3.4.0)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/tests)

# Create a library with all source files (excluding main.cpp and problematic files)
file(GLOB_RECURSE CORE_SOURCES 
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/include/core/*.cpp"
)

# Remove main.cpp and problematic files
list(FILTER CORE_SOURCES EXCLUDE REGEX "main\\.cpp$")
list(FILTER CORE_SOURCES EXCLUDE REGEX "RungeKutta4Solver\\.cpp$")

# Only create library if we have sources
if(CORE_SOURCES)
    add_library(CoreLayerLib STATIC ${CORE_SOURCES})
    target_include_directories(CoreLayerLib PUBLIC 
        ${CMAKE_SOURCE_DIR}/include/core
        ${CMAKE_SOURCE_DIR}/libs/eigen-3.4.0
        ${CMAKE_SOURCE_DIR}/src
    )
else()
    # Create empty library for tests that don't need the main codebase
    add_library(CoreLayerLib INTERFACE)
    target_include_directories(CoreLayerLib INTERFACE 
        ${CMAKE_SOURCE_DIR}/include/core
        ${CMAKE_SOURCE_DIR}/libs/eigen-3.4.0
        ${CMAKE_SOURCE_DIR}/src
    )
endif()

# Function to create test executable
function(create_test_executable TEST_NAME TEST_SOURCES)
    if(TEST_SOURCES)
        add_executable(${TEST_NAME} ${TEST_SOURCES})
        
        # Link with CoreLayerLib and Catch2
        get_target_property(LIB_TYPE CoreLayerLib TYPE)
        if(LIB_TYPE STREQUAL "STATIC_LIBRARY")
            target_link_libraries(${TEST_NAME} PRIVATE CoreLayerLib)
        else()
            target_link_libraries(${TEST_NAME} PRIVATE CoreLayerLib)
        endif()
        
        # Link with Catch2 if available
        if(Catch2_FOUND)
            if(TARGET Catch2::Catch2WithMain)
                target_link_libraries(${TEST_NAME} PRIVATE Catch2::Catch2WithMain)
            elseif(TARGET Catch2)
                target_link_libraries(${TEST_NAME} PRIVATE Catch2)
            endif()
        endif()
        
        # Register tests with CTest
        if(COMMAND catch_discover_tests)
            catch_discover_tests(${TEST_NAME}
                TEST_PREFIX "${TEST_NAME}:"
                REPORTER xml
                OUTPUT_DIR ${CMAKE_BINARY_DIR}/test_results
            )
        else()
            add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
        endif()
        
        message(STATUS "Created test executable: ${TEST_NAME}")
    else()
        message(STATUS "No test sources found for: ${TEST_NAME}")
    endif()
endfunction()

# 1. UNIT TESTS - Collect unit test files
file(GLOB_RECURSE UNIT_TEST_SOURCES "unit/*.cpp")
create_test_executable(unit_tests "${UNIT_TEST_SOURCES}")

# 2. INTEGRATION TESTS - Collect integration test files
file(GLOB_RECURSE INTEGRATION_TEST_SOURCES "integration/*.cpp")
if(INTEGRATION_TEST_SOURCES)
    create_test_executable(integration_tests "${INTEGRATION_TEST_SOURCES}")
else()
    # Create placeholder integration test if no files exist
    message(STATUS "No integration test files found. Creating placeholder.")
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/placeholder_integration.cpp
        "#include <catch2/catch_test_macros.hpp>\n"
        "TEST_CASE(\"Integration Tests Placeholder\", \"[integration]\") {\n"
        "    REQUIRE(true); // Placeholder test\n"
        "}\n"
    )
    create_test_executable(integration_tests "${CMAKE_CURRENT_BINARY_DIR}/placeholder_integration.cpp")
endif()

# 3. MOCK TESTS - Collect mock test files
file(GLOB_RECURSE MOCK_TEST_SOURCES "mocks/*.cpp")
if(MOCK_TEST_SOURCES)
    create_test_executable(mock_tests "${MOCK_TEST_SOURCES}")
else()
    # Create placeholder mock test if no files exist
    message(STATUS "No mock test files found. Creating placeholder.")
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/placeholder_mock.cpp
        "#include <catch2/catch_test_macros.hpp>\n"
        "TEST_CASE(\"Mock Tests Placeholder\", \"[mock]\") {\n"
        "    REQUIRE(true); // Placeholder test\n"
        "}\n"
    )
    create_test_executable(mock_tests "${CMAKE_CURRENT_BINARY_DIR}/placeholder_mock.cpp")
endif()

# Create combined test target that runs all three test suites
add_custom_target(all_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS unit_tests integration_tests mock_tests
    COMMENT "Running all three test suites"
)

# Coverage target (if gcov/lcov available)
find_program(GCOV_PATH gcov)
find_program(LCOV_PATH lcov)
find_program(GENHTML_PATH genhtml)

if(GCOV_PATH AND LCOV_PATH AND GENHTML_PATH AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_custom_target(coverage
        # Cleanup lcov
        COMMAND ${LCOV_PATH} --directory . --zerocounters
        
        # Run tests
        COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
        
        # Capture coverage data
        COMMAND ${LCOV_PATH} --directory . --capture --output-file coverage.info
        
        # Filter out system headers and test files
        COMMAND ${LCOV_PATH} --remove coverage.info '/usr/*' '*/tests/*' '*/libs/*' --output-file coverage_filtered.info
        
        # Generate HTML report
        COMMAND ${GENHTML_PATH} coverage_filtered.info --output-directory coverage_html
        
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        DEPENDS unit_tests integration_tests mock_tests
        COMMENT "Generating coverage report"
    )
    
    message(STATUS "Coverage reporting enabled. Use 'make coverage' to generate reports.")
else()
    message(STATUS "Coverage reporting disabled. Install gcov, lcov, and genhtml for coverage support.")
endif()

# Print test configuration summary
message(STATUS "=== THREE TEST SYSTEM CONFIGURATION ===")
message(STATUS "  1. Unit tests: ${UNIT_TEST_SOURCES}")
message(STATUS "  2. Integration tests: ${INTEGRATION_TEST_SOURCES}")
message(STATUS "  3. Mock tests: ${MOCK_TEST_SOURCES}")
message(STATUS "  Catch2 found: ${Catch2_FOUND}")
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "  Coverage enabled: TRUE")
else()
    message(STATUS "  Coverage enabled: FALSE")
endif()
message(STATUS "=======================================")