# Enable testing
enable_testing()

# Find GTest package
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Function to add a gtest executable
function(add_gtest test_name)
    add_executable(${test_name} ${ARGN})
    target_link_libraries(${test_name}
        ${GTEST_BOTH_LIBRARIES}
    )
    add_test(NAME ${test_name} COMMAND ${test_name})
endfunction()

# Add test executable
add_gtest(gtest_slot_map
    gtest/gtest_slot_map.cpp
)