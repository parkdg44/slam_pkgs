
################## dependency ##################

list(APPEND LIBS Eigen3)
list(APPEND LIBS Ceres)
list(APPEND LIBS OpenCV)

foreach(LIB ${LIBS})
	find_package(${LIB} REQUIRED)
endforeach()

# target link only
list(APPEND TARGET_LINK_LIBS Eigen3::Eigen)
list(APPEND TARGET_LINK_LIBS Ceres::ceres)
list(APPEND TARGET_LINK_LIBS ${OpenCV_LIBS})

################## 3rd party ##################

# subdirectory
MESSAGE(STATUS "----------------------------")
MESSAGE(STATUS "load Pangolin...")

add_subdirectory(${CMAKE_SOURCE_DIR}/3rdparty/Pangolin)
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
list(APPEND TARGET_LINK_LIBS ${Pangolin_LIBRARY})

MESSAGE(STATUS "Pangolin is loaded!")
MESSAGE(STATUS "----------------------------")

# for header-only
include_directories(
	${CMAKE_SOURCE_DIR}/3rdparty
)

# set(THIRD_PARTY_SRC ~~~)