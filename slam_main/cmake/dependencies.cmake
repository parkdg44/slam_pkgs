
################## dependency ##################

list(APPEND LIBS Eigen3)
list(APPEND LIBS Ceres)

foreach(LIB ${LIBS})
	find_package(${LIB} REQUIRED)
endforeach()

# target link only
list(APPEND TARGET_LINK_LIBS Eigen3::Eigen)
list(APPEND TARGET_LINK_LIBS Ceres::ceres)

################## 3rd party ##################

# for header-only
include_directories(
	${CMAKE_SOURCE_DIR}/3rdparty
)

# set(THIRD_PARTY_SRC ~~~)