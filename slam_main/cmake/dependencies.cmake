
################## dependency ##################

find_package(Ceres 2.1.0 REQUIRED)
set(LIBS ${LIBS} Ceres::ceres)


################## 3rd party ##################

# set(THIRD_PARTY_SRC ~~~)

include_directories(
	${CMAKE_SOURCE_DIR}/3rdparty
)