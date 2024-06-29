
################## dependency ##################

set(LIBS ${LIBS} Eigen3)
set(LIBS ${LIBS} Ceres)
set(LIBS ${LIBS} OpenCV)

foreach(LIB ${LIBS})
	find_package(${LIB} REQUIRED)
endforeach()

# target link only
set(TARGET_LINK_LIBS ${TARGET_LINK_LIBS} Eigen3::Eigen)
set(TARGET_LINK_LIBS ${TARGET_LINK_LIBS} Ceres::ceres)
set(TARGET_LINK_LIBS ${TARGET_LINK_LIBS} ${OpenCV_LIBS})

################## 3rd party ##################

include_directories(
	${CMAKE_SOURCE_DIR}/3rdparty
)

# set(THIRD_PARTY_SRC ~~~)