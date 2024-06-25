
################## dependency ##################

set(LIBS ${LIBS} Ceres)
set(LIBS ${LIBS} OpenCV)

foreach(LIB ${LIBS})
	find_package(${LIB} REQUIRED)
endforeach()

# target link only
set(LIBS ${LIBS} Ceres::ceres)
set(LIBS ${LIBS} ${OpenCV_LIBS})

################## 3rd party ##################

include_directories(
	${CMAKE_SOURCE_DIR}/3rdparty
)

# set(THIRD_PARTY_SRC ~~~)