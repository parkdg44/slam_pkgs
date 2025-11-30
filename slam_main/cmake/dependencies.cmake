################## dependency ##################

list(APPEND LIBS Eigen3)
list(APPEND LIBS TBB)

foreach(LIB ${LIBS})
    find_package(${LIB} REQUIRED)
endforeach()

# target link only
list(APPEND TARGET_LINK_LIBS Eigen3::Eigen)
list(APPEND TARGET_LINK_LIBS TBB::tbb)

################## 3rd party ##################

# for header-only
include_directories(SYSTEM INTERFACE
	${CMAKE_SOURCE_DIR}/3rdparty
)