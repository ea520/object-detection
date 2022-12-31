set(ZBAR_DIR ${CMAKE_BINARY_DIR}/install)

if(NOT EXISTS ${ZBAR_DIR}/lib/libzbar.so)
	include(ExternalProject)
	ExternalProject_Add(zbar_mchehab
	PREFIX ${ZBAR_DIR}
	GIT_REPOSITORY https://github.com/mchehab/zbar
	GIT_TAG 0.23.92
	CONFIGURE_COMMAND autoreconf -vfi && ./configure CC=${CMAKE_C_COMPILER} CXX=${CMAKE_CXX_COMPILER} "CFLAGS=${EXTERNAL_C_FLAGS} -march=native -Ofast" "CXXFLAGS=${EXTERNAL_CXX_FLAGS} -march=native -Ofast" "LDFLAGS=${EXTERNAL_LD_FLAGS}" --prefix=${ZBAR_DIR} --enable-codes=qrcode --without-dbus
	BUILD_COMMAND make -j
	BUILD_IN_SOURCE 1
	INSTALL_COMMAND make -j install
	)
else()
	add_custom_target(zbar_mchehab)	
endif()