
set(LIBUAVCAN_DIR ${PX4_SOURCE_DIR}/src/drivers/uavcan/libuavcan)
set(LIBUAVCAN_DIR_DRIVERS ${PX4_SOURCE_DIR}/src/drivers/uavcan/uavcan_drivers)


set(UAVCAN_USE_CPP03 ON CACHE BOOL "uavcan cpp03")
set(UAVCAN_PLATFORM "generic")

set(UAVCAN_DRIVER "stm32")
set(UAVCAN_TIMER 5) # The default timer is TIM5

if(NOT DEFINED UAVCAN_DRIVER)
	message(FATAL_ERROR "UAVCAN_DRIVER not set")
endif()

if(NOT config_uavcan_num_ifaces)
	message(FATAL_ERROR "config_uavcan_num_ifaces not set")
endif()


string(TOUPPER "${PX4_PLATFORM}" OS_UPPER)
string(TOUPPER "${UAVCAN_DRIVER}" UAVCAN_DRIVER_UPPER)
add_definitions(
	-DUAVCAN_${UAVCAN_DRIVER_UPPER}_${OS_UPPER}=1
	-DUAVCAN_${UAVCAN_DRIVER_UPPER}_NUM_IFACES=${config_uavcan_num_ifaces}
	-DUAVCAN_${UAVCAN_DRIVER_UPPER}_TIMER_NUMBER=${UAVCAN_TIMER}
	-DUAVCAN_CPP_VERSION=UAVCAN_CPP03
	-DUAVCAN_DRIVER=uavcan_${UAVCAN_DRIVER}
	-DUAVCAN_IMPLEMENT_PLACEMENT_NEW=1
	-DUAVCAN_MEM_POOL_BLOCK_SIZE=48
	-DUAVCAN_NO_ASSERTIONS
	-DUAVCAN_PLATFORM=${UAVCAN_PLATFORM}
)

add_compile_options(
	-Wno-cast-align # TODO: fix and enable
	-Wno-deprecated-copy # TODO: fix
	-Wno-address-of-packed-member
)

# driver
#add_subdirectory(${LIBUAVCAN_DIR_DRIVERS}/${UAVCAN_DRIVER}/driver libuavcan_drivers EXCLUDE_FROM_ALL)
target_include_directories(uavcan_${UAVCAN_DRIVER}_driver PUBLIC
	${LIBUAVCAN_DIR}/libuavcan/include
)


px4_add_module(
	MODULE modules__can_integrale
	MAIN can_integrale
	STACK_MAIN 4096
	INCLUDES
		${LIBUAVCAN_DIR}/libuavcan/include
		${LIBUAVCAN_DIR}/libuavcan_drivers/posix/include
		${LIBUAVCAN_DIR_DRIVERS}/${UAVCAN_DRIVER}/driver/include
	SRCS
		can_integrale_module.cpp
	DEPENDS
		modules__uORB
		uavcan_${UAVCAN_DRIVER}_driver
	)
