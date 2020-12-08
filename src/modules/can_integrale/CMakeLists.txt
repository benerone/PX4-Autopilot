px4_add_module(
	MODULE modules__can_integrale
	MAIN can_integrale
	SRCS
		can_integrale_module.cpp
	DEPENDS
		modules__uORB
	)
