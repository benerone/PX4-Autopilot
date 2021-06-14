

px4_add_module(
	MODULE modules__sgb
	MAIN sbg
	STACK_MAIN 4096
	SRCS
		sbg_module.cpp
	DEPENDS
		modules__uORB
	)
