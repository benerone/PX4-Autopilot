

px4_add_module(
	MODULE modules__sgb
	MAIN sbg
	STACK_MAIN 10000
	SRCS
		sbg_module.cpp
	DEPENDS
		modules__uORB
		git_ecl
		ecl_geo
		utilitieslib
		sbgECom
	)
