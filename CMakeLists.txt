px4_add_module(
	MODULE ME136
	COMPILE_FLAGS
        -D_MICROCONTROLLER
	STACK_MAIN 1500
	SRCS
		UserCode.cpp
		UtilityFunctions.cpp
	DEPENDS
		platforms__common
	)
