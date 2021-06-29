
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v5
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	ROMFSROOT px4fmu_common
	IO px4_io-v2_default
	TESTING
	UAVCAN_INTERFACES 2
	SERIAL_PORTS
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
		TEL3:/dev/ttyS4
		TEL4:/dev/ttyS3
	DRIVERS
		adc
		lights/rgbled_pwm
		pwm_out
		pwm_out_sim
		px4io
		safety_button
		tone_alarm
		uavcan
	MODULES
		battery_status
		can_integrale
		sbg
		commander
		dataman
		events
		land_detector
		load_mon
		logger
		mavlink
		mc_att_control
		mc_hover_thrust_estimator
		mc_pos_control
		mc_rate_control
		navigator
		rc_update
		sensors
	SYSTEMCMDS
		bl_update
		dmesg
		dumpfile
		esc_calib
		gpio
		hardfault_log
		i2cdetect
		led_control
		mixer
		motor_ramp
		motor_test
		mtd
		nshterm
		param
		perf
		pwm
		reboot
		reflect
		sd_bench
		#tests # tests and test runner
		top
		topic_listener
		tune_control
		usb_connected
		ver
		work_queue
	EXAMPLES
		#fake_magnetometer
		#fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		#hello
		#hwtest # Hardware test
		#matlab_csv_serial
		#px4_mavlink_debug # Tutorial code from http://dev.px4.io/en/debug/debug_values.html
		#px4_simple_app # Tutorial code from http://dev.px4.io/en/apps/hello_sky.html
		#rover_steering_control # Rover example app
		#uuv_example_app
		#work_item
	)
