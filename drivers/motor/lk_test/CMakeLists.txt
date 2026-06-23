zephyr_library()
zephyr_library_sources_ifdef(CONFIG_MOTOR_LK_TEST
	motor_lk_test.c
	motor_lk_test_codec.c
)
