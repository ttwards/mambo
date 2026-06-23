#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lk_rom_zero, LOG_LEVEL_INF);

#define LK_ZERO_RETRIES 3
#define LK_ZERO_GAP_MS  150

static const struct device *const lk_motors[] = {
	DEVICE_DT_GET(DT_NODELABEL(motor_steer0)),
	DEVICE_DT_GET(DT_NODELABEL(motor_steer1)),
	DEVICE_DT_GET(DT_NODELABEL(motor_steer2)),
	DEVICE_DT_GET(DT_NODELABEL(motor_steer3)),
};

static int check_motors_ready(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(lk_motors); i++) {
		if (!device_is_ready(lk_motors[i])) {
			LOG_ERR("%s not ready", lk_motors[i]->name);
			return -ENODEV;
		}
	}

	return 0;
}

int main(void)
{
	int ret = check_motors_ready();

	if (ret < 0) {
		return ret;
	}

	LOG_INF("LK ROM zero writer started");
	k_msleep(500);

	for (int retry = 0; retry < LK_ZERO_RETRIES; retry++) {
		for (size_t i = 0; i < ARRAY_SIZE(lk_motors); i++) {
			LOG_INF("SET_ZERO ROM: %s", lk_motors[i]->name);
			motor_control(lk_motors[i], SET_ZERO);
			k_msleep(LK_ZERO_GAP_MS);
		}
	}

	k_msleep(300);

	for (size_t i = 0; i < ARRAY_SIZE(lk_motors); i++) {
		motor_control(lk_motors[i], DISABLE_MOTOR);
	}

	LOG_INF("LK ROM zero commands sent. Power-cycle motors to use the stored zero.");

	while (1) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
