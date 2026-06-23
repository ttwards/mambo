#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define PWM0_NODE DT_NODELABEL(pwm0)
#define PWM1_NODE DT_NODELABEL(pwm1)

const struct device *pwm0_dev = DEVICE_DT_GET(PWM0_NODE);
const struct device *pwm1_dev = DEVICE_DT_GET(PWM1_NODE);

#define PWM_PERIOD_USEC 20000 // 20ms周期
#define MIN_PULSE_USEC  1000  // 1ms最小脉宽
#define MAX_PULSE_USEC  2000  // 2ms最大脉宽

// 角度转占空比
static uint32_t angle_to_pulse_width(float angle, float max_angle)
{
	float range = MAX_PULSE_USEC - MIN_PULSE_USEC;
	return MIN_PULSE_USEC + (uint32_t)((angle / max_angle) * range);
}

void main(void)
{
	// printk("Servomotor control starting...\n");

	if (!device_is_ready(pwm0_dev)) {
		// printk("Error: PWM0 device not ready\n");
		return;
	}

	if (!device_is_ready(pwm1_dev)) {
		// printk("Error: PWM1 device not ready\n");
		return;
	}

	// printk("PWM devices ready\n");
	k_msleep(100);

	while (1) {
		// 0度
		// printk("Setting 0 degrees\n");
		pwm_set(pwm0_dev, 1, PWM_USEC(PWM_PERIOD_USEC),
			PWM_USEC(angle_to_pulse_width(0, 180.0f)), 0);
		pwm_set(pwm1_dev, 1, PWM_USEC(PWM_PERIOD_USEC),
			PWM_USEC(angle_to_pulse_width(0, 270.0f)), 0);
		k_msleep(2000);

		// 90度/135度
		// printk("Setting middle position\n");
		pwm_set(pwm0_dev, 1, PWM_USEC(PWM_PERIOD_USEC),
			PWM_USEC(angle_to_pulse_width(90, 180.0f)), 0);
		pwm_set(pwm1_dev, 1, PWM_USEC(PWM_PERIOD_USEC),
			PWM_USEC(angle_to_pulse_width(135, 270.0f)), 0);
		k_msleep(2000);

		// 最大角度
		// printk("Setting maximum angles\n");
		pwm_set(pwm0_dev, 1, PWM_USEC(PWM_PERIOD_USEC),
			PWM_USEC(angle_to_pulse_width(180, 180.0f)), 0);
		pwm_set(pwm1_dev, 1, PWM_USEC(PWM_PERIOD_USEC),
			PWM_USEC(angle_to_pulse_width(270, 270.0f)), 0);
		k_msleep(2000);
	}
}
