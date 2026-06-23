#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <ares/ares_comm.h>
#include <ares/interface/usb/usb_bulk.h>
#include <ares/protocol/dual/dual_protocol.h>

LOG_MODULE_REGISTER(connector_wye_debug, LOG_LEVEL_INF);

#define WYE_CMD_SYNC_ID 0x0205

static const struct device *const connector_wye_motor =
	DEVICE_DT_GET(DT_NODELABEL(connector_wye_motor));

/* 上电自动 prepare 的位置；调试出合适位置后改这里。 */
static float connector_wye_prepare_angle = -800.0f;

static float wye_cmd_angle;
static float pending_wye_angle;

K_SEM_DEFINE(wye_cmd_sem, 0, 1);

DUAL_PROPOSE_PROTOCOL_DEFINE(tool_protocol);
ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);

static int set_wye_angle(float angle)
{
	return motor_set(connector_wye_motor,
			 &(motor_status_t){
				 .angle = angle,
				 .mode = ML_ANGLE,
			 });
}

static int enable_wye_motor(void)
{
	if (!device_is_ready(connector_wye_motor)) {
		LOG_ERR("connector_wye_motor is not ready");
		return -ENODEV;
	}

	motor_set(connector_wye_motor,
		  &(motor_status_t){
			  .rpm = 0.0f,
			  .mode = ML_SPEED,
		  });
	k_msleep(100);

	motor_control(connector_wye_motor, ENABLE_MOTOR);
	k_msleep(100);

	return 0;
}

static void wye_cmd_cb(int status)
{
	if (status != SYNC_PACK_STATUS_DONE) {
		return;
	}

	pending_wye_angle = wye_cmd_angle;
	k_sem_give(&wye_cmd_sem);
}

static void wye_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		k_sem_take(&wye_cmd_sem, K_FOREVER);

		float angle = pending_wye_angle;
		int ret = set_wye_angle(angle);

		if (ret == 0) {
			LOG_INF("connector_wye_motor target angle: %.2f", (double)angle);
		} else {
			LOG_ERR("failed to set connector_wye_motor angle %.2f: %d",
				(double)angle, ret);
		}
	}
}

K_THREAD_DEFINE(wye_cmd, 1024, wye_thread, NULL, NULL, NULL, 6, 0, 0);

int main(void)
{
	int ret;

	LOG_INF("connector_wye_debug start");

	ret = ares_bind_interface(&usb_bulk_interface, &tool_protocol);
	if (ret != 0) {
		LOG_ERR("failed to initialize ARES USB interface: %d", ret);
		return ret;
	}

	dual_sync_add(&tool_protocol, WYE_CMD_SYNC_ID, (uint8_t *)&wye_cmd_angle,
		      sizeof(wye_cmd_angle), wye_cmd_cb);

	ret = enable_wye_motor();
	if (ret != 0) {
		return ret;
	}

	ret = set_wye_angle(connector_wye_prepare_angle);
	if (ret != 0) {
		LOG_ERR("failed to prepare connector_wye_motor: %d", ret);
		return ret;
	}
	LOG_INF("connector_wye_motor prepared at %.2f",
		(double)connector_wye_prepare_angle);

	while (1) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
