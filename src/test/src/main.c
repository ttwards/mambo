#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include "ares/protocol/dual/dual_protocol.h"
#include <ares/interface/usb/usb_bulk.h>
#include <ares/ares_comm.h>
#include <ares/protocol/dual/dual_protocol.h>

/* 使用节点标签 (Node Label) 获取光电开关的配置 */
#define SENSOR1_NODE DT_NODELABEL(sensor_1)
#define SENSOR2_NODE DT_NODELABEL(sensor_2)

/* 检查设备树中是否存在这两个节点，防止编译报错不明显 */
#if !DT_NODE_EXISTS(SENSOR1_NODE) || !DT_NODE_EXISTS(SENSOR2_NODE)
#error "设备树中缺少 sensor_1 或 sensor_2 节点！"
#endif
uint8_t actionbuf[36] = {0};

/* 获取 GPIO spec 结构体 */
static const struct gpio_dt_spec sensor1 = GPIO_DT_SPEC_GET(SENSOR1_NODE, gpios);
static const struct gpio_dt_spec sensor2 = GPIO_DT_SPEC_GET(SENSOR2_NODE, gpios);

/* 定义回调数据结构 */
static struct gpio_callback sensor1_cb_data;
static struct gpio_callback sensor2_cb_data;
DUAL_PROPOSE_PROTOCOL_DEFINE(dual_protocol);
ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);
int action_rx_cb(int status)
{
	// LOG_ERR("action_rx_cb called with status: %d", status);
	if(status == SYNC_PACK_STATUS_DONE) {
	
	}else{
		return;
	}
	return 0;
}
/* * 统一的光电开关回调函数 (运行在中断上下文中)
 */
void unified_sensor_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	/* 通过比对 port 和 pins 掩码来区分是哪个传感器触发了中断 */

	if (port == sensor1.port && (pins & BIT(sensor1.pin))) {
		printk("[ISR] 光电开关 1 被触发！(Pin: %d)\n", sensor1.pin);
		/* 注意：这里是中断上下文，不要调用会导致睡眠的函数如 k_msleep */
	}

	if (port == sensor2.port && (pins & BIT(sensor2.pin))) {
		printk("[ISR] 光电开关 2 被触发！(Pin: %d)\n", sensor2.pin);
	}
}

int main(void)
{
	int ret;
	ares_bind_interface(&usb_bulk_interface, &dual_protocol);
	printk("启动光电开关检测程序 (无别名版本)...\n");
	dual_sync_add(&dual_protocol, 0x0101, &actionbuf, 36, (dual_trans_cb_t)action_rx_cb);
	/* 1. 检查 GPIO 控制器设备是否就绪 */
	if (!gpio_is_ready_dt(&sensor1)) {
		printk("错误: 光电开关 1 的 GPIO 设备未就绪\n");
		return -ENODEV;
	}
	if (!gpio_is_ready_dt(&sensor2)) {
		printk("错误: 光电开关 2 的 GPIO 设备未就绪\n");
		return -ENODEV;
	}

	/* 2. 配置引脚为输入模式 */
	ret = gpio_pin_configure_dt(&sensor1, GPIO_INPUT);
	if (ret != 0) {
		printk("错误: 无法配置光电开关 1, 错误码: %d\n", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&sensor2, GPIO_INPUT);
	if (ret != 0) {
		printk("错误: 无法配置光电开关 2, 错误码: %d\n", ret);
		return ret;
	}

	/* 3. 配置中断触发方式 (上升沿/有效电平触发) */
	ret = gpio_pin_interrupt_configure_dt(&sensor1, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("错误: 无法配置光电开关 1 的中断, 错误码: %d\n", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&sensor2, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("错误: 无法配置光电开关 2 的中断, 错误码: %d\n", ret);
		return ret;
	}

	/* 4. 初始化回调结构体并绑定到统一的回调函数 */
	gpio_init_callback(&sensor1_cb_data, unified_sensor_callback, BIT(sensor1.pin));
	gpio_init_callback(&sensor2_cb_data, unified_sensor_callback, BIT(sensor2.pin));

	/* 5. 将回调添加到相应的 GPIO 驱动实例中 */
	gpio_add_callback(sensor1.port, &sensor1_cb_data);
	gpio_add_callback(sensor2.port, &sensor2_cb_data);

	printk("光电开关配置成功，等待触发...\n");

	/* 主线程可以在这里执行其他任务 */
	while (1) {
		k_msleep(2000);
	}

	return 0;
}