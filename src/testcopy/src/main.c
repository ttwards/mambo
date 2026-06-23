#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <ares/protocol/dual/dual_protocol.h>
#include <ares/ares_comm.h>
#define SENSOR1_NODE DT_NODELABEL(sensor_1)
#define SENSOR2_NODE DT_NODELABEL(sensor_2)
#include <ares/interface/usb/usb_bulk.h>
#include <ares/board/init.h>
#include <ares/protocol/dual/dual_protocol.h>
#include <ares/ares_comm.h>
#if !DT_NODE_EXISTS(SENSOR1_NODE) || !DT_NODE_EXISTS(SENSOR2_NODE)
#error "设备树中缺少 sensor_1 或 sensor_2 节点！"
#endif

static const struct gpio_dt_spec sensor1 = GPIO_DT_SPEC_GET(SENSOR1_NODE, gpios);
static const struct gpio_dt_spec sensor2 = GPIO_DT_SPEC_GET(SENSOR2_NODE, gpios);

static struct gpio_callback sensor1_cb_data;
static struct gpio_callback sensor2_cb_data;
sync_table_t *angle_tx;
uint8_t anglebuf[36]= {0};
float angle[9] = {0};
DUAL_PROPOSE_PROTOCOL_DEFINE(dual_protocol);
ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);
/**
 * 统一处理函数：判断传感器编号及当前状态
 */
void handle_sensor_event(const struct gpio_dt_spec *spec, const char *label)
{
    /* 读取引脚当前物理电平 (Read the current pin state) */
    int val = gpio_pin_get_dt(spec);
    int const cdg =0;

    if (val > 0) {
        /* 传感器被遮挡 (Sensor Blocked / Active) */
        printk("[ISR] %s: 检测到物体遮挡！\n", label);
        if(strcmp(label, "传感器 1") == 0) {
            angle[0] = 1.0f;  // 示例：当传感器1被遮挡时，将angle[0]设置为1.0
        } else if(strcmp(label, "传感器 2") == 0) {
            angle[1] = 1.0f;  // 示例：当传感器2被遮挡时，将angle[1]设置为1.0
        }
    } else {
        /* 遮挡取消 (Sensor Cleared / Inactive) */
        printk("[ISR] %s: 遮挡已取消。\n", label);
        if (strcmp(label, "传感器 1") == 0) {
            angle[0] = 0.0f;  // 示例：当传感器1遮挡取消时，将angle[0]设置为0.0
        } else if(strcmp(label, "传感器 2") == 0) {
            angle[1] = 0.0f;  // 示例：当传感器2遮挡取消时，将angle[1]设置为0.0
        {
            /* code */
        }
        
    }
}
}
void unified_sensor_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    /* 区分是哪个传感器触发的中断 */
    if (port == sensor1.port && (pins & BIT(sensor1.pin))) {
        handle_sensor_event(&sensor1, "传感器 1");
        

    }

    if (port == sensor2.port && (pins & BIT(sensor2.pin))) {
        handle_sensor_event(&sensor2, "传感器 2");
        
    }
}

int main(void)
{
    int ret;

    printk("启动双向边缘检测程序 (Edge Both)...\n");

    if (!gpio_is_ready_dt(&sensor1) || !gpio_is_ready_dt(&sensor2)) {
        return -ENODEV;
    }
ares_bind_interface(&usb_bulk_interface, &dual_protocol);
    /* 配置为输入 */
    gpio_pin_configure_dt(&sensor1, GPIO_INPUT);
    gpio_pin_configure_dt(&sensor2, GPIO_INPUT);
angle_tx = dual_sync_add(&dual_protocol, 0x0201, anglebuf, 36, NULL);
    /* * 关键修改：使用 GPIO_INT_EDGE_BOTH 
     * 使其在电平跳变（上升沿和下降沿）时都会触发中断 
     */
    ret = gpio_pin_interrupt_configure_dt(&sensor1, GPIO_INT_EDGE_BOTH);
    if (ret != 0) return ret;

    ret = gpio_pin_interrupt_configure_dt(&sensor2, GPIO_INT_EDGE_BOTH);
    if (ret != 0) return ret;

    gpio_init_callback(&sensor1_cb_data, unified_sensor_callback, BIT(sensor1.pin));
    gpio_init_callback(&sensor2_cb_data, unified_sensor_callback, BIT(sensor2.pin));

    gpio_add_callback(sensor1.port, &sensor1_cb_data);
    gpio_add_callback(sensor2.port, &sensor2_cb_data);

    while (1) {
        k_msleep(10);
        memcpy(anglebuf, angle, sizeof(angle));
        dual_sync_flush(&dual_protocol, angle_tx);
    }
    return 0;
}