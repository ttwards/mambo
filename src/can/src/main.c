#include "ares/board/init.h"
#include "math.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
#define can_node DT_NODELABEL(can1)

 
const struct device *can_dev = DEVICE_DT_GET(can_node);

 
 /* CAN接收回调函数 */
 void can_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
 {
     printk("收到CAN消息: ID=0x%03X, DLC=%d, 数据=", frame->id, frame->dlc);
     for (int i = 0; i < frame->dlc; i++) {
         printk("%02X ", frame->data[i]);
     }
     printk("\n");
 }
 
 /* CAN消息发送函数 */
 int send_can_msg(uint32_t id, uint8_t *data, uint8_t len)
 {
     struct can_frame frame = {
         
         
         .id = id,
         .dlc = len
     };
     
     memcpy(frame.data, data, len);
     
     int ret = can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);
     if (ret == 0) {
         LOG_INF("CAN发送成功: ID=0x%03X, 长度=%d\n", id, len);
     } else {
      LOG_INF("CAN发送失败: %d\n", ret);
     }
     return ret;
 }

 /* CAN初始化 */
 int init_can(void)
 {
     
     if (!device_is_ready(can_dev)) {
         printk("CAN设备未就绪\n");
         return -1;
     }
 
    
 
     /* 添加接收过滤器 - 接收ID 0x123的消息 */
     struct can_filter filter = {
       
         .id = 0x00000000,
         .mask = 0x00000000
     };
     can_add_rx_filter(can_dev, can_rx_callback, NULL, &filter);
 
     /* 启动CAN */
     can_start(can_dev);
     
     LOG_INF("CAN初始化完成\n");
     return 0;
 }
 

 /* 主函数 */
 int main(void)
 {
    
 
     /* 初始化设备 */
     if (init_can() != 0 ) {
         printk("设备初始化失败\n");
         return -1;
     }
 
     /* 测试数据 */
     uint8_t test_data[] = {0x11, 0x22, 0x33, 0x44};
     int counter = 0;
 
     while (1) {
         /* 每2秒发送一次CAN消息 */
         test_data[0] = counter & 0xFF;  // 更新第一个字节
         send_can_msg(0x123, test_data, 4);
         
    
     }
 
     return 0;
 }