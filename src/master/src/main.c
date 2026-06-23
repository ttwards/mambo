#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
// #include <cstddef>

LOG_MODULE_REGISTER(spi_master, LOG_LEVEL_INF);

#define SPI_NODE DT_NODELABEL(spi2)
static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);
#define CS_PORT_NODE DT_NODELABEL(gpiob)
static const struct device *cs_port = DEVICE_DT_GET(CS_PORT_NODE);
#define CS_PIN 12
static struct spi_config spi_cfg = {
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA | SPI_OP_MODE_MASTER,
    .frequency = 21000000, // 提高波特率以确保 0.5ms 内能传完数据
    .cs = {
      .gpio = SPI_CS_GPIOS_DT_SPEC_GET(SPI_NODE),
      .delay = 0,
    },
    
};

#define BUF_SIZE 16
uint8_t tx_buf_data[BUF_SIZE] __aligned(4);
uint8_t rx_buf_data[BUF_SIZE] __aligned(4);

int main(void)
{
	k_msleep(1000);
    tx_buf_data[0]=15;
    if (!device_is_ready(spi_dev)) {
      LOG_ERR("SPI 设备未就绪");
     
  }int ret = gpio_pin_configure(cs_port, CS_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH);
  if (ret < 0) {
      LOG_ERR("引脚配置失败，错误码：%d (大概率是引脚被其他外设占用了)", ret);
      
  }
    struct spi_buf tx_b = { .buf = tx_buf_data, .len = BUF_SIZE };
    struct spi_buf_set tx_bufs = { .buffers = &tx_b, .count = 1 };
    struct spi_buf rx_b = { .buf = rx_buf_data, .len = BUF_SIZE };
    struct spi_buf_set rx_bufs = { .buffers = &rx_b, .count = 1 };

    struct k_timer master_timer;
    k_timer_init(&master_timer, NULL, NULL);
    
    // 500微秒触发一次 (2000Hz)
    k_timer_start(&master_timer, K_MSEC(1), K_USEC(100));
    // gpio_pin_set_raw(cs_port, CS_PIN, 0);
	// k_msleep(1);
	// int err = spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs);
	// gpio_pin_set_raw(cs_port, CS_PIN, 1);
    uint32_t total_sent = 0;
    while (1) {
        k_timer_status_sync(&master_timer);
	gpio_pin_set_raw(cs_port, CS_PIN, 0);
	// k_msleep(1);
	int err = spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs);
	gpio_pin_set_raw(cs_port, CS_PIN, 1);
        if (err == 0) {
            total_sent++;
            // 每 2000 帧打印一次，避免阻塞
            if (total_sent % 2000 == 0) {
                LOG_INF("Master: Sent %u frames", total_sent);
            }
        }
    }
}