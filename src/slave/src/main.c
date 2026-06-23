#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <sys/_stdint.h>

LOG_MODULE_REGISTER(spi_slave, LOG_LEVEL_INF);

#define SPI_NODE DT_NODELABEL(spi2)
static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

static struct spi_config spi_cfg = {
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA | SPI_OP_MODE_SLAVE,
    .frequency = 21000000,
    .cs = NULL, 
};

#define BUF_SIZE 16
uint8_t slave_rx[BUF_SIZE] __aligned(4);
uint8_t slave_tx[BUF_SIZE] __aligned(4);

int main(void) {
    k_msleep(1000); // 等待主设备准备好

    struct spi_buf tx_b = { .buf = slave_tx, .len = BUF_SIZE };
    struct spi_buf_set tx_bufs = { .buffers = &tx_b, .count = 1 };
    struct spi_buf rx_b = { .buf = slave_rx, .len = BUF_SIZE };
    struct spi_buf_set rx_bufs = { .buffers = &rx_b, .count = 1 };

    uint32_t frame_counter = 0;
    uint32_t loss_counter = 0;
    LOG_INF("Slave ready, waiting for 2000Hz stream...");

    while (1) {
	    // k_msleep(10);
        // 阻塞等待下一次 DMA 传输完成
        int err = spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs);
        
        if (err >= 0) {
            frame_counter++;
            // LOG_INF("...Received frame %u: %02X %02X %02X ...", frame_counter, slave_rx[0], slave_rx[1], slave_rx[2]);
            // 检查逻辑：例如每秒报告一次接收情况 (2000Hz * 1s = 2000帧)
            if (frame_counter % 20000 == 0) {
                LOG_INF("Slave status: Received %u frames total", frame_counter);
                LOG_ERR("Slave status: Detected %u lost frames", loss_counter);
                /* 这里可以加入数据完整性检查 */
                if (slave_rx[0] != 15) { 
                    loss_counter++;
                } 
            }
        } else {
            LOG_ERR("SPI Slave error: %d", err);
        }
    }
}