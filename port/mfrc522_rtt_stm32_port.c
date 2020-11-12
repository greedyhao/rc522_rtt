#include "mfrc522.h"
#include <drv_spi.h>

static int rt_hw_spi_rc522_init()
{
    rt_err_t res = RT_EOK;
    //此处的 GPIOA 和 GPIO_PIN_4 需要根据实际使用的 NSS 引脚编号配置。
    res = rt_hw_spi_device_attach(MFRC522_SPI_BUS_NAME, MFRC522_SPI_DEVICE_NAME, GPIOA, GPIO_PIN_4);
    if (res != RT_EOK)
    {
        rt_kprintf("[RC522] Failed to attach device %s\n", MFRC522_SPI_DEVICE_NAME);
        return res;
    }
    struct rt_spi_device *spi_dev = (struct rt_spi_device *)rt_device_find(MFRC522_SPI_DEVICE_NAME);
    // Set device SPI Mode
    struct rt_spi_configuration cfg = {0};
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB | RT_SPI_NO_CS;
    cfg.max_hz = MFRC522_SPICLOCK;
    rt_spi_configure(spi_dev, &cfg);

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(rt_hw_spi_rc522_init);
