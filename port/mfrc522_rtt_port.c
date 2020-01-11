#include "mfrc522.h"

static struct rt_spi_device mfrc522_spi_dev;
struct rt_hw_spi_cs
{
    rt_uint32_t pin;
};
static struct rt_hw_spi_cs spi_cs; 

static int rt_hw_spi_rc522_init()
{
    rt_err_t res = RT_EOK;

    // Attach Device
    spi_cs.pin = MFRC522_SS_PIN;
    rt_pin_mode(spi_cs.pin, PIN_MODE_OUTPUT);
    res = rt_spi_bus_attach_device(&mfrc522_spi_dev, MFRC522_SPI_DEVICE_NAME, MFRC522_SPI_BUS_NAME, (void*)&spi_cs);
    if (res != RT_EOK)
    {
        rt_kprintf("[RC522] Failed to attach device %s\n", MFRC522_SPI_DEVICE_NAME);
        return res;
    }

    // Set device SPI Mode
    struct rt_spi_configuration cfg = {0};
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB | RT_SPI_NO_CS;
    cfg.max_hz = MFRC522_SPICLOCK;

    rt_spi_configure(&mfrc522_spi_dev, &cfg);

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(rt_hw_spi_rc522_init);
