#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "ohm3_driver.h"

/* TWI instance ID. */
static uint8_t gapI2cAddress = OHM_ADDR_READ;
#define TWI_INSTANCE_ID     0
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);


//change the PIN to device
const nrf_drv_twi_config_t twi_acc_config =
{
    .scl                = 9,
    .sda                = 6,
    .frequency          = NRF_DRV_TWI_FREQ_400K,
    .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
    .clear_bus_init     = true,
    .hold_bus_uninit = false
};

status_t Acc_ReadReg(uint8_t addr, uint8_t *buf)
{
    uint32_t err_code = 0;
    uint8_t readAddr[] = {addr};

    err_code = nrf_drv_twi_tx(&m_twi, gapI2cAddress, readAddr, 1, true);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_rx(&m_twi, gapI2cAddress, buf, 1);
    APP_ERROR_CHECK(err_code);

    return MEMS_SUCCESS;
}

status_t Acc_WriteReg(uint8_t addr, uint8_t buf)
{
    uint8_t txBuf[] = {addr, buf};

    uint32_t err_code = nrf_drv_twi_tx(&m_twi, gapI2cAddress, txBuf, sizeof(txBuf), true);
    APP_ERROR_CHECK(err_code);

    return MEMS_SUCCESS;
}
void makeSureOHMReset(void)
{
    OHM_SoftReset();

    nrf_drv_twi_disable(&m_twi);
    nrf_drv_twi_uninit(&m_twi);
    nrf_delay_ms(15);
    uint32_t err_code = nrf_drv_twi_init(&m_twi, &twi_acc_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(&m_twi);
    while (!OHM_IsResetComplete())
    {
        nrf_delay_ms(5);
    }
}

 /* Number of possible TWI addresses. */
 #define TWI_ADDRESSES      127
void twi_scan()
{
    uint8_t address;
    uint8_t sample_data;

    for (address = 1; address <= TWI_ADDRESSES; address++)
    {
        int err_code = nrf_drv_twi_rx(&m_twi, address, &sample_data, sizeof(sample_data));
        if (err_code == NRF_SUCCESS)
        {
            NRF_LOG_INFO("TWI device detected at address 0x%x.", address);
            NRF_LOG_FLUSH();
        }
    }
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nTWI sensor example started.");
    NRF_LOG_FLUSH();

    uint32_t err_code = nrf_drv_twi_init(&m_twi, &twi_acc_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(&m_twi);

    twi_scan();

    //reset device (option)
    makeSureOHMReset();

    //start 10 hz xyz reading
    OHM_PosDetectEnable(AccOdr10Hz);

    //get xyz
    AxesRaw_t data;
    while (true)
    {
        OHM_GetPosAxesRaw(&data); 

        NRF_LOG_INFO("x:%d, y:%d, z:%d", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);
        NRF_LOG_FLUSH();

        nrf_delay_ms(1000);
    }
}

/** @} */
