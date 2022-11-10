#include "ohm3_driver.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include <stdlib.h>

extern status_t Acc_ReadReg(uint8_t addr, uint8_t *buf);

extern status_t Acc_WriteReg(uint8_t addr, uint8_t buf);

static status_t OHM_ReadReg(uint8_t addr, uint8_t *buf)
{
    return Acc_ReadReg(addr, buf);
}

static status_t OHM_WriteReg(uint8_t addr, uint8_t buf)
{
    return Acc_WriteReg(addr, buf);
}

void OHM_SoftReset(void)
{
    Acc_WriteReg(regAddressCNTL2, 0x80);
}

bool OHM_IsResetComplete(void)
{
    uint8_t CNTL2 = 0;
    Acc_ReadReg(regAddressCNTL2, &CNTL2);
    return ((CNTL2 & 0x55) == 0);
}


void OHM_SensorStoped(void)
{
    uint8_t readData = 0;
    
    OHM_WriteReg(regAddressCNTL1, 0x0);
    OHM_ReadReg(regAddressINTREL, &readData);

    //clear
    AxesRaw_t raw;
    OHM_GetAccAxesRaw(&raw);
    
    NRF_LOG_INFO("OHM sensor stop complete");
}

void OHM_PosDetectEnable(EAccDecHz decHz)
{
    uint8_t readData = 0;

    //clear
    AxesRaw_t raw;
    
    //heigh current, 16bit sensitive, standby
    OHM_WriteReg(regAddressCNTL1, 0x00);
    OHM_ReadReg(regAddressINTREL, &readData);

    //set odh = 50hz
    if (decHz == AccOdr1Hz)
    {
        OHM_WriteReg(regAddressCNTL2, 0x1);
        OHM_WriteReg(regAddressDataCtrl, 0x9);
    }
    else
    {
        OHM_WriteReg(regAddressCNTL2, 0x4);
        OHM_WriteReg(regAddressDataCtrl, 0x0);
    }

    //setCNTL1reg to operator mode
    OHM_WriteReg(regAddressCNTL1, 0x80); //0x80

    NRF_LOG_INFO("OHM sensor pos enable:%d", (int)decHz);
}


uint32_t OHM_StartSelfTest(void)
{
    NRF_LOG_INFO("success test");

    OHM_PosDetectEnable(AccOdr50Hz);
    nrf_delay_ms(30);

    AxesRaw_t raw;
    OHM_GetAccAxesRaw(&raw);
    uint32_t result = abs(raw.AXIS_X) + abs(raw.AXIS_Y) + abs(raw.AXIS_Z);
    if (result < 100)
    {
        nrf_delay_ms(25);
        OHM_GetAccAxesRaw(&raw);
        result = abs(raw.AXIS_X) + abs(raw.AXIS_Y) + abs(raw.AXIS_Z);
        if (result < 100)
        {
            result = 0;
        }
    }

    return result;
}


bool OHM_MotionDetectEnable(uint8_t odr, uint8_t accRange, uint8_t nDuration)
{
    if (accRange <= 1)
    {
        accRange = 1;
    }

    uint8_t readData = 0;

    /* to set the accelerometer in stand-by mode,
    to set the performance mode to High Resolution (full power), G-range to ¡À2g, and
    enable the Wake Up (motion detect) function.
    */
    OHM_WriteReg(regAddressCNTL1, 0x0);  //0x2

    //Read  from  the  Interrupt  Latch  Release  Register  (INT_REL)  to  clear  any  outstanding interrupts.
    OHM_ReadReg(regAddressINTREL, &readData);

    //for the wake up function  50hz
    if (odr == AccOdr1Hz)
    {
        OHM_WriteReg(regAddressCNTL2, 0x1);
        OHM_WriteReg(regAddressDataCtrl, 0x9);
    }
    else if (odr == AccOdr10Hz)
    {
        OHM_WriteReg(regAddressCNTL2, 0x4);
        OHM_WriteReg(regAddressDataCtrl, 0x0);
    }
    else if (odr == AccOdr25Hz)
    {
        OHM_WriteReg(regAddressCNTL2, 0x5);
        OHM_WriteReg(regAddressDataCtrl, 0x1);
    }
    else
    {
        OHM_WriteReg(regAddressCNTL2, 0x6);
        OHM_WriteReg(regAddressDataCtrl, 0x2);
    }
    
    //for the wake up for 1/odr = 1/50 * nDuration = 5ms
    OHM_WriteReg(regAddressWAKEUPCount, nDuration);

    //motion range
    uint16_t nRange = accRange * 4;//accRange * 1;
    nRange = (nRange << 4);
    OHM_WriteReg(regAddressWAKEUP1, ((nRange >> 8) & 0xFF));
    OHM_WriteReg(regAddressWAKEUP2, (nRange & 0xFF));


    // Write 0x30 to Interrupt Control Register 1 (INT_CTRL_REG1) to configure the hardware interrupt
    OHM_WriteReg(regAddressINTCtrlReg1, 0x30);

    OHM_WriteReg(regAddressCNTL1, 0x82);  //0x2


    uint8_t nIntValue = 0;
    OHM_GetInt1Src(&nIntValue);
    NRF_LOG_INFO("OHM sensor motion enable, odr:%d, %d", odr, (nRange >>4));

    return true;

}

status_t OHM_GetInt1Src(uint8_t* val)
{
    uint8_t INT_REL = 0;
    //read acc source
    OHM_ReadReg(regAddressIntSrc1, val);

    if( MEMS_ERROR == OHM_ReadReg(regAddressINTREL, &INT_REL) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

status_t OHM_GetPosAxesRaw(AxesRaw_t* buff)
{
    //uint8_t valueL = 0;
    uint8_t valueH = 0;

    //get x
    if( MEMS_ERROR == OHM_ReadReg(OHM_OUT_X_H, &valueH) )
        return MEMS_ERROR;
    buff->AXIS_X = (int16_t)((int8_t)valueH * 15.625f);

    //get y
    if( MEMS_ERROR == OHM_ReadReg(OHM_OUT_Y_H, &valueH) )
        return MEMS_ERROR;
    buff->AXIS_Y = (int16_t)((int8_t)valueH * 15.625f);

    //get z
    if( MEMS_ERROR == OHM_ReadReg(OHM_OUT_Z_H, &valueH) )
        return MEMS_ERROR;
    buff->AXIS_Z = (int16_t)((int8_t)valueH * 15.625f);

    return MEMS_SUCCESS;
}


status_t OHM_GetAccAxesRaw(AxesRaw_t* buff)
{
    return OHM_GetPosAxesRaw(buff);
}
