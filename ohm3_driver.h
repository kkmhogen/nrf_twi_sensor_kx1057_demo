#ifndef __OHM_DRIVER__H
#define __OHM_DRIVER__H

#include <stdbool.h>
#include <stdint.h>

#define OHM_ADDR_READ 0xE  
#define OHM_READ_VALUE 0x35

#define regAddressCNTL1  0x1B
#define regAddressCNTL2  0x1D
#define regAddressINTREL  0x1A

#define regAddressWAKEUPCount  0x29
#define regAddressWAKEUP1  0x6A
#define regAddressWAKEUP2  0x6B

#define regAddressINTCtrlReg1  0x1E

#define regAddressIntSrc1  0x16

#define regAddressSelftest 0x3A

#define regAddressDataCtrl 0x21 



//OUTPUT REGISTER
#define OHM_OUT_X_L					0x6
#define OHM_OUT_X_H					0x7
#define OHM_OUT_Y_L					0x8
#define OHM_OUT_Y_H					0x9
#define OHM_OUT_Z_L					0xA
#define OHM_OUT_Z_H					0xB

typedef enum eAccOdrHz
{
    AccOdr1Hz,
    AccOdr10Hz,
    AccOdr25Hz,
    AccOdr50Hz,
    AccMaxOdr
}EAccDecHz;

typedef enum {
  MEMS_SUCCESS				=		0x0,
  MEMS_ERROR				=		0x1	
} status_t;

typedef struct {
  int16_t AXIS_X;
  int16_t AXIS_Y;
  int16_t AXIS_Z;
} AxesRaw_t;

void OHM_SoftReset(void);

bool OHM_IsResetComplete(void);

void OHM_SensorStoped(void);

uint32_t OHM_StartSelfTest(void);

void OHM_PosDetectEnable(EAccDecHz decHz);

bool OHM_MotionDetectEnable(uint8_t odr, uint8_t accRange, uint8_t nDuration);

status_t OHM_GetPosAxesRaw(AxesRaw_t* buff) ;

status_t OHM_GetAccAxesRaw(AxesRaw_t* buff) ;

status_t OHM_GetInt1Src(uint8_t* val) ;

#endif
