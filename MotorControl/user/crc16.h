
#ifndef __CRC16_H__
#define __CRC16_H__
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


//******************************************* public attribute ******************************************
uint16_t crc16(uint8_t *puchHeadAddr,uint8_t uchLen);

#ifdef __cplusplus
}
#endif
#endif
