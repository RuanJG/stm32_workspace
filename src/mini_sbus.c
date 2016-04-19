#include "mini_sbus.h"

#if 0
#include <libopencm3/stm32/crc.h>
uint32_t sbus_crc_calculate(const uint8_t* pBuffer, uint16_t length)
{
	uint32_t crc;
	int i;
	crc_reset();

	for( i=0; i< length; )
		CRC_DR = pBuffer[i++]|(pBuffer[i++]<<8)|(pBuffer[i++]<<16)|(pBuffer[i++]<<24);
	crc = CRC_DR;
	return crc;
}
#else

/**
 * @brief Initiliaze the buffer for the X.25 CRC
 *
 * @param crcAccum the 16 bit X.25 CRC
 */
#define X25_INIT_CRC 0xffff
static inline void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
        /*Accumulate one byte of data into the CRC*/
        uint8_t tmp;

        tmp = data ^ (uint8_t)(*crcAccum &0xff);
        tmp ^= (tmp<<4);
        *crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
}
static inline void crc_init(uint16_t* crcAccum)
{
        *crcAccum = X25_INIT_CRC;
}
uint16_t mini_sbus_crc_calculate(const uint8_t* pBuffer, uint16_t length)
{
        uint16_t crcTmp;
        crc_init(&crcTmp);
	while (length--) {
                crc_accumulate(*pBuffer++, &crcTmp);
        }
        return crcTmp;
}
#endif


// values [num_values] : the rcs for encode to frame
// frame_channel : the sbus flag bit[0-3] 0~15 , frame_channel[0] = channel (1,2,3,4,5,6,7,8) 
// 			frame_channel[1] = channel ( 9,10,11,12,13,14,15,16 ) ...
void encode_mini_sbus_frame(uint16_t *values, uint16_t num_values, int frame_channel, uint8_t *oframe)
{
	uint16_t crc;
  	//memset(oframe, 0x0, CRC_SBUS_FRAME_SIZE);
	encode_sbus_frame(values,num_values,oframe);
	if( frame_channel < 0x0f )
		oframe[MINI_SBUS_FLAG_IDX]= frame_channel;
	else
		oframe[MINI_SBUS_FLAG_IDX]= 0;
	crc = mini_sbus_crc_calculate(oframe, MINI_SBUS_CRC_DATA_SIZE);
	oframe[MINI_SBUS_CRC_L_IDX]= (crc&0xff);
	oframe[MINI_SBUS_CRC_H_IDX]= ((crc>>8)&0xff);
	oframe[MINI_SBUS_ENDBYTE_IDX]= SBUS_END_BYTE; //end flag
}


static uint8_t mini_oframe[SBUS_FRAME_SIZE] = { 0x0f };
void send_mini_sbus_out (uint16_t *rc_chans, uint16_t rc_count, void (*cb)(uint8_t * data, int len))
{
	encode_mini_sbus_frame(rc_chans,rc_count,0,mini_oframe);
	cb( mini_oframe, MINI_SBUS_FRAME_SIZE);
}

