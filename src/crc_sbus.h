#ifndef _CRC_SBUS_H
#define _CRC_SBUS_H



#include "sbus.h"
#include <mcu.h>
#include "mcu_periph/sys_time.h"
#include <stdio.h>
#define CRC_SBUS_FRAME_SIZE 27 //sbus + 2 crc : sbus_frame[0-23]+[crc][crc]+[sbus end falg]
#define CRC_SBUS_CRC_L_IDX 24 //sbus + 2 crc : sbus_frame[0-23]+[crc][crc]+[sbus end falg]
#define CRC_SBUS_CRC_H_IDX 25 //sbus + 2 crc : sbus_frame[0-23]+[crc][crc]+[sbus end falg]
#define CRC_SBUS_ENDBYTE_IDX 26 //sbus + 2 crc : sbus_frame[0-23]+[crc][crc]+[sbus end falg]
#define CRC_SBUS_CRC_DATA_SIZE 24 //sbus_frame[0-23]
//crc is check the byte[1]-byte[23]

uint16_t sbus_crc_calculate(const uint8_t* pBuffer, uint16_t length);
void encode_crc_sbus_frame(uint16_t *values, uint16_t num_values, uint8_t *oframe);
void send_crc_sbus_out (uint16_t *rc_chans, uint16_t rc_count, void (*cb)(uint8_t * data, int len));
inline int parse_crc_sbus_frame(uint8_t rbyte , struct sbus_buffer *sbus_p)
{
      uint8_t ret = 0;
      uint16_t crc,crc_cal;

      switch (sbus_p->status) {
        case SBUS_STATUS_UNINIT:
          // Wait for the start byte
          if (rbyte == SBUS_START_BYTE) {
            sbus_p->buffer[0] = rbyte;
            sbus_p->status++;
            sbus_p->idx = 1;
          }
          break;
        case SBUS_STATUS_GOT_START:
          // Store buffer
          sbus_p->buffer[sbus_p->idx] = rbyte;
          sbus_p->idx++;
          if (sbus_p->idx == CRC_SBUS_FRAME_SIZE) {
            // Decode if last byte is the correct end byte
            if (rbyte == SBUS_END_BYTE) {
		crc = sbus_p->buffer[CRC_SBUS_CRC_L_IDX] | (sbus_p->buffer[CRC_SBUS_CRC_H_IDX]<<8);
		crc_cal = sbus_crc_calculate(sbus_p->buffer,CRC_SBUS_CRC_DATA_SIZE);
		if( crc == crc_cal ){
              		decode_sbus_frame(sbus_p->buffer, sbus_p->rc_chans, &sbus_p->frame_available);
	      		ret = 1;
		}else{
			if(sbus_p->frame_decode_faile < 0x7fff )
				sbus_p->frame_decode_faile ++;
		}
            }else{
		if(sbus_p->frame_capture_faile < 0x7fff )
			sbus_p->frame_capture_faile ++;
	    }
            sbus_p->status = SBUS_STATUS_UNINIT;
          }
          break;
        default:
          break;
      }	

      return ret;
}


#endif
