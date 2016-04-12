#include "mcu_periph/sys_time.h"
#include "sbus.h"
/*
 *
 * *********************************** sbus function 
 *
 */


void sbus_buffer_init(struct sbus_buffer *sbus_b)
{
	sbus_b->frame_available = FALSE;
	sbus_b->status = SBUS_STATUS_UNINIT;
}

#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f
#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f
/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))
void encode_sbus_frame(uint16_t *values, uint16_t num_values, uint8_t *oframe)
{
	static float sbus_scale_factor = SBUS_SCALE_FACTOR;// = ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN));
	static int  sbus_scale_offset = SBUS_SCALE_OFFSET ;//= (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f));

	uint8_t byteindex = 1; /*Data starts one byte into the sbus frame. */
	uint8_t offset = 0;
	uint16_t value;
	int i;

  	memset(oframe, 0x0, SBUS_FRAME_SIZE);
	oframe[0]=SBUS_START_BYTE;

	/* 16 is sbus number of servos/channels minus 2 single bit channels.
	* currently ignoring single bit channels.  */
	for (i = 0; (i < num_values) && (i < SBUS_MAX_RC_COUNT); ++i) {
		//value = (uint16_t)(((values[i] - sbus_scale_offset) / sbus_scale_factor) + 0.5f);
    		value = (uint16_t)values[i];
	    	/*protect from out of bounds values and limit to 11 bits*/
	    	if (value > 0x07ff) {
			value = 0x07ff;
	    	}
		while (offset >= 8) {
		++byteindex;
			offset -= 8;
      		}
      		oframe[byteindex] |= (value << (offset)) & 0xff;
      		oframe[byteindex + 1] |= (value >> (8 - offset)) & 0xff;
      		oframe[byteindex + 2] |= (value >> (16 - offset)) & 0xff;
      		offset += 11;
    	}

	oframe[SBUS_FLAGS_BYTE_IDX]= 0;
	oframe[SBUS_END_BYTE_IDX]= SBUS_END_BYTE;
}


/** Decode the raw buffer */
void decode_sbus_frame(const uint8_t *src, uint16_t *dst, bool_t *available)
{
  // reset counters
  uint8_t byteInRawBuf = 0;
  uint8_t bitInRawBuf = 0;
  uint8_t channel = 0;
  uint8_t bitInChannel = 0;

  // clear bits
  memset(dst, 0, SBUS_MAX_RC_COUNT * sizeof(uint16_t));

  // decode sbus data
  for (uint8_t i = 0; i < (SBUS_MAX_RC_COUNT * SBUS_BIT_PER_CHANNEL); i++) {
    if (src[byteInRawBuf] & (1 << bitInRawBuf)) {
      dst[channel] |= (1 << bitInChannel);
    }

    bitInRawBuf++;
    bitInChannel++;

    if (bitInRawBuf == SBUS_BIT_PER_BYTE) {
      bitInRawBuf = 0;
      byteInRawBuf++;
    }
    if (bitInChannel == SBUS_BIT_PER_CHANNEL) {
      bitInChannel = 0;
      channel++;
    }
  }
  // test frame lost flag
  *available = !bit_is_set(src[SBUS_FLAGS_BYTE_IDX-1], SBUS_FRAME_LOST_BIT);
}

#if 0
static const struct sbus_bit_pick sbus_decoder[SBUS_MAX_RC_COUNT][3] = {
	/*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
	/*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
	/*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
	/*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
	/* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
	/* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};
void decode_sbus_frame_pix(const uint8_t *frame, uint16_t *dst, bool_t *available)
{
	static float sbus_scale_factor = SBUS_SCALE_FACTOR;// = ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN));
	static int  sbus_scale_offset = SBUS_SCALE_OFFSET ;//= (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f));

	for (unsigned channel = 0; channel < SBUS_MAX_RC_COUNT; channel++) {
		unsigned value = 0;
		for (unsigned pick = 0; pick < 3; pick++) {
			const struct sbus_bit_pick *decode = &sbus_decoder[channel][pick];

			if (decode->mask != 0) {
				unsigned piece = frame[1 + decode->byte];
				piece >>= decode->rshift;
				piece &= decode->mask;
				piece <<= decode->lshift;

				value |= piece;
			}
		}


		/* convert 0-2048 values to 1000-2000 ppm encoding in a not too sloppy fashion */
		dst[channel] = (uint16_t)(value * sbus_scale_factor + .5f) + sbus_scale_offset;
	}
	/* decode and handle failsafe and frame-lost flags */
	if (frame[SBUS_FLAGS_BYTE_IDX] & (1 << SBUS_FRAME_FAILSAFE_BIT)) { /* failsafe */
		/* report that we failed to read anything valid off the receiver */
		//*sbus_failsafe = true;
		//*sbus_frame_drop = true;
		*available = FALSE;

	} else if (frame[SBUS_FLAGS_BYTE_IDX] & (1 << SBUS_FRAME_LOST_BIT)) { /* a frame was lost */
		/* set a special warning flag
		 *
		 * Attention! This flag indicates a skipped frame only, not a total link loss! Handling this
		 * condition as fail-safe greatly reduces the reliability and range of the radio link,
		 * e.g. by prematurely issueing return-to-launch!!! */

		//*sbus_failsafe = false;
		//*sbus_frame_drop = true;
		*available = FALSE;

	} else {
		*available = TRUE;
		//*sbus_failsafe = false;
		//*sbus_frame_drop = false;
	}
}

#endif


/*
*
* byte : read frome uart or spi
* sbus_buff : static , store the byte for the whole sbus frame  
 		if get a sbus frame , decode it to sbus_buff->rc_chans 
		and return 1
*/
int parse_sbus_frame(uint8_t rbyte , struct sbus_buffer *sbus_p)
{
	uint8_t ret = 0;
      switch (sbus_p->status) {
        case SBUS_STATUS_UNINIT:
          // Wait for the start byte
          if (rbyte == SBUS_START_BYTE) {
            sbus_p->status++;
            sbus_p->idx = 1;
          }
          break;
        case SBUS_STATUS_GOT_START:
          // Store buffer
          sbus_p->buffer[sbus_p->idx] = rbyte;
          sbus_p->idx++;
          if (sbus_p->idx == SBUS_FRAME_SIZE) {
            // Decode if last byte is the correct end byte
            if (rbyte == SBUS_END_BYTE) {
              decode_sbus_frame(&sbus_p->buffer[1], sbus_p->rc_chans, &sbus_p->frame_available);
	      ret = 1;
            }
            sbus_p->status = SBUS_STATUS_UNINIT;
          }
          break;
        default:
          break;
      }	

      return ret;
}

static uint8_t oframe[SBUS_FRAME_SIZE] = { 0x0f };
void send_sbus_out (uint16_t *rc_chans, uint16_t rc_count, void (*cb)(uint8_t * data, int len))
{
	encode_sbus_frame(rc_chans,rc_count, oframe);
	cb( oframe, SBUS_FRAME_SIZE);
}
void send_sbus_out_loop(uint16_t *rc_chans, uint16_t rc_count, void (*cb)(uint8_t * data, int len))
{
	static float last_time=0;
	float time = get_sys_time_float();
	float diff_time = time - last_time;

	diff_time = diff_time*1000;//to ms
	if (diff_time>7 || last_time==0) {
		last_time = time;
		encode_sbus_frame(rc_chans,rc_count, oframe);
		cb( oframe, SBUS_FRAME_SIZE);
	}
}
