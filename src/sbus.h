#define SBUS_START_BYTE 0x0f
#define SBUS_END_BYTE 0x00
#define SBUS_BIT_PER_CHANNEL 11
#define SBUS_BIT_PER_BYTE 8
#define SBUS_FRAME_LOST_BIT 2
#define SBUS_FRAME_FAILSAFE_BIT 3
#define SBUS_FLAGS_BYTE_IDX 23
#define SBUS_END_BYTE_IDX 24
#define SBUS_FRAME_SIZE 25
#define SBUS_MAX_RC_COUNT 16 //[startbyte] [data1] [data2] .... [data22] [flags][endbyte]  {data1-data22}=8bit*22=11bit*16
/*
flags：
bit7 = ch17 = digital channel (0x80)
bit6 = ch18 = digital channel (0x40)
bit5 = Frame lost, equivalent red LED on receiver (0x20)
bit4 = failsafe activated (0x10)
bit3 = n/a
bit2 = n/a
bit1 = n/a
bit0 = n/a
*/

#define SBUS_STATUS_UNINIT      0
#define SBUS_STATUS_GOT_START   1
struct sbus_buffer {
  uint16_t rc_chans[SBUS_MAX_RC_COUNT]; ///< decoded values
  bool_t frame_available;           ///< new frame available
  uint8_t buffer[SBUS_FRAME_SIZE];  ///< input buffer
  uint8_t idx;                      ///< input index
  uint8_t status;                   ///< decoder state machine status
};
/*
 * S.bus decoder matrix.
 *
 * Each channel value can come from up to 3 input bytes. Each row in the
 * matrix describes up to three bytes, and each entry gives:
 *
 * - byte offset in the data portion of the frame
 * - right shift applied to the data byte
 * - mask for the data byte
 * - left shift applied to the result into the channel value
 */
struct sbus_bit_pick {
	uint8_t byte;
	uint8_t rshift;
	uint8_t mask;
	uint8_t lshift;
};



void encode_sbus_frame(uint16_t *values, uint16_t num_values, uint8_t *oframe);
void decode_sbus_frame(const uint8_t *src, uint16_t *dst, bool_t *available);

//reciver use these function
void sbus_buffer_init(struct sbus_buffer *sbus_b);
int parse_sbus_frame(uint8_t byte , struct sbus_buffer *sbus_p);
//sender just use these
void send_sbus_out_loop(uint16_t *rc_chans, uint16_t rc_count, void (*cb)(uint8_t * data, int len));
void send_sbus_out (uint16_t *rc_chans, uint16_t rc_count, void (*cb)(uint8_t * data, int len));
