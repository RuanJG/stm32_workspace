#include "junyue_mpu6050.h"
#define DEBUG_APP 1
#include <stdio.h>
#define log(format, ...) if( DEBUG_APP ) printf(format, ## __VA_ARGS__)

struct i2c_periph *mpu6050_i2c_dev;
//uint8_t mpu6050_i2c_addr = 0;
struct i2c_transaction mpu6050_i2c_trans ;
struct Uint16Vect3 gry;
struct Uint16Vect3 accel;
struct Uint16Vect3 mag;
struct mpu6050_arhs mpu6050_arhs_data;
volatile uint8_t mpu6050_data_update = 0;

void mpu6050_i2c_init(struct i2c_periph *i2c_p)
{
  /* set i2c_peripheral */
  mpu6050_i2c_dev = i2c_p;

  /* slave address */
  mpu6050_i2c_trans.slave_addr = JUNYUE_MPU6050_I2C_ADDR;
  /* set inital status: Success or Done */
  mpu6050_i2c_trans.status = I2CTransDone;

	mpu6050_arhs_data.roll = 0;
	mpu6050_arhs_data.pitch = 0;
	mpu6050_arhs_data.yaw = 0;

	mpu6050_arhs_data.ax = 0;
	mpu6050_arhs_data.ay = 0;
	mpu6050_arhs_data.az = 0;

	mpu6050_arhs_data.gx = 0;
	mpu6050_arhs_data.gy = 0;
	mpu6050_arhs_data.gz = 0;

	mpu6050_arhs_data.mx = 0;
	mpu6050_arhs_data.my = 0;
	mpu6050_arhs_data.mz = 0;

}


void mpu6050_i2c_write_to_reg(uint8_t _reg, uint8_t _val)
{
  mpu6050_i2c_trans.buf[0] = _reg;
  mpu6050_i2c_trans.buf[1] = _val;
  i2c_transmit(mpu6050_i2c_dev, &(mpu6050_i2c_trans), mpu6050_i2c_trans.slave_addr, 2);
}

void mpu6050_i2c_read(uint8_t reg, uint16_t len)
{
  if (mpu6050_i2c_trans.status == I2CTransDone) {
    /* set read bit and multiple byte bit, then address */
    mpu6050_i2c_trans.buf[0] = reg;
    i2c_transceive(mpu6050_i2c_dev, &(mpu6050_i2c_trans), mpu6050_i2c_trans.slave_addr, 1, len);
  }
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))
#define FloatFromBuf(_buf,_idx) ((float)((_buf[_idx]<<8) | _buf[_idx+1]))

void mpu6050_i2c_event()
{
    if (mpu6050_i2c_trans.status == I2CTransFailed) {
      mpu6050_i2c_trans.status = I2CTransDone;
      //log("xxxxxx \r\n");
    } else if (mpu6050_i2c_trans.status == I2CTransSuccess) {
      // Successfull reading
      //if (bit_is_set(mpu6050_i2c_trans.buf[0], 0)) {
      if (mpu6050_i2c_trans.type == I2CTransRx) {

        mpu6050_arhs_data.ax = FloatFromBuf(mpu6050_i2c_trans.buf, 0)/32768*16;
        mpu6050_arhs_data.ay = FloatFromBuf(mpu6050_i2c_trans.buf, 2)/32768*16;
        mpu6050_arhs_data.az = FloatFromBuf(mpu6050_i2c_trans.buf, 4)/32768*16;

        mpu6050_arhs_data.gx = FloatFromBuf(mpu6050_i2c_trans.buf, 4)/32768*2000;
        mpu6050_arhs_data.gy = FloatFromBuf(mpu6050_i2c_trans.buf, 6)/32768*2000;
        mpu6050_arhs_data.gz = FloatFromBuf(mpu6050_i2c_trans.buf, 8)/32768*2000;
	
        mpu6050_arhs_data.mx = Int16FromBuf(mpu6050_i2c_trans.buf, 10);
        mpu6050_arhs_data.my = Int16FromBuf(mpu6050_i2c_trans.buf, 12);
        mpu6050_arhs_data.mz = Int16FromBuf(mpu6050_i2c_trans.buf, 14);

		
#if 0
        mpu6050_arhs_data.roll = FloatFromBuf(mpu6050_i2c_trans.buf, 0)/32768*180;
        mpu6050_arhs_data.pitch = FloatFromBuf(mpu6050_i2c_trans.buf, 2)/32768*180;
        mpu6050_arhs_data.yaw = FloatFromBuf(mpu6050_i2c_trans.buf, 4)/32768*180;
#endif
	mpu6050_data_update = 1;

      }else
	log("w %x \r\n",mpu6050_i2c_trans.type);
      mpu6050_i2c_trans.status = I2CTransDone;
    }
}

void mpu6050_i2c_periodic()
{
    //mpu6050_i2c_read(ROLL_REG,6);
    mpu6050_i2c_read(AX_REG,9);
}

bool is_mpu6050_valiable()
{
	return (mpu6050_data_update==1);
}
void set_mpu6050_data_has_read()
{
	mpu6050_data_update = 0;
}
