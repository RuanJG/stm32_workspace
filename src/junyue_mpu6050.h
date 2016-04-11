#ifndef _JUNYUE_MPU6050_H
#define _JUNYUE_MPU6050_H



#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/i2c.h"

#define JUNYUE_MPU6050_I2C_ADDR 0xA0
#define AX_REG 0x34
#define AY_REG 0x35
#define AZ_REG 0x36
#define GX_REG 0x37
//
//
#define HX_REG 0x3A
//
//
#define ROLL_REG 0x3D
#define PITCH_REG 0x3E
#define YAW_REG 0x3F

struct mpu6050_arhs{
	float ax;
	float ay;
	float az;

	float gx;
	float gy;
	float gz;

	int16_t mx;
	int16_t my;
	int16_t mz;

	float roll;
	float pitch;
	float yaw;
};

void mpu6050_i2c_write_to_reg(uint8_t _reg, uint8_t _val);

extern struct mpu6050_arhs mpu6050_arhs_data;
void mpu6050_i2c_init(struct i2c_periph *i2c_p);
void mpu6050_i2c_event();
void mpu6050_i2c_periodic();
bool is_mpu6050_valiable();
void set_mpu6050_data_has_read();



#endif
