#ifndef _IMU_READ_H_
#define _IMU_READ_H_

#define BMX055_ACC_SLAVE_ADDRESS_DEFAULT 0x19
#define BMX055_GYRO_SLAVE_ADDRESS_DEFAULT 0x69
#define BMX055_MAG_SLAVE_ADDRESS_DEFAULT 0x13
/// @}

/** Who_am_i register */
#define BMX055_WHO_AM_I_REG 0x00
/** Device check register */
#define BMX055_ACC_DEVICE   0xFA
/** Reset register */
#define BMX055_RESET_REG	0x14
/** Soft reset parameter */
#define BMX055_INITIATED_SOFT_RESET		0xB6

// BMX055 Accel Register
#define BMX055_ACC_DATA_START_REG		0x82 //0x02 for writing but 0x82 for reading
#define BMX055_ACC_PMU_RANGE_REG		0x0F
#define BMX055_ACC_PMU_BW_REG			0x10
#define BMX055_ACC_PMU_LPW_REG			0x11

// BMX055 Gyro Register
#define BMX055_GYRO_DATA_START_REG		0x82
#define BMX055_GYRO_RANGE_REG			0x0F
#define BMX055_GYRO_BW_REG				0x10
#define BMX055_GYRO_LPM1_REG			0x11

// BMX055 Mag Register
#define BMX055_MAG_DATA_START_REG		0xC2
#define BMX055_MAG_POW_CTL_REG			0x4B
#define BMX055_MAG_ADV_OP_OUTPUT_REG	0x4C
#define BMX055_MAG_AXES_REG				0x4E
#define BMX055_MAG_REP_XY_REG       	0x51
#define BMX055_MAG_REP_Z_REG          	0x52

//Gyro Low Power Mode Parameter
#define BMX055_GYRO_LPM1_MODE_NOMAL		0b00
#define BMX055_GYRO_LPM1_MODE_DEEP_SUSPEND	0b01
#define BMX055_GYRO_LPM1_MODE_SUSPEND	0b10

//Accel MPU Range Parameter
#define BMX055_ACC_RANGE_2				0b0011
#define BMX055_ACC_RANGE_4				0b0101
#define BMX055_ACC_RANGE_8				0b1000
#define BMX055_ACC_RANGE_16				0b1100

//Accel MPU Band Width Parameter(Hz)
#define BMX055_ACC_PMU_BW_7_81	  		0b01000
#define BMX055_ACC_PMU_BW_15_63			0b01001
#define BMX055_ACC_PMU_BW_31_25			0b01010
#define BMX055_ACC_PMU_BW_62_5			0b01011
#define BMX055_ACC_PMU_BW_125			0b01100
#define BMX055_ACC_PMU_BW_250			0b01101
#define BMX055_ACC_PMU_BW_500			0b01110
#define BMX055_ACC_PMU_BW_1000			0b01111

// Accel Low Power Mode Parameter
#define BMX055_ACC_PMU_LPW_MODE_NOMAL	0b00000000
#define BMX055_ACC_PMU_LPW_MODE_DEEP_SUSPEND	0b00100000
#define BMX055_ACC_PMU_LPW_MODE_LOW_POWER		0b01000000
#define BMX055_ACC_PMU_LPW_MODE_SUSPEND			0b10000000

//Accel Low Power Mode sleep phase duration Parameter
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_0_5MS	0b00000
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_1MS	0b01100
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_2MS	0b01110
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_4MS	0b10000
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_6MS	0b10010
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_10MS	0b10100
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_25MS	0b10110
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_50MS	0b11000
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_100MS	0b11010
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_500MS	0b11100
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_1S		0b11110

// Gyro Measurement Range Parameter(Resolution:LSB/°/s)
#define BMX055_GYRO_RANGE_16_4			0b000
#define BMX055_GYRO_RANGE_32_8			0b001
#define BMX055_GYRO_RANGE_65_6			0b010
#define BMX055_GYRO_RANGE_131_2			0b011
#define BMX055_GYRO_RANGE_262_4			0b100

// Gyro MPU Band Width Parameter(Hz)
#define BMX055_GYRO_BW_32				0b0111
#define BMX055_GYRO_BW_64				0b0110
#define BMX055_GYRO_BW_12				0b0101
#define BMX055_GYRO_BW_23				0b0100
#define BMX055_GYRO_BW_47				0b0011
#define BMX055_GYRO_BW_116				0b0010
#define BMX055_GYRO_BW_230				0b0001
#define BMX055_GYRO_BW_523				0b0000

// Gyro Sleep Duration Parameter(ms)
#define BMX055_GYRO_LPM1_SLEEP_DUR_2MS		0b000
#define BMX055_GYRO_LPM1_SLEEP_DUR_4MS		0b001
#define BMX055_GYRO_LPM1_SLEEP_DUR_5MS		0b010
#define BMX055_GYRO_LPM1_SLEEP_DUR_8MS		0b011
#define BMX055_GYRO_LPM1_SLEEP_DUR_10MS		0b100
#define BMX055_GYRO_LPM1_SLEEP_DUR_15MS		0b101
#define BMX055_GYRO_LPM1_SLEEP_DUR_18MS		0b110
#define BMX055_GYRO_LPM1_SLEEP_DUR_20MS		0b111

// Mag Advance Self Test Control Parameter
#define BMX055_MAG_ADV_SELF_TEST_NORMAL		0b00000000
#define BMX055_MAG_ADV_SELF_TEST_NEGATIVE	0b10000000
#define BMX055_MAG_ADV_SELF_TEST_POSITIVE	0b11000000

// Mag Datarate Control Parameter
#define BMX055_MAG_DATA_RATE_10 		0b000000
#define BMX055_MAG_DATA_RATE_2			0b001000
#define BMX055_MAG_DATA_RATE_6			0b010000
#define BMX055_MAG_DATA_RATE_8			0b011000
#define BMX055_MAG_DATA_RATE_15			0b100000
#define BMX055_MAG_DATA_RATE_20			0b101000
#define BMX055_MAG_DATA_RATE_25			0b110000
#define BMX055_MAG_DATA_RATE_30			0b111000

// Mag Operation Mode Control Parameter
#define BMX055_MAG_OP_MODE_NORMAL		0b000
#define BMX055_MAG_OP_MODE_FORCED 		0b010
#define BMX055_MAG_OP_MODE_SLEEP		0b110

// Mag Self Test Control Parameter
#define BMX055_MAG_TEST_NORMAL			0b0
#define BMX055_MAG_TEST_SELF_TEST 		0b1

// Mag Power Control Parameter
#define BMX055_MAG_POW_CTL_SOFT_RESET		0b10000010
#define BMX055_MAG_POW_CTL_SLEEP_MODE		0b00000001
#define BMX055_MAG_POW_CTL_SUSPEND_MODE		0b00000000

void IMU_config(SPI_HandleTypeDef *hspi);
void IMU_acc_read(SPI_HandleTypeDef *hspi, int *accel_data);
void IMU_gyro_read(SPI_HandleTypeDef *hspi, int *gyro_data);
void IMU_mag_read(SPI_HandleTypeDef *hspi, int *mag_data);

#endif
