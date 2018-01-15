package tijos.framework.sensor.mpu6050;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;
import tijos.util.BigBitConverter;

/*TiJOS-MPU6050 Triple Axis Gyroscope & Accelerometer  
 * Based on original C library from Ardunio by Jarzębski  
 * https://github.com/jarzebski/Arduino-MPU6050
 */

class Vector {
	public double XAxis;
	public double YAxis;
	public double ZAxis;
}

class Activities {

	public int isOverflow;
	public int isFreeFall;
	public int isInactivity;
	public int isActivity;
	public int isPosActivityOnX;
	public int isPosActivityOnY;
	public int isPosActivityOnZ;
	public int isNegActivityOnX;
	public int isNegActivityOnY;
	public int isNegActivityOnZ;
	public int isDataReady;
}

class TiMPU6050Register {

	public static final int MPU6050_REG_ACCEL_XOFFS_H = 0x06;
	public static final int MPU6050_REG_ACCEL_XOFFS_L = 0x07;
	public static final int MPU6050_REG_ACCEL_YOFFS_H = 0x08;
	public static final int MPU6050_REG_ACCEL_YOFFS_L = 0x09;
	public static final int MPU6050_REG_ACCEL_ZOFFS_H = 0x0A;
	public static final int MPU6050_REG_ACCEL_ZOFFS_L = 0x0B;
	public static final int MPU6050_REG_GYRO_XOFFS_H = 0x13;
	public static final int MPU6050_REG_GYRO_XOFFS_L = 0x14;
	public static final int MPU6050_REG_GYRO_YOFFS_H = 0x15;
	public static final int MPU6050_REG_GYRO_YOFFS_L = 0x16;
	public static final int MPU6050_REG_GYRO_ZOFFS_H = 0x17;
	public static final int MPU6050_REG_GYRO_ZOFFS_L = 0x18;
	public static final int MPU6050_REG_CONFIG = 0x1A;

	// Gyroscope Configuration
	public static final int MPU6050_REG_GYRO_CONFIG = 0x1B;

	// Accelerometer Configuration
	public static final int MPU6050_REG_ACCEL_CONFIG = 0x1C;
	public static final int MPU6050_REG_FF_THRESHOLD = 0x1D;
	public static final int MPU6050_REG_FF_DURATION = 0x1E;
	public static final int MPU6050_REG_MOT_THRESHOLD = 0x1F;
	public static final int MPU6050_REG_MOT_DURATION = 0x20;
	public static final int MPU6050_REG_ZMOT_THRESHOLD = 0x21;
	public static final int MPU6050_REG_ZMOT_DURATION = 0x22;

	// INT Pin. Bypass Enable Configuration
	public static final int MPU6050_REG_INT_PIN_CFG = 0x37;

	// INT Enable
	public static final int MPU6050_REG_INT_ENABLE = 0x38;
	public static final int MPU6050_REG_INT_STATUS = 0x3A;
	public static final int MPU6050_REG_ACCEL_XOUT_H = 0x3B;
	public static final int MPU6050_REG_ACCEL_XOUT_L = 0x3C;
	public static final int MPU6050_REG_ACCEL_YOUT_H = 0x3D;
	public static final int MPU6050_REG_ACCEL_YOUT_L = 0x3E;
	public static final int MPU6050_REG_ACCEL_ZOUT_H = 0x3F;
	public static final int MPU6050_REG_ACCEL_ZOUT_L = 0x40;
	public static final int MPU6050_REG_TEMP_OUT_H = 0x41;
	public static final int MPU6050_REG_TEMP_OUT_L = 0x42;
	public static final int MPU6050_REG_GYRO_XOUT_H = 0x43;
	public static final int MPU6050_REG_GYRO_XOUT_L = 0x44;
	public static final int MPU6050_REG_GYRO_YOUT_H = 0x45;
	public static final int MPU6050_REG_GYRO_YOUT_L = 0x46;
	public static final int MPU6050_REG_GYRO_ZOUT_H = 0x47;
	public static final int MPU6050_REG_GYRO_ZOUT_L = 0x48;
	public static final int MPU6050_REG_MOT_DETECT_STATUS = 0x61;
	public static final int MPU6050_REG_MOT_DETECT_CTRL = 0x69;

	// User Control
	public static final int MPU6050_REG_USER_CTRL = 0x6A;

	// Power Management 1
	public static final int MPU6050_REG_PWR_MGMT_1 = 0x6B;

	// Who Am I
	public static final int MPU6050_REG_WHO_AM_I = 0x75;
}

/**
 * MPU-6050 Accelerometer + Gyro sensor driver
 * 
 * @author TiJOS
 *
 */
public class TiMPU6050 {

	public static final int MPU6050_ADDRESS = 0x68;

	/**
	 * Definitions
	 */
	public static final int MPU6050_RANGE_2G = 0;
	public static final int MPU6050_RANGE_4G = 1;
	public static final int MPU6050_RANGE_8G = 2;
	public static final int MPU6050_RANGE_16G = 3;

	public static final int MPU6050_SCALE_2000DPS = 3;
	public static final int MPU6050_SCALE_1000DPS = 2;
	public static final int MPU6050_SCALE_500DPS = 1;
	public static final int MPU6050_SCALE_250DPS = 0;

	// clock source
	public static final int MPU6050_CLOCK_KEEP_RESET = 0b111;
	public static final int MPU6050_CLOCK_EXTERNAL_19MHZ = 0b101;
	public static final int MPU6050_CLOCK_EXTERNAL_32KHZ = 0b100;
	public static final int MPU6050_CLOCK_PLL_ZGYRO = 0b011;
	public static final int MPU6050_CLOCK_PLL_YGYRO = 0b010;
	public static final int MPU6050_CLOCK_PLL_XGYRO = 0b001;
	public static final int MPU6050_CLOCK_INTERNAL_8MHZ = 0b000;

	public static final int MPU6050_DELAY_3MS = 0b11;
	public static final int MPU6050_DELAY_2MS = 0b10;
	public static final int MPU6050_DELAY_1MS = 0b01;
	public static final int MPU6050_NO_DELAY = 0b00;

	public static final int MPU6050_DHPF_HOLD = 0b111;
	public static final int MPU6050_DHPF_0_63HZ = 0b100;
	public static final int MPU6050_DHPF_1_25HZ = 0b011;
	public static final int MPU6050_DHPF_2_5HZ = 0b010;
	public static final int MPU6050_DHPF_5HZ = 0b001;
	public static final int MPU6050_DHPF_RESET = 0b000;

	/**
	 * TiI2CMaster object
	 */
	private TiI2CMaster i2cmObj;

	private int i2cSlaveAddr = MPU6050_ADDRESS;

	private byte[] data = new byte[8];

	float dpsPerDigit, rangePerDigit;
	int actualThreshold;
	boolean useCalibrate;
	int mpuAddress;

	Vector ra = new Vector();
	Vector rg = new Vector(); // Raw vectors
	Vector na = new Vector();
	Vector ng = new Vector(); // Normalized vectors
	Vector tg = new Vector();
	Vector dg = new Vector(); // Threshold and Delta for Gyro
	Vector th = new Vector(); // Threshold

	Activities a = new Activities(); // Activities

	/**
	 * Initialize object with i2c communication object, default slave address is
	 * 0x69
	 * 
	 * @param i2c
	 *            I2C master object for communication
	 */
	public TiMPU6050(TiI2CMaster i2c) {
		this(i2c, MPU6050_ADDRESS);
	}

	/**
	 * Initialize object with i2c communication object
	 * 
	 * @param i2c
	 *            I2C master object for communication
	 * @param address
	 *            I2C slave address
	 */
	public TiMPU6050(TiI2CMaster i2c, int address) {
		this.i2cmObj = i2c;
		this.i2cSlaveAddr = address;
	}

	public void begin(int scale, int range) throws IOException {
		this.i2cmObj.setBaudRate(400);

		// Reset calibrate values
		dg.XAxis = 0;
		dg.YAxis = 0;
		dg.ZAxis = 0;
		useCalibrate = false;

		// Reset threshold values
		tg.XAxis = 0;
		tg.YAxis = 0;
		tg.ZAxis = 0;
		actualThreshold = 0;

		// Check MPU6050 Who Am I Register
		if (whoAmI() != 0x68) {
			throw new IOException("Invalid address from WhoAmI register");
		}

		// Set Clock Source
		setClockSource(MPU6050_CLOCK_PLL_XGYRO);

		// Set Scale & Range
		setScale(scale);
		setRange(range);

		// Disable Sleep Mode
		setSleepEnabled(false);
	}

	public void setScale(int scale) throws IOException {
		int value;

		switch (scale) {
		case MPU6050_SCALE_250DPS:
			dpsPerDigit = .007633f;
			break;
		case MPU6050_SCALE_500DPS:
			dpsPerDigit = .015267f;
			break;
		case MPU6050_SCALE_1000DPS:
			dpsPerDigit = .030487f;
			break;
		case MPU6050_SCALE_2000DPS:
			dpsPerDigit = .060975f;
			break;
		default:
			break;
		}

		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_GYRO_CONFIG, data, 0, 1);
		value = data[0];
		value &= 0b11100111;
		value |= (scale << 3);

		data[0] = (byte) value;
		this.i2cmObj.write(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_GYRO_CONFIG, data, 0, 1);

	}

	public int getScale() throws IOException {

		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_GYRO_CONFIG, data, 0, 1);
		int value = data[0];
		value &= 0b00011000;
		value >>>= 3;

		return value;
	}

	public void setRange(int range) throws IOException {
		int value;

		switch (range) {
		case MPU6050_RANGE_2G:
			rangePerDigit = .000061f;
			break;
		case MPU6050_RANGE_4G:
			rangePerDigit = .000122f;
			break;
		case MPU6050_RANGE_8G:
			rangePerDigit = .000244f;
			break;
		case MPU6050_RANGE_16G:
			rangePerDigit = .0004882f;
			break;
		default:
			break;
		}

		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_ACCEL_CONFIG, data, 0, 1);
		value = data[0];
		value &= 0b11100111;
		value |= (range << 3);

		data[0] = (byte) value;
		this.i2cmObj.write(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_ACCEL_CONFIG, data, 0, 1);

	}

	public int getRange() throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_ACCEL_CONFIG, data, 0, 1);
		int value = data[0];
		value &= 0b00011000;
		value >>= 3;

		return value;
	}

	public void setDHPFMode(int dhpf) throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_ACCEL_CONFIG, data, 0, 1);
		int value = data[0];
		value &= 0b11111000;
		value |= dhpf;

		data[0] = (byte) value;
		this.i2cmObj.write(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_ACCEL_CONFIG, data, 0, 1);

	}

	public void setDLPFMode(int dlpf) throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_CONFIG, data, 0, 1);
		int value = data[0];
		value &= 0b11111000;
		value |= dlpf;

		data[0] = (byte) value;
		this.i2cmObj.write(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_CONFIG, data, 0, 1);
	}

	public void setClockSource(int source) throws IOException {

		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_PWR_MGMT_1, data, 0, 1);
		int value = data[0];
		value &= 0b11111000;
		value |= source;

		data[0] = (byte) value;
		this.i2cmObj.write(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_PWR_MGMT_1, data, 0, 1);

	}

	public int getClockSource() throws IOException {

		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_PWR_MGMT_1, data, 0, 1);
		int value = data[0];
		value &= 0b00000111;

		return value;

	}

	public boolean getSleepEnabled() throws IOException {
		return readRegisterBit(TiMPU6050Register.MPU6050_REG_PWR_MGMT_1, 6);
	}

	public void setSleepEnabled(boolean state) throws IOException {
		writeRegisterBit(TiMPU6050Register.MPU6050_REG_PWR_MGMT_1, 6, state);
	}

	public boolean getIntZeroMotionEnabled() throws IOException {
		return readRegisterBit(TiMPU6050Register.MPU6050_REG_INT_ENABLE, 5);
	}

	public void setIntZeroMotionEnabled(boolean state) throws IOException {
		writeRegisterBit(TiMPU6050Register.MPU6050_REG_INT_ENABLE, 5, state);
	}

	public boolean getIntMotionEnabled() throws IOException {
		return readRegisterBit(TiMPU6050Register.MPU6050_REG_INT_ENABLE, 6);
	}

	public void setIntMotionEnabled(boolean state) throws IOException {
		writeRegisterBit(TiMPU6050Register.MPU6050_REG_INT_ENABLE, 6, state);
	}

	public boolean getIntFreeFallEnabled() throws IOException {
		return readRegisterBit(TiMPU6050Register.MPU6050_REG_INT_ENABLE, 7);
	}

	public void setIntFreeFallEnabled(boolean state) throws IOException {
		writeRegisterBit(TiMPU6050Register.MPU6050_REG_INT_ENABLE, 7, state);
	}

	public int getMotionDetectionThreshold() throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_MOT_THRESHOLD, data, 0, 1);
		return data[0];
	}

	public void setMotionDetectionThreshold(int threshold) throws IOException {
		data[0] = (byte) threshold;
		this.i2cmObj.write(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_MOT_THRESHOLD, data, 0, 1);
	}

	public int getMotionDetectionDuration() throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_MOT_DURATION, data, 0, 1);
		return data[0];
	}

	public void setMotionDetectionDuration(int duration) throws IOException {
		data[0] = (byte) duration;
		this.i2cmObj.write(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_MOT_DURATION, data, 0, 1);

	}

	public int getZeroMotionDetectionThreshold() throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_ZMOT_THRESHOLD, data, 0, 1);
		return data[0];
	}

	public void setZeroMotionDetectionThreshold(int threshold) throws IOException {
		data[0] = (byte) threshold;
		this.i2cmObj.write(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_ZMOT_THRESHOLD, data, 0, 1);
	}

	public int getZeroMotionDetectionDuration() throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_ZMOT_DURATION, data, 0, 1);
		return data[0];
	}

	public void setZeroMotionDetectionDuration(int duration) throws IOException {
		data[0] = (byte) duration;
		this.i2cmObj.write(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_ZMOT_DURATION, data, 0, 1);
	}

	public int getFreeFallDetectionThreshold() throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_FF_THRESHOLD, data, 0, 1);
		return data[0];
	}

	public void setFreeFallDetectionThreshold(int threshold) throws IOException {
		data[0] = (byte) threshold;
		this.i2cmObj.write(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_FF_THRESHOLD, data, 0, 1);
	}

	public int getFreeFallDetectionDuration() throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_FF_DURATION, data, 0, 1);
		return data[0];
	}

	public void setFreeFallDetectionDuration(int duration) throws IOException {
		data[0] = (byte) duration;
		this.i2cmObj.write(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_FF_DURATION, data, 0, 1);
	}

	public boolean getI2CMasterModeEnabled() throws IOException {
		return readRegisterBit(TiMPU6050Register.MPU6050_REG_USER_CTRL, 5);
	}

	public void setI2CMasterModeEnabled(boolean state) throws IOException {
		writeRegisterBit(TiMPU6050Register.MPU6050_REG_USER_CTRL, 5, state);
	}

	public void setI2CBypassEnabled(boolean state) throws IOException {
		writeRegisterBit(TiMPU6050Register.MPU6050_REG_INT_PIN_CFG, 1, state);
	}

	public boolean getI2CBypassEnabled() throws IOException {
		return readRegisterBit(TiMPU6050Register.MPU6050_REG_INT_PIN_CFG, 1);
	}

	public void setAccelPowerOnDelay(int delay) throws IOException {

		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_MOT_DETECT_CTRL, data, 0, 1);
		int value = data[0];

		value &= 0b11001111;
		value |= (delay << 4);

		data[0] = (byte) value;
		this.i2cmObj.write(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_MOT_DETECT_CTRL, data, 0, 1);
	}

	public int getAccelPowerOnDelay() throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_MOT_DETECT_CTRL, data, 0, 1);
		int value = data[0];
		value &= 0b00110000;

		return (value >> 4);
	}

	public int getIntStatus() throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_INT_STATUS, data, 0, 1);
		return data[0];
	}

	public Activities readActivites() throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_INT_STATUS, data, 0, 1);
		int value = data[0];

		a.isOverflow = ((value >> 4) & 1);
		a.isFreeFall = ((value >> 7) & 1);
		a.isInactivity = ((value >> 5) & 1);
		a.isActivity = ((value >> 6) & 1);
		a.isDataReady = ((value >> 0) & 1);

		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_MOT_DETECT_STATUS, data, 0, 1);
		value = data[0];

		a.isNegActivityOnX = ((value >> 7) & 1);
		a.isPosActivityOnX = ((value >> 6) & 1);

		a.isNegActivityOnY = ((value >> 5) & 1);
		a.isPosActivityOnY = ((value >> 4) & 1);

		a.isNegActivityOnZ = ((value >> 3) & 1);
		a.isPosActivityOnZ = ((value >> 2) & 1);

		return a;
	}

	public Vector readRawAccel() throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_ACCEL_XOUT_H, data, 0, 6);

		ra.XAxis = BigBitConverter.ToUInt16(data, 0);
		ra.YAxis = BigBitConverter.ToUInt16(data, 2);
		ra.ZAxis = BigBitConverter.ToUInt16(data, 4);

		return ra;
	}

	public Vector readNormalizeAccel() throws IOException {
		readRawAccel();

		na.XAxis = ra.XAxis * rangePerDigit * 9.80665f;
		na.YAxis = ra.YAxis * rangePerDigit * 9.80665f;
		na.ZAxis = ra.ZAxis * rangePerDigit * 9.80665f;

		return na;
	}

	public Vector readScaledAccel() throws IOException {
		readRawAccel();

		na.XAxis = ra.XAxis * rangePerDigit;
		na.YAxis = ra.YAxis * rangePerDigit;
		na.ZAxis = ra.ZAxis * rangePerDigit;

		return na;
	}

	public Vector readRawGyro() throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_GYRO_XOUT_H, data, 0, 6);

		rg.XAxis = BigBitConverter.ToUInt16(data, 0);
		rg.YAxis = BigBitConverter.ToUInt16(data, 2);
		rg.ZAxis = BigBitConverter.ToUInt16(data, 4);

		return rg;
	}

	public Vector readNormalizeGyro() throws IOException {
		readRawGyro();

		if (useCalibrate) {
			ng.XAxis = (rg.XAxis - dg.XAxis) * dpsPerDigit;
			ng.YAxis = (rg.YAxis - dg.YAxis) * dpsPerDigit;
			ng.ZAxis = (rg.ZAxis - dg.ZAxis) * dpsPerDigit;
		} else {
			ng.XAxis = rg.XAxis * dpsPerDigit;
			ng.YAxis = rg.YAxis * dpsPerDigit;
			ng.ZAxis = rg.ZAxis * dpsPerDigit;
		}

		if (actualThreshold > 0) {
			if (Math.abs(ng.XAxis) < tg.XAxis)
				ng.XAxis = 0;
			if (Math.abs(ng.YAxis) < tg.YAxis)
				ng.YAxis = 0;
			if (Math.abs(ng.ZAxis) < tg.ZAxis)
				ng.ZAxis = 0;
		}

		return ng;
	}

	public double readTemperature() throws IOException {
		double T = readRegister16(TiMPU6050Register.MPU6050_REG_TEMP_OUT_H);

		return (T / 340.0) + 36.53;
	}

	public int getGyroOffsetX() throws IOException {
		return readRegister16(TiMPU6050Register.MPU6050_REG_GYRO_XOFFS_H);
	}

	public int getGyroOffsetY() throws IOException {
		return readRegister16(TiMPU6050Register.MPU6050_REG_GYRO_YOFFS_H);
	}

	public int getGyroOffsetZ() throws IOException {
		return readRegister16(TiMPU6050Register.MPU6050_REG_GYRO_ZOFFS_H);
	}

	public void setGyroOffsetX(int offset) throws IOException {
		writeRegister16(TiMPU6050Register.MPU6050_REG_GYRO_XOFFS_H, offset);
	}

	public void setGyroOffsetY(int offset) throws IOException {
		writeRegister16(TiMPU6050Register.MPU6050_REG_GYRO_YOFFS_H, offset);
	}

	public void setGyroOffsetZ(int offset) throws IOException {
		writeRegister16(TiMPU6050Register.MPU6050_REG_GYRO_ZOFFS_H, offset);
	}

	public int getAccelOffsetX() throws IOException {
		return readRegister16(TiMPU6050Register.MPU6050_REG_ACCEL_XOFFS_H);
	}

	public int getAccelOffsetY() throws IOException {
		return readRegister16(TiMPU6050Register.MPU6050_REG_ACCEL_YOFFS_H);
	}

	public int getAccelOffsetZ() throws IOException {
		return readRegister16(TiMPU6050Register.MPU6050_REG_ACCEL_ZOFFS_H);
	}

	public void setAccelOffsetX(int offset) throws IOException {
		writeRegister16(TiMPU6050Register.MPU6050_REG_ACCEL_XOFFS_H, offset);
	}

	public void setAccelOffsetY(int offset) throws IOException {
		writeRegister16(TiMPU6050Register.MPU6050_REG_ACCEL_YOFFS_H, offset);
	}

	public void setAccelOffsetZ(int offset) throws IOException {
		writeRegister16(TiMPU6050Register.MPU6050_REG_ACCEL_ZOFFS_H, offset);
	}

	// Calibrate algorithm
	public void calibrateGyro(int samples) throws IOException {
		// Set calibrate
		useCalibrate = true;

		// Reset values
		float sumX = 0;
		float sumY = 0;
		float sumZ = 0;
		float sigmaX = 0;
		float sigmaY = 0;
		float sigmaZ = 0;

		// Read n-samples
		for (int i = 0; i < samples; ++i) {
			readRawGyro();
			sumX += rg.XAxis;
			sumY += rg.YAxis;
			sumZ += rg.ZAxis;

			sigmaX += rg.XAxis * rg.XAxis;
			sigmaY += rg.YAxis * rg.YAxis;
			sigmaZ += rg.ZAxis * rg.ZAxis;
		}

		// Calculate delta vectors
		dg.XAxis = sumX / samples;
		dg.YAxis = sumY / samples;
		dg.ZAxis = sumZ / samples;

		// Calculate threshold vectors
		th.XAxis = Math.sqrt((sigmaX / 50) - (dg.XAxis * dg.XAxis));
		th.YAxis = Math.sqrt((sigmaY / 50) - (dg.YAxis * dg.YAxis));
		th.ZAxis = Math.sqrt((sigmaZ / 50) - (dg.ZAxis * dg.ZAxis));

		// If already set threshold, recalculate threshold vectors
		if (actualThreshold > 0) {
			setThreshold(actualThreshold);
		}
	}

	// Get current threshold value
	public int getThreshold() {
		return actualThreshold;
	}

	// Set threshold value
	public void setThreshold(int multiple) throws IOException {
		if (multiple > 0) {
			// If not calibrated, need calibrate
			if (!useCalibrate) {
				calibrateGyro(50);
			}

			// Calculate threshold vectors
			tg.XAxis = th.XAxis * multiple;
			tg.YAxis = th.YAxis * multiple;
			tg.ZAxis = th.ZAxis * multiple;
		} else {
			// No threshold
			tg.XAxis = 0;
			tg.YAxis = 0;
			tg.ZAxis = 0;
		}

		// Remember old threshold value
		actualThreshold = multiple;
	}

	/**
	 * Communication test: WHO_AM_I register reading
	 * 
	 * @return slave address
	 * @throws IOException
	 */
	public int whoAmI() throws IOException {

		i2cmObj.read(this.i2cSlaveAddr, TiMPU6050Register.MPU6050_REG_WHO_AM_I, data, 0, 1);
		return data[0];
	}

	// Read register bit
	private boolean readRegisterBit(int reg, int pos) throws IOException {

		this.i2cmObj.read(this.i2cSlaveAddr, reg, data, 0, 1);
		int value = data[0];

		value = ((value >> pos) & 1);
		if (value > 0)
			return true;

		return false;
	}

	// Write register bit
	private void writeRegisterBit(int reg, int pos, boolean state) throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, reg, data, 0, 1);
		int value = data[0];

		if (state) {
			value |= (1 << pos);
		} else {
			value &= ~(1 << pos);
		}

		data[0] = (byte) value;
		this.i2cmObj.write(this.i2cSlaveAddr, reg, data, 0, 1);
	}

	private int readRegister16(int reg) throws IOException {
		this.i2cmObj.read(this.i2cSlaveAddr, reg, data, 0, 2);

		return BigBitConverter.ToInt16(data, 0);
	}

	private void writeRegister16(int reg, int value) throws IOException {
		data[0] = (byte) (value >>> 8);
		data[1] = (byte) value;

		this.i2cmObj.write(this.i2cSlaveAddr, reg, data, 0, 2);
	}

}
