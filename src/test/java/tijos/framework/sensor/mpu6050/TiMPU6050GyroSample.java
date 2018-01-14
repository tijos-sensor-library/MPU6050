package tijos.framework.sensor.mpu6050;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;
import tijos.framework.sensor.mpu6050.TiMPU6050;
import tijos.framework.sensor.mpu6050.MpuVector;
import tijos.util.Delay;

public class TiMPU6050GyroSample {

	public static void main(String[] args) {
		try {

			/*
			 * 定义使用的TiI2CMaster port
			 */
			int i2cPort0 = 0;

			/*
			 * 资源分配， 将i2cPort0分配给TiI2CMaster对象i2c0
			 */
			TiI2CMaster i2c0 = TiI2CMaster.open(i2cPort0);

			TiMPU6050 mpu6050 = new TiMPU6050(i2c0);

			mpu6050.begin(TiMPU6050.MPU6050_SCALE_2000DPS, TiMPU6050.MPU6050_RANGE_2G);

			  // Calibrate gyroscope. The calibration must be at rest.
			  // If you don't want calibrate, comment this line.
			mpu6050.calibrateGyro(50);
			  
			  // Set threshold sensivity. Default 3.
			  // If you don't want use threshold, comment this line or set 0.
			mpu6050.setThreshold(3);
			  
			checkSettings(mpu6050);

			int num = 10000;
			while (num-- > 0) {
				try {
					  MpuVector rawGyro = mpu6050.readRawGyro();
					  MpuVector normGyro = mpu6050.readNormalizeGyro();

					  System.out.print(" Xraw = ");
					  System.out.println(rawGyro.XAxis);
					  
					  System.out.print(" Yraw = ");
					  System.out.println(rawGyro.YAxis);
					  
					  System.out.print(" Zraw = ");
					  System.out.println(rawGyro.ZAxis);

					  System.out.print(" Xnorm = ");
					  System.out.println(normGyro.XAxis);
					  
					  System.out.print(" Ynorm = ");
					  System.out.println(normGyro.YAxis);
					  
					  System.out.print(" Znorm = ");
					  System.out.println(normGyro.ZAxis);
					  
					Delay.msDelay(5);

				} catch (Exception ex) {

					ex.printStackTrace();
				}

			}
		} catch (IOException ie) {
			ie.printStackTrace();
		}

	}

	static void checkSettings(TiMPU6050 mpu) throws IOException {
		System.out.println();

		System.out.print(" * Sleep Mode:        ");
		System.out.println(mpu.getSleepEnabled());

		System.out.print(" * Clock Source:      ");
		switch (mpu.getClockSource()) {
		case TiMPU6050.MPU6050_CLOCK_KEEP_RESET:
			System.out.println("Stops the clock and keeps the timing generator in reset");
			break;
		case TiMPU6050.MPU6050_CLOCK_EXTERNAL_19MHZ:
			System.out.println("PLL with external 19.2MHz reference");
			break;
		case TiMPU6050.MPU6050_CLOCK_EXTERNAL_32KHZ:
			System.out.println("PLL with external 32.768kHz reference");
			break;
		case TiMPU6050.MPU6050_CLOCK_PLL_ZGYRO:
			System.out.println("PLL with Z axis gyroscope reference");
			break;
		case TiMPU6050.MPU6050_CLOCK_PLL_YGYRO:
			System.out.println("PLL with Y axis gyroscope reference");
			break;
		case TiMPU6050.MPU6050_CLOCK_PLL_XGYRO:
			System.out.println("PLL with X axis gyroscope reference");
			break;
		case TiMPU6050.MPU6050_CLOCK_INTERNAL_8MHZ:
			System.out.println("Internal 8MHz oscillator");
			break;
		}

		System.out.print(" * Gyroscope:         ");
		switch (mpu.getScale()) {
		case TiMPU6050.MPU6050_SCALE_2000DPS:
			System.out.println("2000 dps");
			break;
		case TiMPU6050.MPU6050_SCALE_1000DPS:
			System.out.println("1000 dps");
			break;
		case TiMPU6050.MPU6050_SCALE_500DPS:
			System.out.println("500 dps");
			break;
		case TiMPU6050.MPU6050_SCALE_250DPS:
			System.out.println("250 dps");
			break;
		}

		System.out.print(" * Gyroscope offsets: ");
		System.out.print(mpu.getGyroOffsetX());
		System.out.print(" / ");
		System.out.print(mpu.getGyroOffsetY());
		System.out.print(" / ");
		System.out.println(mpu.getGyroOffsetZ());

		System.out.println();
	}

}
