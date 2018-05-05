package tijos.framework.sensor.mpu6050;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;
import tijos.framework.sensor.mpu6050.TiMPU6050;
import tijos.framework.util.Delay;

public class TiMPU6050AccelSample {

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

			checkSettings(mpu6050);

			int num = 10000;
			while (num-- > 0) {
				try {
					Vector rawAccel = mpu6050.readRawAccel();
					Vector normAccel = mpu6050.readNormalizeAccel();

					System.out.print(" Xraw = ");
					System.out.println(rawAccel.XAxis);
					System.out.print(" Yraw = ");
					System.out.println(rawAccel.YAxis);
					System.out.print(" Zraw = ");

					System.out.println(rawAccel.ZAxis);

					System.out.print(" Xnorm = ");
					System.out.println(normAccel.XAxis);
					System.out.print(" Ynorm = ");
					System.out.println(normAccel.YAxis);
					System.out.print(" Znorm = ");
					System.out.println(normAccel.ZAxis);

					Delay.msDelay(5);

				} catch (Exception ex) {

					ex.printStackTrace();
				}

			}
		} catch (IOException ie) {
			ie.printStackTrace();
		}

	}

	static public void checkSettings(TiMPU6050 mpu) throws IOException {
		System.out.println();

		System.out.print(" * Sleep Mode:            ");
		System.out.println(mpu.getSleepEnabled());

		System.out.print(" * Clock Source:          ");
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

		System.out.print(" * Accelerometer:         ");
		switch (mpu.getRange()) {
		case TiMPU6050.MPU6050_RANGE_16G:
			System.out.println("+/- 16 g");
			break;
		case TiMPU6050.MPU6050_RANGE_8G:
			System.out.println("+/- 8 g");
			break;
		case TiMPU6050.MPU6050_RANGE_4G:
			System.out.println("+/- 4 g");
			break;
		case TiMPU6050.MPU6050_RANGE_2G:
			System.out.println("+/- 2 g");
			break;
		}

		System.out.print(" * Accelerometer offsets: ");
		System.out.print(mpu.getAccelOffsetX());
		System.out.print(" / ");
		System.out.print(mpu.getAccelOffsetY());
		System.out.print(" / ");
		System.out.println(mpu.getAccelOffsetZ());

		System.out.println();
	}

}
