package tijos.framework.sensor.mpu6050;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;
import tijos.framework.sensor.mpu6050.MpuActivities;
import tijos.framework.sensor.mpu6050.MpuVector;
import tijos.framework.sensor.mpu6050.TiMPU6050;

public class TiMPU6050MotionSample {
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

			TiMPU6050 mpu = new TiMPU6050(i2c0);

			mpu.begin(TiMPU6050.MPU6050_SCALE_2000DPS, TiMPU6050.MPU6050_RANGE_16G);

			mpu.setAccelPowerOnDelay(TiMPU6050.MPU6050_DELAY_3MS);

			mpu.setIntFreeFallEnabled(false);
			mpu.setIntZeroMotionEnabled(false);
			mpu.setIntMotionEnabled(false);

			mpu.setDHPFMode(TiMPU6050.MPU6050_DHPF_5HZ);

			mpu.setMotionDetectionThreshold(2);
			mpu.setMotionDetectionDuration(5);

			mpu.setZeroMotionDetectionThreshold(4);
			mpu.setZeroMotionDetectionDuration(2);

			checkSettings(mpu);

			while (true) {
				try {
					MpuVector rawAccel = mpu.readRawAccel();
					MpuActivities act = mpu.readActivites();



					  System.out.print(act.isActivity);
					  System.out.print(" ");
					  System.out.print(act.isInactivity);

					  System.out.print(" ");
					  System.out.print(act.isPosActivityOnX);
					  System.out.print(" ");
					  System.out.print(act.isNegActivityOnX);
					  System.out.print(" ");

					  System.out.print(act.isPosActivityOnY);
					  System.out.print(" ");
					  System.out.print(act.isNegActivityOnY);
					  System.out.print(" ");

					  System.out.print(act.isPosActivityOnZ);
					  System.out.print(" ");
					  System.out.print(act.isNegActivityOnZ);
					  System.out.print("\n");
					 
					  
				} catch (IOException ie) {
					ie.printStackTrace();
				}

			}

		} catch (IOException ie) {
			ie.printStackTrace();
		}

	}

	public static void checkSettings(TiMPU6050 mpu) throws IOException {
		System.out.println();

		System.out.print(" * Sleep Mode:                ");
		System.out.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

		System.out.print(" * Motion Interrupt:     ");
		System.out.println(mpu.getIntMotionEnabled() ? "Enabled" : "Disabled");

		System.out.print(" * Zero Motion Interrupt:     ");
		System.out.println(mpu.getIntZeroMotionEnabled() ? "Enabled" : "Disabled");

		System.out.print(" * Free Fall Interrupt:       ");
		System.out.println(mpu.getIntFreeFallEnabled() ? "Enabled" : "Disabled");

		System.out.print(" * Motion Threshold:          ");
		System.out.println(mpu.getMotionDetectionThreshold());

		System.out.print(" * Motion Duration:           ");
		System.out.println(mpu.getMotionDetectionDuration());

		System.out.print(" * Zero Motion Threshold:     ");
		System.out.println(mpu.getZeroMotionDetectionThreshold());

		System.out.print(" * Zero Motion Duration:      ");
		System.out.println(mpu.getZeroMotionDetectionDuration());

		System.out.print(" * Clock Source:              ");
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

		System.out.print(" * Accelerometer:             ");
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

		System.out.print(" * Accelerometer offsets:     ");
		System.out.print(mpu.getAccelOffsetX());
		System.out.print(" / ");
		System.out.print(mpu.getAccelOffsetY());
		System.out.print(" / ");
		System.out.println(mpu.getAccelOffsetZ());

		System.out.print(" * Accelerometer power delay: ");
		switch (mpu.getAccelPowerOnDelay()) {
		case TiMPU6050.MPU6050_DELAY_3MS:
			System.out.println("3ms");
			break;
		case TiMPU6050.MPU6050_DELAY_2MS:
			System.out.println("2ms");
			break;
		case TiMPU6050.MPU6050_DELAY_1MS:
			System.out.println("1ms");
			break;
		case TiMPU6050.MPU6050_NO_DELAY:
			System.out.println("0ms");
			break;
		}

		System.out.println();
	}
}
