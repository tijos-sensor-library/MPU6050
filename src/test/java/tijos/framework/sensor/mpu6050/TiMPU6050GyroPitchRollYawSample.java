package tijos.framework.sensor.mpu6050;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;
import tijos.framework.sensor.mpu6050.TiMPU6050;
import tijos.util.Delay;

public class TiMPU6050GyroPitchRollYawSample {

	public static void main(String[] args) {
		try {

			// Timers
			long timer = 0;

			double timeStep = 0.01;

			// Pitch, Roll and Yaw values
			double pitch = 0;
			double roll = 0;
			double yaw = 0;

			/*
			 * 定义使用的TiI2CMaster port
			 */
			int i2cPort0 = 0;

			/*
			 * 资源分配， 将i2cPort0分配给TiI2CMaster对象i2c0
			 */
			TiI2CMaster i2c0 = TiI2CMaster.open(i2cPort0);

			TiMPU6050 mpu = new TiMPU6050(i2c0);

			mpu.begin(TiMPU6050.MPU6050_SCALE_2000DPS, TiMPU6050.MPU6050_RANGE_2G);

			// Calibrate gyroscope. The calibration must be at rest.
			// If you don't want calibrate, comment this line.
			mpu.calibrateGyro(50);

			// Set threshold sensivty. Default 3.
			// If you don't want use threshold, comment this line or set 0.
			mpu.setThreshold(3);

			int num = 10000;
			while (num-- > 0) {
				try {

					timer = System.currentTimeMillis();

					// Read normalized values
					Vector norm = mpu.readNormalizeGyro();

					// Calculate Pitch, Roll and Yaw
					pitch = pitch + norm.YAxis * timeStep;
					roll = roll + norm.XAxis * timeStep;
					yaw = yaw + norm.ZAxis * timeStep;

					// Output raw
					System.out.print(" Pitch = ");
					System.out.print((int) pitch);
					System.out.print(" Roll = ");
					System.out.print((int) roll);
					System.out.print(" Yaw = ");
					System.out.println((int) yaw);

					// Wait to full timeStep period
					Delay.msDelay((int) ((timeStep * 1000) - (System.currentTimeMillis() - timer)));

				} catch (Exception ex) {

					ex.printStackTrace();
				}

			}
		} catch (IOException ie) {
			ie.printStackTrace();
		}

	}

}
