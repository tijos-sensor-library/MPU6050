package tijos.framework.sensor.mpu6050;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;
import tijos.framework.sensor.mpu6050.TiMPU6050;
import tijos.util.Delay;

public class TiMPU6050TemperatureSample {

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

			int num = 10000;
			while (num-- > 0) {
				try {

					  double temp = mpu6050.readTemperature();

					  System.out.printf("Temp = %.2f *C \n",temp);
					  
					Delay.msDelay(1000);

				} catch (Exception ex) {

					ex.printStackTrace();
				}

			}
		} catch (IOException ie) {
			ie.printStackTrace();
		}
	}
}
