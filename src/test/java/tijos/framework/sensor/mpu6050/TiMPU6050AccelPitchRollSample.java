package tijos.framework.sensor.mpu6050;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;
import tijos.framework.sensor.mpu6050.MpuVector;
import tijos.framework.sensor.mpu6050.TiMPU6050;

public class TiMPU6050AccelPitchRollSample {

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

			mpu.begin(TiMPU6050.MPU6050_SCALE_2000DPS, TiMPU6050.MPU6050_RANGE_2G);

			int num = 10000;
			while (num-- > 0) {
				try {

					 // Read normalized values 
					  MpuVector normAccel = mpu.readNormalizeAccel();

					  // Calculate Pitch & Roll
					  double pitch = -(Math.atan2(normAccel.XAxis, Math.sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/Math.PI;
					  double roll = (Math.atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/Math.PI;

					  // Output
					  System.out.print(" Pitch = ");
					  System.out.print((int)pitch);
					  System.out.print(" Roll = ");
					  System.out.print((int)roll);
					  
					  System.out.println();
					  

				} catch (Exception ex) {

					ex.printStackTrace();
				}

			}
		} catch (IOException ie) {
			ie.printStackTrace();
		}

	}

}
