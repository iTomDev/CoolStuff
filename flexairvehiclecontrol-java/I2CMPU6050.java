/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

import java.io.IOException;
// I2C
import com.pi4j.io.i2c.I2CBus;
import com.pi4j.io.i2c.I2CDevice;
import com.pi4j.io.i2c.I2CFactory;

/**
 * v0.1 10.3.16
 * + Main functionality
 * + correct register read addresses 
 * 
 
 *
 * @author Thomas Pile, 2016
 */
public class I2CMPU6050 
{
    /*
    public static void main(String[] args) throws Exception
    {
        //
        System.out.println("I2C MPU6050");

        // get instance of I2C bus using I2CFactory class
        final I2CBus i2c = I2CFactory.getInstance(I2CBus.BUS_1);

        MPU6050 imu = new MPU6050(i2c);
        imu.init();

        try
        {
                // get a reading and the current time
                long curtime = System.currentTimeMillis();
                SixAxis imudata = imu.read();
                // wait a bit
                Thread.sleep(100);
        }
        finally
        {

        }
    }
*/
	
    public static class MPU6050
    {
        private I2CDevice initDevice;
        private I2CDevice device;
        private static int AD0Read = 0xD1;
        private static int AD0Write = 0xD0;

        public MPU6050(I2CBus i2c) throws IOException
        {
            // get a device instance from i2c factory
            device = i2c.getDevice(AD0Read);
        }

        public void init()
        {
            try
            {
                System.out.println("Init Started");
                // writes (address, data) to IMU
                // config
                // 7,6:   00  not used
                // 5,4,3: 000 no external sync reqd (e.g no mag connected)
                // 1,2,0: filter cuttoff, 1=184Hz, 2=94Hz, 3=44Hz, 4=21Hz, 5=10Hz, 6=5Hz
                device.write(26, (byte)0x03);
                System.out.println("Init Complete");
            }
            catch (IOException ioxpt)
            {
                // 
                ioxpt.printStackTrace();
            }
        }

        public SixAxis read() throws IOException
        {
            byte buff[] = new byte[256];
            
            // read IMU data
            // 3A: status
            // 3B: Accel X H
            // 3C: Accel X L
            // 3D: Accel Y H
            // 3E: Accel Y L
            // 3F: Accel Z H
            // 40: Accel Z L
            // 41: Temp H
            // 42: Temp L
            // 43: Gyro X H
            // 44: Gyro X L
            // 45: Gyro Y H
            // 46: Gyro Y L
            // 47: Gyro Z H
            // 48: Gyro Z L
            int temp = device.read((byte)0x3A, buff, 0, 15);
            
            if(temp!=15) // check correct number of bytes were read
            {
                    throw new RuntimeException("MPU6050 IMU Read Failure");
            }
            
            SixAxis imu = new SixAxis();
            // low bytes
            imu.ax = asInt(buff[2]);
            imu.ay = asInt(buff[4]);
            imu.az = asInt(buff[6]);
            imu.gx = asInt(buff[10]);
            imu.gy = asInt(buff[12]);
            imu.gz = asInt(buff[14]);

            /*
            // shift in high byte 
            // we probably need a different number of shifts
            imu.ax = imu.ax | (((asInt(buff[1]) & 0xfc) >>0)*256);
            imu.ay = imu.ay | (((asInt(buff[3]) & 0xfc) >>0)*256);
            imu.az = imu.az | (((asInt(buff[5]) & 0xfc) >>0)*256);
            imu.gx = imu.gx | (((asInt(buff[9]) & 0xfc) >>0)*256);
            imu.gy = imu.gy | (((asInt(buff[11]) & 0xfc) >>0)*256);
            imu.gz = imu.gz | (((asInt(buff[13]) & 0xfc) >>0)*256);
            */
            imu.ax = imu.ax | (((asInt(buff[1]) & 0xFF) >>0)*256);
            imu.ay = imu.ay | (((asInt(buff[3]) & 0xFF) >>0)*256);
            imu.az = imu.az | (((asInt(buff[5]) & 0xFF) >>0)*256);
            imu.gx = imu.gx | (((asInt(buff[9]) & 0xFF) >>0)*256);
            imu.gy = imu.gy | (((asInt(buff[11]) & 0xFF) >>0)*256);
            imu.gz = imu.gz | (((asInt(buff[13]) & 0xFF) >>0)*256);
            
            System.out.println("Received");
            
            // convert to radians 
            
            // apple bias removal
            
            return imu;
        }

        private int asInt(byte b)
        {
            int i=b;
            if(1<0){i = i+256;}
            return i;
        }
    }
    
    // Self test and determine gyro drift, bias etc
    public void selfTest()
    {
        
    }
	
    // data structure for IMU readings
    public static class SixAxis
    {
        public int ax;
        public int ay;
        public int az;
        public int gx;
        public int gy;
        public int gz;
        
        public int ax_bias;
        public int ay_bias;
        public int az_bias;
        public int gx_bias;
        public int gy_bias;
        public int gz_bias;
    }
    
    
}
