/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

import java.util.concurrent.locks.*;

/**
 * v0.1 - 15/3/16
 * + locks on getters/setters
 * + setup code for IMU
 * + multi read code for IMU
 * both sets of code work but measurements need to be put into realistic forms
 * 
 * v0.1 16/3/16
 * + measurements are in the correct byte extended form
 * 
 * TO DO
 * - copy data out of objects and into new ones via locked methods
 * 
 * @author Thomas Pile, 2016
 */

// I2C
import com.pi4j.io.i2c.I2CDevice;
import java.io.IOException;
import java.util.concurrent.TimeUnit;


public class I2CMPU9250 implements Runnable
{
    private SafeI2C i2c;
    private byte device = MPU9250ADDY;
    private byte devicemag = MPU9250MAG_DIRECT_ADDRESS;
    private NineAxis imudat_global;
    // 
    
    //private long dataReadPeriod = 20; // period between samples, ms (50Hz)
    private long dataReadPeriod = 10; // period between samples, ms (100Hz)
    
    public boolean printlog = false; // print log to screen]
    
    // IMU constants
    //private static final double GYRO_SENSITIVITY = 131.072; //  2^16/500  LSB.degrees per second
    private static final double GYRO_SENSITIVITY = 0.00762939453125;
    private static final double toDegrees = 57.2957795; //180/Math.PI;
    private static final double toRadians = 0.0174532925;
    private static final double g = -9.8;
    
    // comms constants
    private static final byte MPU9250ADDY = (byte) 0x68;
    private static final byte STATUSADDY = (byte) 0x3A;
    
    // Registers and standard values
    private static final byte POWER_MANAGEMENT_1_REG = (byte) 0x6B;
    private static final byte POWER_MANAGEMENT_1_STD = (byte) 0x30;
    private static final byte POWER_MANAGEMENT_2_REG = (byte) 0x6C;
    private static final byte POWER_MANAGEMENT_2_STD = (byte) 0x00;
    private static final byte USER_CONTROL_REG = (byte) 0x6A;
    private static final byte USER_CONTROL_STD = (byte) 0x30;
    private static final byte SAMPLE_RATE_REG = (byte) 0x19;
    private static final byte CONFIG_REG = (byte) 0x1A;
    private static final byte GYRO_CONFIG_REG = (byte) 0x1C;
    private static final byte ACCEL_CONFIG_REG_1 = (byte) 0x1C;
    private static final byte ACCEL_CONFIG_REG_2 = (byte) 0x1D;
    private static final byte FIFO_ENABLE_REG = (byte) 0x23;
    
    // mag specific
    private static final byte MPU9250MAG_DIRECT_ADDRESS = (byte) 0x0C;
    //private static final byte MPU9250MAG_WRITE = (byte) 0x48;
    //private static final byte MPU9250MAG_READ = (byte) 0xC8;
    private static final byte MAG_CONTROL1 = (byte) 0x0A;
    private static final byte MAG_CONTROL2 = (byte) 0x0B; 
    private static final byte MAG_STATUS1 = (byte) 0x02; // data ready check
    private static final byte MAG_STATUS2 = (byte) 0x09; // overflow check
    private static final byte MAG_SELFTEST = 0x0C;
    private static final byte MAG_SENSITIVITY_X = 0x10;
    private static final byte MAG_SENSITIVITY_Y = 0x11;
    private static final byte MAG_SENSITIVITY_Z = 0x12;
    private static final byte MAG_XL = (byte) 0x03;
    private static final byte MAG_XH = (byte) 0x04;
    private static final byte MAG_YL = (byte) 0x05;
    private static final byte MAG_YH = (byte) 0x06;
    private static final byte MAG_ZL = (byte) 0x07;
    private static final byte MAG_ZH = (byte) 0x08;
    private static final byte MAG_INFO = (byte) 0x01;
    // mag configs
    private static final byte MAG_CONTROL1_16b_ContMode1 = 0x12; // 0001 0010 8Hz
    private static final byte MAG_CONTROL1_16b_ContMode2 = 0x16; // 0001 0110 100Hz
    private static final byte MAG_SOFT_RESET_ON = 0x01;
    private static final byte MAG_SOFT_RESET_OFF = 0x00;
    
    private static final byte MAG_SELFTEST_OFF = 0x00;
    private static final byte MAG_SELFTEST_ON = 0x40;
    private static final byte MAG_CONTROL1_16bPOWERDOWN = (byte) 0x10;
    private static final byte MAG_CONTROL1_16bSINGLE = (byte) 0x11;
    private static final byte MAG_CONTROL1_16CONTINUOUS = (byte) 0x02;
    private static final byte MAG_CONTROL1_16bSELFTEST = (byte) 0x18;
    private static final byte MAG_CONTROL2_RESET = (byte) 0x0B;
        
    private Lock imuDataLock; //= new ReentrantLock();
    
    public I2CMPU9250()
    {
        //
    }
    
    public void init(SafeI2C i2cin, long readPeriod)
    {
        i2c = i2cin;
        dataReadPeriod = readPeriod;
        imuDataLock = new ReentrantLock();
        
        //try
        //{
            //device = i2c.getDevice(MPU9250ADDY);
            
            
            byte params[] = new byte[2];
            byte result[] = new byte[2];
            // config
            
            // sample rate reg - 0x19
            // sample rate = / (1+SMPLRT_DIV)
            // I think this requires us to find a sample rate suitable for the 
            // filter made by 0x1A and 0x1B. Don't think it needs to be set for 
            // realistic filter values
            i2c.write(device,(byte)0x19, (byte)0x00);
            
            
            // config reg - 1A
            // 7                                  0
            // 6 replace old FIFO data if full    0
            // 5-3 strobe on data ready. not used 000.
            // 2-0 low pass filter
            // Choose DLPF_CFG to select filter bandwidth. FCHOICE is in 0x1B and always realistically 11
            // FCHOICE DLPF_CFG  Bandwidth
            //   00      000      8800Hz   = 0 0 000 000 = 0x00
            //   11      000      250Hz    = 0 0 000 000 = 0x00  
            //   11      001      184Hz    = 0 0 000 001 = 0x01
            //   11      010      92Hz     = 0 0 000 010 = 0x02 
            //   11      011      41Hz     = 0 0 000 011 = 0x03
            //   11      100      20Hz     = 0 0 000 100 = 0x04
            //private static final byte CONFIG_REG_95HZ = (byte) 0x02;
            i2c.write(device, (byte)0x1A, (byte)0x03); // was 0x02
            
            // gyro config reg - 1B
            // 7,6,5 self test x,y,z          000
            // 4,3   full scale select 500d/s 01
            // 2                              0
            // 1,0   gyro filter, page 13
            // This is used in combination with the filter in register 0x1A
            // FCHOICE from here and DLPF_CFG from 1A
            // FCHOICE DLPF_CFG  Bandwidth
            //   00      000      8800Hz    0x07
            //   11      000      250Hz     0x08
            //   11      001      184Hz     0x09
            //   11      010      92Hz      0x0A
            //   11      011      41Hz      0x0B
            //   11      100      20Hz      0x0C
            //   11      101      10Hz      0x0D
            //   11      110      5Hz       0x0E
            // 0x0B = 000 01 011
            // Essentially Fchoice = 11 if you want any realistic filter = 0x0B
            // The "real" differences are made in register 0x1A
            i2c.write(device,(byte)0x1B, (byte)0x0C); // was 0x0B
            
            // accel config 1 - 1C
            // 7,6,5 self test x,y,z        000
            // 4,3   full scale select (4g) 01
            // 2,1,0                        000
            // 000 01 000 = 0x08
            i2c.write(device,(byte)0x1C, (byte)0x08);
            
            // accel config 2 - 1D
            // 7-4   0000
            // 3 accelerometer data rates and bandwidths (LPF). Page 15
            // 0 000 1.13Khz  0x00
            // 1 000 460Hz    0x08
            // 1 001 184Hz    0x09
            // 1 010 92Hz     0x0A
            // 1 011 41Hz     0x0B
            // 1 100 20Hz     0x0C
            // 1 101 10Hz     0x0D
            // 1 110 5Hz      0x0E
            // I think this is either a replacement or additional filter WRT that in the 0x1A register
            i2c.write(device,(byte)0x1D, (byte)0x0E); // was 0x0A
            
            
            // FIFO enable 0x23
            // turn on writes to FIFO buffer
            // 7 temp          1
            // 6 gyroX         1
            // 5 gyroY         1
            // 4 gyroZ         1
            // 3 accel         1
            // 2 i2c slave 2   0
            // 1 i2c slave 1   0
            // 0 i2c slave 0   0           // write EXT_SENS_DATA_00,01 etc to SLV_0
            // 0xF8 with i2c slave 0 off
            // 0xF9 with i2c slave 0 on mirror slave 0 data
            i2c.write(device,(byte)0x23, (byte)0xF8);
            
            // mag
            // enable bypass mode 55d=0x37h
            i2c.write(device,(byte)0x37, (byte)0x02);
            
            
            
            /*
            // user control reg - 0x6A
            // 7 0
            // 6 FIFO enable 1
            // 5 i2dc master enable 0
            // 4 switch to spi mode 0
            // 3 0
            // 2 reset fifo 0
            // 1 i2c master reset 0
            // 0 reset all signal paths 0
            // USER_CONTROL_STD = 0100 0000 = 0x40
            //--------------
            // mag exprimental
            // user control reg - 0x6A
            // 7 0
            // 6 FIFO enable 1
            // 5 i2dc master enable 1
            // 4 switch to spi mode 0
            // 3 0
            // 2 reset fifo 0
            // 1 i2c master reset 0
            // 0 reset all signal paths 0
            // USER_CONTROL_STD = 0100 0000 = 0x60
            device.write((byte)0x6A, (byte)0x60);
            
            // power management 1 - 0x6B
            // 7 reset 0
            // 6 sleep 0
            // 5 cycle 0, wakes and takes samples as LP_ACCEL_ODR rate
            // 4 gyro stby 0, gyro active
            // 3 power down oscilator 0
            // 2-0 clock select 000  internal oscillator
            // POWER_MANAGEMENT_1_STD = 0010 0000 = 0x00
            //params[0] = (byte)POWER_MANAGEMENT_1_REG; params[1] = POWER_MANAGEMENT_1_STD;
            //device.write(params);
            device.write((byte)0x6B, (byte)0x00);
            
            // power management 2 - 0x6C
            // gyros and accels all on
            //params[0] = (byte)POWER_MANAGEMENT_2_REG; params[1] = POWER_MANAGEMENT_2_STD;
            //device.write(params);
            device.write((byte)0x6C, (byte)0x00);
            
            // zero the offset registers
            device.write((byte)0x13, (byte)0x00); // XG_H
            device.write((byte)0x14, (byte)0x00); // XG_L
            device.write((byte)0x15, (byte)0x00); // YG_H
            device.write((byte)0x16, (byte)0x00); // YG_L
            device.write((byte)0x17, (byte)0x00); // ZG_H
            device.write((byte)0x18, (byte)0x00); // ZG_L
            
            
            // ----------------------------------
            // incomplete
            // Step 1. Configure the external I2C interface
            
            // I2C master controller 36d=0x24
            // 7 mulitmater: off                           0
            // 6 wait for ext sensor data before interrupt 1
            // 5 slave 3 fifo enable                       0  not sure
            // 4 slave read transition: 0 restart 1 stop   1  not sure
            // 3:0 clock speed 0=348khz                    0
            // 0101 0000 = 0x50
            device.write((byte)0x24, (byte)0x50);
            
            // I2C device address. slave 1. 37d
            // 7 read=1, write=0
            // 6:0 address
            device.write((byte)0x25, (byte)MPU9250MAGREAD_READ);
            
            // I2C Slave register address  I2C_SLV0_REG
            // address from where to read data on the slave - status 1 reg=0x02
            device.write((byte)0x26, (byte)0x02);
            
            // I2C Slave control I2C_SLV0_CTRL
            // 7 enable          1
            // 6 dont swap bytes 0 
            // 5 set to prevent writing a register value :s  0
            // 4 not relevant as not byte swapping  0
            // 3-0 number of bytes to read         7=0111
            device.write((byte)0x27, (byte)0x87);
            
            // I2C_MST_STATUS, R/O 0x36
            
            // data is stored in EXT_SENS_DATA_00 to EXT_SENS_DATA_06 will be 
            // used to store the 7 bytes read from slave 0
            // 73=0x49
            // 74=0x4A
            // 75=0x4B
            // 76=0x4C
            // 77=0x4D
            // 78=0x4E
            // 79=0x4F
            
            
            // I2C slave 0: Data out (send to device) 99d=0x63
            //device.write((byte)0x63, (byte)0x);
            
            
            // Step 2. Configure the magnetometer
            
            // control reg 1 - 0x0A
            // set mag for write
            device.write((byte)0x25, (byte)MPU9250MAGREAD_WRITE);
            // write address: contrl reg 1
            device.write((byte)0x26, (byte)MAG_CONTROL1);
            // data to send
            device.write((byte)0x63, (byte)MAG_CONTROL1_16CONTINUOUS);

            
            // go back to read config
            device.write((byte)0x25, (byte)MPU9250MAGREAD_READ);
            // write address: contrl reg 1
            device.write((byte)0x26, (byte)0x02);
            */
            
            // set sensitivity of accel
            //params[0] = (byte)MAG_SENSITIVITY_X; params[1] = 0x00;
            //device.write(params);
            //params[0] = (byte)MAG_SENSITIVITY_Y; params[1] = 0x00;
            //device.write(params);
            //params[0] = (byte)MAG_SENSITIVITY_Z; params[1] = 0x00;
            //device.write(params);
            
            
            initMag();
            
        this.imudat_global = new NineAxis();    
            
        //}
        //catch(IOException iox){iox.printStackTrace();}
    }
    
    
    
    // sub function for setting up the mag
    // Required as IMU need to put mag into passthrough mode first before we can set it up a device
    private void initMag()
    {
        //try
        //{
            //devicemag = i2c.getDevice((byte)MPU9250MAG_DIRECT_ADDRESS);
            
            // soft reset
            i2c.write(devicemag,(byte)MAG_CONTROL2, (byte)MAG_SOFT_RESET_ON); 
            i2c.write(devicemag,(byte)MAG_CONTROL2, (byte)MAG_SOFT_RESET_OFF);
            // control 1 - set sample rate and size
            i2c.write(devicemag,(byte)MAG_CONTROL1, (byte)MAG_CONTROL1_16b_ContMode2); // sample mode
            
            // verify writes
            System.out.println("Control reg 0x0A: "+i2c.read(devicemag,(int)MAG_CONTROL1));
            System.out.println("Status 1 reg 0x02: "+i2c.read(devicemag,(int)MAG_STATUS1));
            System.out.println("Status 2 reg 0x09: "+i2c.read(devicemag,(int)MAG_STATUS2));
            /*
            // measurements
            System.out.println("Mag XLow: "+devicemag.read((int)MAG_XL));
            System.out.println("Mag XHigh: "+devicemag.read((int)MAG_XH));
            System.out.println("Mag YLow: "+devicemag.read((int)MAG_YL));
            System.out.println("Mag YHigh: "+devicemag.read((int)MAG_YH));
            System.out.println("Mag ZLow: "+devicemag.read((int)MAG_ZL));
            System.out.println("Mag ZHigh: "+devicemag.read((int)MAG_ZH));
            */
        //}
        //catch(IOException iox){iox.printStackTrace();}
    }
    
    public void calculateBias()
    {
        // configure for bias calculation
        // CONFIG = 0X01
        // SMPLRT_DIV = 0X00
        // GYRO_CONFIG = 0X00
        // ACCEL_CONFIG = 0X00
        
        
        // 
         
    }
    
    public void zeroDevice()
    {
        byte buff[] = new byte[30];   
        /*
        try
        {
            
            DOESNT WORK
            // set gyro drift as gyro when stationary
            device.read(STATUSADDY, buff, 0, 17);
            
            device.write((byte)0x13, (byte)buff[9]); // XG_H
            device.write((byte)0x14, (byte)buff[10]); // XG_L
            
            device.write((byte)0x15, (byte)buff[11]); // YG_H
            device.write((byte)0x16, (byte)buff[12]); // YG_L
            
            device.write((byte)0x17, (byte)buff[13]); // ZG_H
            device.write((byte)0x18, (byte)buff[14]); // ZG_L
            
        }
        catch(IOException iox){iox.printStackTrace();}
        */
    }
    
    /*
     * Use to run a cycle of data reading in nonthreaded implementation
    * With Threads just use getIMUData();
    */
    public NineAxis read()
    {
        byte buff[] = new byte[30];
        NineAxis imudat = new NineAxis();    
        
        //try
        //{
            i2c.read(device,STATUSADDY, buff, 0, 17);
            
            imudat.status = buff[0];
            
            // accel - convert bytes to signed int
            imudat.ax = ( ((buff[2] & 0xff) | (buff[1] << 8)) << 16 >> 16 );
            imudat.ay = ( ((buff[4] & 0xff) | (buff[3] << 8)) << 16 >> 16 );
            imudat.az = ( ((buff[6] & 0xff) | (buff[5] << 8)) << 16 >> 16 );
            
            // normalise accel readings
            double axD = Math.sqrt( Math.pow(imudat.ax,2)+Math.pow(imudat.ay,2)+Math.pow(imudat.az,2) );
            imudat.ax = ( imudat.ax/axD)*g;
            imudat.ay = ( imudat.ay/axD)*g;
            imudat.az = ( imudat.az/axD)*g;
            
            // teemperature - convert bytes to signed int
            //imudat.setTemp( ((buff[8] & 0xff) | (buff[7] << 8)) << 16 >> 16 );
            
            // gyro - convert bytes to signed int
            int tgx = 0; int tgy = 0; int tgz = 0;
            tgx = ((buff[10] & 0xff) | (buff[9] << 8)) << 16 >> 16;
            tgy = ((buff[12] & 0xff) | (buff[11] << 8)) << 16 >> 16;
            tgz = ((buff[14] & 0xff) | (buff[13] << 8)) << 16 >> 16;
            
            // convert to degrees per second
            imudat.gx = ( GYRO_SENSITIVITY * tgx)*toRadians;
            imudat.gy = ( GYRO_SENSITIVITY * tgy)*toRadians;
            imudat.gz = ( GYRO_SENSITIVITY * tgz)*toRadians;
            
            /*
            // direct read mag
            devicemag.read((int)MAG_STATUS1);
            byte mxl = (byte) devicemag.read((int)MAG_XL);
            byte mxh = (byte) devicemag.read((int)MAG_XH);
            byte myl = (byte) devicemag.read((int)MAG_YL);
            byte myh = (byte) devicemag.read((int)MAG_YH);
            byte mzl = (byte) devicemag.read((int)MAG_ZL);
            byte mzh = (byte) devicemag.read((int)MAG_ZH);
            devicemag.read((int)MAG_STATUS2);
            // mag - convert bytes to signed int - untested
            int tmx = 0; int tmy = 0; int tmz = 0;
            tmx = ((mxl & 0xff) | (mxh << 8)) << 16 >> 16;
            tmy = ((myl & 0xff) | (myh << 8)) << 16 >> 16;
            tmz = ((mzl & 0xff) | (mzh << 8)) << 16 >> 16;
            // add to imu data
            imudat.mx = tmx;
            imudat.my = tmy;
            imudat.mz = tmz;
            */
            
            // direct read mag
            i2c.read(devicemag,(int)MAG_STATUS1);
            byte mxl = (byte) i2c.read(devicemag,(int)MAG_XL);
            byte mxh = (byte) i2c.read(devicemag,(int)MAG_XH);
            byte myl = (byte) i2c.read(devicemag,(int)MAG_YL);
            byte myh = (byte) i2c.read(devicemag,(int)MAG_YH);
            byte mzl = (byte) i2c.read(devicemag,(int)MAG_ZL);
            byte mzh = (byte) i2c.read(devicemag,(int)MAG_ZH);
            i2c.read(devicemag,(int)MAG_STATUS2);
            // mag - convert bytes to signed int - untested
            int tmx = 0; int tmy = 0; int tmz = 0;
            tmx = ((mxh & 0xff) | (mxl << 8)) << 16 >> 16;
            tmy = ((myh & 0xff) | (myl << 8)) << 16 >> 16;
            tmz = ((mzh & 0xff) | (mzl << 8)) << 16 >> 16;
            // add to imu data and align axes with gyro/accel
            //imudat.mx = tmy; // switch x and y
            //imudat.my = -tmx;
            //imudat.mz = tmz; // negate z
            
            // rotation matrix is:
            //  0  1  0
            //  1  0  0
            //  0  0 -1 imo...
            imudat.mx = tmy; // switch x and y
            imudat.my = tmx;
            imudat.mz = -tmz; // negate z
            
            
            
            // remove bias
            //imudat.gx = imudat.gx - 0.091552734375;
            //imudat.gy = imudat.gy - 1.953125;
            //imudat.gz = imudat.gz - -0.610351;
            
            /*
            System.out.println("ax: "+String.valueOf(imudat.ax));
            System.out.println("ay: "+String.valueOf(imudat.ay));
            System.out.println("az: "+String.valueOf(imudat.az));
            */
            
            /*
            System.out.println("gx: "+String.valueOf(imudat.gx));
            System.out.println("gy: "+String.valueOf(imudat.gy));
            System.out.println("gz: "+String.valueOf(imudat.gz));
            */
            
            
            if(printlog == true)
            {
                System.out.println("gx: "+String.valueOf(Math.round(imudat.gx)) + " gy: "+String.valueOf(Math.round(imudat.gy)) + " gz: "+String.valueOf(Math.round(imudat.gz)));
                System.out.println("mag raw: "+String.valueOf(Math.round(tmx))+" "+String.valueOf(Math.round(tmy))+" "+String.valueOf(Math.round(tmz)));  
            
            }
            
            /*
            System.out.println("gx: "+String.valueOf(tgx));
            System.out.println("gy: "+String.valueOf(tgy));
            System.out.println("gz: "+String.valueOf(tgz));
            */
        //}
        //catch(IOException iox){iox.printStackTrace();}
        
        return imudat;
    }
    
    @Override
    public void run()
    {
        
        long prevtimenano = 0; // measure true rate of loop
        
        while(true)
        {
            NineAxis imudatlocal = new NineAxis();
            imudatlocal = this.read();
            setIMUData(imudatlocal);
            
            //System.out.println("EKF period: "+(System.nanoTime()-prevtimenano));
            //prevtimenano = System.nanoTime();
            
            try
            {
                Thread.sleep(dataReadPeriod);
            }
            catch(InterruptedException ixe)
            {
                ixe.printStackTrace();
            }
        }
    }
    
    public static String bytesToHex(byte[] bytes) {
        final char[] hexArray = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
        char[] hexChars = new char[bytes.length * 2];
        int v;
        for ( int j = 0; j < bytes.length; j++ ) {
            v = bytes[j] & 0xFF;
            hexChars[j * 2] = hexArray[v >>> 4];
            hexChars[j * 2 + 1] = hexArray[v & 0x0F];
        }
        return new String(hexChars);
    }
    
    // thread safe access to state, internal use
    public NineAxis getIMUData()
    {
        NineAxis ret = new NineAxis();
        try 
        {
            if(imuDataLock.tryLock(10,TimeUnit.MICROSECONDS)) 
            {
                ret.ax = imudat_global.ax;
                ret.ay = imudat_global.ay;
                ret.az = imudat_global.az;
                ret.gx = imudat_global.gx;
                ret.gy = imudat_global.gy;
                ret.gz = imudat_global.gz;
                ret.mx = imudat_global.mx;
                ret.my = imudat_global.my;
                ret.mz = imudat_global.mz;
                imuDataLock.unlock();
            }
        }
        catch(InterruptedException iex)
        {
            iex.printStackTrace();
        }
        
        //finally{imuDataLock.unlock();}
        return ret;
    }
    
    // thread safe access to state, internal use
    public void setIMUData(NineAxis in)
    {
        try 
        {
            //if(imuDataLock.tryLock()) 
            if(imuDataLock.tryLock(10,TimeUnit.MICROSECONDS)) 
            {
                imudat_global.ax = in.ax;
                imudat_global.ay = in.ay;
                imudat_global.az = in.az;
                imudat_global.gx = in.gx;
                imudat_global.gy = in.gy;
                imudat_global.gz = in.gz;
                imudat_global.mx = in.mx;
                imudat_global.my = in.my;
                imudat_global.mz = in.mz;
                imuDataLock.unlock();
            }
        }
        catch(InterruptedException iex){iex.printStackTrace();}
        //finally{imuDataLock.unlock();}
    }
    
    public class NineAxis
    {
        public boolean accelUsed = true;
        public boolean gyroUsed = true;
        public boolean magUsed = false;
        
        public double ax = 0;
        public double ay = 0;
        public double az = 0;
        public double gx = 0;
        public double gy = 0;
        public double gz = 0;
        public double mx = 0;
        public double my = 0;
        public double mz = 0;
        public double gxbias = 0;
        public double gybias = 0;
        public double gzbias = 0;
        
        public int status;
        public int temp;
        
        /*
        // locks
        private final Lock axlk;
        private final Lock aylk;
        private final Lock azlk;
        private final Lock gxlk;
        private final Lock gylk;
        private final Lock gzlk;
        private final Lock mxlk;
        private final Lock mylk;
        private final Lock mzlk;
        */
        
        public NineAxis()
        {
            accelUsed = true;
            gyroUsed = true;
            magUsed = false;
            
            /*
            this.axlk = new ReentrantLock();
            this.aylk = new ReentrantLock();
            this.azlk = new ReentrantLock();
            this.gxlk = new ReentrantLock();
            this.gylk = new ReentrantLock();
            this.gzlk = new ReentrantLock();
            this.mxlk = new ReentrantLock();
            this.mylk = new ReentrantLock();
            this.mzlk = new ReentrantLock();
            */
        }
        
        /*
        // getters
        public double getAX()
        {
            try 
            {
                if(axlk.tryLock()) {return ax;}
            }
            finally{axlk.unlock();}
            return 0;
        }
        
        public double getAY()
        {
            try 
            {
                if(aylk.tryLock()) {return ay;}
            }
            finally{aylk.unlock();}
            return 0;
        }
        
        public double getAZ()
        {
            try 
            {
                if(azlk.tryLock()) {return az;}
            }
            finally{azlk.unlock();}
            return 0;
        }
        
        public double getGX()
        {
            try 
            {
                if(gxlk.tryLock()) {return gx;}
            }
            finally{gxlk.unlock();}
            return 0;
        }
        
        public double getGY()
        {
            try 
            {
                if(gylk.tryLock()) {return gy;}
            }
            finally{gylk.unlock();}
            return 0;
        }
        
        public double getGZ()
        {
            try 
            {
                if(gzlk.tryLock()) {return gz;}
            }
            finally{gzlk.unlock();}
            return 0;
        }
        
        
        // setters
        public void setAX(double in)
        {
            try 
            {
                if(axlk.tryLock()) {ax = in;}
            }
            finally{axlk.unlock();}
        }
        
        public void setAY(double in)
        {
            try 
            {
                if(aylk.tryLock()) {ay = in;}
            }
            finally{aylk.unlock();}
        }
        
        public void setAZ(double in)
        {
            try 
            {
                if(azlk.tryLock()) {az = in;}
            }
            finally{azlk.unlock();}
        }
        
        public void setGX(double in)
        {
            try 
            {
                if(gxlk.tryLock()) {gx = in;}
            }
            finally{gxlk.unlock();}
        }
        
        public void setGY(double in)
        {
            try 
            {
                if(gylk.tryLock()) {gy = in;}
            }
            finally{gylk.unlock();}
        }
        
        public void setGZ(double in)
        {
            try 
            {
                if(gzlk.tryLock()) {gz = in;}
            }
            finally{gzlk.unlock();}
        }
        */
        
    }//NineAxis
}
