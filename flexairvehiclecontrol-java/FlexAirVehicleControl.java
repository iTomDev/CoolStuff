/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

import CTOL.CTOL_SysIdentLoop;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

// I2C
import com.pi4j.io.i2c.I2CBus;
import com.pi4j.io.i2c.I2CDevice;
import com.pi4j.io.i2c.I2CFactory;
 
// SPI
import com.pi4j.io.spi.SpiChannel;
import com.pi4j.io.spi.SpiDevice;
import com.pi4j.io.spi.SpiFactory;

// internal
import flexairvehiclecontrol.I2CMPU6050.*;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.pi4j.io.serial.*;
import static com.pi4j.wiringpi.Gpio.delayMicroseconds;
import flexairvehiclecontrol.SimpleAttitudeController.FCSData;
import java.io.IOException;
import org.ujmp.core.Matrix;

import Quadrotor.*;

/**
 *
 * @author Thomas Pile, 2016
 * 
 * main stuff to do in order of priority
 * + I2C read and write to IMU
 * + read serial from GPS, parse
 * + read servo angles
 * + log data to file
 * + kalman filter
 * + rf link
 * + controller executer, SS system import, SS controller import
 * + finally: command packet processor, security
 * 
 * 
 * v0.1 14.3.16
 * + got serial rs232 to work
 * 
 * 39.3.2016
 * + EKF works stationary, logging works, mag should work
 * - GPS parsing still has issues
 * 
 * 
 * 
 * 
 * Usage:
 * cd Desktop
 * sudo java -classpath .:classes:/opt/pi4j/lib/'*' -jar FlexAirVehicleControl.jar
 */
public class FlexAirVehicleControl 
{
    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) throws IOException 
    {
        /*
        // MPU6050 use
        System.out.println("I2C MPU6050");
        
        RS232Test rs232 = new RS232Test();
        rs232.init();
        */
        
        /*
        // get instance of I2C bus using I2CFactory class
        final I2CBus i2c;
        
        MPU6050 imu;
        try
        {
            // get instance of I2C bus using I2CFactory class
            i2c = I2CFactory.getInstance(I2CBus.BUS_1);
            
            imu = new MPU6050(i2c);
            imu.init();
            // get a reading and the current time
            long curtime = System.currentTimeMillis();
            SixAxis imudata = imu.read();
            // wait a bit
            Thread.sleep(100);
        }
        catch (IOException | InterruptedException ioxpt)
        {
            // 
            ioxpt.printStackTrace();
        }
        finally
        {

        }
        */
        
        
        
        
        /*
        // Serial GPS use
        RS232GPS rs232gps = new RS232GPS();
        try 
        {
            rs232gps.init();
        }
        catch (InterruptedException | IOException ex) 
        {
            //Logger.getLogger(FlexAirVehicleControl.class.getName()).log(Level.SEVERE, null, ex);
            ex.printStackTrace();
        }
        rs232gps.close();
        
        System.out.println("Closing...");
        */
        
        
        
        /*
        // Working Serial Code
        
        final Serial serial = SerialFactory.createInstance();
        
        //serial.addListener(new SerialDataEventListener() {
        serial.addListener((SerialDataEventListener) (SerialDataEvent event) -> {
//            @Override
            //public void dataReceived(SerialDataEvent event) {

                
                
                
                // print out the data received to the console
                try 
                {
                    //System.out.println("[HEX DATA]   " + event.getHexByteString());
                    //System.out.println("[ASCII DATA] " + event.getAsciiString());
                    //byte dat[] = new byte[512];
                    //dat = serial.read(12);
                    
                    serial.flush();
                    
                    System.out.println("finished read");
                    
                    System.out.println("received");
                    try {
                        // NOTE! - It is extremely important to read the data received from the
                        // serial port.  If it does not get read from the receive buffer, the
                        // buffer will continue to grow and consume memory.
                        
                        serial.discardInput();
                        
                    } catch (IllegalStateException ex) {
                        Logger.getLogger(FlexAirVehicleControl.class.getName()).log(Level.SEVERE, null, ex);
                    } catch (IOException ex) {
                        Logger.getLogger(FlexAirVehicleControl.class.getName()).log(Level.SEVERE, null, ex);
                    }
                    
                } 
                catch (IOException e) 
                {
                    e.printStackTrace();
                }
//            }
        });
        //System.out.println("3");
        try 
        {
            //System.out.println("4");
            
            // by default, use the DEFAULT com port on the Raspberry Pi
            //String serialPort = Serial.DEFAULT_COM_PORT;
            
            // open the default serial port provided on the GPIO header
            //serial.open(serialPort, Baud._9600, DataBits._8, Parity.NONE, StopBits._1, FlowControl.NONE);
            //serial.open(Serial.DEFAULT_COM_PORT, (int)9600);
            //serial.open(Serial.DEFAULT_COM_PORT, 9600);
            
            serial.open(Serial.DEFAULT_COM_PORT, Baud._9600, DataBits._8, Parity.NONE, StopBits._1, FlowControl.SOFTWARE);
            serial.setBufferingDataReceived(false);
            
            //System.out.println("5");
            // continuous loop to keep the program running until the user terminates the program
            while(true) 
            {
                try {
                    System.out.println("send");
                    // write a individual bytes to the serial transmit buffer
                    //serial.write((byte) 13);
                    //System.out.println("6");
                }
                catch(IllegalStateException ex){
                    ex.printStackTrace();
                    
                }

                try {Thread.sleep(1000);} 
                catch (InterruptedException ex) 
                {
                    ex.printStackTrace();
                    Logger.getLogger(FlexAirVehicleControl.class.getName()).log(Level.SEVERE, null, ex);
                }
            }

        }
        catch(IOException ex) {
            System.out.println(" ==>> SERIAL SETUP FAILED : " + ex.getMessage());
            return;
        }
        
        */
        
        
        
        // do this
        // https://www.raspberrypi.org/forums/viewtopic.php?f=44&t=98318
        // Problem: Unknown bus error
        //  Solution: Check spec sheet and see which bus it is e.g I2C1 is BUS1
        // Problem: Can't read from address
        //  Solution: use i2cdetect -y 1 . The number it gives is the address to 
        //  use when trying to access the device e.g for 68 you do
        //  I2CDevice device = i2c.getDevice(0x68)
        
        // Basic attempt at I2C
        //final I2CBus i2c = I2CFactory.getInstance(I2CBus.BUS_1);
        //I2CDevice device = i2c.getDevice(0x68);
        //int response = device.read((byte)0x75); // same result
       // int response = device.read((byte)117);    // same result
       // System.out.println("result is " + String.valueOf(response));
        // This is the corres response. 104d = 68
        
        /*
        // test write (turn on)
        //device.write(0x1A, (byte)0x03);
        device.write((byte)38, (byte)0x03); // 0x26
        device.write((byte)28, (byte)0x08); // 0x1C
        device.write((byte)35, (byte)0xF8); // 0x23
        device.write((byte)55, (byte)0xC8); // 0x37
        device.write((byte)56, (byte)0x01); // 0x38
        device.write((byte)106, (byte)0x40); // 0x6A
        */
        /*
        device.write((byte)0x26, (byte)0x03); // 0x26
        device.write(0x28, (byte)0b00001000); // 0x1C = 28d    0x08
        device.write(0x23, (byte)0xF8); // 0x23
        device.write(0x37, (byte)0xC8); // 0x37
        device.write(0x38, (byte)0x01); // 0x38
        device.write(0x6A, (byte)0x40); // 0x6A
        device.write(0x6C, (byte)0x80);  //0x6C
        */
        byte params[] = new byte[2];
        //params[0] = (byte)0x1C; params[1] = (byte)0x08;
        //device.write(params);
        
        // it writes, but only writes zero
        // 107 before write
        // writes aren't taking place
        //response = device.read((byte)107);
        //System.out.println("0x6B before is " + String.valueOf(response));
        
        params[0] = (byte)0x6B; params[1] = (byte)0xF8; 
//        device.write(params);
        //device.write((byte)107,(byte)248);
        //response = device.read((byte)107);
        //System.out.println("0x6B after is " + String.valueOf(response));
        
        //response = device.read((byte)107);
        //System.out.println("0x6B after is " + String.valueOf(response));
        
        //response = device.read((byte)107);
        //System.out.println("result is " + String.valueOf(response));
        //int response = device.read((byte)0x3B);
        
        /*
        for(int i=0; i<100; i++)
        {
        // test multiread
        // it reads fixed registers, but gyro, accel etc are all zero
        byte resp[] = new byte[128];
        resp[0] = 1; resp[1] = 1; resp[2] = 1;
        //response = device.read((byte)0x3B, resp, 0, 14);
        response = device.read((byte)59, resp, 0, 14);
        System.out.println("Number of bytes read: " + String.valueOf(response));
        System.out.println("Data: "+String.valueOf(resp[0]) + " " +
                                    String.valueOf(resp[1]) + " " +
                                    String.valueOf(resp[2]) + " " +
                                    String.valueOf(resp[3]) + " " +
                                    String.valueOf(resp[4]) + " " +
                                    String.valueOf(resp[5]) + " " +
                                    String.valueOf(resp[6]) + " " +
                                    String.valueOf(resp[7]) + " " +
                                    String.valueOf(resp[8]) + " " +
                                    String.valueOf(resp[9]) + " " +
                                    String.valueOf(resp[10]) + " " +
                                    String.valueOf(resp[11]) + " " +
                                    String.valueOf(resp[12]) 
                                    );
        // works, reads 14 bytes
        }
        */
        
        /*
        // Real time clock I2C test - read and writes verified
        // detect devices on 50 and 68
        final I2CBus i2c = I2CFactory.getInstance(I2CBus.BUS_1);
        I2CDevice device = i2c.getDevice(0x68);
        int response;
        response = device.read((byte)0x00); // same result
        //int response = device.read((byte)117);    // same result
        System.out.println("result is " + String.valueOf(response));
        // test set to 0x13 and verify
        byte params[] = new byte[2];
        params[0] = (byte)0x07; params[1] = (byte)0x13;
        device.write(params);
        //
        response = device.read((byte)0x07);
        System.out.println("0x07 is " + String.valueOf(response));
        // test set to 0x12 and verify
        params[0] = (byte)0x07; params[1] = (byte)0x12;
        device.write(params);
        //
        response = device.read((byte)0x07);
        System.out.println("0x07 is " + String.valueOf(response));
        */
        
        /*
        // test mag
        final I2CBus i2c = I2CFactory.getInstance(I2CBus.BUS_1);
        //I2CDevice device = i2c.getDevice(0x68);
        I2CHMC5883 mag = new I2CHMC5883();
        mag.init(i2c);
        mag.read();
        */
        
        /*
        
        
        
        // RS232 test
        RS232GPS gpsserial = new RS232GPS();
        gpsserial.init(20);
        //gpsserial.read();
        
        
        // EKF
        EKF ekf = new EKF(imu,gpsserial);
        ekf.setFilterParams(20, 20, false);
        //ekf.runFilter(imu.read(), gpsserial.read());
        ekf.enableLogging();
        //ekf.getState();
        
        // Control
        
        // start threads
        new Thread(imu).start();
        //new Thread(gpsserial).start();
        new Thread(ekf).start();
        
        */
        
        // full system
        
        StatusLed leds = new StatusLed();
        leds.init();
        //leds.flashLED1();
        //leds.flashLED2();
        
        // RS232 GPS
        Servo servo = new Servo();
        RS232 serial = new RS232();
        servo.init(serial);
        new Thread(serial).start();
        
        
        // Safe I2C
        //final I2CBus i2c = I2CFactory.getInstance(I2CBus.BUS_1);
        SafeI2C i2c = new SafeI2C();
        i2c.init();
        
        /*
        //I2CDevice device = i2c.getDevice(0x68);
        I2CMPU9250 imu = new I2CMPU9250();
        imu.init(i2c, 20);
        new Thread(imu).start();
        //imu.zeroDevice();
        //imu.read();
        */
        
        /*
        // EKF
        EKF ekf = new EKF();
        ekf.init(imu, serial.getGPSData(), leds, servo);
        ekf.setFilterParams(20, 80, false);
        new Thread(ekf).start();
        */
        
        // controller
        //SimpleAttitudeController fcs = new SimpleAttitudeController();
        //fcs.init();
        //new Thread(fcs).start();
        
        SafeSPI spi = new SafeSPI();
        spi.init();
        
        // nRF24L01
        Communications comms = new Communications();
        comms.init(spi);
        //new Thread(comms).start();
        //comms.setFlightController(fcs);
        
        SPIServo spiServo = new SPIServo();
        spiServo.init(spi);
        //new Thread(spiServo).start();
        
        
        /*
        // sys ident
        SystemIdentification sysident = new SystemIdentification();
        sysident.init(servo, comms, ekf);
        new Thread(sysident).start();
        */
        
        /*
        Trirotor_Controller control = new Trirotor_Controller();
        control.init(ekf, servo);
        new Thread(control).start();
        */
        
        /*
        Trirotor_Controller_AttOnly control2 = new Trirotor_Controller_AttOnly();
        control2.init(ekf, servo);
        new Thread(control2).start();
        */
        
        /*
        QuadBasic control3 = new QuadBasic();
        control3.init(ekf, spiServo);
        new Thread(control3).start();
        */
        
        /*
        //=========================
        // Quadrotor mode
        
        // set up control loop, it is a thread!
        // set up non-runnable control
        QuadBasic_NoRun control = new QuadBasic_NoRun();
        control.init(spiServo);
        new Thread(control).start();
        
        // set up the ekf, imu, control single block loop
        IMUEKFControlLoop imuekf = new IMUEKFControlLoop();
        imuekf.init(i2c, serial, spiServo, control);
        new Thread(imuekf).start();
        */
        
        
        //=========================
        // CTOL Mode
        CTOL_SysIdentLoop control = new CTOL_SysIdentLoop();
        control.init(spiServo);
        new Thread(control).start();
        
        // set up the ekf, imu, control single block loop
        IMUEKFControlLoop imuekf = new IMUEKFControlLoop();
        imuekf.init(i2c, serial, spiServo, control);
        new Thread(imuekf).start();
        
        
        // todo: you need to extend some kind of controllablke interface and use for quad and ctol
        
        //timertest timer = new timertest();
        //IMU_Timer imutimer = new IMU_Timer(imuekf);
        
    }//main 
}