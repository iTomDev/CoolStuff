/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

/**
 * Controls servos and motor controllers via the AVR I2c system
 * This is compatible with SB03 version of the board
 * Code written 10/5/16
 * 
 * Tested on 26.5.16 to work with SB03 for 45,90 and 130 degrees
 * Send the desired angle in hex!
 * @author Thomas Pile, 2016
 */

// I2C
import com.pi4j.io.i2c.I2CDevice;
import java.io.IOException;


public class I2CAVRSERVO 
{
    private SafeI2C i2c;
    
    
    // I2C addresses and registers
    private static final byte AVRSB03ADDY = (byte) 0x19;
    private static final byte AVRSB03A_MOTOR1 = (byte) 0x01;
    private static final byte AVRSB03A_MOTOR2 = (byte) 0x02;
    private static final byte AVRSB03A_MOTOR3 = (byte) 0x03;
    private static final byte AVRSB03A_SERVO1 = (byte) 0x04;
    private static final byte AVRSB03A_SERVO2 = (byte) 0x05;
    private static final byte AVRSB03A_SERVO3 = (byte) 0x06;
    private static final byte AVRSB03A_SERVO4= (byte) 0x07;
    private static final byte SELECT_OUTPUT = (byte) 0x09;
    private static final byte AVRSB03A_OUTPUT_STATUS = (byte) 0x0A;
    
    // common reg values
    private static final byte OUTPUT_STATUS_DISABLED = (byte) 0xcc; // 0x00
    private static final byte OUTPUT_STATUS_ENABLED = (byte) 0x00; // 0x01
    private static final byte MOTOR_INIT = (byte) 0x80;
    private static final byte MOTOR_GENTLE_SPIN = (byte) 0x75;
    
    private static final byte SELECT_OUTPUT_RC = (byte) 0xaa; // 0xaa
    private static final byte SELECT_OUTPUT_AVR = (byte) 0x00; // 0x00
    
    private final byte device = AVRSB03ADDY;
    
    public void init(SafeI2C i2cin)
    {
        //
        i2c = i2cin;
        //try
        //{
            //device = i2c.getDevice(AVRSB03ADDY);
            
            /*
            // set up for pwm1,2,3 as motors, pm4,5,6,7 as servos
            i2c.write(device,(byte)AVRSB03A_MOTOR1, (byte)MOTOR_INIT); avrTwiDelay();
            i2c.write(device,(byte)AVRSB03A_MOTOR2, (byte)MOTOR_INIT); avrTwiDelay();
            i2c.write(device,(byte)AVRSB03A_MOTOR3, (byte)MOTOR_INIT); avrTwiDelay();
            */
            
            // start PWM running
            i2c.write(device,(byte)AVRSB03A_OUTPUT_STATUS, (byte)OUTPUT_STATUS_ENABLED); avrTwiDelay();
        //}
        //catch(IOException iox){iox.printStackTrace();}
    }// init
    
    public void setMotor(int pwmchannel, int speed)
    {
        //try
        //{
            i2c.write(device,(byte)pwmchannel, (byte)speed); avrTwiDelay();
        //}
        //catch(IOException iox){iox.printStackTrace();}
    }
    
    public void setServo(int pwmchannel, int angle)
    {
        //try
        //{
            i2c.write(device,(byte)pwmchannel, (byte)angle); avrTwiDelay();
        //}
        //catch(IOException iox){iox.printStackTrace();}
    }
    
    public void motorTestSequence()
    {
        setMotor(AVRSB03A_MOTOR1, 0x80);
        try
        {
            Thread.sleep(5000);
            setMotor(AVRSB03A_MOTOR1, 0x80); avrTwiDelay();
            Thread.sleep(1000);
            setMotor(AVRSB03A_MOTOR1, 0x74); avrTwiDelay();
            Thread.sleep(10000); 
            setMotor(AVRSB03A_MOTOR1, 0x80); avrTwiDelay();
            Thread.sleep(1000);
        }
        catch(InterruptedException iex){iex.printStackTrace();}
    }
    
    public void disableOutputs()
    {
        //try
        //{
            //i2c.write(device,(byte)SELECT_OUTPUT, (byte)SELECT_OUTPUT_RC);
            i2c.write(device,(byte)0x09, (byte)0x33); avrTwiDelay();
            //i2c.write(device,(byte)AVRSB03A_OUTPUT_STATUS, (byte)OUTPUT_STATUS_DISABLED);
            i2c.write(device,(byte)0x0A, (byte)0x00); avrTwiDelay();
        //}
        //catch(IOException iox){iox.printStackTrace();}
    }
    
    public void enableOutputs()
    {
        //try
        //{
            //i2c.write(device,(byte)0x09, (byte)SELECT_OUTPUT_AVR);
            i2c.write(device,(byte)0x09, (byte)0x00); avrTwiDelay();
            //i2c.write(device,(byte)AVRSB03A_OUTPUT_STATUS, (byte)OUTPUT_STATUS_ENABLED);
            i2c.write(device,(byte)0x0A, (byte)0x33); avrTwiDelay();
        //}
        //catch(IOException iox){iox.printStackTrace();}
    }
    
    // this is used to stop the AVR crashing the I2C interface
    // a huge pain to resolve. Delay period is only a guess.
    public void avrTwiDelay()
    {
        try
        {
            Thread.sleep(20);
        }
        catch(InterruptedException iex){iex.printStackTrace();}
    }
}//class

