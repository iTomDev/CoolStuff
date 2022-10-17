/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;


import com.pi4j.io.i2c.I2CBus;
import com.pi4j.io.i2c.I2CDevice;
import com.pi4j.io.i2c.I2CFactory;
import com.pi4j.wiringpi.I2C;
import java.io.IOException;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * A thread safe I2C wrapper. 
 * The j4Pi implementation seems to have issues when two bits of code try to simultaneously 
 * use the I2C comms. I'm not 100% sure if threaded access is the cause but its possible.
 * 
 * Usage
 * 1. set up an instance of safeI2C 
 * 2. get device instance from safeI2C
 * 3. communicate with I2C device through safe I2C by sending the device instance and the send/receive requests
 *    returns true if data was sent, false is not
 *    multi receive requires a byte array to be supplied as an arg, pointer style
 *    syntax mirrors original lib, but with device added as first arg
 * 
 * @author Thomas Pile
 */
public class SafeI2C 
{
    private boolean I2C_InUse = false;
    private final Lock I2CLock = new ReentrantLock();
    
    private I2CBus i2c;
    private I2CDevice Dev68h; // MPU9250 gyro/accel
    private I2CDevice Dev0Ch; // MPU9250 mag
    //private I2CDevice Dev19h; // AVR I2C Servo
    
    public void init()
    {
        try
        {
            //final I2CBus i2c = I2CFactory.getInstance(I2CBus.BUS_1);
            i2c = I2CFactory.getInstance(I2CBus.BUS_1);
            
            Dev68h = i2c.getDevice(0x68);
            Dev0Ch = i2c.getDevice(0x0C);
            //Dev19h = i2c.getDevice(0x19);
        }
        catch(IOException iex){System.out.println("I2C Bus could not be registered");}
    }
    
    /*
    // I don't want it runnable, but I have little choice. Ned to have delays and all I2C in the same thread!
    public void run()
    {
        try
        {
            
            
            Thread.sleep(1);
        }
        catch(InterruptedException iex){iex.printStackTrace();}
    }
    */
    
    /*
    public I2CDevice getDevice(byte deviceAddy)
    {
        I2CDevice device;
        
        // if device isnt registered, register it
        try
        {
            device = i2c.getDevice(deviceAddy);
        }
        catch(IOException iex){System.out.println("I2C Device already registered"); device=null;}
        
        return device;
    }
    */
    
    /* 
    * returns true if data was sent
    * return -1 if fail
    */
    public void write(byte device, byte reg, byte data)
    {
        boolean ret = false;
        
        try
        {
            if(I2CLock.tryLock(1,TimeUnit.MILLISECONDS))
            //if(I2CLock.tryLock())
            {
                try
                {
                    switch(device)
                    {   
                        case 0x68: //104d
                        {
                            Dev68h.write(reg, data);
                            ret = true;
                            I2CLock.unlock();
                            break;
                        }
                        case 0x0C: //12d
                        {
                            Dev0Ch.write(reg, data);
                            ret = true;
                            I2CLock.unlock();
                            break;
                        }/*
                        case 0x19:
                        {
                            Dev19h.write(reg, data);
                            ret = true;
                            I2CLock.unlock();
                            break;
                        }*/
                    }//switch
                    
                }
                catch(IOException iex){ret = false; System.out.println("Could not send I2C single write to "+device);}
            }//lock
        }
        catch(InterruptedException iex){ret = false; System.out.println("Could not acquire I2C lock for single write");}
        //finally{I2CLock.unlock();}
        //return ret;
    }
    
    /*
    * Multi read
    * return -1 if failed
    * return 0 if success
    */
    public int read(byte device, byte reg, byte[] dataBuffer, int offset, int numToRead)
    {
        //
        int ret = 0;
        
        try
        {
            //f(I2CLock.tryLock(2,TimeUnit.MILLISECONDS))
            if(I2CLock.tryLock(1,TimeUnit.MILLISECONDS))
            {
                try
                {
                    switch(device)
                    {
                        case 0x68:
                        {
                            Dev68h.read(reg, dataBuffer, 0, numToRead);
                            ret = 0;
                            I2CLock.unlock();
                            break;
                        }
                        case 0x0C:
                        {
                            Dev0Ch.read(reg, dataBuffer, 0, numToRead);
                            ret = 0;
                            I2CLock.unlock();
                            break;
                        }/*
                        case 0x19:
                        {
                            Dev19h.read(reg, dataBuffer, 0, numToRead);
                            ret = 0;
                            I2CLock.unlock();
                            break;
                        }*/
                    }
                }
                catch(IOException iex){ret = -1; System.out.println("Could not send I2C multi read to "+device);}
            }//lock
        }
        catch(InterruptedException iex){ret = -1; System.out.println("Could not acquire I2C lock for multi read");}
        //finally{I2CLock.unlock();}
        
        return ret;
    }//read
    
    /*
    * Single read. Returns data received. 
    * return -1 if failed
    * return 0 if success
    */
    public byte read(byte device, int reg)
    {
        //
        byte ret = 0;
        
        try
        {
            //if(I2CLock.tryLock(2,TimeUnit.MILLISECONDS))
            if(I2CLock.tryLock(1,TimeUnit.MILLISECONDS))
            {
                try
                {
                    switch(device)
                    {   
                        case 0x68:
                        {
                            ret = (byte)Dev68h.read(reg);
                            I2CLock.unlock();
                            break;
                        }
                        case 0x0C:
                        {
                            ret = (byte)Dev0Ch.read(reg);
                            I2CLock.unlock();
                            break;
                        }/*
                        case 0x19:
                        {
                            ret = (byte)Dev19h.read(reg);
                            I2CLock.unlock();
                            break;
                        }*/
                    }
                }
                catch(IOException iex){ret = -1; System.out.println("Could not send I2C multi read to "+device);}
            }//lock
        }
        catch(InterruptedException iex){ret = -1; System.out.println("Could not acquire I2C lock for multi read");}
        //finally{I2CLock.unlock();}
        
        return ret;
    }//read
}
