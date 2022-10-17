/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

// SPI
import com.pi4j.io.spi.SpiChannel;
import com.pi4j.io.spi.SpiDevice;
import com.pi4j.io.spi.SpiFactory;

// GPIO
import com.pi4j.io.gpio.*;
import com.pi4j.io.gpio.event.GpioPinDigitalStateChangeEvent;
import com.pi4j.io.gpio.event.GpioPinListenerDigital;
import com.pi4j.io.spi.SpiMode;
import java.io.IOException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 *
 * @author NeonGreen
 */
public class SafeSPI 
{
    private final Lock SPILock = new ReentrantLock();
    private SpiDevice spi0 = null;
    private SpiDevice spi1 = null;
    private GpioController gpio;
    private GpioPinDigitalOutput Pin3;
    private GpioPinDigitalInput RFint;
    
    public void init()
    {
        try
        {
            //spi0 = SpiFactory.getInstance(SpiChannel.CS0, SpiDevice.DEFAULT_SPI_SPEED, SpiMode.MODE_0);
            //spi1 = SpiFactory.getInstance(SpiChannel.CS1, SpiDevice.DEFAULT_SPI_SPEED, SpiMode.MODE_0);
            
            // lower speed for V17 board
            spi0 = SpiFactory.getInstance(SpiChannel.CS0, 400000, SpiMode.MODE_0);
            spi1 = SpiFactory.getInstance(SpiChannel.CS1, 400000, SpiMode.MODE_0);
            
        }
        catch(IOException ex){ex.printStackTrace();}
    }
    
    public byte[] write(byte[] packet,int start, int length, int channel)
    {
        byte[] received = new byte[length+1]; // cant be more than this
        
        try
        {
            if(SPILock.tryLock(20,TimeUnit.MILLISECONDS))
            {
                switch(channel)
                {
                    case 0:
                        //
                        try
                        {
                            received = spi0.write(packet, start, length);
                            SPILock.unlock();
                        }
                        catch(IOException iox){iox.printStackTrace();SPILock.unlock();}
                        break;
                    case 1:
                        //
                        try
                        {
                            received = spi1.write(packet, start, length);
                            SPILock.unlock();
                        }
                        catch(IOException iox){iox.printStackTrace();SPILock.unlock();}
                        break;
                            
                }
            }//spilock
        }
        catch(InterruptedException iex){}
        
            return received;
    }//write
    
    public byte[] write(byte[] packet,int channel)
    {
        byte[] received = new byte[packet.length+1]; // cant be more than this
        
        try
        {
            if(SPILock.tryLock(20,TimeUnit.MILLISECONDS))
            {
                switch(channel)
                {
                    case 0:
                    {
                        //
                        try
                        {
                            received = spi0.write(packet);
                            SPILock.unlock();
                        }
                        catch(IOException iox){iox.printStackTrace();SPILock.unlock();}
                        break;
                    }
                    case 1:
                    {
                        //
                        try
                        {
                            received = spi1.write(packet);
                            SPILock.unlock();
                        }
                        catch(IOException iox){iox.printStackTrace();SPILock.unlock();}
                        break;
                    }
                    
                }
            }//spilock
        }
        catch(InterruptedException iex){}
        
            return received;
    }//write
    
}
