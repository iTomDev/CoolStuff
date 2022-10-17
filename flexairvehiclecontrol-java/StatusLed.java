/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.PinState;
import com.pi4j.io.gpio.RaspiPin;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * control the two status LEDs
 *
 * @author Thomas Pile, 201
 */
public class StatusLed implements Runnable
{
    private GpioController gpio;
    private GpioPinDigitalOutput Pin24;
    private GpioPinDigitalOutput Pin25;
    
    private volatile int LED1Counter = 0;
    private volatile int LED2Counter = 0;
    private Lock LEDLock = new ReentrantLock();
    
    //
    //public void init(GpioController gpio)
    public void init()
    {
        // set up pins
        gpio = GpioFactory.getInstance();
        Pin24 = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_24, "Pin24", PinState.LOW);
        Pin24.low();
        
        Pin25 = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_25, "Pin25", PinState.LOW);
        Pin25.low();
    }
    
    public boolean LED1_OFF()
    {
        if(LED1Counter<=0){return true;}
        return false;
    }
    
    public boolean LED2_OFF()
    {
        if(LED2Counter<=0){return true;}
        return false;
    }
    
    public void flashLED1()
    {
        //if(LED1Counter>0){LED1Counter=2;} // 2*250ms = 500ms
        LED1Counter=4;
    }
    
    public void flashLED2()
    {
        //if(LED2Counter>0){LED2Counter=2;}
        LED2Counter=4;
    }
    
    public void run()
    {
        while(true)
        {
            try
            {
                // if counter > 0, LED on
                if(LED1Counter>0){Pin24.high(); LED1Counter--;}
                else{Pin24.low();}

                // if counter > 0, LED on
                if(LED2Counter>0){Pin25.high(); LED2Counter--;}
                else{Pin25.low();}
                
                Thread.sleep(250);
                
            }
            catch(InterruptedException iex){iex.printStackTrace();}
        }
    }//run
}
