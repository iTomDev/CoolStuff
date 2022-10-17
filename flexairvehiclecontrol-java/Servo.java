/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

import com.pi4j.component.servo.*;
import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPinDigitalInput;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.GpioPinPwmOutput;
import com.pi4j.io.gpio.PinMode;
import com.pi4j.io.gpio.*;
import com.pi4j.io.gpio.event.GpioPinDigitalStateChangeEvent;
import com.pi4j.io.gpio.event.GpioPinListenerDigital;
import com.pi4j.io.gpio.PinPullResistance;
import com.pi4j.io.gpio.PinState;
import com.pi4j.io.gpio.RaspiPin;
import com.pi4j.io.gpio.event.GpioPinDigitalStateChangeEvent;
import com.pi4j.io.gpio.event.GpioPinListenerDigital;
import static flexairvehiclecontrol.SPINRF24L.PACKET_SIZE;
import static flexairvehiclecontrol.SPINRF24L.PACKET_SIZE_DECIMAL;
import static flexairvehiclecontrol.SPINRF24L.STATUS_R;

import java.util.ArrayList;
import java.util.List;

import java.util.Timer;
import java.util.concurrent.locks.ReentrantLock;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Provides abstraction from the physical layer to the software Layer
 * Can handle different aircraft types and can switch more easily between I2C or serial
 * 
 * @author Thomas Pile, 2016
 */
public class Servo //implements Runnable
{
    // other class functions
    private List<CommunicationsListener> listeners = new ArrayList<CommunicationsListener>();
    public RS232 serial;
    
    // hardware interface
    private GpioPinDigitalInput AVR_Int;
    private GpioController gpio;
    private GpioPinDigitalOutput Pin3;
    
    // servo info
    public int thrust = 0;
    public int elevator = 0;
    public int aileron = 0;
    public int aux1 = 0;
    public int aux2 = 0;
    public int aux3 = 0;
    public volatile boolean PI_Has_Controls = false; // PI control or RC control 
    
    public boolean running = true;
    private boolean setupcompleted = false;
    
    
    public Servo()
    {
        //servoLock = new ReentrantLock();
    }
    
    public void init(RS232 serialIn)
    {
        serial = serialIn;
        
        // set up listener for interrupt
        GpioController gpio = GpioFactory.getInstance();
        AVR_Int = gpio.provisionDigitalInputPin(RaspiPin.GPIO_06, PinPullResistance.PULL_DOWN);

        AVR_Int.setShutdownOptions(true);
        AVR_Int.addListener(new GpioPinListenerDigital()
        {
            @Override
            public void handleGpioPinDigitalStateChangeEvent(GpioPinDigitalStateChangeEvent evt)
            {
                // this fires upon the interrupt going low, verified!

                PinState ps;
                ps = evt.getState();
                
                if(ps.isHigh())
                {
                    //System.out.println("PI Has Control");
                    // set PI_Has_Controls as false
                    PI_Has_Controls = true;
                }
                else
                {
                    //System.out.println("RC Has Control");
                    // set PI_Has_Controls as false
                    PI_Has_Controls = false;
                }
            }
        });
    }
    
    /*
        @param Servo: 1 to 8
        @Param Angle: 0 to 255 [1 to 180 degs]
    */
    public void setServo(int servo, int angle)
    {
        if(angle==0){angle=1;}
        if(angle>170){angle=170;} // some kind of error breaks output above this
        
        // copy the desired servo setting
        switch(servo)
        {
            case 1:
                thrust=angle;
                break;
            case 2:
                elevator=angle;
                break;
            case 3:
                aileron=angle;
                break;
            case 4:
                aux1=angle;
                break;
            case 5:
                aux2=angle;
                break;
            case 6:
                aux3=angle;
                break;
        }
        
        // convert
        // 255/180 = 1.4166666666666666666666666666667;
        angle*=1.4166666666666666666666666666667;
        
        serial.send((byte) 255);
        serial.send((byte) servo);
        serial.send((byte) angle);
    }
    
    /*
        @param Servo: Thrust, Elevator, Aileron
        @Param Angle: 0 to 255 [1 to 180 degs]
    */
    public void setServoCTOLConfig(String servoin, int angle)
    {
        int servo = 0;
        if(angle==0){angle=1;}
        if(angle>170){angle=170;} // some kind of error breaks output above this
        // convert
        // 255/180 = 1.4166666666666666666666666666667;
        angle*=1.4166666666666666666666666666667;

        if(servoin.equalsIgnoreCase("Thrust")){servo=1;}
        if(servoin.equalsIgnoreCase("Elevator")){servo=2;}
        if(servoin.equalsIgnoreCase("Aileron")){servo=3;}
        
        // copy the desired servo setting
        switch(servo)
        {
            case 1:
                thrust=angle;
                break;
            case 2:
                elevator=angle;
                break;
            case 3:
                aileron=angle;
                break;
            case 4:
                aux1=angle;
                break;
            case 5:
                aux2=angle;
                break;
            case 6:
                aux3=angle;
                break;
        }
        
        serial.send((byte) 255);
        serial.send((byte) servo);
        serial.send((byte) angle);
    }
    
    /*
        @param Servo: Thrust, Elevator, Aileron
        @Param Angle: 0 to 255 [1 to 180 degs]
    */
    public void setServoSTOVLConfig(String servoin, int angle)
    {
        int servo = 0;
        if(angle==0){angle=1;}
        if(angle>170){angle=170;} // some kind of error breaks output above this
        // convert
        // 255/180 = 1.4166666666666666666666666666667;
        angle*=1.4166666666666666666666666666667;
        
        System.out.println(servoin + " " + angle);
        
        if(servoin.equalsIgnoreCase("ThrustLeft")){servo=1;}
        if(servoin.equalsIgnoreCase("TiltLeft")){servo=4;}
        if(servoin.equalsIgnoreCase("ThrustRight")){servo=2;}
        if(servoin.equalsIgnoreCase("TiltRight")){servo=5;}
        if(servoin.equalsIgnoreCase("ThrustTail")){servo=3;}
        if(servoin.equalsIgnoreCase("TilerTail")){servo=6;}
        
        // copy the desired servo setting
        switch(servo)
        {
            case 1:
                thrust=angle;
                break;
            case 2:
                elevator=angle;
                break;
            case 3:
                aileron=angle;
                break;
            case 4:
                aux1=angle;
                break;
            case 5:
                aux2=angle;
                break;
            case 6:
                aux3=angle;
                break;
        }
        
        serial.send((byte) 255);
        serial.send((byte) servo);
        serial.send((byte) angle);
    }
    
    public void setServoQuadConfig(String servoin, int angle)
    {
        int servo = 0;
        if(angle==0){angle=1;}
        if(angle>170){angle=170;} // some kind of error breaks output above this
        // convert
        // 255/180 = 1.4166666666666666666666666666667;
        angle*=1.4166666666666666666666666666667;
        
        System.out.println(servoin + " " + angle);
        
        if(servoin.equalsIgnoreCase("1")){servo=1;}
        if(servoin.equalsIgnoreCase("2")){servo=2;}
        if(servoin.equalsIgnoreCase("3")){servo=3;}
        if(servoin.equalsIgnoreCase("4")){servo=4;}
        
        // copy the desired servo setting
        switch(servo)
        {
            case 1:
                thrust=angle;
                break;
            case 2:
                elevator=angle;
                break;
            case 3:
                aileron=angle;
                break;
            case 4:
                aux1=angle;
                break;
            case 5:
                aux2=angle;
                break;
            case 6:
                aux3=angle;
                break;
        }
        
        serial.send((byte) 255);
        serial.send((byte) servo);
        serial.send((byte) angle);
    }
    
    public void setOutputSelectPI()
    {
        serial.send((byte) 255);
        serial.send((byte) 9); 
        serial.send((byte) 1); // S
    }
    
    public void setOutputSelectRC()
    {
        serial.send((byte) 255);
        serial.send((byte) 9);
        serial.send((byte) 0); // R
    }
    
    /*
    @Override
    public void run()
    {
        while(running==true)
        {
        }
    }//run
    */
    
    public enum STOVL {ThrustLeft, ThrustRight, ThrustFront, TiltLeft, TiltRight, TiltFront }
}
