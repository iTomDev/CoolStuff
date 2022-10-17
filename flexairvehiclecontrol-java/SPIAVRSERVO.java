/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 *
 * 6.7.16
 * + can send to the AVR but dont know whether pwms are working
 * + avr sends on miso but signal needs proper level shifting. currenly signal is trashed by diode
 *
 */
package flexairvehiclecontrol;

import java.util.logging.Level;
import java.util.logging.Logger;



/**
 *
 * @author NeonGreen
 */
public class SPIAVRSERVO implements Runnable
{
    private SafeSPI spi;
    
    public void init(SafeSPI spi_in)
    {
        spi = spi_in;
    }
    
    public void run()
    {
        byte packet[] = new byte[3];
        int pwm1 = 0;
        
        while(true)
        {
            // testing SPI works
            packet[0] = (byte)0x00;
            packet[1] = (byte)0x01;
            packet[2] = (byte)pwm1;
            //spi.write(packet, 0, 3, 1);
            spi.write(packet, 0);
            
            
            // set angle
            packet[0] = (byte)0x00;
            packet[1] = (byte)0x01;
            packet[2] = (byte)pwm1;
            //spi.write(packet, 0, 3, 1);
            spi.write(packet, 1);
            
            // set angle
            
            
            //pwm1++; if(pwm1>249){pwm1=70;}            
            try 
            {
                Thread.sleep(10);
            }catch (InterruptedException ex) {Logger.getLogger(SPIAVRSERVO.class.getName()).log(Level.SEVERE, null, ex);}
            
        }
    }
    
    public void eServoTest()
    {
        byte packet[] = new byte[3];
        
        /*
        // enable outputs
        packet[0] = 0x00;
        packet[1] = 0x0A;
        packet[2] = 0x01;
        //spi.write(packet, 0, 3, 1);
        spi.write(packet, 1);
        
        // enable outputs
        packet[0] = 0x00;
        packet[1] = 0x0B;
        packet[2] = 0x01;
        //spi.write(packet, 0, 3, 1);
        spi.write(packet, 1);
        */
        while(true)
        {
            // set angle
            packet[0] = (byte)0x00;
            packet[1] = (byte)0x01;
            packet[2] = (byte)0xaa;
            //spi.write(packet, 0, 3, 1);
            spi.write(packet, 1);
            
            
        }
        
    }
}
