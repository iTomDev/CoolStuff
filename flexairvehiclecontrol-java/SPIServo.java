/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

/**
 *
 * @author Thomas Pile
 *  Only works for AVR FCS code post June 2017.
 *  
 */
public class SPIServo 
{
    private SafeSPI spi;
    
    // these values are kept for logging purposes
    public int out1 = 0;
    public int out2 = 0;
    public int out3 = 0;
    public int out4 = 0;
    public int out5 = 0;
    public int out6 = 0;
    public int out7 = 0;
    public boolean rcAutopilotEnabled = false;
    public int rc_ch1_roll = 0;
    public int rc_ch2_thrust = 0;
    public int rc_ch3_pitch = 0;
    public int ultrasonic_height = 0;
    
    public void init(SafeSPI spiin)
    {
        spi = spiin;
    }
    
    /*
    Set servo values. 
    */
    public void setServo(byte servo, byte value)
    {
        final int SERVO_OFFSET = 0x80;
        
        byte packet[] = new byte[4];
                
        packet[0] = (byte)(SERVO_OFFSET+servo); //servo ch
        packet[1] = (byte)value; 
        packet[2] = (byte)0xFF;
        packet[3] = (byte)0x00;
        spi.write(packet,0, 4, 1);
        
        switch(servo)
        {
            case 1: out1 = value; break;
            case 2: out2 = value; break;
            case 3: out3 = value; break;
            case 4: out4 = value; break;
            case 5: out5 = value; break;
            case 6: out6 = value; break;
            case 7: out7 = value; break;
        }
    }
    
    /*
    This get RC controller measurements from the AVR
    Important: The response to request 1 comes in request 2
    */
    public void getRCInput()
    {
        final byte RC_CH0_BASE_VALUE = 0;
        final byte RC_CH1_BASE_VALUE = 70;
        final byte RC_CH2_BASE_VALUE = 28;
        final byte RC_CH3_BASE_VALUE = 50;
        
        final int RC_CH0_TOP_VALUE = 0;
        final int RC_CH1_TOP_VALUE = 250;
        final int RC_CH2_TOP_VALUE = 248;
        final int RC_CH3_TOP_VALUE = 228;
        
        /*
        Important: The response to request 1 comes in request 2 in all cases!
        // example 1. read rc val 0 - long version
        packet[0] = (byte)0x11; //ch
        packet[1] = (byte)0xFF; 
        packet[2] = (byte)0xFF;
        packet[3] = (byte)0x00;
        byte[] ret =  spi.write(packet,0, 4, 1);
        System.out.println("Rc input: "+Byte.toUnsignedInt(ret[0])+" "+ret[1]+" "+ret[2]+" "+ret[3]);
        
        // example 2. read rc val 0 - short version
        packet[0] = (byte)0x11; //ch
        packet[1] = (byte)0x00;
        byte[] ret =  spi.write(packet,0, 2, 1);
        System.out.println("Rc input: "+Byte.toUnsignedInt(ret[0])+" "+ret[1]);
        */
        
        int rcval[] = new int[7];
        byte packet[] = new byte[7];
        byte ret[] = new byte[5];
        
        // request ch1 roll
        packet[0] = (byte)0x11; 
        packet[1] = (byte)0x00;
        ret[0] =  spi.write(packet,0, 2, 1)[0];
        //System.out.println("Rc input: "+Byte.toUnsignedInt(ret[0])+" "+ret[1]);
        
        // ch2 thrust
        packet[0] = (byte)0x12; 
        packet[1] = (byte)0x00;
        ret[1] =  spi.write(packet,0, 2, 1)[0];
        ret[1] -= RC_CH1_BASE_VALUE;
        //System.out.println("Rc input: "+Byte.toUnsignedInt(ret[0])+" "+ret[1]);
        
        // ch3 elev
        packet[0] = (byte)0x13;
        packet[1] = (byte)0x00;
        ret[2] =  spi.write(packet,0, 2, 1)[0];
        ret[2] -= RC_CH2_BASE_VALUE;
        //System.out.println("Rc input: "+Byte.toUnsignedInt(ret[0])+" "+ret[1]);
        
        // ch4 - just to get the last val
        packet[0] = (byte)0x14; //
        packet[1] = (byte)0x00;
        ret[3] =  spi.write(packet,0, 2, 1)[0];
        ret[3] -= RC_CH3_BASE_VALUE;
        //System.out.println("Rc input: "+Byte.toUnsignedInt(ret[0])+" "+ret[1]);
        
        System.out.println("Rc input: Roll "+Byte.toUnsignedInt(ret[1])+
                                       ", Thrust: "+Byte.toUnsignedInt(ret[2])+
                                       " Pitch: "+Byte.toUnsignedInt(ret[3])+" ");
        
        // copy across
        //rcval[0] = ret[0];
        //rcval[1] = ret[1];
        //rcval[2] = ret[2];
        //rcval[3] = ret[3];
        
        //if(rc_ch1_roll<0 || rc_ch2_thrust<0 || rc_ch3_pitch<0)
        {
            rc_ch1_roll = Byte.toUnsignedInt(ret[1]);
            rc_ch2_thrust = Byte.toUnsignedInt(ret[2]);
            rc_ch3_pitch = Byte.toUnsignedInt(ret[3]);
        }
        
        //int fakeret[] = {0,0,0,0,0};
        //return rcval;
}
    
    public int getUltrasonicHeight()
    {
        /*
        int height = 0;
        byte packet[] = new byte[5];
        byte resp[] = new byte[5];
        packet[0] = (byte)0xff;
        packet[1] = (byte)0xfd;
        packet[2] = (byte)0x01;
        packet[3] = (byte)0x00;
        packet[4] = (byte)0x00;
        height = spi.write(packet,0, 5, 1)[0];
        //System.out.println("Rc input"+height);
        */
        int height = 0;
        byte packet[] = new byte[5];
        byte ret = 0;
        
        // send height request
        packet[0] = (byte)0x31;
        packet[1] = (byte)0x00;
        spi.write(packet,0, 2, 1);
        
        // send request again to read the value
        packet[0] = (byte)0x31;
        packet[1] = (byte)0x00;
        ret =  spi.write(packet,0, 2, 1)[0];
        
        ultrasonic_height = ret;
        
        return ret;
        //System.out.println("Height: "+Byte.toUnsignedInt(ret)+" ");
    }
    
    public boolean getRCAutopilotEnabled()
    {
        /*
        boolean enabled = false;
        byte packet[] = new byte[5];
        byte resp[] = new byte[5];
        packet[0] = (byte)0xff;
        packet[1] = (byte)0xFC;
        packet[2] = (byte)0x00;
        packet[3] = (byte)0x00;
        packet[4] = (byte)0x00;
        resp = spi.write(packet,0, 5, 1);
        */
        
        byte packet[] = new byte[7];
        byte ret = 0;
        boolean enabled = false;
        
        // request value of PWM_Output_Select
        packet[0] = (byte)0x30; 
        packet[1] = (byte)0x00;
        spi.write(packet,0, 2, 1);
        
        packet[0] = (byte)0x30; 
        packet[1] = (byte)0x00;
        ret =  spi.write(packet,0, 2, 1)[0];
        //System.out.println("Rc input: "+Byte.toUnsignedInt(ret[0])+" "+ret[1]);
        
        if(ret>0){enabled=true; rcAutopilotEnabled=true;}
        return enabled;
    }
}
