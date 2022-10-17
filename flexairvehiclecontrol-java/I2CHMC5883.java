/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

import com.pi4j.io.i2c.*;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 *
 * @author Thomas Pile, 2016
 */
public class I2CHMC5883 
{
    // control reg 2
    public static final byte GAIN_1370LSBG = (byte)0x00;
    public static final byte GAIN_1090LSBG = (byte)0x20;
    public static final byte GAIN_820LSBG = (byte)0x40;
    public static final byte GAIN_660LSBG = (byte)0x60;
    public static final byte GAIN_440LSBG = (byte)0x80;
    public static final byte GAIN_390LSBG = (byte)0xA0;
    public static final byte GAIN_330LSBG = (byte)0xC0;
    public static final byte GAIN_230LSBG = (byte)0xE0;
    // Mode Register
    public static final byte MODE_CONTINUOUS = (byte) 0x00;
    public static final byte MODE_SINGLE = (byte)0x01;
    
    //
    public I2CBus i2c;
    public I2CDevice device;
    
    public void init (I2CBus i2cin)
    {
        i2c = i2cin;
        
        byte params[] = new byte[2];
        
        try
        {
            // setup code
            device = i2c.getDevice(0x1E);
            // HMC5883L - Magnetometer 0x00
            // Config reg A
            // 0 11 101 00
            params[0] = (byte)0x00; params[1] = (byte)0x74;
            device.write(params);
            // config B
            params[0] = (byte)0x01; params[1] = GAIN_1090LSBG;
            device.write(params);
            // mode register
            params[0] = (byte)0x02; params[1] = MODE_SINGLE;
            device.write(params);
        }
        catch(IOException iox){iox.printStackTrace();}  
    }
    
    /*
     * 
     *       // convert to int
     *       int magX = ((buff[0] &0xFF) | ((buff[1] & 0xFF) << 8));
     *       // or to signed int
     *       int magX2 = ((buff[0] & 0xff) | (buff[1] << 8)) << 16 >> 16;
    */
    public ThreeAxis read()
    {
        ThreeAxis magdat = new ThreeAxis();
        
        try
        {
            // test multiread
            //
            byte buff[] = new byte[128];
            byte buff2[] = new byte[2];
            device.read((byte)03, buff, 0, 7);
            
            
            // convert bytes to signed int
            magdat.magX = ((buff[0] & 0xff) | (buff[1] << 8)) << 16 >> 16;
            magdat.magX = ((buff[2] & 0xff) | (buff[3] << 8)) << 16 >> 16;
            magdat.magX = ((buff[4] & 0xff) | (buff[5] << 8)) << 16 >> 16;
            
            System.out.println("Data: "+String.valueOf(magdat.magX));
            
            // correct for bias
            
            
            /*
            
            
            //response = device.read((byte)0x3B, resp, 0, 14);
            device.read((byte)03, buff, 0, 7);
            buff2[0] = asInt(buff[0]);
            buff2[1] = asInt(buff[2]);
            buff2[2] = asInt(buff[4]);
            buff2[0] = buff2[0] | (((asInt(buff[1]) & 0xFF) >>0)*256);
            buff2[1] = buff2[1] | (((asInt(buff[3]) & 0xFF) >>0)*256);
            buff2[2] = buff2[2] | (((asInt(buff[5]) & 0xFF) >>0)*256);
            buff3[0] = (float) buff2[0];
            
            //System.out.println("Number of bytes read: " + String.valueOf(response));
            System.out.println("Data: "+String.valueOf(buff3[0]) + " " +
                                        String.valueOf(buff2[1]) + " " +
                                        String.valueOf(buff2[2]) 
                                        );
            // works, reads 14 bytes
*/
        }
        catch(IOException iox){iox.printStackTrace();}
        
        return magdat;
    }
    
    /*
    private float asInt(byte b)
    {
        float i=b;
        if(1<0){i = i+256;}
        return i;
    }
    */
    
    public static class ThreeAxis
    {
        public int magX;
        public int magY;
        public int magZ;
    }
    
}
