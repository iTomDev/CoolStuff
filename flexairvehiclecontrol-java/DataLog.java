/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

import flexairvehiclecontrol.I2CMPU6050.SixAxis;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

/**
 * v0.1 11.3.16
 * + create the log, close log
 * + IMU log 
 * 
 * @author Thomas Pile, 2016
 */
public class DataLog 
{
    private FileWriter logfile;
    private BufferedWriter buffw;
    private PrintWriter log;
    
    public void init()
    {
        try
        {
            logfile = new FileWriter("FCSDat.csv");
            buffw = new BufferedWriter(logfile);
            log = new PrintWriter(buffw);
        }
        catch(IOException iox)
        {
            iox.printStackTrace();
        }
    }
    
    public void updateLog(SixAxis imudat)
    {
        // IMU Log
        // MPU6050, ax, ay, az, gx, gy, gz
        log.println("MPU6050," + formatInt(imudat.ax)
                + "," + formatInt(imudat.ay)
                + "," + formatInt(imudat.az)
                + "," + formatInt(imudat.az)
                + "," + formatInt(imudat.gx)
                + "," + formatInt(imudat.gy)
                + "," + formatInt(imudat.gz)
                );
        
        // GPS Log
        //log.println()
        
        // Aircraft data log
    }
    
    public void close()
    {
        try
        {
            buffw.flush();
            buffw.close();
            logfile.close();
        }
        catch(IOException iox)
        {
            iox.printStackTrace();
        }
    }
    
    private static String formatInt(int i) {
        String x = "         " + Integer.toString(i);
        x = x.substring(x.length() - 6, x.length());
        return x;
    }
    
    private static String formatLong(long i) {
        String x = "         " + Long.toString(i);
        x = x.substring(x.length() - 6, x.length());
        return x;
    }
}
