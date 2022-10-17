/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

import flexairvehiclecontrol.EKF.EKFData;
import flexairvehiclecontrol.I2CMPU6050.SixAxis;
import flexairvehiclecontrol.I2CMPU9250_NoRun.NineAxis;
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
public class EKFDataLog 
{
    FileWriter logfile;
    BufferedWriter buffw;
    PrintWriter log;
    long time = 0; 
    
    public void open()
    {
        try
        {
            // make unique names for log files to prevent overwriting
            time = System.nanoTime();
            String stime = String.valueOf(time);
            String logpath = "FCSDat";
            logpath = logpath.concat(stime);
            logpath = logpath.concat(".csv");
            
            //logfile = new FileWriter("FCSDat.csv");
            logfile = new FileWriter(logpath);
            buffw = new BufferedWriter(logfile);
            log = new PrintWriter(buffw);
            
            log.println(
                "$, " +
                "Quat 1, " + // Quat 1
                "Quat 2, " + // Quat 2
                "Quat 3, " + // Quat 3
                "Quat 4, " + // Quat 4
                "Velocity X, " + // Velocity
                "Velocity Y, " + // Velocity
                "Velocity Z, " + // velocity 
                "Gyro X, " +      // Gyro X
                "Gyro Y, " +      // Gyro Y
                "Gyro Z," +       // Gyro Z
                "Accel X," +       // Accel X
                "Accel Y," +       // Accel Y
                "Accel Z," +       // Accel Z
                "headingT,"+       // True heading
                "speedKM,"+         // Speed KM
                "servo_out1,"+
                "servo_out2,"+
                "servo_out3,"+
                "servo_out4,"+
                "servo_out5,"+
                "servo_out6,"+
                "rc_ch1_roll,"+
                "rc_ch2_thrust,"+
                "rc_ch3_pitch,"+
                "ultrasonic_height,"+
                "dt(nanoseconds)"
            );
        }
        catch(IOException iox)
        {
            iox.printStackTrace();
        }
    }
    
    
    
    /*
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
    */
    
    /*
    // log EKF state components
    */
    public void updateLog(double[] stateEKF, NineAxis imu, double gps_courseT, double gps_speedKM, SPIServo servo)
    {
        // find dt
        long dt = (System.nanoTime()-time);
        time = System.nanoTime();
        
        log.println(
            "0," +
            stateEKF[0]+ "," + // Quat 1
            stateEKF[1]+ "," + // Quat 2
            stateEKF[2]+ "," + // Quat 3
            stateEKF[3]+ "," + // Quat 4
            stateEKF[4]+ "," + // Velocity
            stateEKF[5]+ "," + // Velocity
            stateEKF[6]+ "," + // velocity 
            imu.gx+ "," +      // Gyro X
            imu.gy+ "," +      // Gyro Y
            imu.gz+ "," +      // Gyro Z
            imu.ax+ "," +      // Accel X
            imu.ay+ "," +      // Accel Y
            imu.az+ "," +      // Accel Z
            gps_courseT+ "," + //gps.courseT
            gps_speedKM+","+        //gps.speedKM
            servo.out1+ "," +
            servo.out2+ "," +
            servo.out3+ "," +
            servo.out4+ "," +
            servo.out5+ "," +
            servo.out6+ "," +
            servo.rc_ch1_roll+ "," +
            servo.rc_ch2_thrust+ "," +
            servo.rc_ch3_pitch+","+
            servo.ultrasonic_height+","+
            dt
        );
        try
        {
            buffw.flush();
            logfile.flush();
        }
        catch(IOException iox)
        {
            iox.printStackTrace();
        }

    }
    
    
    public void openSimpleLog()
    {
        try
        {
            // make unique names for log files to prevent overwriting
            time = System.nanoTime();
            String stime = String.valueOf(time);
            String logpath = "FCSDat";
            logpath = logpath.concat(stime);
            logpath = logpath.concat(".csv");
            
            //logfile = new FileWriter("FCSDat.csv");
            logfile = new FileWriter(logpath);
            buffw = new BufferedWriter(logfile);
            log = new PrintWriter(buffw);
            
            log.println(
                "$, " +
                "Quat 1, " + // Quat 1
                "Quat 2, " + // Quat 2
                "Quat 3, " + // Quat 3
                "Quat 4, " + // Quat 4
                "Velocity X, " + // Velocity
                "Velocity Y, " + // Velocity
                "Velocity Z, " + // velocity 
                "headingT,"+       // True heading
                "speedKM,"+         // Speed KM
                "rc_ch1_roll,"+
                "rc_ch2_thrust,"+
                "rc_ch3_pitch,"+
                "ultrasonic_height"+
                "dt(nanoseconds)"
            );
        }
        catch(IOException iox)
        {
            iox.printStackTrace();
        }
    }
    
    /*
    // log EKF state components
    */
    public void updateSimpleLog(double[] stateEKF, NineAxis imu, double gps_courseT, double gps_speedKM, SPIServo servo)
    {
        long dt = (System.nanoTime()-time);
        time = System.nanoTime();
        log.println(
            "0," +
            stateEKF[0]+ "," + // Quat 1
            stateEKF[1]+ "," + // Quat 2
            stateEKF[2]+ "," + // Quat 3
            stateEKF[3]+ "," + // Quat 4
            stateEKF[4]+ "," + // Velocity
            stateEKF[5]+ "," + // Velocity
            stateEKF[6]+ "," + // velocity 
            gps_courseT+ "," + //gps.courseT
            gps_speedKM+","+        //gps.speedKM
            servo.rc_ch1_roll+ "," +
            servo.rc_ch2_thrust+ "," +
            servo.rc_ch3_pitch+","+
            servo.ultrasonic_height+","+
            dt
        );
        try
        {
            buffw.flush();
            logfile.flush();
        }
        catch(IOException iox)
        {
            iox.printStackTrace();
        }

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
}
