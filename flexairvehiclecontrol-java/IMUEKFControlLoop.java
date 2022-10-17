/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

import flexairvehiclecontrol.I2CMPU9250_NoRun.NineAxis;
import flexairvehiclecontrol.EKF_NoRun;
import Quadrotor.QuadBasic_NoRun;
import java.util.Timer;
import java.util.TimerTask;
import org.ujmp.core.Matrix;

/**
 *
 * @author Thomas Pile
 * An attempt to combine the EKF, IMU reading and stabilisation into a single loop
 * so data doesn't have to share data locks which can cause errors and may be 
 * slowing computation down.
 */
public class IMUEKFControlLoop implements Runnable
{
    // thread
    boolean running = true;
    boolean cycle = true;
    long timeold = 0;
    private final boolean DISPLAY_LOOP_TIME = false;
    
    // logging
    private EKFDataLog log;
    private boolean enableLogging = true;
    
    // data
    Matrix State;       // this maintains state of the EKF, was in the class before
    
    // objects
    I2CMPU9250_NoRun imu;
    EKF_NoRun ekf;
    //QuadBasic_NoRun control;
    flexairvehiclecontrol.FlightController control;
    RS232 serial;
    SPIServo servo;
    SafeI2C i2c;
    
    // imu FIR filters
    FIR_Filter imufilter_ax;
    FIR_Filter imufilter_ay;
    FIR_Filter imufilter_az;
    FIR_Filter imufilter_gx;
    FIR_Filter imufilter_gy;
    FIR_Filter imufilter_gz;
    AverageFilter gyroFilt;     
    AverageFilter accelFilt;  
    
    
    private static final double toDegrees = 57.2957795; //180/Math.PI;
    private static final double toRadians = 0.0174532925;
    
    //public void init(SafeI2C i2cin, RS232 serialin, SPIServo servoin, QuadBasic_NoRun controlin)
    public void init(SafeI2C i2cin, RS232 serialin, SPIServo servoin, flexairvehiclecontrol.FlightController controlin)
    {
        i2c = i2cin;
        serial = serialin;
        servo = servoin;
        control = controlin;
        
        // start log
        log = new EKFDataLog();
        log.open();
        
        // set up serial access and gps comms
        serial = serialin;
        
        // set up a non-runnable imu
        imu = new I2CMPU9250_NoRun(); 
        imu.init(i2cin);
        
        // set up FIR filter for imu data
        imufilter_ax = new FIR_Filter();
        imufilter_ay = new FIR_Filter();
        imufilter_az = new FIR_Filter();
        imufilter_gx = new FIR_Filter();
        imufilter_gy = new FIR_Filter();
        imufilter_gz = new FIR_Filter();
        
        // set up non-runnable imu
        ekf = new EKF_NoRun();
        ekf.init();
        
        // average filter to move vibrations between 0.5s and 0.25s
        // cant filter this with FIR as it will ruin the signal
        gyroFilt = new AverageFilter();
        gyroFilt.init();
        accelFilt = new AverageFilter();
        accelFilt.init();
        
        //timertest timer = new timertest();
        IMU_Timer imutimer = new IMU_Timer(this);
        
        double State_arr[] = {1,0,0,0,0,0,0};
        //double State_arr[] = {0,0,-1,0,0,0,0}; // 180 degrees yaw
        //double State_arr[] = {0,0,0,-1,0,0,0}; //180 roll
        State = Matrix.Factory.importFromArray(State_arr);
        State = State.transpose();
        
        
    }
    
    @Override
    public void run()
    {
        while(running==true)
        {
            if(cycle==true) // go through a cycle
            {
                timeold = System.nanoTime();
                
                // read a non-runnable imu
                I2CMPU9250_NoRun.NineAxis imudat = imu.read();
                
                
                // filter imu data
                //System.out.println("gy: "+imudat.gy);
                imudat.ax = imufilter_ax.next(imudat.ax);
                imudat.ay = imufilter_ay.next(imudat.ay);
                imudat.az = imufilter_az.next(imudat.az);
                imudat.gx = imufilter_gx.next(imudat.gx);
                imudat.gy = imufilter_gy.next(imudat.gy);
                imudat.gz = imufilter_gz.next(imudat.gz);
                //System.out.print("filtered gy: "+imudat.gy);
                
                
                // apply average filter to gyro data
                double[] gyroFiltDat = {0,0,0};
                gyroFiltDat[0] = imudat.gx;
                gyroFiltDat[1] = imudat.gy;
                gyroFiltDat[2] = imudat.gz;
                gyroFiltDat = gyroFilt.next(gyroFiltDat);
                if(gyroFiltDat==null){} // make sure its initialised
                else
                {
                    imudat.gx = gyroFiltDat[0];
                    imudat.gy = gyroFiltDat[1];
                    imudat.gz = gyroFiltDat[2];
                }
                
                // apply average filter to accel data
                double[] accelFiltDat = {0,0,0};
                accelFiltDat[0] = imudat.ax;
                accelFiltDat[1] = imudat.ay;
                accelFiltDat[2] = imudat.az;
                accelFiltDat = accelFilt.next(accelFiltDat);
                if(accelFiltDat==null){} // make sure its initialised
                else
                {
                    imudat.ax = accelFiltDat[0];
                    imudat.ay = accelFiltDat[1];
                    imudat.az = accelFiltDat[2];
                }
                
                // get gps data (has to be threaded)
                RS232.GPSData gpsdat = serial.getGPSData();
                
                // send it to the EKF
                EKF_NoRun.EKFData ekfdat;
                ekfdat = ekf.runEKF(State,imudat, gpsdat);
                State = ekfdat.state;
                
                // get altitude
                //ekfdat.ultrasonic_height = servo.getUltrasonicHeight();
                
                // align IMU
                //double[] imu_align = {0,-180,0};
                //ekfdat.state = EKF_NoRun.RotateStateByQuaternion(ekfdat.state, EKF_NoRun.EulerToQuat(imu_align));
                
                /*
                // apply average filter
                double[] rpy = avgFilt.next(ekfdat.getRPY());
                if(rpy==null){} // make sure its initialised
                else
                {
                    ekfdat.filtRPY=rpy;
                    // convert euler to quaternion for logging only! (remove when working)
                    ekfdat.setStateQuat(ekf.EulerToQuat(ekfdat.filtRPY));
                }
                */
                
                if(DISPLAY_LOOP_TIME == true)
                {
                    System.out.println("time taken: "+(System.nanoTime()-timeold)+ " ");
                    
                }
                
                //System.out.println("EKF RPY: "+ekfdat.getRPY()[0]+" "+ekfdat.getRPY()[1]+" "+ekfdat.getRPY()[2]);
                //System.out.println("Accel data: "+imudat.ax+" "+imudat.ay+" "+imudat.az);
                
                /*
                // works
                ekfdat.getRPY();
                // does not work
                double[] tst = {1,2,3};
                avgFilt.next(tst);
                */
                
                
                // get rc controller inputs
                servo.getRCInput();
                
                // thread safe ekf update
                control.setEKFData(ekfdat);
                
                // logging
                if(enableLogging==true)
                {
                    log.updateLog(EKF_NoRun.StateToDouble(ekfdat.state), imudat, gpsdat.courseT, gpsdat.speedKM, servo);
                }
                
                cycle = false;
                
                
                /*
                try
                {
                    Thread.sleep(10);
                }
                catch(InterruptedException iex){iex.printStackTrace();}
                */
            }//cycle
        }//running
    }//run
    
    
}//class
