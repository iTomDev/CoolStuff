/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

// ujmp - matrix algebra

import flexairvehiclecontrol.I2CMPU9250.NineAxis;
import flexairvehiclecontrol.RS232.GPSData;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.ujmp.core.*;

/**
 * v0.1 16.3.16 - 17.3.16
 * Basic EKF implementation based on something I did in Matlab a few years back.
 * Functionality:
 * 1. Makes state predictions of attitude in Quaternion form by integrating gyro
 *    rates.
 * 2. State corrections performed with low frequency measurements from 
 *    accelerometer for roll/pitch, and GPS for yaw. 
 * 3. Can do a little under 30k full predict/correct cycles per second, 
 *    including finding K, on an Intel i5 2520m, with fixed measurements
 * 
 * Doing full predict/correct cycles inc finding K the filter achieves 2000 cycles/0.9 secs on Rasp Pi 2
 * 
 * 17.5.16
 * + gyro, accel, mag, gps and some resistance to forward movement.
 * 
 * Usage:
 * EKF ekf = new EKF();
 * ekf.runFilter(imu.read(), gpsserial.read());
 * ekf.getState();
 * 
 * IMU up is roll 180*, pitch = 0
 * 
 * Threaded usage:
 * 
 * 
 * To do :
 * - option for constant kalman gain
 * - mag measurements (separate filter)
 * 
 * @author Thomas Pile, 2016
 */
public class EKF implements Runnable
{
    private boolean running;
    private Lock EKFDataLock;
    private EKFData ekfdat; // return data from EKF
    //private NineAxis imu;   // IMU data from mag/gyro/accel combo
    private I2CMPU9250 imu;
    private NineAxis imudat;
    private GPSData gps;    // GPS data
    private Servo servo;
    
    //private I2CMPU9250 imu;
    //private RS232GPS gps;
    //private ThreeAxis mag; // Mag data
    private EKFDataLog log;
    private StatusLed leds;
    
    // filter logic
    private long correctionPeriod_ms = 10; 
    private long correctionCount_ms = 0;
    private boolean constantKalman = false;
    private boolean enableLogging = true;
    public boolean printlog = false; // print log to screen]
    
    Lock stateLock;
    
    // filter maths
    private Matrix State;       // State vector [Quat, Velocities]
    private Matrix Covariance;  
    private Matrix Q;           // process noise model
    private Matrix R;           // measurement noise model
    private Matrix h;           // measurement matrix
    private Matrix H;           // measurement jacobian matrix
    private Matrix Z;           // measurement vector
    private Matrix K;           // Kalman gain
    private Matrix Residual;    
    private Matrix S;
    private long dt = 20; // filter sample period, milliseconds
    //private long dt = 1;    
    private long sleept = 1; // how long for thread to sleep (not the same thing as dt!!!)
    
    // filter constants
    private static final Matrix I77 = DenseMatrix.Factory.eye(7, 7);
    private static final double g = -9.8;
    private static final double toSeconds = 0.001;
    private static final double toDegrees = 57.2957795; //180/Math.PI;
    private static final double toRadians = 0.0174532925;
    
    /*
    // filter parameters
    private double gps_heading_variance = 0.0001;
    private double accelerometer_variance = 0.0001;
    private double gyro_variance = 0.15; // was 0.015
    private int bias_found = 0;
    */
    
    // filter parameters
    private double gps_heading_variance = 0.0001;
    private double accelerometer_variance = 0.001;  // was 0.0001
    private double gyro_variance = 0.15; // was 0.015
    public int bias_found = 0;
    
    // freq filter
    FIR_Filter pitchfilter = new FIR_Filter();
    
    
    // bias calcs
    private double gxbias = 0;
    private double gybias = 0;
    private double gzbias = 0;
    private double gxold= 0;
    private double gyold= 0;
    private double gzold= 0;
    
    /*
     * Constructor for EKF.
     * Initialises the EKF by setting up the internal matrices.
    */
    public EKF()
    {
        //
    }
    
    /*
     * Constructor for Runnable EKF.
     * Initialises the EKF by setting up the internal matrices.
    */
    //public EKF(NineAxis nineaxisin, GPSData gpsdatain)
    public void init(I2CMPU9250 imuin, GPSData gpsin, StatusLed ledsIn, Servo servoin)//, RS232GPS gpsin)
    {
        servo = servoin;
        leds = ledsIn;
        
        // Q - Noise model of "prediction" process (the gyro integration part)
        Matrix I77 = DenseMatrix.Factory.eye(7, 7);
        double wvar = gyro_variance;
        Q = I77.times(wvar);
        
        // R - Noise model of measurement model in "correction" stage
        double avar = accelerometer_variance; // accelerometer variance
        double ghvar = gps_heading_variance; // gps heading variance
        double R_arr[][] = {{avar, 0   , 0   , 0    }, 
                            {0   , avar, 0   , 0    },
                            {0   , 0   , avar, 0    },
                            {0   , 0   , 0   , ghvar}};
        R = Matrix.Factory.importFromArray(R_arr);
        
        // init state vector col(ones)
        double State_arr[] = {1,0,0,0,0,0,0};
        State = Matrix.Factory.importFromArray(State_arr);
        State = State.transpose();
        
        // init covariance matrix
        Covariance = Matrix.Factory.zeros(7,7); // init
        
        // copy data from I2CMPU9250 data object to the local one
        imu = imuin;
        gps = gpsin;
        
        EKFDataLock = new ReentrantLock();
        stateLock = new ReentrantLock(); // conocurrency lock on state
        running = true;        
        /*
        // set up logging
        if (enableLogging==true)
        {
            log = new EKFDataLog();
            log.open();
        }*/
    }
    
    
    // thread safe access to state, internal use
    public Matrix getState()
    {
        Matrix ret = Matrix.Factory.zeros(7,1);
        

        //{ret = State; }
        
        
        try 
        {
            if(stateLock.tryLock()) {ret = State; }
        }
        finally{stateLock.unlock();}
        
        return ret;
    }
    
    // thread safe access to state, internal use
    public void setState(Matrix in)
    {
        //{State = in;}
        
        
        try 
        {
            
            if(stateLock.tryLock()) {State = in;}
        }
        finally{stateLock.unlock();}
        
    }
    
    
    
    // Direct access to the state vector, can be used for results
    // 
    public double[] getStateComponents()
    {
        Matrix ls = Matrix.Factory.zeros(7,1);
        try 
        {
            if(stateLock.tryLock()) {ls = State;}
        }
        finally{stateLock.unlock();}
        
        double x[] = {0,0,0,0,0,0,0};
        x[0] = ls.getAsDouble(0,0);
        x[1] = ls.getAsDouble(1,0);
        x[2] = ls.getAsDouble(2,0);
        x[3] = ls.getAsDouble(3,0);
        x[4] = ls.getAsDouble(4,0);
        x[5] = ls.getAsDouble(5,0);
        x[6] = ls.getAsDouble(6,0);
        return x;
    }
    
    /* safelty returns state as Euler angles in degrees*/
    public double[] getStateAsEuler()
    {
        double r[] = new double[3];
        Matrix ls = Matrix.Factory.zeros(7,1);
        
        try
        {
            try 
            {
                //if(stateLock.tryLock()) {ls = State; }
                if(stateLock.tryLock(20, TimeUnit.MILLISECONDS)){ls = State; }
            }
            catch(InterruptedException iex){iex.printStackTrace();}
            finally{stateLock.unlock();}
        }
        catch(IllegalMonitorStateException imse){imse.printStackTrace();}
        
        
        r = stateToEuler(ls);
        r[0]*= toDegrees;
        r[1]*= toDegrees;
        r[2]*= toDegrees;
        return r;
    }
    
    
    public double[] getGyroPQR()
    {
        double[] ret = {0.0,0,0};
        
        ret[0] = imudat.gx*toDegrees;
        ret[1] = imudat.gy*toDegrees;
        ret[2] = imudat.gz*toDegrees;
        return ret;
    }
    
    /*
     * @param updatePeriod = dt = period between prediction cycles milliseonds
     * @param correctPeriod: perform a correction every [correctPeriod_ms] milliseonds
     * @param constantKalman: Should filter converge after a few iterations to constant gain?
    */
    public void setFilterParams(int updatePeriod, int correctionPeriodin, boolean constantKalmanin)
    {
        dt = updatePeriod;
        correctionPeriod_ms = correctionPeriodin;
        constantKalman = constantKalmanin;
    }
    /*
    // performs a full filter cycle if the class isn't being used as Runnable
    public EKFData runFilter(I2CMPU9250 imuin, RS232GPS gpsin)
    {
        gps = gpsin.getGPSData();
        imu = imuin.getIMUData();
        imu.ay = 10;
        this.predict();
        this.correct();
        
        return this.ekfdat;
    }
    */
    public void enableLogging()
    {
        enableLogging = true;
    }
    
    /*
     *               INTERNAL METHODS
     *
    */
    
    @Override
    public void run()
    {
        int i = 0;
        log = new EKFDataLog();
        log.open();
        long before = System.nanoTime();
        
        
        
        //while (i<2000)
        while(running)
        {
            RunLoop();
            
            try
            {
                Thread.sleep(sleept);

            }
            catch(InterruptedException ixe)
            {
                ixe.printStackTrace();
            }
        }
        
        
        /*
        // close log
        if (enableLogging==true)
        {
            enableLogging = false;
            log.close();
        }
        */
    }
    
    public void RunLoop()
    {
        // update local imu data with new measurements
        imudat = imu.getIMUData();

        // invert Z axis
        imudat.az = -imudat.az;

        // get heading from mag
        if(imudat.magUsed==true)
        {
            ekfdat.mag_heading = Magto2DYaw(imudat.my,imudat.mx);
        }
        /*
        {
        // fake IMU data  - remove for proper usage
        imudat.gx = 0;
        imudat.gy = 0;
        imudat.gz = 0;

        imudat.setAX(0);
        imudat.setAY(0);
        imudat.setAZ(-9.8);
        gps.courseT = 359;
        */

        correctionCount_ms+=dt;
        predict();
        
        
        
        // print state matrix as euler
        //printMatrix(State);
        double Qo[] = {0.0, 0.0, 0.0};
        Qo = stateToEuler(State);

        // low pass filter results
        // filter. 10ms period=100Hz
        // doesnt work
        //double pitch = pitchfilter.next(Qo[2]);
        //System.out.println(pitch*toDegrees);
        
        
        
        if(correctionCount_ms >= correctionPeriod_ms)
        {
            // correct with accel and gps
            //correct();
            // correct with accel and mag
            correct_Accel_Mag();
            correctionCount_ms = 0;
        }

        /*
        if(enableLogging==true)
        {
            log.updateLog(this.getStateComponents(), imu.getIMUData(), gps.courseT, gps.speedKM, servo);
        }
        */
        
        // flash LEDS
        if(leds.LED1_OFF()==true){leds.flashLED1();}
        if(leds.LED2_OFF()==true){leds.flashLED2();}

        //getStateAsEuler();

        
    }
    
    public void predict()
    {
        Matrix LocalState = this.getState();
        
        double X[] = {0,0,0,0}; // quaternion state vector
        X[0] = LocalState.getAsDouble(0,0);
        X[1] = LocalState.getAsDouble(1,0);
        X[2] = LocalState.getAsDouble(2,0);
        X[3] = LocalState.getAsDouble(3,0);
        
        // subtract some bias
        imudat.gx = imudat.gx - imudat.gxbias;
        imudat.gy = imudat.gy - imudat.gybias;
        imudat.gz = imudat.gz - imudat.gzbias;
        
        // State prediction Jacobian Matrix
        // [dQuat/dQuat]       0
        //       0       [dEuler/dQuat]
        /*
        // The second block here factors in the bias... 
        // working
        double J[][] = {{0.5*0        , 0.5*-imudat.gx, 0.5*-imudat.gy, 0.5*-imudat.gz, -0.5*-X[1], -0.5*-X[2] , -0.5*-X[3]},
                        {0.5*imudat.gx, 0.5*0         , 0.5*imudat.gz , 0.5*-imudat.gy, -0.5* X[0], -0.5*-X[3] , -0.5* X[2]},
                        {0.5*imudat.gy, 0.5*-imudat.gz, 0.5*0         , 0.5*-imudat.gx, -0.5* X[3], -0.5* X[0] , -0.5*-X[1]},
                        {0.5*imudat.gz, 0.5*imudat.gy , 0.5*-imudat.gx, 0.5*0         , -0.5*-X[2], -0.5* X[1] , -0.5* X[0]},
                        {0            ,0              ,0              ,0              ,0          ,0           ,0          },
                        {0            ,0              ,0              ,0              ,0          ,0           ,0          },
                        {0            ,0              ,0              ,0              ,0          ,0           ,0          }};
        */
        // experimental
        // Adding -0.5 here massively reduces gyro drift. Started at -2,0 for roll,pitch, 5 mins stationary same vals.
        // This is inspired by p182 grwel and andrews, though it's not the same thing. Their uses a Hessian etc, this
        // is just a crude experiment.
        // It essentially eliminates drift
        double J[][] = {{0.5*0        , 0.5*-imudat.gx, 0.5*-imudat.gy, 0.5*-imudat.gz, -0.5*-X[1], -0.5*-X[2] , -0.5*-X[3]},
                        {0.5*imudat.gx, 0.5*0         , 0.5*imudat.gz , 0.5*-imudat.gy, -0.5* X[0], -0.5*-X[3] , -0.5* X[2]},
                        {0.5*imudat.gy, 0.5*-imudat.gz, 0.5*0         , 0.5*-imudat.gx, -0.5* X[3], -0.5* X[0] , -0.5*-X[1]},
                        {0.5*imudat.gz, 0.5*imudat.gy , 0.5*-imudat.gx, 0.5*0         , -0.5*-X[2], -0.5* X[1] , -0.5* X[0]},
                        {0            ,0              ,0              ,0              ,-0.5       ,0           ,0          },
                        {0            ,0              ,0              ,0              ,0          ,-0.5        ,0          },
                        {0            ,0              ,0              ,0              ,0          ,0        ,-0.5          }};
        
        /*
        double J[][] = {{0.5*0             , 0.5*-imudat.getGX(), 0.5*-imudat.getGY(), 0.5*-imudat.getGZ(), -0.5*-X[1], -0.5*-X[2] , -0.5*-X[3]},
                        {0.5*imudat.getGX(), 0.5*0              , 0.5*imudat.getGZ() , 0.5*-imudat.getGY(), -0.5* X[0], -0.5*-X[3] , -0.5* X[2]},
                        {0.5*imudat.getGY(), 0.5*-imudat.getGZ(), 0.5*0              , 0.5*-imudat.getGX(), -0.5* X[3], -0.5* X[0] , -0.5*-X[1]},
                        {0.5*imudat.getGZ(), 0.5*imudat.getGY() , 0.5*-imudat.getGX(), 0.5*0              , -0.5*-X[2], -0.5* X[1] , -0.5* X[0]},
                        {0                 ,0                   ,0                   ,0                   ,0          ,0           ,0          },
                        {0                 ,0                   ,0                   ,0                   ,0          ,0           ,0          },
                        {0                 ,0                   ,0                   ,0                   ,0          ,0           ,0          }};
        */
        Matrix J2 = DenseMatrix.Factory.importFromArray(J);
        
        // State predict equations     
        // X = (eye(7) + F*dt) * X; 
        // P = F*P*F' + Q;        
        LocalState = (I77.plus(J2.times(dt*toSeconds))).mtimes(LocalState); 
        Covariance = J2.mtimes(Covariance.mtimes(J2.transpose())).plus(Q);
        
        //printMatrix(State);        
        LocalState = normaliseState(LocalState);
        //printMatrix(LocalState);
        
        setState(LocalState);
        
    }
    
    public void correct()
    {
        Matrix LocalState = this.getState();
        
        // get state vector quat in more concise form
        double X[] = {0,0,0,0};
        X[0] = LocalState.getAsDouble(0,0);
        X[1] = LocalState.getAsDouble(1,0);
        X[2] = LocalState.getAsDouble(2,0);
        X[3] = LocalState.getAsDouble(3,0);
        
        // State relation vector
        // converts state to the same form as the measurements ?
        // h = [2*g*(X(2)*X(4)-X(1)*X(3));
        //  2*g*(X(3)*X(4)+X(1)*X(2));
        //  g*(X(1)^2-X(2)^2-X(3)^2-X(4)^2);
        //  atan2(2*(X(1)*X(4)+X(2)*X(3)), 1 - 2*(X(3)^2+X(4)^2))];
        double[] h_arr = {2*g*(X[1]*X[3]-X[0]*X[2]),
                          2*g*(X[2]*X[3]+X[0]*X[1]),
                          g*( Math.pow(X[0],2)-Math.pow(X[1],2)-Math.pow(X[2],2)-Math.pow(X[3],2) ),
                          Math.atan2( 2*(X[0]*X[3]+X[1]*X[2]),  1-2*(Math.pow(X[2],2)+Math.pow(X[3],2)) )};
        h = DenseMatrix.Factory.importFromArray(h_arr);
        h = h.transpose();
        
        // Measurement Jacobian Matrix
        // estimated direction of gravity
        // H = 2*g* [-X(3),  X(4), -X(1),  X(2), 0, 0, 0;
        //            X(2),  X(1),  X(4),  X(3), 0, 0, 0;
        //            X(1), -X(2), -X(3),  X(4), 0, 0, 0;
        //             0  ,   0  ,   0  ,   0  , 0, 0, 0];
        double H_arr[][] = {{-X[2],  X[3], -X[0], X[1], 0, 0, 0},
                            { X[1],  X[0] , X[3], X[2], 0, 0, 0},
                            { X[0], -X[1], -X[2], X[3], 0, 0, 0},
                            {0    ,0     ,     0,    0, 0, 0, 0}};
        H = Matrix.Factory.importFromArray(H_arr);
        H = H.times(2*g);
       
        // State Update equations
        // Residual = h-Z;
        // S = H*P*H' + R; 
        // K = P*H'/S;
        // X = X + K*(Residual);
        // P = (eye(7) - K*H)*P;
        
        // Measurement vector
        double Z_arr[] = {imudat.ax, imudat.ay, imudat.az, gps.courseT};
        Z = Matrix.Factory.importFromArray(Z_arr);
        Z = Z.transpose();
        
        // compute residual = h-z
        Residual = h.minus(Z);
        
        
        while(bias_found<100)
        {
                // calculate bias
                gxbias = (gxbias + (imudat.gx-gxold) )/2;
                gybias = (gybias + (imudat.gy-gyold) )/2;
                gzbias = (gzbias + (imudat.gz-gzold) )/2;
                gxold= imudat.gx;
                gyold= imudat.gy;
                gzold= imudat.gz;
                bias_found++;
        }
        
        S = H.mtimes(Covariance.mtimes(H.transpose())).plus(R);
        
        // compute Kalman gain
        K = Covariance.mtimes(H.transpose().mtimes(S.inv()));    // avoid doing this bit too much!
        
        /*
        System.out.println("S 0: " +S.getAsDouble(0,0));
        System.out.println("S 1: " +S.getAsDouble(1,0));
        System.out.println("S 2: " +S.getAsDouble(2,0));
        System.out.println("S 3: " +S.getAsDouble(3,0));
        
        System.out.println("K 0: " +K.getAsDouble(0,0));
        System.out.println("K 1: " +K.getAsDouble(1,0));
        System.out.println("K 2: " +K.getAsDouble(2,0));
        System.out.println("K 3: " +K.getAsDouble(3,0));
        */
        
        // update state equation
        LocalState = LocalState.plus(K.mtimes(Residual));
        
        
        // compute measurement covariance
        Covariance = I77.minus(K.mtimes(H)).mtimes(Covariance);
        
        
        //printMatrix(LocalState);
        //System.out.println("GPS: " +gps.courseT);
        LocalState = normaliseState(LocalState);
        
        setState(LocalState);
    }
    
    
    public void correct_Accel_Mag()
    {
        Matrix LocalState = this.getState();
        
        // get state vector quat in more concise form
        double X[] = {0,0,0,0};
        X[0] = LocalState.getAsDouble(0,0);
        X[1] = LocalState.getAsDouble(1,0);
        X[2] = LocalState.getAsDouble(2,0);
        X[3] = LocalState.getAsDouble(3,0);
        
        // State relation vector
        // converts state to the same form as the measurements ?
        // h = [2*g*(X(2)*X(4)-X(1)*X(3));
        //  2*g*(X(3)*X(4)+X(1)*X(2));
        //  g*(X(1)^2-X(2)^2-X(3)^2-X(4)^2);
        //  atan2(2*(X(1)*X(4)+X(2)*X(3)), 1 - 2*(X(3)^2+X(4)^2))];
        double[] h_arr = {2*g*(X[1]*X[3]-X[0]*X[2]),
                          2*g*(X[2]*X[3]+X[0]*X[1]),
                          g*( Math.pow(X[0],2)-Math.pow(X[1],2)-Math.pow(X[2],2)-Math.pow(X[3],2) ),
                          Math.atan2( 2*(X[0]*X[3]+X[1]*X[2]),  1-2*(Math.pow(X[2],2)+Math.pow(X[3],2)) )};
        h = DenseMatrix.Factory.importFromArray(h_arr);
        h = h.transpose();
        
        // Measurement Jacobian Matrix
        // estimated direction of gravity
        // H = 2*g* [-X(3),  X(4), -X(1),  X(2), 0, 0, 0;
        //            X(2),  X(1),  X(4),  X(3), 0, 0, 0;
        //            X(1), -X(2), -X(3),  X(4), 0, 0, 0;
        //             0  ,   0  ,   0  ,   0  , 0, 0, 0];
        double H_arr[][] = {{-X[2],  X[3], -X[0], X[1], 0, 0, 0},
                            { X[1],  X[0] , X[3], X[2], 0, 0, 0},
                            { X[0], -X[1], -X[2], X[3], 0, 0, 0},
                            {0    ,0     ,     0,    0, 0, 0, 0}};
        H = Matrix.Factory.importFromArray(H_arr);
        H = H.times(2*g);
       
        // State Update equations
        // Residual = h-Z;
        // S = H*P*H' + R; 
        // K = P*H'/S;
        // X = X + K*(Residual);
        // P = (eye(7) - K*H)*P;
        
        // correct mag and get heading
        // normalise mag
        //imudat.mx = -1*imudat.mx; 
        //imudat.my = -1*imudat.my;
        //imudat.mz = -1*imudat.mz;
        double mag_mag = Math.sqrt(imudat.mx*imudat.mx + imudat.my*imudat.my + imudat.mz*imudat.mz);
        double mx = imudat.mx/mag_mag;
        double my = imudat.my/mag_mag;
        double mz = imudat.mz/mag_mag;
        
        // get euler orientation
        double RPY[] = stateToEuler(LocalState);
        double a = RPY[0]; 
        double b = RPY[1];
        double heading_x = mx*Math.cos(b)+my*Math.sin(b)*Math.sin(a)+mz*Math.sin(b)*Math.cos(a);
        double heading_y = my*Math.cos(a)-mz*Math.sin(a);
        double heading = Math.atan2(-heading_y,heading_x);
        //if(heading<0){heading+=2*Math.PI;}
        //System.out.println("heading x and y: "+heading_x+" "+heading_y);
        double basicheading = Math.atan2(-my, mx);
        
        if(printlog==true)
        {
            //System.out.println("Heading Mag: "+mx+" "+my+" "+mz);
            System.out.println("roll: "+Math.round(RPY[0]*toDegrees)+", Pitch: "+Math.round(RPY[1]*toDegrees)+", Yaw: "+Math.round(RPY[2]*toDegrees));
            //System.out.println("Heading Mag: "+heading*toDegrees + " Heading GPS: "+ gps.courseT + " "+ gps.courseM);
            //System.out.println("speedK: "+String.valueOf(gps.speedKM)+" "+String.valueOf(gps.speedK)+" lat: "+gps.latitude);

            System.out.println("Heading Basic: "+basicheading*toDegrees);
            System.out.println();
        }
        
        // Measurement vector
        double Z_arr[] = {imudat.ax, imudat.ay, imudat.az, heading*toDegrees};//gps.courseT};
        Z = Matrix.Factory.importFromArray(Z_arr);
        Z = Z.transpose();
        
        // compute residual = h-z
        Residual = h.minus(Z);
        
        while(bias_found<100)
        {
                // calculate bias
                gxbias = (gxbias + (imudat.gx-gxold) )/2;
                gybias = (gybias + (imudat.gy-gyold) )/2;
                gzbias = (gzbias + (imudat.gz-gzold) )/2;
                gxold= imudat.gx;
                gyold= imudat.gy;
                gzold= imudat.gz;
                bias_found++;
        }
        
        S = H.mtimes(Covariance.mtimes(H.transpose())).plus(R);
        
        // compute Kalman gain
        K = Covariance.mtimes(H.transpose().mtimes(S.inv()));    // avoid doing this bit too much!
        
        /*
        System.out.println("S 0: " +S.getAsDouble(0,0));
        System.out.println("S 1: " +S.getAsDouble(1,0));
        System.out.println("S 2: " +S.getAsDouble(2,0));
        System.out.println("S 3: " +S.getAsDouble(3,0));
        
        System.out.println("K 0: " +K.getAsDouble(0,0));
        System.out.println("K 1: " +K.getAsDouble(1,0));
        System.out.println("K 2: " +K.getAsDouble(2,0));
        System.out.println("K 3: " +K.getAsDouble(3,0));
        */
        
        // update state equation
        LocalState = LocalState.plus(K.mtimes(Residual));
        
        // compute measurement covariance
        Covariance = I77.minus(K.mtimes(H)).mtimes(Covariance);
        
        //printMatrix(LocalState);
        //System.out.println("GPS: " +gps.courseT);
        LocalState = normaliseState(LocalState);
        
        setState(LocalState);
        //this.State = LocalState;
        
        // trying to fix the random screw up
        //System.out.println("local state: "+stateToEuler(getState())[1]);
    }
    
    /*
    private void computeKalmanGain
    {
        // compute Kalman gain
        K = Covariance.mtimes(H.transpose().mtimes(S.inv()));    // avoid doing this bit too much!
        
    }
    */
    
    // column vector in, columns vector out
    private Matrix normaliseState(Matrix m)
    {
        Matrix q = DenseMatrix.Factory.zeros(7, 1);
        double qMag = Math.sqrt(Math.pow(m.getAsDouble(0,0),2) +
                                Math.pow(m.getAsDouble(1,0),2) +
                                Math.pow(m.getAsDouble(2,0),2) +
                                Math.pow(m.getAsDouble(3,0),2) );
        q.setAsDouble(m.getAsDouble(0,0)/qMag, 0, 0);
        q.setAsDouble(m.getAsDouble(1,0)/qMag, 1, 0);
        q.setAsDouble(m.getAsDouble(2,0)/qMag, 2, 0);
        q.setAsDouble(m.getAsDouble(3,0)/qMag, 3, 0);
        q.setAsDouble(m.getAsDouble(4,0) ,4 , 0);       // pass through
        q.setAsDouble(m.getAsDouble(5,0) ,5 , 0);       // pass through
        q.setAsDouble(m.getAsDouble(6,0) ,6 , 0);       // pass through
        return q;
    }
    
    /*
     *
     * This gets the quaternion as Euler
    */
    private double[] stateToEuler(Matrix m)
    {
        double Q[] = {0.0,0.0,0.0};
        //PosQasE(1) = atan2(2*(PosQ(3)*PosQ(4)+PosQ(1)*PosQ(2)), (1-2*((PosQ(2)^2)+(PosQ(3)^2))));
        //PosQasE(2) = -asin(2*(PosQ(2)*PosQ(4)-PosQ(1)*PosQ(3)));
        //PosQasE(3) = atan2(2*(PosQ(2)*PosQ(3)+PosQ(1)*PosQ(4)), (1-2*((PosQ(3)^2)+(PosQ(4)^2))));
    
        Q[0] = Math.atan2(2*(m.getAsDouble(2,0)*m.getAsDouble(3,0)+m.getAsDouble(0,0)*m.getAsDouble(1,0)), (1-2*(Math.pow(m.getAsDouble(1,0),2)+Math.pow(m.getAsDouble(2,0),2))));
        Q[1] = -Math.asin(2*(m.getAsDouble(1,0)*m.getAsDouble(3,0)-m.getAsDouble(0,0)*m.getAsDouble(2,0)));
        Q[2] = Math.atan2(2*(m.getAsDouble(1,0)*m.getAsDouble(2,0)+m.getAsDouble(0,0)*m.getAsDouble(3,0)), (1-2*(Math.pow(m.getAsDouble(2,0),2)+Math.pow(m.getAsDouble(3,0),2))));
        
        return Q;
    }  
    
    /*
    * Rotate vector A by vector B
    * Useful for correcting orientation of IMU etc
    */
    private double[] rotateEulerVector(Matrix A, Matrix B)
    {
        double Q[] = {0.0,0.0,0.0};
        return Q;
    }
    
    /*
    * Take Euler angle A and return the quaternion form
    * q0= cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2)
    * q1= sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2)
    * q2= cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2)
    * q3= cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2)
    */
    private static Matrix EulerToQuaternion(Matrix A)
    {
        return A;
    }
    
    
    /*
    * Rotate quaternio A by quaternion B - untested
      A and B both col vectors
    
      w -z  y  x
      z  w -x  y
     -y  x  w  z
     -x -y -z  w
    */
    private static Matrix RotateQuaternion(Matrix A, Matrix B)
    {
        // make sure they're col vectors
        if(A.getRowCount()>1){A.transpose();}
        if(B.getRowCount()>1){B.transpose();}
        
        // form the rotation matrix from B
        // this can be done faster with dot products
        Matrix R = DenseMatrix.Factory.zeros(A.getRowCount(), A.getColumnCount());
        R.setAsDouble(B.getAsDouble(0,0),0,0); // row 0
        R.setAsDouble(-B.getAsDouble(3,0),0,1);
        R.setAsDouble(B.getAsDouble(2,0),0,2);
        R.setAsDouble(B.getAsDouble(1,0),0,3);
        R.setAsDouble(B.getAsDouble(3,0),1,0); // row 1
        R.setAsDouble(B.getAsDouble(0,0),1,1);
        R.setAsDouble(-B.getAsDouble(1,0),1,2);
        R.setAsDouble(B.getAsDouble(2,0),1,3);
        R.setAsDouble(-B.getAsDouble(2,0),2,0); // row 2
        R.setAsDouble(B.getAsDouble(1,0),2,1);
        R.setAsDouble(B.getAsDouble(0,0),2,2);
        R.setAsDouble(B.getAsDouble(3,0),2,3);
        R.setAsDouble(-B.getAsDouble(1,0),0,0); // row 3
        R.setAsDouble(-B.getAsDouble(2,0),0,0);
        R.setAsDouble(-B.getAsDouble(3,0),0,0);
        R.setAsDouble(B.getAsDouble(0,0),0,0);
        
        R = R.mtimes(A);
        
        return R;
    }
    
    /*
    * Convert mag X,Y to heading 
    * doesnt adjust for pitch and roll tilt
    */
    public static double Magto2DYaw(double magX, double magY)
    {
        double heading = 0;
        heading = Math.atan2(magY, magX)*toDegrees;
        
        return heading;
    }
    
    public static void printMatrix(Matrix m)
    {
        long r = m.getRowCount(); long c = m.getColumnCount();
        
        System.out.println("\n");
        
        long ri; long ci;
        for(ri=0;ri<r;ri++)
        {
            for(ci=0;ci<c;ci++)
            {
                if(ci==c-1)
                {
                    System.out.print((float)m.getAsDouble(ri,ci)+ "\n");
                }
                else
                {
                    System.out.print((float)m.getAsDouble(ri,ci)+ " ");
                } 
            }//col
        }//row
    }
    
    public class EKFData 
    {
        // kinematics
        private double roll;
        private double pitch;
        private double yaw;
        private double velX;
        private double velY;
        private double velZ;
        // geospatial
        private double GPS_headingT;
        private double latitude;
        private double longitude;
        private double altitude;
        private double mag_heading;
    } 
    
    public EKFData getEKFData()
    {
        EKFData ret = new EKFData();
        
        try
        {
            if(EKFDataLock.tryLock(10,TimeUnit.MICROSECONDS))
            {
                // copy data from local data structure to external
                ret.roll = ekfdat.roll;
                ret.pitch = ekfdat.pitch;
                ret.yaw = ekfdat.yaw;
                ret.velX = ekfdat.velX;
                ret.velY = ekfdat.velY;
                ret.velZ = ekfdat.velZ;
                ret.latitude = ekfdat.latitude;
                ret.longitude = ekfdat.longitude;
                ret.GPS_headingT = ekfdat.GPS_headingT;
                ret.mag_heading = ekfdat.mag_heading;
                EKFDataLock.unlock();
            }
        }
        catch(InterruptedException iex)
        {
            iex.printStackTrace();
        }
        return ret;
    }
    
    public void setEKFData(EKFData in)
    {
        try
        {
            if(EKFDataLock.tryLock(10,TimeUnit.MICROSECONDS))
            {
                // copy data from local data structure to external
                ekfdat.roll = in.roll;
                ekfdat.pitch = in.pitch;
                ekfdat.yaw = in.yaw;
                ekfdat.velX = in.velX;
                ekfdat.velY = in.velY;
                ekfdat.velZ = in.velZ;
                ekfdat.latitude = in.latitude;
                ekfdat.longitude = in.longitude;
                ekfdat.GPS_headingT = in.GPS_headingT;
                ekfdat.mag_heading = in.mag_heading;
                EKFDataLock.unlock();
            }
        }
        catch(InterruptedException iex)
        {
            iex.printStackTrace();
        }
    }
}
