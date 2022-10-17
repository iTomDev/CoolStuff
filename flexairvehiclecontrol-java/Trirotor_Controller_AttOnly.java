/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

import org.ujmp.core.DenseMatrix;
import org.ujmp.core.Matrix;

/**
 *
 * @author SpyroRawr
 * Attitude only controller for Trirotor. Is just the controller formula by Kara Mhd
 * Attitude only to make it easier to debug
 */
public class Trirotor_Controller_AttOnly implements Runnable
{
    // supporting
    private boolean printlog = true;
    public boolean killthread = false;
    public boolean running = false;
    
    // external data objects
    private EKF ekf = null;
    private Servo servo  = null;
    
    // matrices
    private Matrix Psi;
    private Matrix Jinv;
    private Matrix DPsi;
    private Matrix J;
    private Matrix Ht;
    private Matrix Hf;
    private Matrix S;
    private Matrix Temp1;
    private Matrix Beta1;
    private Matrix Beta1inv;
    private Matrix KDM;     // replaces Ht, Hf
    
    // vectors
    private Matrix u;       // motor rates vector - This is to be computed
    private Matrix pqr;     // angular rate derivatives
    private Matrix rpy;     // angular rates
    
    // scalars
    private double length = 0.5; // arm length, meters
    private double Kf = 0.004;       // Newtons/RPM. For us that is 0.8N/180 at a guess
    private double Jxx=0.005; 
    private double Jyy=0.005; 
    private double Jzz=0.005;
    private double g = 9.8;
    private double mass = 0.7;
    private double toRadians = 0.01745329251994329576923690768489;
    
    public void run()
    {
        while(killthread==false)
        {   
            // update altitude
                servo.serial.requestUltrasonicHeight();
                
            while(running==true)
            {
                
                
                // get angles from imu0
                double[] rpy_arr = ekf.getStateAsEuler();
                rpy_arr[0] = rpy_arr[0]*toRadians;
                rpy_arr[1] = rpy_arr[1]*toRadians;
                rpy_arr[2] = rpy_arr[2]*toRadians;     
                rpy = Matrix.Factory.importFromArray(rpy_arr);
                rpy = rpy.transpose();
                // set desired angle
                double rpydes_arr[] = {0.0,0.1,0};
                Matrix rpydes = Matrix.Factory.importFromArray(rpydes_arr);
                rpydes = rpydes.transpose();
                // fake angular rates
                double pqr_arr[] = {0.2,0,0};
                pqr = Matrix.Factory.importFromArray(pqr_arr);
                pqr = pqr.transpose();
                
                // update inputs
                
                Matrix u = inverseController(rpydes, rpy, pqr);
                
                System.out.println("rpy: "+rpy_arr[0]+" "+rpy_arr[1]+" "+rpy_arr[2]);
                //System.out.println("rpy desired: "+rpydes_arr[0]+" "+rpydes_arr[1]+" "+rpydes_arr[2]);
                //System.out.println("Computed u: "+u.toDoubleArray()[0][0]+" "+u.toDoubleArray()[1][0]+" "+u.toDoubleArray()[0][0]);
                
                
                
                try
                {
                    Thread.sleep(100);
                }
                catch(InterruptedException iex){iex.printStackTrace();}
            }
        }
    }
    
    public void init(EKF ekfin, Servo servoin)
    {
        // external sources
        ekf = ekfin;
        servo = servoin;
        
        // initialise variable matrices
        J = DenseMatrix.Factory.zeros(3, 3);
        Jinv = DenseMatrix.Factory.zeros(3, 3);
        S = DenseMatrix.Factory.eye(3, 3);
        u = DenseMatrix.Factory.zeros(3, 1);
        pqr = DenseMatrix.Factory.zeros(3, 1);
        Psi= DenseMatrix.Factory.zeros(3, 3);
        DPsi= DenseMatrix.Factory.zeros(3, 3);
        KDM = DenseMatrix.Factory.zeros(3, 4);
        
        // Inertia matrix and its inverse
        double J_arr[][] = {{Jxx,0,0},{0,Jyy,0},{0,0,Jzz}};
        J = Matrix.Factory.importFromArray(J_arr);
        Jinv = J.inv();
        
        // compute fixed matrices
        // Rotate thrust into body vector. This is not the thrust deflection angle
        double Ht_arr[][] = {{ 0, Math.sqrt(3)/2, Math.sqrt(3)/2}, 
                             {-1, 1/2,            1/2           },
                             { 0,   0,            0.1             }};   // added 3,3 - it should be zero
        Ht = Matrix.Factory.importFromArray(Ht_arr);
        Ht = Ht.times(length);
        
        if(printlog==true)
        {
            System.out.println("");
            System.out.println("Ht matrix");
            System.out.println(Ht.toDoubleArray()[0][0]+" "+Ht.toDoubleArray()[0][1]+" "+Ht.toDoubleArray()[0][2]);
            System.out.println(Ht.toDoubleArray()[1][0]+" "+Ht.toDoubleArray()[1][1]+" "+Ht.toDoubleArray()[2][2]);
            System.out.println(Ht.toDoubleArray()[2][0]+" "+Ht.toDoubleArray()[2][1]+" "+Ht.toDoubleArray()[2][2]);
            System.out.println("Determinant: "+Ht.det());
        }
        
    }
    
    // takes desired output "y"
    // returns control "u" to obtain that desire "y"
    // PQR: 3x1 col vector
    // RPY: 3x1 col vector
    // RPYdes: 3x1 col vector
    public Matrix inverseController(Matrix rpydes, Matrix rpy, Matrix pqr)
    {
        if(rpydes==null)
        {
            //rpydes = DenseMatrix.Factory.zeros(3,1);
        }
        u = DenseMatrix.Factory.zeros(3, 1); // computed rotor angular rates
        
        // fedback linearisation law
        // psi
        // rpy: 0=roll, 1=pitch, 2=yaw
        Psi.setAsDouble(1, 0,0);                                                                // row 0
        Psi.setAsDouble(Math.sin(rpy.getAsDouble(0,0))*Math.tan(rpy.getAsDouble(1,0)), 0,1);
        Psi.setAsDouble(Math.cos(rpy.getAsDouble(0,0))*Math.tan(rpy.getAsDouble(1,0)), 0,2);
        Psi.setAsDouble(0, 1,0);                                                                // row 1
        Psi.setAsDouble(Math.cos(rpy.getAsDouble(0,0)),1,1);
        Psi.setAsDouble(-Math.sin(rpy.getAsDouble(0,0)),1,2);
        Psi.setAsDouble(0, 2,0);                                                                // row 2
        Psi.setAsDouble(Math.sin(rpy.getAsDouble(0,0))*1/Math.cos(rpy.getAsDouble(1,0)), 2,1);  // << Here be dragons
        Psi.setAsDouble(Math.sin(rpy.getAsDouble(0,0))*1/Math.cos(rpy.getAsDouble(1,0)), 2,2);  // << Here be dragons
        
        if(printlog==true)
        {
            // print beta
            System.out.println("");
            System.out.println("Psi matrix");
            System.out.println(Psi.toDoubleArray()[0][0]+" "+Psi.toDoubleArray()[0][1]+" "+Psi.toDoubleArray()[0][2]);
            System.out.println(Psi.toDoubleArray()[1][0]+" "+Psi.toDoubleArray()[1][1]+" "+Psi.toDoubleArray()[2][2]);
            System.out.println(Psi.toDoubleArray()[2][0]+" "+Psi.toDoubleArray()[2][1]+" "+Psi.toDoubleArray()[2][2]);
        }
        
        // Psi derivative (might be wrong)
        DPsi.setAsDouble(0, 0,0);                                                               // row 0
        DPsi.setAsDouble(Math.cos(rpy.getAsDouble(0,0))*Math.tan(rpy.getAsDouble(1,0)), 0,1);
        DPsi.setAsDouble(-Math.sin(rpy.getAsDouble(0,0))*Math.tan(rpy.getAsDouble(1,0)), 0,2);  
        DPsi.setAsDouble(0, 1,0);                                                               // row 1
        DPsi.setAsDouble(-Math.sin(rpy.getAsDouble(0,0)),1,1);
        DPsi.setAsDouble(-Math.cos(rpy.getAsDouble(0,0)),1,2);
        DPsi.setAsDouble(0, 2,0);                                                               // row 2
        DPsi.setAsDouble(Math.cos(rpy.getAsDouble(0,0))/Math.cos(rpy.getAsDouble(1,0)), 2,1);
        DPsi.setAsDouble(-Math.sin(rpy.getAsDouble(0,0))/Math.cos(rpy.getAsDouble(1,0)), 2,2);
        
        /*
        if(printlog==true)
        {
            // print beta
            System.out.println("");
            System.out.println("Derivative of Psi matrix");
            System.out.println(DPsi.toDoubleArray()[0][0]+" "+DPsi.toDoubleArray()[0][1]+" "+DPsi.toDoubleArray()[0][2]);
            System.out.println(DPsi.toDoubleArray()[1][0]+" "+DPsi.toDoubleArray()[1][1]+" "+DPsi.toDoubleArray()[2][2]);
            System.out.println(DPsi.toDoubleArray()[2][0]+" "+DPsi.toDoubleArray()[2][1]+" "+DPsi.toDoubleArray()[2][2]);
            System.out.println("Determinant: "+DPsi.det());
        }
        */
        
        // skew symmetric matrix - skipping zeros
        S.setAsDouble(-pqr.getAsDouble(2,0) ,0 , 1); // row 0
        S.setAsDouble(pqr.getAsDouble(1,0) ,0 , 2);
        S.setAsDouble(pqr.getAsDouble(2,0) ,1 , 0); // row 1
        S.setAsDouble(-pqr.getAsDouble(0,0) ,1 , 2);
        S.setAsDouble(-pqr.getAsDouble(1,0) ,2 , 0); // row 2
        S.setAsDouble(pqr.getAsDouble(0,0) ,2 , 1);
        
        // Temp1 = DPsi-Psi*Jinv*S*J*omega;
        Temp1= DPsi.minus(Psi.mtimes(Jinv.mtimes(S).mtimes(J))).mtimes(pqr);
        
        /*
        Matrix Temp2;
        Temp2 = S.mtimes(J);
        Temp2 = Jinv.mtimes(Temp2);
        Temp2 = Psi.mtimes(Temp2);
        
        if(printlog==true)
        {
            // print beta
            System.out.println("");
            System.out.println("Temp2 matrix");
            System.out.println(Temp2.toDoubleArray()[0][0]+" "+Temp2.toDoubleArray()[0][1]+" "+Temp2.toDoubleArray()[0][2]);
            System.out.println(Temp2.toDoubleArray()[1][0]+" "+Temp2.toDoubleArray()[1][1]+" "+Temp2.toDoubleArray()[2][2]);
            System.out.println(Temp2.toDoubleArray()[2][0]+" "+Temp2.toDoubleArray()[2][1]+" "+Temp2.toDoubleArray()[2][2]);
        }*/
        
        /*
        if(printlog==true)
        {
            // print beta
            System.out.println("");
            System.out.println("temp1 vector");
            System.out.println(Temp1.toDoubleArray()[0][0]);
            System.out.println(Temp1.toDoubleArray()[1][0]);
            System.out.println(Temp1.toDoubleArray()[2][0]);
        }
        */
        /*
        if(printlog==true)
        {
            // print beta
            System.out.println("");
            System.out.println("pqr vector");
            System.out.println(pqr.toDoubleArray()[0][0]);
            System.out.println(pqr.toDoubleArray()[1][0]);
            System.out.println(pqr.toDoubleArray()[2][0]);
        }
        */
        // compute Beta, the control input part
        //Beta1 = Ht.times(Kf);
        //Beta1 = Psi.mtimes(Jinv.mtimes(Ht.times(Kf)));
        Beta1 = Matrix.Factory.eye(3,3);
        Beta1 = Jinv.mtimes(Beta1);
        Beta1 = Psi.mtimes(Beta1);
        Beta1 = Beta1.mtimes(Ht.times(Kf));
        
        
        if(printlog==true)
        {
            // print beta
            System.out.println("");
            System.out.println("Beta1 matrix");
            System.out.println(Beta1.toDoubleArray()[0][0]+" "+Beta1.toDoubleArray()[0][1]+" "+Beta1.toDoubleArray()[0][2]);
            System.out.println(Beta1.toDoubleArray()[1][0]+" "+Beta1.toDoubleArray()[1][1]+" "+Beta1.toDoubleArray()[2][2]);
            System.out.println(Beta1.toDoubleArray()[2][0]+" "+Beta1.toDoubleArray()[2][1]+" "+Beta1.toDoubleArray()[2][2]);
            System.out.println("Determinant: "+Beta1.det());
        }
        
        
        // invert Beta
        Beta1inv = Beta1.pinv();
        
        
        if(printlog==true)
        {
            // print beta
            System.out.println("");
            System.out.println("Beta1 inverse matrix");
            System.out.println(Beta1inv.toDoubleArray()[0][0]+" "+Beta1inv.toDoubleArray()[0][1]+" "+Beta1inv.toDoubleArray()[0][2]);
            System.out.println(Beta1inv.toDoubleArray()[1][0]+" "+Beta1inv.toDoubleArray()[1][1]+" "+Beta1inv.toDoubleArray()[2][2]);
            System.out.println(Beta1inv.toDoubleArray()[2][0]+" "+Beta1inv.toDoubleArray()[2][1]+" "+Beta1inv.toDoubleArray()[2][2]);
        }
        
        // compute  u
        u = Beta1inv.mtimes(rpydes.minus(Temp1));
        
        if(printlog==true)
        {
            System.out.println("Computed u: "+u.toDoubleArray()[0][0]+" "+u.toDoubleArray()[1][0]+" "+u.toDoubleArray()[0][0]);
        } 
        
        // clear data
        Beta1 = null;
        Temp1 = null;
        Psi= DenseMatrix.Factory.zeros(3, 3);
        
        return u;
    }
    
    /*
    void testInverseController()
    {
        // Test Kara inverse control algorithm, angules only
        Trirotor_Controller_AttOnly control1 = new Trirotor_Controller_AttOnly();
        control1.init();
        // test matrices
        // desired angle
        double rpydes_arr[] = {0.2,0,0};
        Matrix rpydes = Matrix.Factory.importFromArray(rpydes_arr);
        rpydes = rpydes.transpose();
        // current angle
        double rpy_arr[] = {0.1,0,0};
        Matrix rpy = Matrix.Factory.importFromArray(rpy_arr);
        rpy = rpy.transpose();
        // current angular rates
        double pqr_arr[] = {0,0,0};
        Matrix pqr = Matrix.Factory.importFromArray(pqr_arr);
        pqr = pqr.transpose();
        // compute
        Matrix u;
        u = control1.inverseController(rpydes, rpy, pqr);
        
        
        
        System.out.println("rpy: "+rpy_arr[0]+" "+rpy_arr[1]+" "+rpy_arr[2]);
        System.out.println("rpy desired: "+rpydes_arr[0]+" "+rpydes_arr[1]+" "+rpydes_arr[2]);
        System.out.println("Computed u: "+u.toDoubleArray()[0][0]+" "+u.toDoubleArray()[1][0]+" "+u.toDoubleArray()[0][0]);
    }
    */
    
    // takes desired output "y"
    // returns control "u" to obtain that desire "y"
    // Uses small angle approximation so d/rpy = pqr which isnt strictly accurate
    // PQR: 3x1 col vector
    // RPY: 3x1 col vector
    // RPYdes: 3x1 col vector
    public Matrix inverseController2(Matrix rpydes, Matrix rpy, Matrix pqr)
    {
        if(rpydes==null)
        {
            //rpydes = DenseMatrix.Factory.zeros(3,1);
        }
        u = DenseMatrix.Factory.zeros(3, 1); // computed rotor angular rates
        
        // form KDM, this is actually static
        KDM.setAsDouble(0 , 0, 0);
        KDM.setAsDouble(Math.sqrt(3)/2 , 0, 1);     // row 0
        KDM.setAsDouble(-Math.sqrt(3)/2 , 0, 2);
        KDM.setAsDouble(0, 0, 3);
        KDM.setAsDouble(-1, 1, 0);                  // row 1
        KDM.setAsDouble(1/2, 1, 1);
        KDM.setAsDouble(1/2, 1, 2);
        KDM.setAsDouble(0, 1, 3);
        KDM.setAsDouble(1, 2, 0);                   // row 2
        KDM.setAsDouble(-1, 2, 1);
        KDM.setAsDouble(1, 2, 2);
        KDM.setAsDouble(-1, 2, 3);
        
        // skew symmetric matrix - skipping zeros
        S.setAsDouble(-pqr.getAsDouble(2,0) ,0 , 1); // row 0
        S.setAsDouble(pqr.getAsDouble(1,0) ,0 , 2);
        S.setAsDouble(pqr.getAsDouble(2,0) ,1 , 0); // row 1
        S.setAsDouble(-pqr.getAsDouble(0,0) ,1 , 2);
        S.setAsDouble(-pqr.getAsDouble(1,0) ,2 , 0); // row 2
        S.setAsDouble(pqr.getAsDouble(0,0) ,2 , 1);
        
        // Temp1 = DPsi-Psi*Jinv*S*J*omega;
        //Temp1= DPsi.minus(Psi.mtimes(Jinv.mtimes(S).mtimes(J))).mtimes(pqr);
        Temp1 = Jinv.mtimes(S.mtimes(J)).mtimes(rpy);
        
        // compute Beta, the control input part
        //Beta1 = Ht.times(Kf);
        //Beta1 = Psi.mtimes(Jinv.mtimes(Ht.times(Kf)));
        Beta1 = Jinv.mtimes(KDM);
        // invert Beta
        Beta1inv = Beta1.pinv();
        
        
        /*
        if(printlog==true)
        {
            // print beta
            System.out.println("");
            System.out.println("Temp1 matrix");
            System.out.println(Temp1.toDoubleArray()[0][0]+" "+Temp1.toDoubleArray()[0][1]+" "+Temp1.toDoubleArray()[0][2]);
            System.out.println(Temp1.toDoubleArray()[1][0]+" "+Temp1.toDoubleArray()[1][1]+" "+Temp1.toDoubleArray()[2][2]);
            System.out.println(Temp1.toDoubleArray()[2][0]+" "+Temp1.toDoubleArray()[2][1]+" "+Temp1.toDoubleArray()[2][2]);
        }*/
        
        /*
        if(printlog==true)
        {
            // print beta
            System.out.println("");
            System.out.println("temp1 vector");
            System.out.println(Temp1.toDoubleArray()[0][0]);
            System.out.println(Temp1.toDoubleArray()[1][0]);
            System.out.println(Temp1.toDoubleArray()[2][0]);
        }
        */
        /*
        if(printlog==true)
        {
            // print beta
            System.out.println("");
            System.out.println("pqr vector");
            System.out.println(pqr.toDoubleArray()[0][0]);
            System.out.println(pqr.toDoubleArray()[1][0]);
            System.out.println(pqr.toDoubleArray()[2][0]);
        }
        */
        
        
        if(printlog==true)
        {
            // print beta
            System.out.println("");
            System.out.println("Beta1 matrix");
            System.out.println(Beta1.toDoubleArray()[0][0]+" "+Beta1.toDoubleArray()[0][1]+" "+Beta1.toDoubleArray()[0][2]);
            System.out.println(Beta1.toDoubleArray()[1][0]+" "+Beta1.toDoubleArray()[1][1]+" "+Beta1.toDoubleArray()[2][2]);
            System.out.println(Beta1.toDoubleArray()[2][0]+" "+Beta1.toDoubleArray()[2][1]+" "+Beta1.toDoubleArray()[2][2]);
            System.out.println("Determinant: "+Beta1.det());
        }

        if(printlog==true)
        {
            // print beta
            System.out.println("");
            System.out.println("Beta1 inverse matrix");
            System.out.println(Beta1inv.toDoubleArray()[0][0]+" "+Beta1inv.toDoubleArray()[0][1]+" "+Beta1inv.toDoubleArray()[0][2]);
            System.out.println(Beta1inv.toDoubleArray()[1][0]+" "+Beta1inv.toDoubleArray()[1][1]+" "+Beta1inv.toDoubleArray()[2][2]);
            System.out.println(Beta1inv.toDoubleArray()[2][0]+" "+Beta1inv.toDoubleArray()[2][1]+" "+Beta1inv.toDoubleArray()[2][2]);
        }
        
        // compute  u
        u = Beta1inv.mtimes(rpydes.minus(Temp1));
        
        if(printlog==true)
        {
            System.out.println("Computed u: "+u.toDoubleArray()[0][0]+" "+u.toDoubleArray()[1][0]+" "+u.toDoubleArray()[0][0]);
        } 
        
        // clear data
        Beta1 = null;
        Temp1 = null;
        Psi= DenseMatrix.Factory.zeros(3, 3);
        
        return u;
    }
}

