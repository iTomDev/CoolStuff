/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

import org.ujmp.core.DenseMatrix;
import org.ujmp.core.Matrix;
import org.ujmp.core.calculation.Calculation;

/**
 *
 * @author NeonGreen
 */
public class Trirotor_Controller implements Runnable
{
    // PI control
    double PI_Ref[] = new double[3];
    double PI_Meas[] = new double[3];
    double PI_Err[] = new double[3];
    double PI_Int[] = new double[3];
    double PI_Output[] = new double[3];
    double PI_WProp = 5;
    double PI_WInt = 2;
    double dt = 0.1; // sampling period
    private static final double toDegrees = 57.2957795; //180/Math.PI;
    
    // objects
    private EKF ekf = null;
    private Servo servo  = null;
    
    int m1 = 60;
    int m2 = 60;
    int m3 = 60;
    
    public void init(EKF ekfin, Servo servoin)
    {
        // PI controller
        PI_Ref[0] = 0; PI_Ref[1] = 0; PI_Ref[2] = 0; 
        PI_Meas[0] = 0; PI_Meas[1] = 0; PI_Meas[2] = 0;
        
        ekf = ekfin;
        servo = servoin;
    }
    
    public void run()
    {
        while(true)
        {
            //if(servo.PI_Has_Controls==true)
            {
                ComputePI();
                try
                {
                    Thread.sleep(100);
                }
                catch(InterruptedException iex){iex.printStackTrace();}
            }
        }
    }
    
    /*
    * 
    */
    public void ComputePI()
    {
        // use 4 angles to determine the relative angle of each prop
        
        //if(servo.PI_Has_Controls)
        
            /*
        for(int i=0;i<3;i++)
        {
            // compute the error from where it is and should be
            PI_Err[i] = PI_Meas[i] - PI_Ref[i];

            // compute weighted integral of error
            PI_Int[i] = PI_Int[i] + PI_Err[i]*dt;
            
            // final correction for each prop
            PI_Output[i] = PI_WProp*PI_Err[i] + PI_WInt*PI_Int[i];
        }
        //PI_Output[i] are the angles to aimed for by the motors
        
        // have to compute the output for the 3 props from the three angles
        // Ideally this would use inversion to run the model in reverse
        // Here we just want to keep it flat, not actually control it yet
        //if(pitch>10){MC--;}
        //if(roll>10){B--;A++}
        //if(roll<-10){A++;B--}
        
        */
        
        int threshold = 3;
        
        // get state as euler
        double euler[] = new double[3];
        euler = ekf.getStateAsEuler();
        
        /*
        // convert rad to deg
        euler[0] *= toDegrees;
        euler[1] *= toDegrees;
        euler[2] *= toDegrees;
        */
        
        // switch roll and pitch to align
        double temp = euler[1];
        euler[1] = euler[0];
        euler[0] = temp;
        
        System.out.println("Trirotor Roll: "+euler[0]);
        System.out.println("Trirotor Pitch: "+euler[1]);
        System.out.println("Trirotor Yaw: "+euler[2]);
        System.out.println("");
        
        // roll correction
        if(euler[0]<-threshold)
        {
            servo.setServoSTOVLConfig("ThrustRight", m2--); 
            servo.setServoSTOVLConfig("ThrustLeft", m1++);
        }
        if(euler[0]>threshold)
        {
            servo.setServoSTOVLConfig("ThrustRight", m2++); 
            servo.setServoSTOVLConfig("ThrustLeft", m1--);
        }
        
        // pitch correct
        if(euler[1]<-threshold)
        {
            servo.setServoSTOVLConfig("ThrustFront", m3++);
            servo.setServoSTOVLConfig("ThrustRight", m2--); 
            servo.setServoSTOVLConfig("ThrustLeft", m1--);
        }
        if(euler[1]>threshold)
        {
            servo.setServoSTOVLConfig("ThrustFront", m3--);
            servo.setServoSTOVLConfig("ThrustRight", m2++); 
            servo.setServoSTOVLConfig("ThrustLeft", m1++);
        }
        
        // limit
        if(m1>170){m1=170;} 
        if(m1<0){m1=0;}
        if(m2>170){m2=170;} 
        if(m2<0){m2=0;}
        if(m3>170){m3=170;} 
        if(m3<0){m3=0;}
    }
    
    /*
    * Compute the input required for a desired output
    */
    public double[] ComputeInverseController(double[] desiredOutput)
    {
        double[] requiredInput = new double[3];
        requiredInput[0] = 0; requiredInput[1] = 0; requiredInput[2] = 0;
        
        // inverse model! 
        double Ixx = 0;
        double Iyy = 0;
        double Izz = 0;
        double Ixz = 0;
        double x[] = {0,0,0, 0,0,0};
        double u[] = {0,0,0};
        
        // example triotor dynamic model. Relates thruster input to euler angles
        // from 
        // zero indexed here
        double roll = ((Iyy-Izz)/Ixx)*x[3]*x[5] + (Ixz/Ixx)*x[5] + (Ixz/Ixx)*x[1]*x[3] + u[0]/Ixx;
        double pitch = ((Izz-Ixx)/Iyy)*x[1]*x[5] + (Ixz/Iyy)*(Math.pow(x[5],2)-Math.pow(x[1],2)) + u[1]/Iyy;
        double yaw = ((Ixx-Iyy)/Izz)*x[1]*x[3] + (Ixz/Izz)*x[3]*x[5] + u[2]/Izz;
        
        
        
        return requiredInput;
    }
    
    
    // matrices
    private Matrix J;
    private Matrix Ht;
    private Matrix Hg;
    private Matrix Hf;
    private Matrix Jinv;
    private Matrix S;
    private Matrix R;
    // inv law
    private Matrix Psi;
    private Matrix DPsi;
    private Matrix Beta;
    private Matrix Beta1;
    private Matrix Beta2;
    private Matrix Betainv;
    // vectors
    private Matrix p;       // motor rates vector
    private Matrix pqr;     // angular rate derivatives
    private Matrix rpy;     // angular rates
    private Matrix v;
    private Matrix u;
    private Matrix Temp;
    private Matrix Temp1;
    private Matrix Temp2;
    // specs of the model
    private double length = 1; // arm length, meters
    private double Kf = 1;
    private double Jxx=0.5; 
    private double Jyy=0.5; 
    private double Jzz=0.5;
    private double g = 9.8;
    private double mass = 0.7;
        
    /* 
    Inverse controller based on the Trirotor controller by Kara Mohammed at UoM
    + Differs in that no rotor deflection is considered, dynamics rewritten to
        reflect that. This is used for testing with the fixed rotor uav
    + 
    */
    public void initKaraInverseController_V1()
    {
        // initialise variable matrices
        J = DenseMatrix.Factory.zeros(3, 3);
        Jinv = DenseMatrix.Factory.zeros(3, 3);
        S = DenseMatrix.Factory.eye(3, 3);
        p = DenseMatrix.Factory.zeros(3, 1);
        pqr = DenseMatrix.Factory.zeros(3, 1);
        Psi= DenseMatrix.Factory.zeros(3, 3);
        R = DenseMatrix.Factory.zeros(3, 3);
        Hg = DenseMatrix.Factory.zeros(3, 1);
        v = DenseMatrix.Factory.zeros(6, 1);
        u = DenseMatrix.Factory.zeros(6, 1);
        
        // compute fixed matrices
        // Rotate thrust into body vector. This is not the thrust deflection angle
        double Ht_arr[][] = {{ 0, Math.sqrt(3)/2, Math.sqrt(3)/2}, 
                             {-1, 1/2,            1/2           },
                             { 0,   0,            0             }};
        Ht = Matrix.Factory.importFromArray(Ht_arr);
        Ht = Ht.times(length);
        
        // Inertia matrix and its inverse
        double J_arr[][] = {{Jxx,0,0},{0,Jyy,0},{0,0,Jzz}};
        J = Matrix.Factory.importFromArray(J_arr);
        Jinv = J.inv();
        
    }
    
    public Matrix updateKaraInverseController_V1(Matrix pqr)
    {
        //
        
        // update matrices
        // motor rotation velocity 
        p.setAsDouble(Math.pow(m1,2) ,0 , 0);
        p.setAsDouble(Math.pow(m2,2) ,0 , 0);
        p.setAsDouble(Math.pow(m3,2) ,0 , 0);
        
        // set to the current roll, pitch, yaw rate values
        //pqr.setAsDouble(Math.pow(m1,2) ,0 , 0);
        //pqr.setAsDouble(Math.pow(m2,2) ,0 , 0);
        //pqr.setAsDouble(Math.pow(m3,2) ,0 , 0);
        
        // skew symmetric matrix  
        S.setAsDouble(-p.getAsDouble(2,0) ,0 , 1); // row 0
        S.setAsDouble(p.getAsDouble(1,0) ,0 , 2);
        S.setAsDouble(p.getAsDouble(2,0) ,1 , 0); // row 1
        S.setAsDouble(-p.getAsDouble(0,0) ,1 , 2);
        S.setAsDouble(-p.getAsDouble(2,0) ,2 , 0); // row 2
        S.setAsDouble(p.getAsDouble(0,0) ,1 , 1);
        
        // update dynamics
        // angular acceleration equation
        // -Jinv*S*J*omega + Jinv*(Kf*Ht-Kt*Hf)*p
        Jinv.times(-1.0).mtimes(S).mtimes(J).mtimes(p).plus(  Jinv.mtimes( Ht.times(Kf).mtimes(p) )  );
        
        // angular rate equation
        rpy = Psi.mtimes(pqr);
        
        
        // ---------------------------------------------------------------------
        
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
        
        // Rbe (I think)
        R.setAsDouble(Math.cos(rpy.getAsDouble(1,0))*Math.cos(rpy.getAsDouble(2,0)),0,0);       // row 0
        R.setAsDouble(Math.cos(rpy.getAsDouble(1,0))*Math.sin(rpy.getAsDouble(2,0)),0,1);
        R.setAsDouble(-Math.sin(rpy.getAsDouble(1,0)),0,2);
        R.setAsDouble(-Math.cos(rpy.getAsDouble(0,0))*Math.sin(rpy.getAsDouble(2,0)) + Math.sin(rpy.getAsDouble(0,0))*Math.sin(rpy.getAsDouble(1,0))*Math.cos(rpy.getAsDouble(2,0)),1,0);   // row 1
        R.setAsDouble(Math.cos(rpy.getAsDouble(0,0))*Math.cos(rpy.getAsDouble(2,0)) + Math.sin(rpy.getAsDouble(0,0))*Math.sin(rpy.getAsDouble(1,0))*Math.sin(rpy.getAsDouble(2,0)),1,1);
        R.setAsDouble(Math.sin(rpy.getAsDouble(0,0))*Math.cos(rpy.getAsDouble(1,0)),1,2);
        R.setAsDouble(Math.sin(rpy.getAsDouble(0,0))*Math.sin(rpy.getAsDouble(2,0)) + Math.cos(rpy.getAsDouble(0,0))*Math.sin(rpy.getAsDouble(1,0))*Math.cos(rpy.getAsDouble(2,0)),2,0);    // row 2
        R.setAsDouble(-Math.sin(rpy.getAsDouble(0,0))*Math.cos(rpy.getAsDouble(2,0)) + Math.cos(rpy.getAsDouble(0,0))*Math.sin(rpy.getAsDouble(1,0))*Math.sin(rpy.getAsDouble(2,0)),2,1);
        R.setAsDouble(Math.cos(rpy.getAsDouble(0,0))*Math.cos(rpy.getAsDouble(1,0)),2,2);
        
        // Hg
        Hg.setAsDouble(Math.sin(rpy.getAsDouble(1,0)),0,0);
        Hg.setAsDouble(-Math.sin(rpy.getAsDouble(0,0))*Math.cos(rpy.getAsDouble(1,0)),1,0);
        Hg.setAsDouble(-Math.cos(rpy.getAsDouble(0,0))*Math.cos(rpy.getAsDouble(1,0)),2,0);
        
        
        // This is split up a bit from the original equations to make it less messy
        // The main equations in the brackets are handed a row at a time
        //                 [psidot - psi* J^-1 * S * J * omega]
        // u = beta^-1 (v- [g*Reb*Hg                          ] )
        
        // Temp1 = DPsi-Psi*Jinv*S*J*omega;
        Temp1= DPsi.minus(Psi.mtimes(Jinv.mtimes(S).mtimes(J))).mtimes(pqr);
        Temp2= R.times(g).mtimes(Hg);
        // form final Temp
        Temp.setAsDouble(Temp1.getAsDouble(0,0), 0,0);
        Temp.setAsDouble(Temp1.getAsDouble(1,0), 1,0);
        Temp.setAsDouble(Temp1.getAsDouble(2,0), 2,0);
        Temp.setAsDouble(Temp2.getAsDouble(0,0), 3,0);
        Temp.setAsDouble(Temp2.getAsDouble(1,0), 4,0);
        Temp.setAsDouble(Temp2.getAsDouble(2,0), 5,0);
        // u1 = Binv1 * (v1-Temp1)
        // u2 = Binv2 * (v2-Temp2)
        // form beta
        Beta1 = Psi.mtimes(Jinv.mtimes(Ht.times(Kf)));
        Beta2 = R.mtimes(Hf).times(Kf/mass); 
        // copy elements from Beta1 to Beta
        Beta.setAsDouble(Beta1.getAsDouble(0,0),0,0);   // row 0
        Beta.setAsDouble(Beta1.getAsDouble(0,1),0,1);
        Beta.setAsDouble(Beta1.getAsDouble(0,2),0,2);
        Beta.setAsDouble(Beta1.getAsDouble(1,0),1,0);   // row 1
        Beta.setAsDouble(Beta1.getAsDouble(1,1),1,1);
        Beta.setAsDouble(Beta1.getAsDouble(1,2),1,2);
        Beta.setAsDouble(Beta1.getAsDouble(2,0),2,0);   // row 2
        Beta.setAsDouble(Beta1.getAsDouble(2,1),2,1);
        Beta.setAsDouble(Beta1.getAsDouble(2,2),2,2);
        // copy elements from Beta2 to Beta
        Beta.setAsDouble(Beta1.getAsDouble(0,0),3,0);   // row 0
        Beta.setAsDouble(Beta1.getAsDouble(0,1),3,1);
        Beta.setAsDouble(Beta1.getAsDouble(0,2),3,2);
        Beta.setAsDouble(Beta1.getAsDouble(1,0),4,0);   // row 1
        Beta.setAsDouble(Beta1.getAsDouble(1,1),4,1);
        Beta.setAsDouble(Beta1.getAsDouble(1,2),4,2);
        Beta.setAsDouble(Beta1.getAsDouble(2,0),5,0);   // row 2
        Beta.setAsDouble(Beta1.getAsDouble(2,1),5,1);
        Beta.setAsDouble(Beta1.getAsDouble(2,2),5,2);
        // invert Beta
        Betainv = Beta.inv();
        // compute  u
        u = Betainv.mtimes(v.minus(Temp));
        
        return u;
    }
    
    
    /*
    *
    *
    *
    *   roll only controller
    *
    *
    */
    
    
}
