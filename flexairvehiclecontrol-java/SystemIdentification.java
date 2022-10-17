/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

/**
 * Performs sequences of outputs depending on the command received from the ground
 * 1. Initialise it with pwm, comms and EKF access. Block other tests
 * 2. send command request from comms to here, execute relevant test
 * 3. start log running of ekf results
 * 4. send servo commands
 * 5. close log, end test, wait for more tests
 * 
 * @author Thomas Pile, 2016
 */
public class SystemIdentification implements Runnable
{
    // control of tests
    private boolean runningLoop = true; // set false to exit run()
    private boolean performingIdent = false; // are we in the process of doing an ident test?
    private int testType = 0; // what type of test is being done
    private int testStage = 0; // test are multistaged, which stage are we at?
    
    // aircraft equilibrium values
    private static final int EQ_ELEVATOR = 90;
    private static final int EQ_THRUST = 30;
    
    // motor settings
    private int m1 = 30;
    private int m2 = 30;
    private int m3 = 30;
    
    // objects
    //private I2CAVRSERVO servo;
    private Servo servo;
    private Communications comms;
    private EKF ekf;
    
    @Override
    public void run()
    {
        while(true)
        {
            while(runningLoop==true)
            {
                test0_QuadrotorTest();
            }
        }
        /*
        //
        while(runningLoop==true)
        {
            // FOR TESTING
            //test3FrequencyTestElevator();
            
            // process requests for tests, reject if in process
            // set testType which locks out other tests and determines which test to run
            try
            {
                if(comms.ftpLock.tryLock())
                {
                    // check if flight test packet is in use
                    if(comms.ftp.inUse==true){}
                    else
                    {
                        // check if we already testing
                        if(performingIdent==false)
                        {
                            testType = comms.ftp.testType;
                            // clear the transfer object
                            comms.ftp.testType = 0;
                        }
                        
                        // clear usage of exchange packet
                        comms.ftp.inUse = false;
                    }
                }// lock
            }//try
            finally{comms.ftpLock.unlock();}  
            
            // start or continue a test
            switch(testType)
            {
                case 0:
                    // no test
                    // we're done
                    testType = 0;
                    testStage = 0;
                    performingIdent = false;
                    break;
                case 1:
                    test1ShortPeriodA();
                    break;
                case 2:
                    test2PhugoidModeA();
                    break;
                case 3:    
                    break;
                case 4:
                    break;
                case 5:
                    
                    break;
            }
            
            // sleep
            try
            {
                Thread.sleep(10);

            }
            catch(InterruptedException ixe)
            {
                ixe.printStackTrace();
            }
        }//while running
        */
        
    }//run
    
    
    public void init(Servo servoIn, Communications commsIn, EKF ekfIn)
    {
        // init
        servo = servoIn;
        comms = commsIn;
        ekf = ekfIn;
    }
    
    private void IOTest()
    {
        switch(testStage)
        {
            case 0:
                // init test
                performingIdent = true;
                break;
            case 1:
                // 
                //servo.setOutputSelectPI();
                servo.setServo(1, 100);
                servo.setServo(2, 100);
                servo.setServo(3, 100);
                try
                {
                    Thread.sleep(1000);
                }
                catch(InterruptedException iex){iex.printStackTrace();}
                break;
            case 2:
                //
                servo.setServo(1, 200);
                servo.setServo(2, 200);
                servo.setServo(3, 200);
                try
                {
                    Thread.sleep(3000);
                }
                catch(InterruptedException iex){iex.printStackTrace();}
                break;
            case 3:
                // 
                servo.setServo(1, 90);
                servo.setServo(2, 90);
                servo.setServo(3, 90);
                testStage = 0;
                try
                {
                    Thread.sleep(1000);
                }
                catch(InterruptedException iex){iex.printStackTrace();}
                break;
        }
        testStage++;
    }
    
    public void test0_QuadrotorTest()
    {
        if(servo.PI_Has_Controls==true)
        {
            //
            servo.setServoSTOVLConfig("ThrustLeft", m1);
            servo.setServoSTOVLConfig("ThrustRight", m2);
            servo.setServoSTOVLConfig("ThrustTail", m3);
            
            m1++; if(m1>80){m1=30;}
            m2++; if(m2>80){m2=30;}
            m3++; if(m3>80){m3=30;}
            
            System.out.println("M1 setting: "+m1);
            
            sleep(1000);
            
            
            
            // if !logging, start logging
            
        }
        /*
        else
        {
            m1 = 0; m2 = 0; m3 = 0;
        }*/
    }
    
        public void test1_QuadrotorTest()
    {
        if(servo.PI_Has_Controls)
        {
            switch(testStage)
            {
                case 0:
                    break; //skip
                case 1:
                {
                    //
                    servo.setServoSTOVLConfig("ThrustLeft", 0);
                    servo.setServoSTOVLConfig("ThrustRight", 0);
                    servo.setServoSTOVLConfig("ThrustTail", 0);
                    
                    sleep(5000);
                    break;
                }
                case 2:
                {
                    //
                    servo.setServoSTOVLConfig("ThrustLeft", 100);
                    servo.setServoSTOVLConfig("ThrustRight", 100);
                    servo.setServoSTOVLConfig("ThrustTail", 100);
                    sleep(2000);
                    break;
                }
                case 3:
                {
                    servo.setServoSTOVLConfig("ThrustLeft", 50);
                    servo.setServoSTOVLConfig("ThrustRight", 50);
                    servo.setServoSTOVLConfig("ThrustTail", 50);
                    sleep(2000);
                    testStage = 0;
                    break;
                }
            }//switch
            testStage++;
        }
    }
        
    
    public void test0_ElevatorPulse()
    {
        if(servo.PI_Has_Controls)
        {
            switch(testStage)
            {
                case 0:
                    // init test
                    performingIdent = true;
                    break;
                case 1:
                    // 
                    //servo.setOutputSelectPI();
                    servo.setServo(1, 100);
                    servo.setServo(2, 100);
                    servo.setServo(3, 100);
                    try
                    {
                        Thread.sleep(3000);
                    }
                    catch(InterruptedException iex){iex.printStackTrace();}
                    break;
                case 2:
                    //
                    servo.setServo(1, 200);
                    servo.setServo(2, 200);
                    servo.setServo(3, 200);
                    try
                    {
                        Thread.sleep(3000);
                    }
                    catch(InterruptedException iex){iex.printStackTrace();}
                    break;
                case 3:
                    // 
                    servo.setServo(1, 10);
                    servo.setServo(2, 10);
                    servo.setServo(3, 10);
                    testStage = 0;
                    try
                    {
                        Thread.sleep(5000);
                    }
                    catch(InterruptedException iex){iex.printStackTrace();}
                    break;
            }
            testStage++;
        }
        else
        {
            testStage=0;
        }
    }
    
    public void test0_parseTest()
    {
        if(servo.PI_Has_Controls)
        {
            // read in flight test file and work through it
            // cols: millis, throttle, elevator, aileron
        }
        else
        {
            testStage=0;
        }
    }
    
    // short period mode test 1
    private void test1ShortPeriodA()
    {
        
        
    }//test1ShortPeriodA()
    
    // phugoid mode test 1
    private void test2PhugoidModeA()
    {
        switch(testStage)
        {
            case 0:
                // init test
                performingIdent = true;
                break;
            case 1:
                // 
                servo.setServo(4, 140);
                try
                {
                    Thread.sleep(500); // will take a while to spin up more etc
                }
                catch(InterruptedException iex){iex.printStackTrace();}
                break;
            case 2:
                // reset elevator angle
                servo.setServo(4, 0); // go back to an equilibrium speed
                try
                {
                    Thread.sleep(500); // will take a while to spin up more etc
                }
                catch(InterruptedException iex){iex.printStackTrace();}
                break;
            case 3:
                // 
                servo.setServo(4, 140);
                try
                {
                    Thread.sleep(500); // will take a while to spin up more etc
                }
                catch(InterruptedException iex){iex.printStackTrace();}
                break;
            case 4:
                // reset elevator angle
                servo.setServo(4, 0); // go back to an equilibrium speed
                try
                {
                    Thread.sleep(500); // will take a while to spin up more etc
                }
                catch(InterruptedException iex){iex.printStackTrace();}
                break;
            case 5:
                // we're done
                testType = 0;
                testStage = 0;
                performingIdent = false;
                System.out.println("Test 2 complete");
                break;
        }//switch
    }//test2PhugoidModeA()
    
    // phugoid mode test 1
    private void test3FrequencyTestElevator()
    {
        switch(testStage)
        {
            case 0:
                // init test
                performingIdent = true;
                //servo.enableOutputs();
                try
                {
                    Thread.sleep(300); 
                }
                catch(InterruptedException iex){iex.printStackTrace();} 
                break;
            case 1:
                // reset elevator angle 
                //servo.setServo27Angle(EQ_ELEVATOR);
                servo.setServo(1, 90);
                try
                {
                    Thread.sleep(3000); 
                }
                catch(InterruptedException iex){iex.printStackTrace();} 
                break;
            case 2:
                // elev up up
                servo.setServo(1, 60);
                try
                {
                    Thread.sleep(1000); 
                }
                catch(InterruptedException iex){iex.printStackTrace();}
                break;
            case 3:
                // elev up up
                servo.setServo(1, 110);
                try
                {
                    Thread.sleep(1000); 
                }
                catch(InterruptedException iex){iex.printStackTrace();}
                break;
            case 4:
                // elev up up
                servo.setServo(1, 60);
                 try
                {
                    Thread.sleep(500); 
                }
                catch(InterruptedException iex){iex.printStackTrace();}
                break;
            case 5:
                // elev up up
                servo.setServo(1, 110);
                try
                {
                    Thread.sleep(500); 
                }
                catch(InterruptedException iex){iex.printStackTrace();}
                break;
            case 6:
                // elev up up
                servo.setServo(1, 60);
                 try
                {
                    Thread.sleep(250); 
                }
                catch(InterruptedException iex){iex.printStackTrace();}
                break;
            case 7:
                // elev up up
                servo.setServo(1, 110);
                try
                {
                    Thread.sleep(250); 
                }
                catch(InterruptedException iex){iex.printStackTrace();}
                break;
            case 8:
                // elev up up
                servo.setServo(1, 60);
                 try
                {
                    Thread.sleep(250);
                }
                catch(InterruptedException iex){iex.printStackTrace();}
                break;
            case 9:
                // elev up up
                servo.setServo(1, 110);
                try
                {
                    Thread.sleep(250); 
                }
                catch(InterruptedException iex){iex.printStackTrace();}
                break;
            // ending test    
            case 10:
                // reset elevator angle 
                servo.setServo(1, 90);
                try
                {
                    Thread.sleep(3000); 
                }
                catch(InterruptedException iex){iex.printStackTrace();}
                break;
            case 11:
                // we're done
                
                //servo.disableOutputs();
                try
                {
                    Thread.sleep(300); 
                }
                catch(InterruptedException iex){iex.printStackTrace();} 
                testType = 0;
                testStage = 0;
                performingIdent = false;
                System.out.println("Test 3 complete");
                break;
        }//switch
        testStage++;
    }//test3
    
    // go to GPS waypoint, kill motors
    public void emergency_RTB()
    {
        //
    }
    
    private void sleep(int i)
    {
        try
        {
            Thread.sleep(i);
        }
        catch(InterruptedException iex){iex.printStackTrace();}           
    }
}//class


