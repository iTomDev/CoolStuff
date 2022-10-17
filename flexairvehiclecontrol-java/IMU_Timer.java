/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

import java.util.Timer;
import java.util.TimerTask;

/**
 *
 * @author Thomas Pile
 * This starts a timer which fires every 10ms
 * Usage from main:
 * timertest timer = new timertest();
 * 
 * Accuracy: +- 0.5ms
 */
public class IMU_Timer 
{
    Timer timer;
    IMUEKFControlLoop imuekf;
    final int PERIOD = 10;
    
    public IMU_Timer(IMUEKFControlLoop imuekfin)
    {
        // link to the imu read loop
        imuekf = imuekfin;
        
        // event timer
        timer = new Timer();
        timer.scheduleAtFixedRate(new IMUReadTask(), PERIOD, PERIOD);
        
        new IMUReadTask();
        /*
        while(true)
        {
            
            try
            {
                Thread.sleep(1);
            }
            catch(InterruptedException iex){iex.printStackTrace();}
            
        }*/
    }
    
    class IMUReadTask extends TimerTask
    {
        @Override
        public void run()
        {
            imuekf.cycle = true;
            //System.out.println("Read");
        }
    }
}
