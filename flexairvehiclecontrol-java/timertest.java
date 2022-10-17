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
 */
public class timertest 
{
    Timer timer;
    
    public timertest()
    {
        
        // event timer
        timer = new Timer();
        timer.scheduleAtFixedRate(new IMUReadTask(), 10, 10);
        
        new IMUReadTask();
        
        while(true)
        {
        }
    }
    
    class IMUReadTask extends TimerTask
    {
        @Override
        public void run()
        {
            System.out.println("Read");
        }
    }
}
