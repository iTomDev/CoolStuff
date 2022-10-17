/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

/**
 *
 * @author Thomas Pile, 2016
 */
public interface FlightController 
{
    //public void init();
    //public void setDesiredRoll(float f);
    public void setEKFData(EKF_NoRun.EKFData ekfdat);
    //public void init(SPIServo spiServo);
    public void init(SPIServo spiservoin);
    // 
    
    public void setDesiredRoll(float f);
    
    /*
    public void setDesiredLocation();
    public void setDesireAttitude();
    public void attitudeControlUpdate();
    public void nagivationControlUpdate();
    */
}
