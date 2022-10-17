/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

/**
 *
 * @author TubeAlloys
 */
public class AverageFilter 
{
    final int length = 4;//100;
    private double[] q1dat;
    private double[] q2dat;
    private double[] q3dat;
    //private double[] q4dat = new double[length];
    private double q1sum;
    private double q2sum;
    private double q3sum;
    
    int setup  = 0;
    
    public void init()
    {
        setup = length+1;
        q1dat = new double[length];
        q2dat = new double[length];
        q3dat = new double[length];
    }
    
    public double[] next(double[] in)
    {
        //
        double[] ret = new double[3];
        
        // shift array components
        System.arraycopy(q1dat, 0, q1dat, 1, length-1);
        System.arraycopy(q2dat, 0, q2dat, 1, length-1);
        System.arraycopy(q3dat, 0, q3dat, 1, length-1);
        
        // add new data
        q1dat[0] = in[0];
        q2dat[0] = in[1];
        q3dat[0] = in[2];
        
        // loop through data arrays
        for(int i=0;i<length;i++)
        {
            ret[0] += q1dat[i];
            ret[1] += q2dat[i];
            ret[2] += q3dat[i];
            //ret[3] += q4dat[i];
        }
        
        // get avg
        ret[0]/=length;
        ret[1]/=length;
        ret[2]/=length;
        
        if(setup>0){setup--; ret = null;}
        
        
        return ret;
    }// next
}
