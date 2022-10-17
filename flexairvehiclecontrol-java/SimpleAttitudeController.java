/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.ujmp.core.Matrix;

/**
 *
 * @author Thomas Pile, 2016
 */
public class SimpleAttitudeController implements Runnable, FlightController
{
    private FCSData fcsdat;
    private Lock fcsdatLock;
    private String controlfilepath = "ControllerGains.ctrl";
    private FileReader controlfile;
    private BufferedReader controlreader;
    
    private Matrix KA;
    private Matrix KB;
    private Matrix KC;
    private Matrix KD;
    
    public void init()
    {
        fcsdat = new FCSData(); 
        fcsdatLock = new ReentrantLock();
        
        try
        {
            // import gain matrices
            controlfile = new FileReader(controlfilepath);
            controlreader = new BufferedReader(controlfile);
            
            int sizeAr = 0; int sizeBr = 0; int sizeCr = 0; int sizeDr = 0; // size of rows
            int sizeAc = 0; int sizeBc = 0; int sizeCc = 0; int sizeDc = 0; // size of cols
            
            try
            {
                // parse file
                // L1: "ControllerGains\n"
                // L2: A rows, cols, B rows, cols, C rows, cols, D rows, cols\n
                // L3 to size(Arows): each row of A. 0,0,0,0\n
                // to size(Brows): each row of B. 0,0,0,0\n
                // to size(Brows): each row of C. 0,0,0,0\n
                // to size(Brows): each row of D
                String rowstr = "";
                rowstr = controlreader.readLine();
                if(rowstr.equalsIgnoreCase("ControllerGains"))
                {
                    // get sizes of matrices
                    rowstr = controlreader.readLine();
                    // split to components
                    String delims = ",";
                    String matsizes[] = rowstr.split(delims);
                    // convert to ints etc
                    sizeAr = Integer.valueOf(matsizes[0]);
                    sizeAc = Integer.valueOf(matsizes[1]);
                    sizeBr = Integer.valueOf(matsizes[2]);
                    sizeBc = Integer.valueOf(matsizes[3]);
                    sizeCr = Integer.valueOf(matsizes[4]);
                    sizeCc = Integer.valueOf(matsizes[5]);
                    sizeDr = Integer.valueOf(matsizes[6]);
                    sizeDc = Integer.valueOf(matsizes[7]);
                    
                    // read in A matrix
                    int irow = 0;
                    int icol = 0;
                    String rowvals[];
                    double KAvals[][] = new double[sizeAr][sizeAc];
                    for(irow=0; irow<sizeAr; irow++);
                    {
                        // read the row components from file and split
                        rowstr = controlreader.readLine();
                        rowvals = rowstr.split(delims);
                        // check sizes wont over run, convert to double and put into temp array
                        if(rowvals.length<=sizeAc)
                        {
                            for(icol=0;icol<rowvals.length;icol++)
                            {
                                KAvals[irow][icol] = Double.valueOf(rowvals[icol]); 
                            }
                        }
                    }
                    KA = Matrix.Factory.importFromArray(KAvals);
                    
                    System.out.println("KAvals[[0][0]: "+KAvals[0][0]);
                    
                    // read in B matrix
                }
            }
            catch(IOException iox)
            {
                iox.printStackTrace();
            }
        }
        catch(FileNotFoundException fnx)
        {
            fnx.printStackTrace();
        }
        // 
    }
    
    private void attitudeControllerUpdate()
    {
        //
        
    }
    
    @Override
    public void run()
    {
        
    }
    
    public FCSData getFCSData()
    {
        FCSData ret = new FCSData();
        
        try 
        {
            if(fcsdatLock.tryLock(10,TimeUnit.MICROSECONDS)) 
            {
                
                fcsdatLock.unlock();
            }
        }
        catch(InterruptedException iex){iex.printStackTrace();}
        //finally{imuDataLock.unlock();}
        
        return ret;
    }
    
    public void setFCSData(FCSData in)
    {
        try 
        {
            if(fcsdatLock.tryLock(10,TimeUnit.MICROSECONDS)) 
            {
                
                fcsdatLock.unlock();
            }
        }
        catch(InterruptedException iex){iex.printStackTrace();}
        //finally{imuDataLock.unlock();}
    }
    
    public class FCSData
    {
        // actual values
        
        // desired values
        private float droll;
    }
    
    /*
    *
    */
    
    private void updateAttitude()
    {
        //
        
    }
    
    /*
    * getters and setters
    */
    public void setDesiredRoll(float rollin)
    {
        try 
            {
                if(fcsdatLock.tryLock()) {fcsdat.droll = rollin;}
            }
            finally{fcsdatLock.unlock();}
    }
    
    public void init(SPIServo spiservoin)
    {
        //
    }
    
    public void setEKFData(EKF_NoRun.EKFData ekfdatin)
    {
        //ekfdat = ekfdatin;
    }
}
