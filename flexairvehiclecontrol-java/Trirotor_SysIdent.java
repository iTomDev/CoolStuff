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
import java.util.ArrayList;
import java.util.List;



/**
 *
 * @author NeonGreen
 */
public class Trirotor_SysIdent 
{
    private static final double toDegrees = 57.2957795; //180/Math.PI;
    private boolean running = true;
    
    public static void Iterate(int testNo, int stage)
    {
        //while(running==true)
        {
            // read evrything in
            try
            {
                
                FileReader f = new FileReader("testfile.csv");
                BufferedReader r = new BufferedReader(f); 
                List<Integer[]> cmds = new ArrayList<Integer[]>();
                int cols= 7; // how many cols
                int irow=0;
                
                String rstr= null;
                String rvals[];
                
                try
                {
                    // read in test data
                    // cols are the settings
                    // rows are the time increments
                    
                    // loop rows
                    while((rstr = r.readLine()) != null)
                    {
                        // parse data
                        String delims = ",";
                        rvals = rstr.split(delims);

                        // temp array
                        Integer[] tempArr = new Integer[7];
                        
                        // loop cols
                        for(int icol=0; icol<cols; icol++) 
                        {
                            // convert to int and put in array
                            tempArr[icol] = Integer.valueOf(rvals[icol]);
                            
                        }
                        cmds.add(irow, tempArr);
                        irow++;
                        rstr = null;
                        
                        // vertify
                        //cmds.get(irow)[0];
                    }
                    
                    //rvals[0]; // time  
                }
                catch(IOException iox){}
                
                // then work through it
            }
            catch(FileNotFoundException fnfe){fnfe.printStackTrace();}
        }
    }//Iterarte
    
}
