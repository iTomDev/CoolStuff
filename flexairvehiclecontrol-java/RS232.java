/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

import com.pi4j.io.serial.SerialFactory;
import java.util.logging.Level;
import java.util.logging.Logger;
import com.pi4j.io.serial.*;
import static com.pi4j.wiringpi.Gpio.delayMicroseconds;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.Charset;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 *
 * @author Thomas Pile, 2016
 * 
 * You can look at the serial output independently in term:
 * sudo cat /dev/ttyAMA0
 * 
 * Usage:
 * You need to reserve the serial port for yourself first or it will crash!
 * sudo vi /boot/cmdline.txt
 * then delete stuff so the file looks like this:
 * dwc_otg.lpm_enable=0 console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline rootwait
 * then :x to save
 * Then do sudo vi /etc/inittab and make sure it doesn't exist. On newer versions it shouldn't
 * This fixed the crashing issue
 * Serial Settings: 9600bps, 8 data, no partity, 1 stop, s/ware flow control 
 * 
 * Input: GPS
 * Output: Servo control

 */
public class RS232 implements Runnable
{
    GPSData gpsdat;
    Lock gpsDataLock;
    private long dataUpdatePeriod = 0;
    private Serial serial;
    BufferedReader serialReader;
    String globalserialstring = "";
    int globalserialstringctr = 0;
    boolean setupcompleted = false;
    boolean displaylogging = false;
    boolean running = true;
    
    volatile boolean height_request_sent = false;
    double ultrasonic_height = 0;
    
    public RS232()
    {
        gpsDataLock = new ReentrantLock();
    }
    
    @Override
    public void run()
    {
        //while(true)
        {
            
            if(setupcompleted==false)
            {
                setupcompleted = true;
                init(0);
            }
            
            try
            {
                //Thread.sleep(dataUpdatePeriod);
                Thread.sleep(100);
            }
            catch(InterruptedException ixe)
            {
                ixe.printStackTrace();
            }
        }
    }
    
    // To use this, call
    //    servo.serial.requestUltrasonicHeight();
    // from a flight control loop 
    public void requestUltrasonicHeight()
    {
        if(serial==null){}
        else
        {
            if(height_request_sent == true){}
            else
            {
                System.out.println("alt request sent");
                // regularly check height
                // send request
                //System.out.println("alt request sent");
                send((byte) 255);
                send((byte) 253);
                send((byte) 1);
                // toggle listener to expect response
                height_request_sent = true;
            }
        }
    }
    
    // thread safe access to state, internal use
    public void setGPSData(GPSData in)
    {
        try 
        {
            if(gpsDataLock.tryLock()) {gpsdat = in;}
        }
        finally{gpsDataLock.unlock();}
    }
    
    public GPSData getGPSData()
    {
        GPSData ret = new GPSData();
        try
        {
            if(gpsDataLock.tryLock()) {ret = gpsdat;}
        }
        finally{gpsDataLock.unlock();}
        
        return gpsdat;
    }
    
    
    
    public void init(long dataUpdatePeriodin)
    {
        dataUpdatePeriod = dataUpdatePeriodin;
        
        // Working Serial Code
        gpsdat = new GPSData();
        gpsdat.courseT = 0;
        
        serial = SerialFactory.createInstance();
        
        //serialReader = new BufferedReader(new InputStreamReader(serial.getInputStream()));
        
        //serial.addListener(new SerialDataEventListener() {
        serial.addListener((SerialDataEventListener) (SerialDataEvent event) -> {
        try 
        {
            /*
            // strings are incomplete!
            String serialStr = event.getString(Charset.defaultCharset());
            System.out.print(serialStr+"\n\n\n");
            //System.out.print(serialStr.substring(0, 1));
            */
            String[] gpsvals;
            String delims = ",";
            String serialStr = event.getAsciiString();
            
            //System.out.print(String.valueOf("Serial: "+serial.available())+"\n");
            
            // if height request has been sent
            if(height_request_sent==true)
            {
                // catch null bytes
                byte[] bytes = event.getBytes();
                if(bytes!=null)
                {
                    // extract height from RS232
                    ultrasonic_height = (int) Byte.toUnsignedInt( event.getBytes()[0]);
                    System.out.println("Ultrasonic height: "+ultrasonic_height);
                    // duplicate into gps data instance
                    gpsdat.ultrasonic_altitude = ultrasonic_height;
                    // clear flag
                    height_request_sent = false;
                }
            }
            else
            {
                // !!! uncomment this to make serial work again
                //globalserialstring = globalserialstring.concat(serialStr);
                //globalserialstringctr++;
                
                // if not, process RS232 data as normal
                if(globalserialstringctr>5)
                {

                    int i=0; int a=0; int b=0; String msg=""; String teststring; int match=0;
                    for(i=0;i<globalserialstring.length();i++)
                    {
                        if(globalserialstring.length()>0)
                        {
                            teststring = String.valueOf(globalserialstring.charAt(i));
                            if(teststring.equalsIgnoreCase("$")){a=i;}
                            if(teststring.equalsIgnoreCase("\r")){b=i;match=1;}

                            if(match==1 && b>a)
                            {
                                match=0;
                                msg = globalserialstring.substring(a,b);

                                //System.out.print(msg+"\n\n");

                                if(msg.startsWith("$GPGGA"))
                                {
                                    //System.out.print(msg+"\n\n");

                                    delims = ",";
                                    gpsvals = msg.split(delims);

                                    if(gpsvals.length>7) // has enough elements
                                    {
                                        // 16 checksum
                                        if(!gpsvals[1].isEmpty()){gpsdat.UTCTime = stringToDouble(gpsvals[1]);}// UTC Time. hhmmss.sss
                                        if(!gpsvals[2].isEmpty()){gpsdat.latitude = stringToDouble(gpsvals[2]);} // Latitude ddmm.mmmm
                                        if(!gpsvals[4].isEmpty()){gpsdat.longitude = stringToDouble(gpsvals[4]);} // Longitude dddmm.mmmm
                                        //if (Integer.valueOf(gpsvals[7])>1){gpsdat.positionfix =true;} // Position Fix Indicator
                                        if(!gpsvals[8].isEmpty()){gpsdat.satsued = stringToDouble(gpsvals[8]);}// Sats used
                                        //if(!gpsvals[10].isEmpty()){gpsdat.altitude = Double.valueOf(gpsvals[10]);}// Altitude

                                        if(displaylogging)
                                        {
                                            System.out.println("GGA UTC Time: "+gpsdat.UTCTime);
                                            System.out.println("GGA latitude: "+gpsdat.latitude);
                                            System.out.println("GGA longitude: "+gpsdat.longitude);
                                            System.out.println("GGA satsued: "+gpsdat.satsued);
                                            System.out.println("GGA altitude: "+gpsdat.altitude+"\n");
                                        }
                                    }
                                }
                                if(msg.startsWith("$GPRMC"))
                                {
                                    //System.out.print(msg+"\n\n");

                                    delims = ",";
                                    gpsvals = msg.split(delims);

                                    if(gpsvals.length>7) // has enough elements
                                    {
                                        if(!gpsvals[1].isEmpty()){gpsdat.UTCTime = stringToDouble(gpsvals[1]);} // 1 UTC Time. hhmmss.sss
                                        if(!gpsvals[3].isEmpty()){gpsdat.latitude = stringToDouble(gpsvals[3]);} // 3 Latitude ddmm.mmmm
                                        if(!gpsvals[5].isEmpty()){gpsdat.longitude = stringToDouble(gpsvals[5]);} // 5 Longitude dddmm.mmmm
                                        if(!gpsvals[7].isEmpty()){gpsdat.speedK = stringToDouble(gpsvals[7]);} // 7 Speed over ground, Knots
                                        if(!gpsvals[8].isEmpty()){gpsdat.courseT = stringToDouble(gpsvals[8]);} // 8 course over ground

                                        if(displaylogging)
                                        {
                                            System.out.println("RMC UTCTime: "+String.valueOf(gpsdat.UTCTime));
                                            System.out.println("latitude: "+String.valueOf(gpsdat.latitude));
                                            System.out.println("longitude: "+String.valueOf(gpsdat.longitude));
                                            System.out.println("speedK: "+String.valueOf(gpsdat.speedK));
                                            System.out.println("courseT: "+String.valueOf(gpsdat.courseT)+"\n");
                                        }
                                    }
                                }

                                if(msg.startsWith("$GPGSA"))
                                {
                                    //System.out.print(msg+"\n\n");
                                }

                                if(msg.startsWith("$GPGSV"))
                                {
                                    //System.out.print(msg+"\n\n");
                                }

                                if(msg.startsWith("$GPGLL"))
                                {
                                    delims = ",";
                                    gpsvals = msg.split(delims);

                                    if(gpsvals.length>4) // has enough elements
                                    {
                                        if(!gpsvals[1].isEmpty()){gpsdat.longitude = stringToDouble(gpsvals[1]);} // Latitude ddmm.mmmm
                                        if(!gpsvals[3].isEmpty()){gpsdat.latitude = stringToDouble(gpsvals[3]);} // Longitude dddmm.mmmm
                                        if(!gpsvals[5].isEmpty()){gpsdat.UTCTime = stringToDouble(gpsvals[5]);} // UTC Time. hhmmss.sss
                                    }
                                }

                                if(msg.startsWith("$GPVTG"))
                                {
                                    delims = ",";
                                    gpsvals = msg.split(delims);

                                    if(gpsvals.length>6) // has enough elements
                                    {
                                        System.out.print(msg+"\n\n");
                                        if(!gpsvals[1].isEmpty()){gpsdat.courseT = stringToDouble(gpsvals[1]);} // true course ddd.mm [degrees]

                                        if(!gpsvals[3].isEmpty()){gpsdat.courseM = stringToDouble(gpsvals[3]);} // magnetic course ddd.mm [degrees]
                                        if(!gpsvals[5].isEmpty()){gpsdat.speedK = stringToDouble(gpsvals[5]);}  // speed N.NN [Knots]
                                        if(!gpsvals[7].isEmpty()){gpsdat.speedKM = stringToDouble(gpsvals[7]);} // speed N.NN [KM/h] 

                                        if(displaylogging)
                                        {
                                            System.out.println("VTG courseT: "+String.valueOf(gpsdat.courseT)+"\n");
                                            System.out.println("VTG courseM: "+String.valueOf(gpsdat.courseM)+"\n");
                                            System.out.println("VTG speedK: "+String.valueOf(gpsdat.speedK)+"\n");
                                            System.out.println("VTG speeKM: "+String.valueOf(gpsdat.speedKM)+"\n");
                                        }
                                    }
                                }
                            }
                        }
                        //System.out.print(String.valueOf(globalserialstring.indexOf("\n"))+"\n");
                        //System.out.print(String.valueOf(globalserialstring.indexOf("\r"))+"\n\n");
                       //System.out.print(globalserialstring);
                    }

                    globalserialstring = "";
                    globalserialstringctr = 0;
                }
            }
            serial.flush();

            //System.out.println("received");
            try 
            {
                // NOTE! - It is extremely important to read the data received from the
                // serial port.  If it does not get read from the receive buffer, the
                // buffer will continue to grow and consume memory.

                serial.discardInput();

            } 
            catch (IOException ex) 
            {
                Logger.getLogger(FlexAirVehicleControl.class.getName()).log(Level.SEVERE, null, ex);
            }

            catch (IllegalStateException ex) 
            {
                Logger.getLogger(FlexAirVehicleControl.class.getName()).log(Level.SEVERE, null, ex);
            } 


        }// try 

        catch (IOException e) 
        {
            e.printStackTrace();
        }
            
        });
        //System.out.println("3");
        
        try 
        {
            //System.out.println("4");
            
            // by default, use the DEFAULT com port on the Raspberry Pi
            //String serialPort = Serial.DEFAULT_COM_PORT;
            
            // open the default serial port provided on the GPIO header
            //serial.open(serialPort, Baud._9600, DataBits._8, Parity.NONE, StopBits._1, FlowControl.NONE);
            //serial.open(Serial.DEFAULT_COM_PORT, (int)9600);
            //serial.open(Serial.DEFAULT_COM_PORT, 9600);
            
            
            serial.setBufferingDataReceived(true);
            serial.open(Serial.DEFAULT_COM_PORT, Baud._9600, DataBits._8, Parity.NONE, StopBits._1, FlowControl.NONE);
            serial.setBufferingDataReceived(false);
            
            
            
            System.out.println("RS232 GPS setup completed");
            // continuous loop to keep the program running until the user terminates the program
            while(true) 
            {
                try {
                    //System.out.println("send");
                    // write a individual bytes to the serial transmit buffer
                    //serial.write((byte) 13);
                    //System.out.println("6");
                }
                catch(IllegalStateException ex){
                    ex.printStackTrace();
                    
                }

                try {Thread.sleep(1000);} 
                catch (InterruptedException ex) 
                {
                    ex.printStackTrace();
                    Logger.getLogger(FlexAirVehicleControl.class.getName()).log(Level.SEVERE, null, ex);
                }
            }

        }
        catch(IOException ex) {
            System.out.println(" ==>> SERIAL SETUP FAILED : " + ex.getMessage());
            return;
        }
        
        
    }// init()
    
    public void send(byte a)
    {
        try 
        {
            //System.out.println("send");
            serial.write((byte) a);
        }
        catch(IOException iox){iox.printStackTrace();}
    }
    
    /*
        @param Servo: 1 to 8
        @Param Angle: 0 to 255 [1 to 180 degs]
    */
    public void setServo(int servo, int angle)
    {
        try
        {
            if(angle==0){angle=1;}
            if(angle>170){angle=170;} // some kind of error breaks output above this
            // convert
            // 255/180 = 1.4166666666666666666666666666667;
            angle*=1.4166666666666666666666666666667;
            
            serial.write((byte) 255);
            serial.write((byte) servo);
            serial.write((byte) angle);
        }
        catch (IOException iox){}
    }
    
    /*
        @param Servo: Thrust, Elevator, Aileron
        @Param Angle: 0 to 255 [1 to 180 degs]
    */
    public void setServoCTOLConfig(String servoin, int angle)
    {
        int servo = 0;
        try
        {
            if(angle==0){angle=1;}
            if(angle>170){angle=170;} // some kind of error breaks output above this
            // convert
            // 255/180 = 1.4166666666666666666666666666667;
            angle*=1.4166666666666666666666666666667;
            
            if(servoin.equalsIgnoreCase("Thrust")){servo=1;}
            if(servoin.equalsIgnoreCase("Elevator")){servo=2;}
            if(servoin.equalsIgnoreCase("Aileron")){servo=3;}
            
            serial.write((byte) 255);
            serial.write((byte) servo);
            serial.write((byte) angle);
        }
        catch (IOException iox){}
    }
    
    public void setOutputSelectPI()
    {
        try
        {
            serial.write((byte) 255);
            serial.write((byte) 9); 
            serial.write((byte) 1); // S
        }
        catch (IOException iox){}
    }
    
    public void setOutputSelectRC()
    {
        try
        {
            serial.write((byte) 255);
            serial.write((byte) 9);
            serial.write((byte) 0); // R
        }
        catch (IOException iox){}
    }
    
    public double stringToDouble(String s)
    {
        double d = 0;
        try
        {
            d = Double.valueOf(s);
        }
        catch(NumberFormatException ex)
        {
            d = 0;
        }
        return d;
    }
   
    
    
    
    public class GPSData
    {
        public Double UTCTime = 0.0;
        public Double latitude = 0.0;
        public Double longitude = 0.0;
        public Double satsued = 0.0;
        public Double altitude = 0.0;
        public Double ultrasonic_altitude = 0.0;
        public boolean positionfix = false;
                
        public double courseT = 0;
        public double courseM = 0;
        public double speedK = 0;
        public double speedKM = 0;
        
        
    }
}











