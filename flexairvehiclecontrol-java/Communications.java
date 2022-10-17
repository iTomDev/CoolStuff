/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

import flexairvehiclecontrol.EKF.EKFData;
import flexairvehiclecontrol.SimpleAttitudeController.FCSData;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.ByteBuffer;

import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.security.Security;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 *
 * @author Thomas Pile
 */
public class Communications implements CommunicationsListener, Runnable
{
    PhysicalCommunicationsInterface pint;
    FlightController  fcs;
    
    // socket info
    private int port = 7100;
    private ServerSocket socket;
    private Socket server;
    private DataInputStream streamIn;
    private DataOutputStream streamOut;
    
    private FCSData fcsdat;
    private EKFData ekf;
    private boolean setupcompleted = false;
    
    public FlightTestPacket ftp;
    public final Lock ftpLock = new ReentrantLock();
    
    @Override
    public void run()
    {
        /*
        if(setupcompleted==false)
        {
            init();
            
        }
        */
        
        while(true)
        {
            receive();
            
            
            /*
            // send and then receive response
            byte data[] = {9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9};
            transmit(data);
            receive();
            */
            
            try
            {
                Thread.sleep(100);
            }
            catch(InterruptedException iex){iex.printStackTrace();}
        }    
    }
    
    public void init(SafeSPI spi)
    {
        // setup physical link
        pint = new SPINRF24L();
        pint.addListener(this);
        pint.init(spi);
        
        ftp = new FlightTestPacket();
        
        
        //pint.startRX();
        
        /*
        try
        {
           
            socket = new ServerSocket(port);
            socket.setSoTimeout(100);
            
            server = socket.accept();
            
            streamIn = new DataInputStream(server.getInputStream());
            streamOut = new DataOutputStream(server.getOutputStream());
            
            //streamIn.readDouble();
            
            
            
            streamIn.close();
            streamOut.close();
            server.close();
            socket.close();
            
            
            
        }
        catch(IOException iox){iox.printStackTrace();}
        */
        setupcompleted = true;
    }
    
    @Override
    public void packetReceived(byte[] data)
    {
        //byte[] data= new byte[2];
        String msgchars;
        char[] msghash;
        String msghashstr;
        
        // process packet, dont bother with sockets
        //msgchars = bytesToHex(data);
        
        byte result[] = data;
        System.out.println("Received: "+result[0]+" "+
                                        result[1]+" "+
                                        result[2]+" "+
                                        result[3]+" "+
                                        result[4]+" "+
                                        result[5]+" "+
                                        result[6]+" "+
                                        result[7]+" "+
                                        result[8]+" "+
                                        result[9]+" "+
                                        result[10]+" "+
                                        result[11]+" "+
                                        result[12]+" "+
                                        result[13]+" "+
                                        result[14]+" "+
                                        result[15]
                                        );
        //
        processPacket(result);
    }
    
    //
    public void transmit(byte[] packet)
    {
        //byte data[] = {9,9,9,9,9,9,9,9};
        pint.transmit(packet);
        //pint.startRX();
    }
    
    public void receive()
    {
        pint.startRX();
    }
    
    /*
    public CommsPacket receive()
    {
        CommsPacket packet = new CommsPacket();
        return packet;
    }
    */
    
    
    public void setPhysicalInterface(PhysicalCommunicationsInterface in)
    {
        pint = in;
    }
    
    public void setFlightController(FlightController fcsin)
    {
        fcs = fcsin;
    }
    
    /*
    *
    *
    */
    
    // SHA256 = 64 bytes long (hex)
    public String computeHash(byte[] msgbuffer)
    {
        String hashval = "";
        
        try
        {
            /*
            byte[] msgbuffer = new byte[10];
            msgbuffer[0] = '1';
            msgbuffer[1] = '2';
            msgbuffer[2] = '3';
            msgbuffer[3] = '4';
            msgbuffer[4] = '5';
            msgbuffer[5] = '6';
            msgbuffer[6] = '7';
            msgbuffer[7] = '8';
            msgbuffer[8] = '9';
            msgbuffer[9] = '1';
            */
            MessageDigest md  = MessageDigest.getInstance("SHA-256");
            //MessageDigest md  = MessageDigest.getInstance("MD5");
            md.update(msgbuffer);
            byte[] tmphashval = md.digest();
            hashval = bytesToHex(tmphashval);
            //System.out.println("Hash: "+bytesToHex(hashval));
        }
        catch(NoSuchAlgorithmException nse){nse.printStackTrace();}
        return hashval;
    }
    
    public static String bytesToBinary(byte[] bytes) {
        StringBuilder sb = new StringBuilder();
        int v;
        for ( int j = 0; j < bytes.length; j++ ) {
            v = bytes[j];
            sb.append(Integer.toBinaryString(v));
        }
        return sb.toString();
    }
    
    public static String bytesToHex(byte[] bytes) {
        final char[] hexArray = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
        char[] hexChars = new char[bytes.length * 2];
        int v;
        for ( int j = 0; j < bytes.length; j++ ) {
            v = bytes[j] & 0xFF;
            hexChars[j * 2] = hexArray[v >>> 4];
            hexChars[j * 2 + 1] = hexArray[v & 0x0F];
        }
        return new String(hexChars);
    }
    
    // process packet
    public void processPacket(byte[] packet)
    {
        
        
        // 
        
        switch(Byte.toUnsignedInt(packet[0]))
        {
            /*
            *      GETTER PACKETS
            */
            case 0:
            {
                // invalid packet
                break;
            }
            
            //
            
            // GET aircraft angle and velocity data (type = "1")
            case 1:
            {
                // send RESPONSE packet with the data
                // 16 byte packet 
                // Get aircraft angle and velocity data (type = "129")
                // p type  1B unsigned byte
                // roll    4B jfloat
                // pitch   4B jfloat
                // yaw     4B jfloat 
                // heading 1B unsigned byte 
                // vel     1B unsigned byte
                // alt     1B unsigned byte
                byte r[] = new byte[16];
                byte temp[] = new byte[4];
                r[0] = (byte) 0x81; // packet type "129"
                //temp = floatToBytes(ekf.getRoll());
                r[1] = temp[0];
                r[2] = temp[1];
                r[3] = temp[2];
                r[4] = temp[3];
                break;
            }

            // 16 byte packet 
            // SET Desired aircraft angles and velocity "2"
            // p type  1B unsigned byte 
            // roll    4B jfloat
            // pitch   4B jfloat
            // yaw     4B jfloat 
            // heading 1B unsigned byte 
            // vel     1B unsigned byte
            // alt     1B unsigned byte
            case 2:
            {
                //
                byte tmp[] = new byte[4];
                
                // roll
                tmp[0] = packet[1]; tmp[1] = packet[2]; 
                tmp[2] = packet[3]; tmp[3] = packet[4]; 
                fcs.setDesiredRoll(bytesToFloat(tmp));
                // pitch 
                break;
            }

            // 16 byte packet 
            // SET Direct desired controls - Aircraft
            // p type         1B unsigned byte "3"
            // aileron angle  1B
            // elevator angle 1B 
            // rudder angle   1B 
            // thrust         1B unsigned byte 
            // 11 zeros
            case 3:
            {
                //
                break;
            }

            // 16 byte
            // SET Direct desired controls - Quadrotor
            // p type     1B unsigned byte "4"
            // thrust 1   1B
            // thrust 2   1B
            // thrust 3   1B 
            // thrust 4   1B
            // 11 zeros
            case 4:
            {
                //
                break;
            }

            // 1 byte
            // SET Run System Identification tests
            // p type     1B unsigned byte "5"
            // test type ID            (if 0, stop test)
            // test type ID repeated   (if 0, stop test)
            // test type ID repeated   (if 0, stop test)
            // e.g
            // Packet: 51000000 00000000 to run test type 1
            // Packet: 53000000 00000000 to run test type 3
            case 5:
            {
                try
                {
                    if(ftpLock.tryLock())
                    {
                        // check if flight test packet is in use
                        //if(ftp.inUse==true){}
                        //else
                        {
                            // set test in use
                            //ftp.inUse = true;
                            // set test type 
                            ftp.testType = (int) packet[1];
                        }
                    }// lock
                }//try
                finally
                {
                    {ftpLock.unlock();}
                }
                break;
            }
        }// switch
    }//processPacket
    
    private float bytesToFloat(byte[] b)
    {
        float r = 0;
        r = ByteBuffer.wrap(b).getFloat();
        return r;
    }
    
    private byte[] floatToBytes(float f)
    {
        byte[] b = new byte[4];
        /*
        // backup option
        int bits = Float.floatToIntBits(f);
        b[0] = (byte)(bits & 0xff);
        b[1] = (byte)((bits >> 8) & 0xff);
        b[2] = (byte)((bits >> 16) & 0xff);
        b[3] = (byte)((bits >> 24) & 0xff);
        */
        b = ByteBuffer.allocate(4).putFloat(f).array();
        return b;
    }
    
    
    public class FlightTestPacket
    {
        boolean inUse = false; // cleared by sys ident when it has the data it wants
        int testType;
    }
    
    public class CommsPacket
    {
        PacketType packetType;
        
    }
    
    /*
    *
    */
    
    public static enum PacketType {EKFDATA,DESIREDCONTROL};
    
    
}
