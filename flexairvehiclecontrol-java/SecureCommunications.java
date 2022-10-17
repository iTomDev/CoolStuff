/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

import java.io.*;
import java.net.*;
import java.security.*;
import javax.net.ssl.*;

/**
 *
 * @author Thomas Pile, 2016
 * 
 * Based on:
 * http://www.herongyang.com/JDK/SSL-Socket-Server-Example-SslReverseEchoer.html
 */
public class SecureCommunications 
{
    PhysicalCommunicationsInterface pint;
    KeyStore keystore;
    String keystorePath;
    char[] keystorePassword;
    
            
    public SecureCommunications()
    {
        //
    }
    
    public void init(PhysicalCommunicationsInterface physint)
    {
        pint = physint;
        keystorePath = "airsidekeys.jks";
        keystorePassword = "password".toCharArray();
        
        
        try
        {
            // set up key material
            keystore = KeyStore.getInstance("AirsideKeystore");
            keystore.load(new FileInputStream(keystorePath), keystorePassword);
            KeyManagerFactory keyManager = KeyManagerFactory.getInstance("SunX509");
            keyManager.init(keystore, keystorePassword);
            
            // server socket
            SSLContext sc = SSLContext.getInstance("TLS"); // or "TLSv1.2"
            sc.init(keyManager.getKeyManagers(), null, null);
            SSLServerSocketFactory ssf = sc.getServerSocketFactory();
            SSLServerSocket socket = (SSLServerSocket) ssf.createServerSocket(7101);
            
            // client socket for loopback testing
            SSLSocket socketClient = (SSLSocket) socket.accept();
            
            // IO
            BufferedWriter bw = new BufferedWriter(new OutputStreamWriter(socketClient.getOutputStream()));
            BufferedReader br = new BufferedReader(new InputStreamReader(socketClient.getInputStream()));
            
            // test
            String m = "test";
            bw.write(m,0,m.length());
            bw.newLine();
            bw.flush();
            
            // read
            while ((m=br.readLine())!= null) 
                    
            // close and cleanup
            br.close();
            bw.close();
            socketClient.close();
            socket.close();
            
        }
        catch(Exception e){e.printStackTrace();}
        /*
        catch(KeyStoreException kse){kse.printStackTrace();}
        catch(FileNotFoundException fne){fne.printStackTrace();}
        catch(IOException iox){iox.printStackTrace();}
        catch(NoSuchAlgorithmException nsee){nsee.printStackTrace();}
        */
    }
    
    
}
