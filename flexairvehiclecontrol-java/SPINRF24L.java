/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package flexairvehiclecontrol;

// SPI
import com.pi4j.io.spi.SpiChannel;
import com.pi4j.io.spi.SpiDevice;
import com.pi4j.io.spi.SpiFactory;

// GPIO
import com.pi4j.io.gpio.*;
import com.pi4j.io.gpio.event.GpioPinDigitalStateChangeEvent;
import com.pi4j.io.gpio.event.GpioPinListenerDigital;
import com.pi4j.io.spi.SpiMode;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author Thomas Pile, 2016
 * April 2016
 * Controls an nRF24L01 transceiver over SPI0. 
 * + Listens for a pin change on the interrupt pin to low from high. It then 
 * checks the device status for whether it was a receive of transmit interrupt.
 * If it was receive then a CommunicationsListener event is sent to listeners
 * with the received packet.
 * + To transmit, use transmit(byte[]) then startRX() to resume receiving
 * + getData() gets packets from the device, but this is handled in the event code
 * + Packets are 16 bytes long
 * + Pin config: GPIO2/IRQ, GPIO3/Chip Enable, CE1/Chip Select, external 3.3 
 *   power supply with lots of decoupling caps
 */
public class SPINRF24L implements PhysicalCommunicationsInterface
{
    private List<CommunicationsListener> listeners = new ArrayList<CommunicationsListener>();
    
    SafeSPI spi= null;
    //private SpiDevice spi = null;
    private GpioController gpio;
    private GpioPinDigitalOutput Pin3;
    private GpioPinDigitalInput RFint;
    
    // SPI registers - read
    public static byte CONFIG_R = (byte) 0x00;
    public static byte EN_AA_R = (byte) 0x01;
    public static byte EN_RXADDR_R = (byte) 0x02;
    public static byte SETUP_RETR_R = (byte) 0x04;
    public static byte RF_CH_R = (byte) 0x05;
    public static byte RF_SETUP_R = (byte) 0x06;
    public static byte STATUS_R = (byte) 0x07;
    public static byte FIFO_STATUS_R = (byte) 0x17;
    
    public static byte RX_PW_P0_R = (byte) 0x11;
    public static byte RX_PW_P1_R = (byte) 0x12;
    public static byte RX_PW_P2_R = (byte) 0x13;
    public static byte RX_PW_P3_R = (byte) 0x14;
    public static byte RX_PW_P4_R = (byte) 0x15;
    public static byte RX_PW_P5_R = (byte) 0x16;
    public static byte DYNPD_R = (byte) 0x1C;
    
    public static byte RX_ADDR_P0 = (byte) 0x0A;
    public static byte RX_ADDR_P1 = (byte) 0x0B;
    public static byte RX_ADDR_P2 = (byte) 0x0C;
    public static byte RX_ADDR_P3 = (byte) 0x0D;
    public static byte RX_ADDR_P4 = (byte) 0x0E;
    public static byte RX_ADDR_P5 = (byte) 0x0F;
    public static byte TX_ADDR_R = (byte) 0x10;
    
    // SPI registers - write
    // Takes the form: 001AAAAA
    public static byte CONFIG_W = (byte) 0x20;       // 00100000
    public static byte EN_AA_W = (byte) 0x21;        // 00100001
    public static byte EN_RXADDR_W = (byte) 0x22;    // 00100002
    public static byte SETUP_RETR_W = (byte) 0x24;
    public static byte RF_CH_W = (byte) 0x25;
    public static byte RF_SETUP_W = (byte) 0x26;
    public static byte STATUS_W = (byte) 0x27;
    
    public static byte RX_PW_P0_W = (byte) 0x31;
    public static byte RX_PW_P1_W = (byte) 0x32;
    public static byte RX_PW_P2_W = (byte) 0x33;
    public static byte RX_PW_P3_W = (byte) 0x34;
    public static byte RX_PW_P4_W = (byte) 0x35;
    public static byte RX_PW_P5_W = (byte) 0x36;
    public static byte DYNPD_W  = (byte) 0x3C;
    
    public static byte RX_ADDR_P0_W = (byte) 0x2A;
    public static byte RX_ADDR_P1_W = (byte) 0x2B;
    public static byte RX_ADDR_P2_W = (byte) 0x2C;
    public static byte RX_ADDR_P3_W = (byte) 0x2D;
    public static byte RX_ADDR_P4_W = (byte) 0x2E;
    public static byte RX_ADDR_P5_W = (byte) 0x2F;
    public static byte TX_ADDR_W = (byte) 0x30;
    
    // Instructions
    public static byte R_RX_PAYLOAD = (byte) 0x61;
    public static byte W_TX_PAYLOAD = (byte) 0xA0;
    public static byte FLUSH_RX = (byte) 0xE2;
    public static byte FLUSH_TX = (byte) 0xE1;
    public static byte NOP = (byte) 0xFF;
    
    // Common default values
    // 0 activates an interrupt, 1 deactivates it!!!
    // interrupt on send or receive, crc 2 bytes, power up, RX = 0001 1111 = 0x1F
    // interrupt on send or receive or max retries, crc 2 bytes, power up, RX = 0000 1111 = 0x0F
    public static byte CONFIG_RX = (byte) 0x0F;
    
    // interrupt on send or receive, crc 2 bytes, power up, TX = 0001 1110 = 0x1E
    // interrupt on send or receive or max retries, crc 2 bytes, power up, TX = 0000 1110 = 0x0E
    public static byte CONFIG_TX = (byte) 0x0E;
    
    // clear all interrupts
    // 0 111 000 0 = 0x70
    public static byte STATUS_CLEAR = (byte) 0x70;
    
    public static byte PACKET_SIZE = (byte) 0x10; // 0x08; 
    public static byte PACKET_SIZE_DECIMAL = (byte) 16; // 8
    
    // instance attributes (non-static)
    //public char[] RX_ADDRESS = {0xC2,0xC2,0xC2,0xC2,0xC2};
    public char[] RX_ADDRESS = {0xE7,0xE7,0xE7,0xE7,0xE7};
    public char[] TX_ADDRESS = {0xE7,0xE7,0xE7,0xE7,0xE7};
    
    @Override
    public void init(SafeSPI spi_in)
    {
        spi = spi_in;
        
        String converted;
        byte[] response = new byte[2];
        
        //try
        {
            //spi = SpiFactory.getInstance(SpiChannel.CS0, SpiDevice.DEFAULT_SPI_SPEED, SpiMode.MODE_0);
            
            // Chip enable GPIO pin
            GpioController gpio = GpioFactory.getInstance();
            Pin3 = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_03, "Pin3", PinState.LOW);
            Pin3.low();
            
            // set up listener for interrupt
            RFint = gpio.provisionDigitalInputPin(RaspiPin.GPIO_02, PinPullResistance.PULL_UP);
            
            RFint.setShutdownOptions(true);
            RFint.addListener(new GpioPinListenerDigital()
            {
                @Override
                public void handleGpioPinDigitalStateChangeEvent(GpioPinDigitalStateChangeEvent evt)
                {
                    // this fires upon the interrupt going low, verified!
                    
                    PinState ps;
                    ps = evt.getState();
                    
                    //System.out.println("Interrupt fired...");
                    //clearInterrupt();
                    
                    if(ps.isLow())
                    {
                        // find out what interrupt is
                        byte packet[] = new byte[2];
                        byte result[] = new byte[PACKET_SIZE_DECIMAL+1];
                        byte trimmedpacket[] = new byte[PACKET_SIZE_DECIMAL];
                        
                        //try
                        {
                            // read status reg
                            packet[0] = (byte) STATUS_R;  
                            packet[1] = (byte) 0x00;  
                            result = spi.write(packet,0);//0,2);

                            // convert to binary
                            int resulttrim = (int) result[1];
                                                   
                            //System.out.println("interrupt: "+(int)resulttrim);
                            
                            if((int)resulttrim > 33 && 64>(int)resulttrim)
                            {
                                //System.out.println("Message sent");
                            }
                            
                            if((int)resulttrim > 63 && 128>(int)resulttrim)
                            {
                                System.out.println("Message received");
                                
                                
                                // get packet from device
                                result = getData();
                                
                                // copy trimmed array
                                int i;
                                for(i=0;i<PACKET_SIZE;i++)
                                {
                                    trimmedpacket[i] = result[i+1];
                                }
                                // send to comms handler
                                notifyPacketReceived(trimmedpacket);
                                
                            }
                        }
                        //catch(IOException iox){iox.printStackTrace();}
                        
                        // notify communications of data to be collected
                        clearInterrupt();
                    }// low
                }
            });
            
            // send setup codes
            byte packet[] = new byte[10];
            byte result[] = new byte[6];
            
            // set status reg
            packet[0] = (byte) STATUS_W;  
            packet[1] = (byte) 0x70;  
            spi.write(packet,0,2,0);
            
            // set RF channel 
            packet[0] = (byte) RF_CH_W;  
            packet[1] = (byte) 0x02;  
            spi.write(packet,0,2,0);
            
            // set up data pipes
                // auto ACK pipe
                packet[0] = (byte) RX_PW_P0_W;  
                packet[1] = (byte) PACKET_SIZE; //0x08;  
                spi.write(packet,0,2,0);
                
                // data pipe 1
                packet[0] = (byte) RX_PW_P1_W;  
                packet[1] = (byte) PACKET_SIZE; //0x08;  
                spi.write(packet,0,2,0);
            //
            
            // rf setup: 1mbps, 0dbm gain, LNA gain setup on
            packet[0] = (byte) RF_SETUP_W;  
            packet[1] = (byte) 0x06;  
            spi.write(packet,0,2,0);
            
            // RF setup: PLL off, 1mbps, 0dB gain, LNA gain setup on
            packet[0] = (byte) RF_SETUP_W;  
            packet[1] = (byte) 0x07;  
            packet[2] = (byte) 0x07;  
            spi.write(packet,0,3,0);
            
            packet[0] = (byte) RF_SETUP_R;  
            packet[1] = (byte) 0x00;  
            response = spi.write(packet,0,2,0);
            converted = bytesToHex(response);
            System.out.println("RF Setup Reg: "+converted.charAt(0)+" "+converted.charAt(1)+" "+converted.charAt(2)+" "+converted.charAt(3));
            
            // all interrupts, crc 1 byte, power on, RX
            packet[0] = (byte) CONFIG_W;  
            packet[1] = (byte) CONFIG_RX;  
            spi.write(packet,0,2,0);
            
            packet[0] = (byte) CONFIG_R;  
            packet[1] = (byte) 0x00;  
            response = spi.write(packet,0,2,0);
            converted = bytesToHex(response);
            System.out.println("Config Reg: "+converted.charAt(0)+" "+converted.charAt(1)+" "+converted.charAt(2)+" "+converted.charAt(3));
            
            // auto ack in enhanced shockburst
            packet[0] = (byte) EN_AA_W;  
            packet[1] = (byte) 0x03;  
            spi.write(packet,0,2,0);
            
            // enable rx address
            packet[0] = (byte) EN_RXADDR_W;  
            packet[1] = (byte) 0x03;  
            spi.write(packet,0,2,0);
            
            //auto retransmit. 1000us and 12 restransmits
            packet[0] = (byte) SETUP_RETR_W;  
            packet[1] = (byte) 0x8F;  
            spi.write(packet,0,2,0);
            
            // no dynamic length
            packet[0] = (byte) DYNPD_W;  
            packet[1] = (byte) 0x00;  
            spi.write(packet,0,2,0);
            
            // set addresses
            //setTXAddress();
            //setRXAddress();
            
            /*
            // clear interrupt
            packet[0] = (byte) STATUS_W;  
            packet[1] = (byte) 0x70;  
            spi.write(packet,0,2);
            */
            
            // read back address
            packet[0] = (byte) TX_ADDR_R;
            packet[1] = (byte) 0x00;  
            packet[2] = (byte) 0x00;
            packet[3] = (byte) 0x00;
            packet[4] = (byte) 0x00;
            packet[5] = (byte) 0x00;
            //result = spi.write(packet);
            result = spi.write(packet,0,6,0);
            
            converted = bytesToHex(result);
            System.out.println("Address: "+converted.charAt(0)+ " "
                                          +converted.charAt(1)+ " "
                                          +converted.charAt(2)+ " "
                                          +converted.charAt(3)+ " "
                                          +converted.charAt(4)+ " "
                                          +converted.charAt(5)+ " "
                                          +converted.charAt(6)+ " "
                                          +converted.charAt(7)+ " "
                                        );
            
            System.out.println("nRF24L Configured");
        }
        //catch(IOException ex){ex.printStackTrace();}
    }
    
    /*
    * The main transmit function
    */
    @Override
    public void transmit( byte[] msg) 
    {
        
        byte packet[] = new byte[PACKET_SIZE_DECIMAL+1];
        byte response[] = new byte[2]; 
        String converted;
        
        //try
        {
            setChipEnable(false);
            
            packet[0] = (byte) FLUSH_TX;  
            packet[1] = (byte) 0x00;  
            spi.write(packet,0,2,0);
            
            packet[0] = (byte) FLUSH_RX;  
            packet[1] = (byte) 0x00;  
            spi.write(packet,0,2,0);
            
            // set as transmitter
            packet[0] = (byte) CONFIG_W;  
            packet[1] = (byte) CONFIG_TX;  
            spi.write(packet,0,2,0);
            
            // copy packet to new one affixed with reg addy
            int i=1;
            for(i=1;i<PACKET_SIZE_DECIMAL+1;i++)
            {
                packet[i] = msg[i-1];
            }
            packet[0] = W_TX_PAYLOAD;
            spi.write(packet,0,packet.length,0);
            
            /*
            // copy packet to new one affixed with reg addy
            int i=1;
            for(i=1;i<9;i++)
            {
                packet[i] = 9;
            }
            packet[0] = W_TX_PAYLOAD;
            // send message 
            spi.write(packet);//,0,packet.length);
            */
            // read status and FIFO status
            packet[0] = (byte) FIFO_STATUS_R;  
            packet[1] = (byte) 0x00;  
            response = spi.write(packet,0,2,0);
            converted = bytesToHex(response);
            //System.out.println("Status: "+converted.charAt(0)+" "+converted.charAt(1)+", FIFO Status: "+converted.charAt(2)+" "+converted.charAt(3));
            
            packet[0] = (byte) CONFIG_R;  
            packet[1] = (byte) 0x00;  
            response = spi.write(packet,0,2,0);
            converted = bytesToHex(response);
            //System.out.println("Config Reg: "+converted.charAt(0)+" "+converted.charAt(1)+" "+converted.charAt(2)+" "+converted.charAt(3));
            
            // turn CE on for 10 microseconds (10000 nanoseconds)
            try
            {
                // set as transmitter
                packet[0] = (byte) CONFIG_W;  
                packet[1] = (byte) CONFIG_TX;  
                spi.write(packet,0,2,0);
                
                Thread.sleep(1, 1);
                
                setChipEnable(true);
                Thread.sleep(0, 1);
                setChipEnable(false);
            }
            catch(InterruptedException iex){iex.printStackTrace();}
            
            // read status and FIFO status
            packet[0] = (byte) FIFO_STATUS_R;  
            packet[1] = (byte) 0x00;  
            response = spi.write(packet,0,2,0);
            converted = bytesToHex(response);
            //System.out.println("Status: "+converted.charAt(0)+" "+converted.charAt(1)+", FIFO Status: "+converted.charAt(2)+" "+converted.charAt(3));
            
            /*
            try
            {
                Thread.sleep(1);
            }
            catch(InterruptedException iex){iex.printStackTrace();}
            */
            
            /*
            // clear interrupt
            packet[0] = (byte) STATUS_W;  
            packet[1] = (byte) STATUS_CLEAR;
            spi.write(packet,0,2);
            */
        }
        //catch(IOException iox){iox.printStackTrace();}
    }
    
    /*
    public void startReceive()
    {
        byte packet[] = new byte[2];
        
        try
        {
            packet[0] = (byte) FLUSH_TX;  
            packet[1] = (byte) 0x00;  
            spi.write(packet,0,2);
            
            packet[0] = (byte) FLUSH_RX;  
            packet[1] = (byte) 0x00;  
            spi.write(packet,0,2);
            
            // set as receiver
            packet[0] = (byte) CONFIG_W;  
            packet[1] = (byte) CONFIG_RX;  
            spi.write(packet,0,2);
            
            // read status
            packet[0] = (byte) STATUS_R;  
            packet[1] = (byte) 0x00;  
            spi.write(packet,0,2);
            
            setChipEnable(true);
        }
        catch(IOException iox){iox.printStackTrace();}
    }
    */
    
    @Override
    public byte[] getData()
    {
        byte packet[] = new byte[PACKET_SIZE_DECIMAL+1];
        byte received[] = new byte[PACKET_SIZE_DECIMAL];
        
        //try
        {
            setChipEnable(false);
            
            // get data
            packet[0] = (byte) R_RX_PAYLOAD;  
            
            // fill rest of packet with zeros
            int i=1;
            for(i=1;i<PACKET_SIZE_DECIMAL+1;i++){packet[i]=0;}
            // send request
            received = spi.write(packet,0);
            
            /*
            // clear interrupt
            packet[0] = (byte) STATUS_W;  
            packet[1] = (byte) 0x7E;  
            spi.write(packet,0,2);
            */
        }
        
        //catch(IOException iox){iox.printStackTrace();}
        return received;
    }
    
    // add a communications listnener to the list to be notified
    public void addListener(CommunicationsListener todo)
    {
        listeners.add(todo);
    }
    
    /*
    * Listener function
    * notifies communications receivers that packet has been received
    */
    
    public void notifyPacketReceived(byte[] packet)
    {
        for(CommunicationsListener cl : listeners)
        {
            //byte packet[];
            
            //packet = this.getData();
            cl.packetReceived(packet);
        }
    }
    
    public void clearInterrupt()
    {
        //try
        {   
            byte packet[] = new byte[2];
            // clear interrupt
            packet[0] = (byte) STATUS_W;  
            packet[1] = (byte) 0x7E;  
            spi.write(packet,0,2,0);
        }
        //catch(IOException iox){iox.printStackTrace();}
    }
    
    /*
    * turn on the receiver
    */ 
    public void startRX()
    {
        byte packet[] = new byte[2];
        byte result[] = new byte[2];
        
        //try
        {
            packet[0] = (byte) FLUSH_TX;  
            packet[1] = (byte) 0x00;  
            spi.write(packet,0,2,0);
            
            packet[0] = (byte) FLUSH_RX;  
            packet[1] = (byte) 0x00;  
            spi.write(packet,0,2,0);
            
            // set inital addresses
            setTXAddress();      
            setRXAddress(); 
            
            setChipEnable(false);
            
            // set as recever
            packet[0] = (byte) CONFIG_W;  
            packet[1] = (byte) CONFIG_RX;  
            spi.write(packet,0,2,0);
            
            // read status
            packet[0] = (byte) STATUS_R;  
            packet[1] = (byte) 0x00;  
            spi.write(packet,0,2,0);
            
            setChipEnable(true);
        }
        //catch(IOException ex){ex.printStackTrace();}
    }
    
    private void startTX()
    {
        byte packet[] = new byte[2];
        byte result[] = new byte[2];
        
        //try
        {
            packet[0] = (byte) FLUSH_TX;  
            packet[1] = (byte) 0x00;  
            spi.write(packet,0,2,0);
            
            packet[0] = (byte) FLUSH_RX;  
            packet[1] = (byte) 0x00;  
            spi.write(packet,0,2,0);
            
            // set as transmitter
            packet[0] = (byte) CONFIG_W;  
            packet[1] = (byte) CONFIG_TX;  
            spi.write(packet,0,2,0);
        }
        //catch(IOException ex){ex.printStackTrace();}
    }
    
    private boolean dataReady()
    {
        byte packet[] = new byte[2];
        byte result[] = new byte[2];
        
        //try
        {
            // read status reg
            packet[0] = (byte) STATUS_R;  
            packet[1] = (byte) 0x00;  
            result = spi.write(packet,0);//,0,2);
            
            // convert to binary
            String stat = bytesToBinary(result); // we want the first array elemet in this case anyway
            
            // check if data ready flag is set
            if(stat.charAt(1)==1){return true;}
        }
        //catch(IOException iox){iox.printStackTrace();}
        return false;
    }
    
    private boolean sendComplete()
    {
        byte packet[] = new byte[2];
        byte result[] = new byte[2];
        
        //try
        {
            // read status reg
            packet[0] = (byte) STATUS_R;  
            packet[1] = (byte) 0x00;  
            result = spi.write(packet,0);//0,2);
            
            // convert to binary
            String stat = bytesToBinary(result); // we want the first array elemet in this case anyway
            
            // check if data ready flag is set
            if(stat.charAt(1)==2){return true;}
        }
        //catch(IOException iox){iox.printStackTrace();}
        return false;
    }
    
    private boolean isSending()
    {
        byte packet[] = new byte[2];
        byte result[] = new byte[2];
        
        //try
        {
            // read status reg
            packet[0] = (byte) STATUS_R;  
            packet[1] = (byte) 0x00;  
            result = spi.write(packet,0);//0,2);
            
            // convert to binary
            String stat = bytesToBinary(result); // we want the first array elemet in this case anyway
            
            // check if data ready flag is set
            if(stat.charAt(2)==1 || stat.charAt(3)==1){return true;} 
        }
        //catch(IOException iox){iox.printStackTrace();}
        return false;
    }
    
    private void close()
    {
        gpio.shutdown();
    }
    
    private void setTXAddress()
    {
        byte packet[] = new byte[6];
        byte result[] = new byte[2];
        
        //try
        {
            // this for doing ACK, its not the data RX
            packet[0] = (byte) RX_ADDR_P0_W;
            packet[1] = (byte) TX_ADDRESS[0];  
            packet[2] = (byte) TX_ADDRESS[1];
            packet[3] = (byte) TX_ADDRESS[2];
            packet[4] = (byte) TX_ADDRESS[3];
            packet[5] = (byte) TX_ADDRESS[4];
            spi.write(packet,0);//0,2);        
            
            // set inital TX address
            packet[0] = (byte) TX_ADDR_W;
            packet[1] = (byte) TX_ADDRESS[0];  
            packet[2] = (byte) TX_ADDRESS[1];
            packet[3] = (byte) TX_ADDRESS[2];
            packet[4] = (byte) TX_ADDRESS[3];
            packet[5] = (byte) TX_ADDRESS[4];
            spi.write(packet,0);//,0,2); 
        }
        //catch(IOException iox){iox.printStackTrace();}
    }
    
    private void setRXAddress()
    {
        byte packet[] = new byte[6];
        byte result[] = new byte[2];
        
        //try
        {      
            setChipEnable(false);
            
            // set inital TX address
            packet[0] = (byte) RX_ADDR_P1_W;
            packet[1] = (byte) RX_ADDRESS[0];  
            packet[2] = (byte) RX_ADDRESS[1];
            packet[3] = (byte) RX_ADDRESS[2];
            packet[4] = (byte) RX_ADDRESS[3];
            packet[5] = (byte) RX_ADDRESS[4];
            spi.write(packet,0);//,0,2); 
            
            setChipEnable(true);
        }
        //catch(IOException iox){iox.printStackTrace();}
    }
    
    /*
    * Set CE pin high or low
    */
    private void setChipEnable(boolean enable)
    {
        if(enable==true){Pin3.high();}
        if(enable==false){Pin3.low();}
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
    
    // byte to binary conversion with padding to 8 bits
    // not used?
    // Tom
    public static String byteToBinary(byte bytes) {
        StringBuilder sb = new StringBuilder();
        StringBuilder sb2 = new StringBuilder();
        
        int v;
        //for ( int j = 0; j < bytes.length; j++ ) {
        v = bytes;
        sb.append(Integer.toBinaryString(v));
        
        for(int i=8; i>0; i--)
        {
            if(i>8-sb.length())
            {
                sb2.append('0');
            }
            else
            {
                sb2.append(sb.charAt(i));
            }
        }
        //sb2 = sb2.reverse();
        //}
        return sb2.toString();
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
}
