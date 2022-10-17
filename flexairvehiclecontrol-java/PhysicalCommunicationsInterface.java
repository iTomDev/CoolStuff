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
public interface PhysicalCommunicationsInterface 
{
    public void init(SafeSPI spi);
    public void transmit( byte[] msg);
    public void startRX();
    public byte[] getData();
    public void addListener(CommunicationsListener todo);
}
