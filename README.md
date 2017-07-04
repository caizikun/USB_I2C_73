# USB to I2C

The USB-I2C eval board provides a complete interface between your PC and the I2C bus. It uses STM32L073xx ST's product as MCU, USB and I2C as peripheral.

## First Step - Get The Drivers ##

You should understand how to work between app and firmware. The following chart shows windows framework.This repository is source code of firmware.-->firmware development.

Please see ["SNOEC_GUI" repository](https://github.com/tclxspy/SNOEC_GUI) for app development.

![](http://i.imgur.com/MP1gyhI.jpg)

## Which COM port? ##

After installing the drivers, and plugging in the USB-I2C module to a spare USB port, you will want to know which COM port it has been assigned to. This will vary from system to system depending on how many COM ports you currently have installed. To find out where it is, right click on your "My Computer" desktop icon and select the "Device Manager" tab. Now scroll down and open the "Ports (COM & LPT)" tab. You should see the USB serial port listed - COM4 in the example below. If you want to change the COM port number - just right click on it, select properties, select advanced and select the COM port number from the available list. The COM port should be set up for 19200 baud, 8 data bits, no parity and two stop bits. 

![](http://i.imgur.com/JC0oCQq.jpg)

## Example by softkit ##

![](http://i.imgur.com/MPvuCdw.jpg)

## Operate with SNOEC_GUI ##

Click link: [https://github.com/tclxspy/SNOEC_GUI](https://github.com/tclxspy/SNOEC_GUI)
