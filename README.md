Lin bus analyzer for sniffing Lin 1.x and 2.x messages, for the purpose of analyzing LIN messages from automotive steering wheel controls. This project utilizes an Arduino nano for development purposes, but will be ported to an STM32 for production at a later time.


-- Wiring --

//TJA1028T/5V0 Pin Name	/ TJA1028T/5V0 Pin Number / Arduino Nano pin

TXD	Pin 1	- Arduino Nano D1 (TX)	
//Transmit Data input from the microcontroller. The Arduino will send data to the LIN bus via this pin.

GND	Pin 2	- Arduino Nano GND, Car Chassis GND	
//Ground connection. Crucially, connect all grounds together.

VCC	Pin 3	- Arduino Nano 5V	
//This is the 5V output from the TJA1028's integrated LDO. This powers your Arduino Nano (connect to its 5V pin, NOT Vin). If your Arduino is powered via USB, ensure its 5V pin isn't back-feeding the TJA1028.

RXD	Pin 4	- Arduino Nano D0 (RX)	
//Receive Data output to the microcontroller. The Arduino will read data from the LIN bus via this pin.

IN_ (No Connect) - Pin 5 (NC)	
//This pin is typically NC or for special features not needed for basic sniffing.

LIN	Pin 6	- Car's LIN Bus Wire	
//The single wire connection to the LIN bus in the car.

VBB	Pin 7	Car's +12V (Battery)	
//Power input for the LIN transceiver and its internal 5V LDO. Connect this to the car's 12V battery line.

INH (or EN)	Pin 8	Arduino Nano GND	
//Inhibit/Enable pin. To keep the transceiver in its normal operating mode (listening to the bus), this pin should typically be pulled LOW (connected to GND). Check datasheet for any internal pull-downs/ups.
