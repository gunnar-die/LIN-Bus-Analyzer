Lin bus analyzer for sniffing Lin 1.x and 2.x messages, for the purpose of analyzing LIN messages from automotive steering wheel controls. This project utilizes an Arduino nano for development purposes, but will be ported to an STM32 for production at a later time.


-- Wiring --

TJA1028T/5V0:

Pin 7 (VBB) -> Car +12V
Pin 2 (GND) -> Car GND, Arduino Nano GND
Pin 3 (VCC) -> Arduino Nano 5V pin
Pin 6 (LIN) -> Car LIN Bus Wire
Pin 1 (TXD) -> Arduino Nano D0 (RX)
Pin 4 (RXD) -> Arduino Nano D1 (TX)
Pin 8 (INH) -> TJA1028 Pin 2 (GND)

Arduino Nano:

5V pin -> TJA1028 Pin 3 (VCC)
GND pin -> TJA1028 Pin 2 (GND)
D0 (RX) -> TJA1028 Pin 1 (TXD)
D1 (TX) -> TJA1028 Pin 4 (RXD)

Capacitors (For Stability):

0.1uF ceramic capacitor across TJA1028 Pin 7 (VBB) and Pin 2 (GND).
0.1uF ceramic capacitor across TJA1028 Pin 3 (VCC) and Pin 2 (GND).
