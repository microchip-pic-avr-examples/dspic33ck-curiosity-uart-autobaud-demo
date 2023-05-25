![image](images/microchip.jpg) 

## dspic33ck curiosity uart autobaud demo

![Board](images/board.jpg)

## Summary

The project demonstrates the usage of UART Autobaud feature and how in realtime the Primary UART tunes the Secondary UART to its baudrate.
We make use of dsPIC33CK Curiosity Development Board and configure the peripherals using MPLAB® Code Configurator.

## Related Documentation

[dsPIC33CK256MP508 datasheet](https://www.microchip.com/dsPIC33CK256MP508) for more information or specifications

## Software Used 
- [MPLAB® X IDE v6.05](https://www.microchip.com/mplabx) or newer
- [MPLAB® XC16 v2.10](https://www.microchip.com/xc16) or newer
- Device Family Pack: dsPIC33CK-MP_DFP v1.9.228
- [MPLAB® Code Configurator (MCC) 5.3.0](https://www.microchip.com/mcc) or newer Version: 
- Data Visualizer or any other serial terminal with the following settings


## Hardware Used
- [dsPIC33CK Curiosity Board](https://www.microchip.com/dm330030)

## Setup
**Hardware Setup**
- Connect a micro-USB cable to port `J7` of Curiosity board to USB port of PC
- Connect a jumper wire from UART1_Tx (RD4) -> UART2_Rx (RD12)
- Connect a jumper wire from Board Switch S1 (RE7) -> Mapped external interrupt PIN (RD15) <br>

![board_connection](images/board_connections.jpg)

**Serial Port Setup**
- Open a serial terminal with the following configurations  
![SerialPort](images/terminal_configs.png)

**MPLAB® X IDE Setup**
- Open the `dspic33ck-curiosity-uart-autobaud-demo.X` project in MPLAB® X IDE
- Build and program the device

## Operation
- On start, the device prints the initial baudrates of the UART Primary and UART Secondary peripherals as 9600 and 115200 as per configuration in MCC.
- UART Primary Transmitter U1TX then starts the autobaud process by sending the 0x55 ie; the ASCII character 'U' to the UART Secondary U2RX
- Upon success of the autobaud process, the notification LED Green will be indicated and LED Red will indicate the failiure of process. Followed by a message on terminal. 
![SerialOutput](images/terminal.png) <br> 


## MCC Configurations
- UART1 Configuration : <br>
![UART1](images/uart_primary_config.png)

- UART2 Configuration : <br>
![UART2](images/uart_secondary_config.png)

- External Interrupt Configuration : <br>
![EXT_INT](images/ext_int_config.png)

- Pins Configuration : <br>
![Pins](images/pin_grid_view.png)<br>
![Pins](images/pins_config.png)

