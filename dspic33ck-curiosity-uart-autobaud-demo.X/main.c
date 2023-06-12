/*
© [2023] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

/* #########################################################################################
 * UART AUTOBAUD DEMO
 * #########################################################################################
 * This projects explains the procedure to use UART Autobaud feature. 
 * Here the objective is to calibrate the Secondary UART to the baud-rate of Primary UART.
 * 
 * In short once the Secondary UART is autobaud feature is enabled, the Primary UART send the 
 * ascii character 'U' ie; 0x55 to the Secondary UART. This makes the Secondary UART to 
 * adapt to the Primary UART baudrate. 
 * #########################################################################################
*/
#include "stdio.h"

#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/system/clock.h"
#include "mcc_generated_files/system/pins.h"
#include "mcc_generated_files/uart/uart1.h"
#include "mcc_generated_files/uart/uart2.h"
#include "mcc_generated_files/system/traps.h"
#include "mcc_generated_files/external_interrupt/ext_int.h"

/*
    Main application
*/

//Macros
#define RETRY_MAX_LIMIT 10
#define QUERY_MAX_LIMIT 50

//Enums
enum APP_STATE{
    APP_INIT_STATE = 0,
    APP_UART_TX_READY_STATE = 1,
    APP_AUTOBAUD_START_STATE = 2,
    APP_AUTOBAUD_QUERY_STATE = 3,
    APP_IDLE_STATE = 4,
};

//Global variables
bool isRetryEnabled = false;
uint8_t currentState = (uint8_t)APP_IDLE_STATE;
uint32_t initialPrimaryBaudRate = 0;
uint32_t initialSecondaryBaudRate = 0;
uint32_t finalPrimaryBaudRate = 0;
uint32_t finalSecondaryBaudRate = 0;


//Static variables
static uint8_t retryCount = 0;
static uint8_t queryCount = 0;

//Global functions
void Button_Event(void);

int main(void)
{
    SYSTEM_Initialize();
    External_Trigger_Switches.CallbackRegister(Autobaud_Trigger_Switch,&Button_Event);            //Callback registry for button event

    LED1_Success_SetLow();
    LED2_Failure_SetLow();
    
    printf("\n#################################################################################################\r\n");
    printf("                                        UART AUTOBAUD DEMO                                       \r\n");
    printf("#################################################################################################\r\n\n");
    printf("This demo explains the usage of UART Autobaud feature\r\n");
    printf("Objective is to calibrate the Secondary UART to the baud-rate of Primary UART\n\n");
    printf("Make sure you have configured different baud-rates for Primary and Secondary UARTs\r\n");
    printf("Connections from UART Primary Tx to UART Secondary Rx is required\r\n");
    printf("Connections from Switch S1 to External Interrupt pin mapped in UI are also mandatory for the demo\r\n\n");
    printf("Press Switch S1 to start the Autobaud Process...\r\n");
    currentState = APP_IDLE_STATE;
   
    while(1)
    {
        switch(currentState)
        {
            case APP_INIT_STATE:                                                   //Initialization state
                isRetryEnabled = false;
                retryCount = 0;
                queryCount = 0;
                initialPrimaryBaudRate = 0;
                initialSecondaryBaudRate = 0;
                finalPrimaryBaudRate = 0;
                finalSecondaryBaudRate = 0;
                LED1_Success_SetLow();
                LED2_Failure_SetLow();
                
                initialPrimaryBaudRate = UART_Serial_Primary.BaudRateGet();          //Getting the initial baud rates of the Primary UART
                initialSecondaryBaudRate = UART_Serial_Secondary.BaudRateGet();      //Getting the initial baud rates of the Secondary UART
                printf("\nThe following are the initial baud-rates before conversion:\r\n");
                printf("Initial baud-rate of Primary UART   : %ld\r\n",initialPrimaryBaudRate);
                printf("Initial baud-rate of Secondary UART : %ld\r\n",initialSecondaryBaudRate);
                if(initialPrimaryBaudRate == initialSecondaryBaudRate)
                {   
                    printf("\n\nSince Primary and Secondary UART baud-rates are same, there is no need for Autobaud process\r\n");
                    currentState = APP_IDLE_STATE;
                    break;
                }
                currentState = APP_UART_TX_READY_STATE;
                break;
                
            case APP_UART_TX_READY_STATE:                                               //State checks whether no current transmission is happening in Primary UART
                if(UART_Serial_Primary.IsTxDone() == true)
                    currentState = APP_AUTOBAUD_START_STATE;
                break;
                
            case APP_AUTOBAUD_START_STATE:                                          //Autobaud process enable and start the routine
                UART_Serial_Secondary.AutoBaudSet(true);                                //Enabling autobaud state
                UART_Serial_Primary.Write(0x55);                                        //Sending 'U' for autobaud process
                if(isRetryEnabled)
                {
                    retryCount++;
                    if(retryCount >= RETRY_MAX_LIMIT)
                    {
                        retryCount = 0;
                        UART_Serial_Secondary.AutoBaudSet(false);                       //Autobaud process disable
                        isRetryEnabled = false;
                        LED2_Failure_SetHigh();
                        printf("\nStatus : Autobaud failure...\r\n");
                        printf("HINT : Please check the connections\r\n");
                        printf("For retrying - Press switch S1, post fixing connections\r\n");
                        currentState = APP_IDLE_STATE;
                        break;
                    }
                }
                currentState = APP_AUTOBAUD_QUERY_STATE; 
                break;
                
            case APP_AUTOBAUD_QUERY_STATE:                                          //State to check whether the auto baud process is completed 
                if(UART_Serial_Secondary.AutoBaudQuery() == false)                      //Check whether autobaud process is completed
                {
                    finalSecondaryBaudRate = UART_Serial_Secondary.BaudRateGet();
                    finalPrimaryBaudRate = UART_Serial_Primary.BaudRateGet();
                    printf("\nAutobaud Process Completed!\r\n");
                    printf("Baudrate of Secondary UART post process : %ld\r\n\n",finalSecondaryBaudRate);

                    if(finalSecondaryBaudRate == finalPrimaryBaudRate)
                    {
                        printf("\nStatus : Autobaud success...\r\n");
                        LED1_Success_SetHigh();
                        currentState = APP_IDLE_STATE;
                    }
                    else
                    {
                        LED1_Success_SetLow();
                        LED2_Failure_SetHigh();
                        currentState = APP_UART_TX_READY_STATE;
                    }
                }
                else
                {
                    queryCount++;
                    if(queryCount >= QUERY_MAX_LIMIT)
                    {
                        queryCount = 0;
                        isRetryEnabled = true;
                        currentState = APP_AUTOBAUD_START_STATE;
                    }
                }
                break;
                
            case APP_IDLE_STATE:                                                   //State idle
                break;
                
            default:
            //Default State
                break;
        } 
    }
        
}

void Button_Event(void)
{
    currentState = APP_INIT_STATE;
}