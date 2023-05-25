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
#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/system/clock.h"
#define FCY CLOCK_PeripheralFrequencyGet()
#include <libpic30.h>
#include "mcc_generated_files/system/pins.h"
#include "mcc_generated_files/uart/uart1.h"
#include "mcc_generated_files/uart/uart2.h"
#include "stdio.h"
#include "mcc_generated_files/system/traps.h"
#include "mcc_generated_files/external_interrupt/ext_int.h"

/*
    Main application
*/

//Macros
#define RETRY_MAX_LIMIT 10
#define QUERY_MAX_LIMIT 50

//Peripheral instances
const struct UART_INTERFACE *UartPrimary = &UART_Serial_Primary;
const struct UART_INTERFACE *UartSecondary = &UART_Serial_Secondary;
const struct EXT_INTERRUPT_INTERFACE *ExternalSwitch = &External_Interrupt;

enum AUTOBAUD_STATE{
    AUTOBAUD_INIT_STATE = 0,
    AUTOBAUD_TX_READY_STATE = 1,
    AUTOBAUD_ROUTINE_START_STATE = 2,
    AUTOBAUD_PROCESS_QUERY_STATE = 3,
    AUTOBAUD_IDLE_STATE = 4,
};

//Global variables
bool is_retry_enabled = false;
uint8_t current_state = (uint8_t)AUTOBAUD_INIT_STATE;
uint32_t initial_baud_rate_primary = 0;
uint32_t initial_baud_rate_secondary = 0;
uint32_t final_baud_rate = 0;
static uint8_t retry_count = 0;
static uint8_t query_count = 0;

//Global functions
void ButtonEvent(void);

int main(void)
{
    SYSTEM_Initialize();
    ExternalSwitch->CallbackRegister(AutobaudSwitch,&ButtonEvent);  //Callback registry for button event

    LED1_Success_SetLow();
    LED2_Failure_SetLow();
   
    while(1)
    {
        switch(current_state)
        {
            case AUTOBAUD_INIT_STATE:
                is_retry_enabled = false;
                LED1_Success_SetLow();
                LED2_Failure_SetLow();
                retry_count = 0;
                query_count = 0;
                initial_baud_rate_primary = 0;
                initial_baud_rate_secondary = 0;
                initial_baud_rate_primary = UartPrimary->BaudRateGet();
                initial_baud_rate_secondary = UartSecondary->BaudRateGet();
                printf("\nInitial baudrate before process UART1: %ld\r\n",initial_baud_rate_primary);
                printf("Initial baudrate before process UART2: %ld\r\n",initial_baud_rate_secondary);
                current_state = AUTOBAUD_TX_READY_STATE;
                break;
            case AUTOBAUD_TX_READY_STATE:
                if(UartPrimary->IsTxDone() == true)
                    current_state = AUTOBAUD_ROUTINE_START_STATE;
                break;
            case AUTOBAUD_ROUTINE_START_STATE:
                UartSecondary->AutoBaudSet(true);
                UartPrimary->Write(0x55);
                if(is_retry_enabled)
                {
                    retry_count++;
                    if(retry_count >= RETRY_MAX_LIMIT)
                    {
                        retry_count = 0;
                        UartSecondary->AutoBaudSet(false);
                        is_retry_enabled = false;
                        LED2_Failure_SetHigh();
                        printf("\nStatus : Autobaud failure...\r\n");
                        current_state = AUTOBAUD_IDLE_STATE;
                        break;
                    }
                }
                current_state = AUTOBAUD_PROCESS_QUERY_STATE; 
                break;
            case AUTOBAUD_PROCESS_QUERY_STATE:
                if(UartSecondary->AutoBaudQuery() == false)
                {
                    final_baud_rate = 0;
                    int final_prim_baud = 0;
            
                    UartSecondary->Write(0x4); //Sample data write
                    UartSecondary->Write(0x14);
                    UartSecondary->Write(0x54);

                    final_baud_rate = UartSecondary->BaudRateGet();
                    final_prim_baud = UartPrimary->BaudRateGet();
                    printf("\nBaudrate of UART2 post process : %ld\r\n",final_baud_rate);

                    if(final_baud_rate == final_prim_baud)
                    {
                        printf("\nStatus : Autobaud success...\r\n");
                        LED1_Success_SetHigh();
                        current_state = AUTOBAUD_IDLE_STATE;
                    }
                    else
                    {
                        LED1_Success_SetLow();
                        LED2_Failure_SetHigh();
                        current_state = AUTOBAUD_TX_READY_STATE;
                    }
                }
                else
                {
                    query_count++;
                    if(query_count >= QUERY_MAX_LIMIT)
                    {
                        query_count = 0;
                        is_retry_enabled = true;
                        current_state = AUTOBAUD_ROUTINE_START_STATE;
                    }
                }
                break;    
            case AUTOBAUD_IDLE_STATE:
                break;
            default:
            //Default State
                break;
        } 
    }
        
}

void ButtonEvent()
{
    current_state = AUTOBAUD_INIT_STATE;
}