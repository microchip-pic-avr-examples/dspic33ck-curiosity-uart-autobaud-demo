/**
 * UART2 Generated Driver Source File
 * 
 * @file      uart2.c
 *            
 * @ingroup   uartdriver
 *            
 * @brief     This is the generated driver source file for the UART2 driver.
 *            
 * @skipline @version   Firmware Driver Version 1.6.1
 *
 * @skipline @version   PLIB Version 1.4.1
 *            
 * @skipline  Device : dsPIC33CK256MP508
*/

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

// Section: Included Files
#include <stdint.h>
#include <stddef.h>
#include <xc.h>
#include <stddef.h>
#include "../uart2.h"

// Section: Macro Definitions
#define UART2_CLOCK 4000000U
#define UART2_BAUD_TO_BRG_WITH_FRACTIONAL(x) (UART2_CLOCK/(x))
#define UART2_BAUD_TO_BRG_WITH_BRGH_1(x) (UART2_CLOCK/(4U*(x))-1U)
#define UART2_BAUD_TO_BRG_WITH_BRGH_0(x) (UART2_CLOCK/(16U*(x))-1U)
#define UART2_BRG_TO_BAUD_WITH_FRACTIONAL(x) (UART2_CLOCK/(x))
#define UART2_BRG_TO_BAUD_WITH_BRGH_1(x) (UART2_CLOCK/(4U*((x)+1U)))
#define UART2_BRG_TO_BAUD_WITH_BRGH_0(x) (UART2_CLOCK/(16U*((x)+1U)))

#define UART2_MIN_ACHIEVABLE_BAUD_WITH_FRACTIONAL 4U
#define UART2_MIN_ACHIEVABLE_BAUD_WITH_BRGH_1 1U

// Section: Driver Interface

const struct UART_INTERFACE UART_Serial_Secondary = {
    .Initialize = &UART2_Initialize,
    .Deinitialize = &UART2_Deinitialize,
    .Read = &UART2_Read,
    .Write = &UART2_Write,
    .IsRxReady = &UART2_IsRxReady,
    .IsTxReady = &UART2_IsTxReady,
    .IsTxDone = &UART2_IsTxDone,
    .TransmitEnable = &UART2_TransmitEnable,
    .TransmitDisable = &UART2_TransmitDisable,
    .AutoBaudSet = &UART2_AutoBaudSet,
    .AutoBaudQuery = &UART2_AutoBaudQuery,
    .AutoBaudEventEnableGet = &UART2_AutoBaudEventEnableGet,
    .BRGCountSet = &UART2_BRGCountSet,
    .BRGCountGet = &UART2_BRGCountGet,
    .BaudRateSet = &UART2_BaudRateSet,
    .BaudRateGet = &UART2_BaudRateGet,
    .ErrorGet = &UART2_ErrorGet,
    .RxCompleteCallbackRegister = NULL,
    .TxCompleteCallbackRegister = NULL,
    .TxCollisionCallbackRegister = NULL,
    .FramingErrorCallbackRegister = NULL,
    .OverrunErrorCallbackRegister = NULL,
    .ParityErrorCallbackRegister = NULL,
};

// Section: Private Variable Definitions
static union
{
    struct
    {
        uint16_t frammingError :1;
        uint16_t parityError :1;
        uint16_t overrunError :1;
        uint16_t txCollisionError :1;
        uint16_t autoBaudOverflow :1;
        uint16_t reserved :11;
    };
    size_t status;
} uartError;

// Section: UART2 APIs

void UART2_Initialize(void)
{
/*    
     Set the UART2 module to the options selected in the user interface.
     Make sure to set LAT bit corresponding to TxPin as high before UART initialization
*/
    // URXEN ; RXBIMD ; UARTEN disabled; MOD Asynchronous 8-bit UART; UTXBRK ; BRKOVR ; UTXEN ; USIDL ; WAKE ; ABAUD ; BRGH ; 
    U2MODE = 0x0;
    // STSEL 1 Stop bit sent, 1 checked at RX; BCLKMOD enabled; SLPEN ; FLO ; BCLKSEL FOSC/2; C0EN ; RUNOVF ; UTXINV ; URXINV ; HALFDPLX ; 
    U2MODEH = 0x800;
    // OERIE ; RXBKIF ; RXBKIE ; ABDOVF ; OERR ; TXCIE ; TXCIF ; FERIE ; TXMTIE ; ABDOVE ; CERIE ; CERIF ; PERIE ; 
    U2STA = 0x80;
    // URXISEL ; UTXBE ; UTXISEL ; URXBE ; STPMD ; TXWRE ; 
    U2STAH = 0x2E;
    // BaudRate 114285.71; Frequency 4000000 Hz; BRG 35; 
    U2BRG = 0x23;
    // BRG 0; 
    U2BRGH = 0x0;
    
    U2MODEbits.UARTEN = 1;   // enabling UART ON bit
    U2MODEbits.UTXEN = 1;
    U2MODEbits.URXEN = 1;
}

void UART2_Deinitialize(void)
{
    U2MODE = 0x0;
    U2MODEH = 0x0;
    U2STA = 0x80;
    U2STAH = 0x2E;
    U2BRG = 0x0;
    U2BRGH = 0x0;
}

uint8_t UART2_Read(void)
{
    while((U2STAHbits.URXBE == 1))
    {
        
    }

    if ((U2STAbits.OERR == 1))
    {
        U2STAbits.OERR = 0;
    }
    
    return U2RXREG;
}

void UART2_Write(uint8_t txData)
{
    while(U2STAHbits.UTXBF == 1)
    {
        
    }

    U2TXREG = txData;    // Write the data byte to the USART.
}

bool UART2_IsRxReady(void)
{
    return (U2STAHbits.URXBE == 0);
}

bool UART2_IsTxReady(void)
{
    return ((!U2STAHbits.UTXBF) && U2MODEbits.UTXEN);
}

bool UART2_IsTxDone(void)
{
    return (bool)(U2STAbits.TRMT && U2STAHbits.UTXBE);
}

void UART2_TransmitEnable(void)
{
    U2MODEbits.UTXEN = 1;
}

void UART2_TransmitDisable(void)
{
    U2MODEbits.UTXEN = 0;
}

void UART2_AutoBaudSet(bool enable)
{
    U2INTbits.ABDIF = 0U;
    U2INTbits.ABDIE = enable;
    U2MODEbits.ABAUD = enable;
}

bool UART2_AutoBaudQuery(void)
{
    return U2MODEbits.ABAUD;
}

bool UART2_AutoBaudEventEnableGet(void)
{ 
    return U2INTbits.ABDIE; 
}

size_t UART2_ErrorGet(void)
{
    uartError.status = 0;
    if(U2STAbits.FERR == 1U)
    {
        uartError.status = uartError.status|UART_ERROR_FRAMING_MASK;
    }
    if(U2STAbits.PERR== 1U)
    {
        uartError.status = uartError.status|UART_ERROR_PARITY_MASK;
    }
    if(U2STAbits.OERR== 1U)
    {
        uartError.status = uartError.status|UART_ERROR_RX_OVERRUN_MASK;
        U2STAbits.OERR = 0;
    }
    if(U2STAbits.TXCIF== 1U)
    {
        uartError.status = uartError.status|UART_ERROR_TX_COLLISION_MASK;
        U2STAbits.TXCIF = 0;
    }
    if(U2STAbits.ABDOVF== 1U)
    {
        uartError.status = uartError.status|UART_ERROR_AUTOBAUD_OVERFLOW_MASK;
        U2STAbits.ABDOVF = 0;
    }
    
    return uartError.status;
}

void UART2_BRGCountSet(uint32_t brgValue)
{
    U2BRG = brgValue & 0xFFFFU;
    U2BRGH = (brgValue >>16U) & 0x000FU;
}

uint32_t UART2_BRGCountGet(void)
{
    uint32_t brgValue;
    
    brgValue = U2BRGH;
    brgValue = (brgValue << 16U) | U2BRG;
    
    return brgValue;
}

void UART2_BaudRateSet(uint32_t baudRate)
{
    uint32_t brgValue;
    
    if((baudRate >= UART2_MIN_ACHIEVABLE_BAUD_WITH_FRACTIONAL) && (baudRate != 0))
    {
        U2MODEHbits.BCLKMOD = 1;
        U2MODEbits.BRGH = 0;
        brgValue = UART2_BAUD_TO_BRG_WITH_FRACTIONAL(baudRate);
    }
    else
    {
        U2MODEHbits.BCLKMOD = 0;
        U2MODEbits.BRGH = 1;
        brgValue = UART2_BAUD_TO_BRG_WITH_BRGH_1(baudRate);
    }
    U2BRG = brgValue & 0xFFFFU;
    U2BRGH = (brgValue >>16U) & 0x000FU;
}

uint32_t UART2_BaudRateGet(void)
{
    uint32_t brgValue;
    uint32_t baudRate;
    
    brgValue = UART2_BRGCountGet();
    if((U2MODEHbits.BCLKMOD == 1) && (brgValue != 0))
    {
        baudRate = UART2_BRG_TO_BAUD_WITH_FRACTIONAL(brgValue);
    }
    else if(U2MODEbits.BRGH == 1)
    {
        baudRate = UART2_BRG_TO_BAUD_WITH_BRGH_1(brgValue);
    }
    else
    {
        baudRate = UART2_BRG_TO_BAUD_WITH_BRGH_0(brgValue);
    }
    return baudRate;
}
