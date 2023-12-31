/**
 * EXT_INT Generated Driver Source File
 * 
 * @file      ext_int.c
 *            
 * @ingroup   ext_interruptdriver
 *            
 * @brief     This is the generated driver source file for EXT_INT driver
 *            
 * @skipline @version   Firmware Driver Version 3.0.1
 *
 * @skipline @version   PLIB Version 2.2.0
 *            
 * @skipline  Device : dsPIC33CK256MP508
*/

/*
� [2023] Microchip Technology Inc. and its subsidiaries.

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

// Section: Includes
#include "../ext_int.h"
#include <stddef.h>

// Section: Driver Interface
const struct EXT_INTERRUPT_INTERFACE External_Trigger_Switches = {
    .Initialize = &EXT_INT_Initialize,
    .Deinitialize = &EXT_INT_Deinitialize,
    .InterruptDisable = &EXT_INT_InterruptDisable,
    .InterruptEnable = &EXT_INT_InterruptEnable,
    .InterruptFlagClear = &EXT_INT_InterruptFlagClear,
    .NegativeEdgeSet = &EXT_INT_NegativeEdgeSet,
    .PositiveEdgeSet = &EXT_INT_PositiveEdgeSet,
    .CallbackRegister = &EXT_INT_CallbackRegister,
};

// Section: Private Variable Definitions
// EXT_INT Default Interrupt Handler
static void (*EXT_INT0_Handler)(void) = NULL;
static void (*EXT_INT1_Handler)(void) = NULL;

void EXT_INT_Initialize(void)
{
    /*******
     * INT0
     * Clear the interrupt flag
     * Set the external interrupt edge detect
     * Enable the interrupt, if enabled in the UI. 
     ********/
    EXT_INT_InterruptFlagClear(INT0);   
    EXT_INT_PositiveEdgeSet(INT0);
    EXT_INT_InterruptEnable(INT0);

    /*******
     * Autobaud_Trigger_Switch
     * Clear the interrupt flag
     * Set the external interrupt edge detect
     * Enable the interrupt, if enabled in the UI. 
     ********/
    EXT_INT_InterruptFlagClear(Autobaud_Trigger_Switch);   
    EXT_INT_PositiveEdgeSet(Autobaud_Trigger_Switch);
    EXT_INT_InterruptEnable(Autobaud_Trigger_Switch);

    // Ext interrupt Callback Register
    EXT_INT_CallbackRegister(INT0, &EXT_INT0_Callback);
    EXT_INT_CallbackRegister(Autobaud_Trigger_Switch, &EXT_INT1_Callback);
}

void EXT_INT_Deinitialize(void)
{
    EXT_INT_InterruptFlagClear(INT0);   
    EXT_INT_InterruptDisable(INT0);
    //default setting is positive edge
    EXT_INT_PositiveEdgeSet(INT0);
    EXT_INT_InterruptFlagClear(Autobaud_Trigger_Switch);   
    EXT_INT_InterruptDisable(Autobaud_Trigger_Switch);
    //default setting is positive edge
    EXT_INT_PositiveEdgeSet(Autobaud_Trigger_Switch);
}

void EXT_INT_PositiveEdgeSet(enum EXT_INT interruptNum)
{
    switch(interruptNum)
    {
        case INT0:
            INTCON2bits.INT0EP = 0;
            break;

        case Autobaud_Trigger_Switch:
            INTCON2bits.INT1EP = 0;
            break;

        default:
            break;
    }
}

void EXT_INT_NegativeEdgeSet(enum EXT_INT interruptNum)
{
    switch(interruptNum)
    {
        case INT0:
            INTCON2bits.INT0EP = 1;
            break;

        case Autobaud_Trigger_Switch:
            INTCON2bits.INT1EP = 1;
            break;

        default:
            break;            
    }
}

void EXT_INT_InterruptEnable(enum EXT_INT interruptNum)
{
    switch(interruptNum)
    {
        case INT0:
            IEC0bits.INT0IE = 1;
            break;
            
        case Autobaud_Trigger_Switch:
            IEC0bits.INT1IE = 1;
            break;
            
        default:
            break;            
    }
}

void EXT_INT_InterruptDisable(enum EXT_INT interruptNum)
{
    switch(interruptNum)
    {
        case INT0:
            IEC0bits.INT0IE = 0;
            break;
            
        case Autobaud_Trigger_Switch:
            IEC0bits.INT1IE = 0;
            break;
            
        default:
            break;            
    }
}

void EXT_INT_InterruptFlagClear(enum EXT_INT interruptNum)
{
    switch(interruptNum)
    {
        case INT0:
            IFS0bits.INT0IF = 0;
            break;

        case Autobaud_Trigger_Switch:
            IFS0bits.INT1IF = 0;
            break;

        default:
            break;
    }
}

void EXT_INT_CallbackRegister(enum EXT_INT interruptNum, void (*handler)(void))
{
    switch(interruptNum)
    {
        case INT0:
            if(NULL != handler)
            {
                EXT_INT0_Handler = handler;
            }
            break;
            
        case Autobaud_Trigger_Switch:
            if(NULL != handler)
            {
                EXT_INT1_Handler = handler;
            }
            break;
            
        default:
            break;            
    }
}

void __attribute__ ((weak)) EXT_INT0_Callback(void)
{ 
    // Add your custom callback code here
} 

void __attribute__ ((weak)) EXT_INT1_Callback(void)
{ 
    // Add your custom callback code here
} 

void __attribute__ ( ( interrupt, no_auto_psv ) ) _INT0Interrupt(void)
{
    if(EXT_INT0_Handler != NULL)
    {
        EXT_INT0_Handler();
    }
    IFS0bits.INT0IF = 0;
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _INT1Interrupt(void)
{
    if(EXT_INT1_Handler != NULL)
    {
        EXT_INT1_Handler();
    }
    IFS0bits.INT1IF = 0;
}

