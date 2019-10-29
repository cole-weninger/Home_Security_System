/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source in_code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#include "main.h"
#include "hal_LCD.h"
#include "keypad.h"
#include <string.h>

//Macros
#define LED1_PIN    GPIO_PIN0
#define LED2_PIN    GPIO_PIN1
#define LED12_PINS  GPIO_PIN0|GPIO_PIN1
#define LED12_PORT  GPIO_PORT_P1
#define LED3_PIN    GPIO_PIN5
#define LED4_PIN    GPIO_PIN7
#define LED34_PINS  GPIO_PIN5|GPIO_PIN7
#define LED34_PORT  GPIO_PORT_P2
#define Z_PORT      GPIO_PORT_P1
#define Z_PINS      GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7
#define Z1_PIN      GPIO_PIN4
#define Z2_PIN      GPIO_PIN5
#define Z3_PIN      GPIO_PIN6
#define Z4_PIN      GPIO_PIN7

#define ALARM_TIME   15
#define ATTEMPTS_LIMIT  5
#define CODE_LENGTH 4

//Prototypes
void Init_GPIO(void);
void Init_Clock(void);
void Init_RTC(void);

/*
// TimerA0 UpMode Configuration Parameter
static const Timer_A_initUpModeParam initUpParam_A0 =
{
    TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
    TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/1 = 2MHz
    30000,                                  // 15ms debounce period
    TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
    TIMER_A_DO_CLEAR,                       // Clear value
    true                                    // Start Timer
};
*/
volatile unsigned char* mode = &BAKMEM4_L; // mode flag
volatile unsigned int Event = 0;
volatile unsigned int RTC_Counter = 0;
volatile unsigned int holdCount = 0;
static char in_code[CODE_LENGTH+1] = {0,0,0,0,0};
static char CODE[CODE_LENGTH+1] = "1234";
volatile unsigned int code_char = 0;
volatile unsigned int prev_code_char = 0;
volatile unsigned int alarm_timer = 0;
volatile unsigned int hold = 0;
volatile char currChar = '\0';
volatile char prevChar = '\0';
unsigned int attempts = 0;

typedef enum {
    DISARMED = 0,
    ARMED = 1,
    ALARM = 2,
    POLICE = 3
}State;

static void displayInput (void);
static void Keypad_Scanner (void);

/*-----------------------------------------------------Main Function-----------------------------------------------------*/

static void Keypad_Scanner (void){
    currChar = Keypad_performScan();
    if (currChar != '\0'){
        if (prevChar == currChar && hold == 20){
            in_code[code_char] = currChar;
            if (code_char < 4)
                code_char++;
            currChar = '\0';
            hold++;
        }
        else if (prevChar == currChar){
            hold++;
        }
        else if (prevChar != currChar || prevChar == '\0'){
            hold = 0;
            prevChar = currChar;
        }
    }
    else if (hold != 0)
        hold = 0;

    displayInput();

    return;
}

static void displayInput (void){
    if (prev_code_char != code_char){
        switch(code_char){
            case 1:
                showChar(in_code[0],pos3);
                break;
            case 2:
                showChar(in_code[1],pos4);
                break;
            case 3:
                showChar(in_code[2],pos5);
                break;
            case 4:
                showChar(in_code[3],pos6);
                prev_code_char = 5;
                __delay_cycles(200000);
        }
    }
    else
        prev_code_char == code_char;

}


int main(void) {
    //Turn off interrupts during initialization
    __disable_interrupt();

    // Stop Watchdog timer
    WDT_A_hold(__MSP430_BASEADDRESS_WDT_A__);     // Stop WDT

    //Stop RTC
    RTC_stop(RTC_BASE);

    // Check if a wakeup from LPMx.5
    if (SYSRSTIV == SYSRSTIV_LPM5WU)
    {
        Init_GPIO();
        __enable_interrupt();
    }
    else
    {
        // Initializations
        Init_GPIO();
        Init_Clock();
        Init_RTC();
        Init_LCD();
        Init_Keypad();

        GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN_ALL16);

        __enable_interrupt();
    }

    State curr_state = DISARMED;
    State prev_state = DISARMED;

    P4OUT |= BIT0;

    while(1) 
    {
        switch(curr_state){
            case DISARMED:
                P4OUT ^= BIT0;
                code_char = 0;
                clearLCD();
                strcpy(in_code, "****");
                if( prev_state == ALARM ){
                    displayScrollText("ALARM OFF ENTER CODE TO ARM");
                }
                else if( prev_state != DISARMED ){
                    displayScrollText("DISARMED ENTER CODE TO ARM");
                }
                else
                    displayScrollText("ENTER PASSCODE TO ARM");
                prev_state = curr_state;
                P1OUT &= 0xFC;
                P2OUT &= 0x5F;
                P4OUT ^= BIT0;
                while(curr_state == DISARMED){
                    Keypad_Scanner();
                    if(code_char == CODE_LENGTH){
                        if( strcmp(in_code, CODE) == 0 ){
                            curr_state = ARMED;
                        }
                        else if (strcmp (in_code, "####" ) == 0){
                            P4OUT ^= BIT0;
                                displayScrollText("ENTER CURRENT PASSCODE");
                            P4OUT ^= BIT0;
                            code_char = 0;
                            while (code_char < CODE_LENGTH){
                                Keypad_Scanner();
                                if(code_char == CODE_LENGTH){
                                    if( strcmp(in_code, CODE) == 0 ){
                                        P4OUT ^= BIT0;
                                        displayScrollText("OKAY");
                                        P4OUT ^= BIT0;
                                        break;
                                    }
                                    else{
                                        P4OUT ^= BIT0;
                                        displayScrollText("WRONG PASSCODE RETURNING TO MENU");
                                        code_char = 0;
                                        P4OUT ^= BIT0;
                                        break;
                                    }
                                }
                            }
                            if (code_char != CODE_LENGTH ) continue;
                            P4OUT ^= BIT0;
                            displayScrollText("ENTER NEW PASSCODE");
                            code_char = 0;
                            strcpy(in_code, "||||");
                            P4OUT ^= BIT0;
                            while (code_char < CODE_LENGTH){
                                Keypad_Scanner();
                                if ( in_code[code_char - 1] == '*' || in_code[code_char - 1] == '#' ){
                                    P4OUT ^= BIT0;
                                    displayScrollText("INVALID INPUT TRY AGAIN");
                                    P4OUT ^= BIT0;
                                    strcpy(in_code, "||||");
                                    code_char = 0;
                                }

                            }
                            strcpy(CODE, in_code);
                            code_char = 0;
                            P4OUT ^= BIT0;
                            displayScrollText("PASSCODE CHANGED");
                            P4OUT ^= BIT0;

                        }
                        else{
                            P4OUT ^= BIT0;
                            displayScrollText("TRY AGAIN");
                            code_char = 0;
                            P4OUT ^= BIT0;
                        }
                    }
                }
                break;
            case ARMED:
                P4OUT ^= BIT0;
                code_char = 0;
                clearLCD();
                strcpy(in_code, "****");
                Event = 0;
                attempts = 0;
                if( prev_state != ARMED ){
                    displayScrollText("ARMED  ENTER PASSCODE TO DISARM");
                    prev_state = curr_state;
                }
                P4OUT ^= BIT0;
                while(curr_state == ARMED){
                    if (Event)
                        curr_state = ALARM;
                    Keypad_Scanner();
                    if(code_char == CODE_LENGTH){
                        if( strcmp(in_code, CODE) == 0 ){
                            curr_state = DISARMED;
                        }
                        else{
                            P4OUT ^= BIT0;
                            displayScrollText("TRY AGAIN");
                            code_char = 0;
                            attempts++;
                            P4OUT ^= BIT0;
                        }
                    }
                    else if (attempts == ATTEMPTS_LIMIT)
                        curr_state = ALARM;
                }
                break;
            case ALARM:
                P4OUT ^= BIT0;
                P1IE = 0;
                code_char = 0;
                clearLCD();
                strcpy(in_code, "****");
                if( prev_state != ALARM ){
                    displayScrollText("15 S TILL POLICE ARE NOTIFIED");
                    prev_state = curr_state;
                    RTC_Counter = 0;
                    RTC_start(RTC_BASE, RTC_CLOCKSOURCE_XT1CLK);
                    P4OUT ^= BIT0;
                    while (curr_state == ALARM){
                        if (RTC_Counter > ALARM_TIME)
                            curr_state = POLICE;
                        Keypad_Scanner();
                        if(code_char == CODE_LENGTH){
                            if( strcmp(in_code, CODE) == 0 ){
                                curr_state = DISARMED;
                                RTC_stop(RTC_BASE);
                                P1IE = 1;
                            }
                            else{
                                P4OUT ^= BIT0;
                                displayScrollText("TRY AGAIN");
                                code_char = 0;
                                P4OUT ^= BIT0;
                            }
                        }
                    }
                }
                break;
            case POLICE:
                P4OUT ^= BIT0;
                P1IE = 0;
                clearLCD();
                displayScrollText("CALLING POLICE");
                while (1){}
                break;
            default:
                displayScrollText("ERROR ERROR");
                P4OUT |= BIT0;
                break;
        }
    }
}

/*-----------------------------------------------------Main Function-----------------------------------------------------*/
    void Init_GPIO()
    {
        // Set all GPIO pins to output low to prevent floating input and reduce power consumption
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN_ALL8);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN_ALL8);
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN_ALL8);
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN_ALL8);
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN_ALL8);
        GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN_ALL8);
        GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN_ALL8);
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN_ALL8);

        GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN_ALL8);
        GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN_ALL8);
        GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN_ALL8);
        GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN_ALL8);
        GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN_ALL8);
        GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN_ALL8);
        GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN_ALL8);
        GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN_ALL8);

        GPIO_setAsInputPinWithPullUpResistor(Z_PORT, Z1_PIN|Z2_PIN|Z3_PIN|Z4_PIN);
        GPIO_selectInterruptEdge(Z_PORT, Z1_PIN|Z2_PIN|Z3_PIN|Z4_PIN, GPIO_HIGH_TO_LOW_TRANSITION);
        GPIO_clearInterrupt(Z_PORT, Z1_PIN|Z2_PIN|Z3_PIN|Z4_PIN);
        GPIO_enableInterrupt(Z_PORT, Z1_PIN|Z2_PIN|Z3_PIN|Z4_PIN);

        // Set P4.1 and P4.2 as Secondary Module Function Input, LFXT.
        GPIO_setAsPeripheralModuleFunctionInputPin(
               GPIO_PORT_P4,
               GPIO_PIN1 + GPIO_PIN2,
               GPIO_PRIMARY_MODULE_FUNCTION
               );

        // Disable the GPIO power-on default high-impedance mode
        // to activate previously configured port settings
        PMM_unlockLPM5();

    }

    /*
     * Clock System Initialization
     */
    void Init_Clock()
    {
        // Initializes the XT1 crystal oscillator
        CS_turnOnXT1(CS_XT1_DRIVE_1);
    }

    /*
     * Real Time Clock counter Initialization
     */
    void Init_RTC()
    {
        // Set RTC modulo to 327-1 to trigger interrupt every ~1s
        RTC_setModulo(RTC_BASE, 32768);
        RTC_enableInterrupt(RTC_BASE, RTC_OVERFLOW_INTERRUPT);
    }
/*
 * PORT1 Interrupt Service Routine
 */
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    //P1IE = 0; // Disable further P1.x interrupts
    switch(__even_in_range(P1IV, P1IV_P1IFG7))
    {
        case P1IV_NONE : break;
        case P1IV_P1IFG0 : break;
        case P1IV_P1IFG1 : break;
        case P1IV_P1IFG2 : break;
        case P1IV_P1IFG3 : break;
        case P1IV_P1IFG4 :
            Event = 1;
            P1OUT ^= BIT0;    // Turn LED_1 On
            P1IES ^= 0x10;
            break;
        case P1IV_P1IFG5 :
            Event = 1;
            P1OUT ^= BIT1;    // Turn LED_2 On
            P1IES ^= 0x20;
            break;
        case P1IV_P1IFG6 :
            Event = 1;
            P2OUT ^= BIT5;    // Turn LED_3 On
            P1IES ^= 0x40;
            break;
        case P1IV_P1IFG7 :
            Event = 1;
            P2OUT ^= BIT7;    // Turn LED_4 On
            P1IES ^= 0x80;
            break;
    }
    P1IFG = 0; //Clear Port 1 Interrupt Flags
}

#pragma vector = RTC_VECTOR
__interrupt void RTC_ISR(void)
{
    if (RTC_Counter == 0){
        P1OUT |= BIT0 | BIT1;
        P2OUT |= BIT5 | BIT7;
    }
    else if (RTC_Counter == ALARM_TIME){
        RTC_setModulo(RTC_BASE, 3200);
    }
    else{
        P1OUT ^= BIT0 | BIT1;
        P2OUT ^= BIT5 | BIT7;
    }

    RTC_Counter++;
    RTC_clearInterrupt(RTC_BASE, RTC_OVERFLOW_INTERRUPT_FLAG);
}
