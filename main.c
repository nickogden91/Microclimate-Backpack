/*
 * File:   main.c
 * Author: Nick
 *
 * Created on September 18, 2019, 8:03 PM
 *
 * Pic 16F1509 Pinout
 * PIN   NAME      USE
 *  1    Vdd       POWER
 *  2    RA5       
 *  3    RA4       
 *  4    Vpp       PROGRAMMER
 *  5    RC5       PWM1: Condenser Fan PWM
 *  6    RC4
 *  7    RC3       PWM2: Water Pump PWM
 *  8    RC6
 *  9    RC7
 * 10    RB7       GPIO: status LED (green)
 * 11    RB6       GPIO: error LED (red)
 * 12    RB5
 * 13    RB4
 * 14    RC2
 * 15    RC1       PWM4: Compressor PWM
 * 16    RC0       GPIO: Flow Sensor Input
 * 17    RA2       AN2: Potentiometer Input
 * 18    ICSPCLK   PROGRAMMER
 * 19    ICSPDAT   PROGRAMMER
 * 20    Vss       GROUND
 * 
 */

#include <xc.h>
#include <pic.h>
#include <pic16F1509.h>
#include <stdbool.h>

// configuration bits
#pragma config LVP      = ON
#pragma config LPBOR    = OFF
#pragma config BORV     = LO
#pragma config STVREN   = OFF
#pragma config WRT      = OFF
#pragma config FCMEN    = OFF
#pragma config IESO     = OFF
#pragma config CLKOUTEN = OFF
#pragma config FCMEN    = OFF
#pragma config BOREN    = OFF
#pragma config CP       = OFF
#pragma config MCLRE    = OFF
#pragma config PWRTE    = OFF
#pragma config WDTE     = OFF
#pragma config FOSC     = INTOSC

#define PWM_PERIOD_10kHz 24                 // number of counts for a period
                                            // of 10kHz (not 25 for some reason)
#define PWM_PERIOD_1kHz 250                 // number of counts for a period
                                            // of 1kHz

#define LED_ON 1
#define LED_OFF 0

#define TMR1H_VAL 0xC2                      // for 1kHz timer interrupt rate
#define TMR1L_VAL 0x10                      //

#define MS_OVERFLOW 1000                    // count up to 1000ms before reset

#define FLOW_OK true
#define FLOW_BAD false

#define FLOW_MS_THRESHOLD 400               // flowmeter period, above which
                                            // the flow is too low and we
                                            // will go into an error state

#define FLOW_MS_HYSTERESIS 25               // amount needed in excess of
                                            // threshold that will get us
                                            // out of error state

#define FLOW_STATE_TRANSITION_TIME_MS 1000  // amount of time before we will
                                            // allow a transition into or out
                                            // of an error state

#define MAINLOOP_DELAY 25000


// table that converts potentiometer percentage to 
const unsigned char pot_pct_to_cmp_power_table[101] = {
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
    40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
    50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
    60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
    70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
    80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
    90, 91, 92, 93, 94, 95, 96, 97, 98, 99,
    100
};


// GLOBAL VARIABLES
unsigned int pwm_period = PWM_PERIOD_1kHz;
unsigned char potentiometer_val = 0;
unsigned long int ms_count   = 0;
bool flow_state = FLOW_OK;


// Sets up registers and variables
void initialize(void)
{
    // disable weak pull ups globally
    OPTION_REGbits.nWPUEN = 1;
    
    // set oscillator frequency
    OSCCONbits.IRCF     = 0b1111;           // 16MHz
    
    // set GPIO outputs
    TRISBbits.TRISB6    = 0;                // RB6 is output
    TRISBbits.TRISB7    = 0;                // RB7 is output
    
    // set GPIO inputs
    TRISCbits.TRISC0    = 1;                // RC0 is input
    ANSELCbits.ANSC0    = 0;                // RC0 is digital
    
    // set up ADC on RA2
    TRISAbits.TRISA2    = 1;                // RA2 is input
    ANSELAbits.ANSA2    = 1;                // AN2 set to analog
    WPUAbits.WPUA2      = 0;                // weak pull up disabled
    ADCON1bits.ADCS     = 0b110;            // clock source is Fosc/64
    ADCON1bits.ADPREF   = 0b00;             // voltage reference is Vdd (5V))
    ADCON0bits.CHS      = 0b00010;          // select channel AN2
    ADCON0bits.ADON     = 1;                // enable ADC module
    ADCON0bits.GO_nDONE = 1;                // start the ADC conversion
    
    // configure timer1
    PIR1bits.TMR1IF     = 0;                // clear timer1 interrupt flag
    T1CONbits.TMR1CS    = 0b01;             // timer1 counts with Fosc
    T1CONbits.T1CKPS    = 0b00;             // prescalar 1:1
    TMR1H               = TMR1H_VAL;        // reset timer1 count
    TMR1L               = TMR1L_VAL;
    T1CONbits.TMR1ON    = 1;                // start timer1
    PIE1bits.TMR1IE     = 1;                // enable timer1 interrupts
    
    // configure timer2
    PIR1bits.TMR2IF     = 0;                // clear timer2 interrupt flag
    T2CONbits.T2CKPS    = 0b10;             // prescalar is 16
    T2CONbits.TMR2ON    = 1;                // start timer2
    PR2                 = PWM_PERIOD_1kHz;  // set timer2 period (default)
    
    // set up PWM on RC5(PWM1) and RC3(PWM2) and RC1(PWM4)
    TRISCbits.TRISC5    = 0;                // set pins to output
    TRISCbits.TRISC3    = 0;
    TRISCbits.TRISC1    = 0;
    
    PWM1DCH             = 0;                // set duty cycle to 0 to start
    PWM1DCL             = 0;                // for all three PWM outputs
    PWM2DCH             = 0;                // 
    PWM2DCL             = 0;                // 
    PWM4DCH             = 0;                // 
    PWM4DCL             = 0;                // 
    
    PWM1CONbits.PWM1OE  = 1;                // PWM outputs enabled
    PWM2CONbits.PWM2OE  = 1;                // 
    PWM4CONbits.PWM4OE  = 1;                // 
    
    PWM1CONbits.PWM1EN  = 1;                // PWM modules enabled
    PWM2CONbits.PWM2EN  = 1;                //
    PWM4CONbits.PWM4EN  = 1;                //
    
    // enable interrupts
    INTCONbits.PEIE     = 1;                // enable peripheral interrupts
    INTCONbits.GIE      = 1;                // enable global interrupts
}


void set_status_led(bool onoff)
{
    LATBbits.LATB7 = onoff;
}


void set_error_led(bool onoff)
{
    LATBbits.LATB6 = onoff;
}


// Checks flowmeter flow rate.
void check_flowmeter()
{
    static unsigned long int flow_ms = 0;
    static unsigned long int ms_since_last_pulse = 0;
    static unsigned long int flow_state_timer = 0;
    
    static unsigned int pulse_state = 0;
    static unsigned int pulse_state_prev = 0;
    static unsigned int pulse_time = 0;
    static unsigned int pulse_time_prev = 0;
    
    // evaluate whether or not we've had another flowmeter pulse
    pulse_state_prev = pulse_state;
    pulse_state     = PORTCbits.RC0;

    // check for rising pulse edges and save the time they occur
    if (pulse_state && !pulse_state_prev) {
        // calculate number of ms since last pulse
        pulse_time_prev = pulse_time;
        pulse_time = ms_count;
        flow_ms = pulse_time - pulse_time_prev;
        ms_since_last_pulse = 0;
    // it's important to also check for a total dropout of the flowmeter
    // so we will keep track of the time since the last pulse as well and
    // set the current pulse width accordingly
    } else {
        ms_since_last_pulse++;
        if (ms_since_last_pulse > flow_ms) {
            flow_ms = ms_since_last_pulse;
        }
    }
    
    // now update flow state
    if (flow_state == FLOW_OK) {
        if (flow_ms > FLOW_MS_THRESHOLD) {
            flow_state_timer++;
            if (flow_state_timer > FLOW_STATE_TRANSITION_TIME_MS) {
                flow_state = FLOW_BAD;
                flow_state_timer = 0;
            }
        } else {
            flow_state_timer = 0;
        }
    } else { // flow_state == FLOW_BAD
        if (flow_ms < (FLOW_MS_THRESHOLD - FLOW_MS_HYSTERESIS)) {
            flow_state_timer++;
            if (flow_state_timer > FLOW_STATE_TRANSITION_TIME_MS) {
                flow_state = FLOW_OK;
                flow_state_timer = 0;
            }
        } else {
            flow_state_timer = 0;
        }
    }
}


// gets the potentiometer value in percent of full scale (1-100))
unsigned int get_potentiometer_pct()
{
    return (potentiometer_val * 100) / 255;
}


// returns true if flow rate is sufficient to sustain proper operation
bool is_flow_sufficient()
{
    return (flow_state == FLOW_OK);
}


// sets the power level of the water_pump (in percent)
void water_pump_set_power(unsigned int pct)
{
    PWM2DCH = (pct * pwm_period) / 100;
}


// sets the power level of the compressor (in percent)
void compressor_set_power(unsigned int pct)
{
    // if we are less than 10pct we turn compressor off
    if (pct < 10) {
        pwm_period = PWM_PERIOD_1kHz;
        PWM4DCH = 0;
    } else {
        pwm_period = 2500 / pct;
        PR2 = pwm_period;
        PWM4DCH = pwm_period >> 1; // for 50% duty cycle
    }
}


// sets the power level of the condenser fan (in percent)
void condenser_fan_set_power(unsigned int pct)
{
    PWM1DCH = (pct * pwm_period) / 100;
}


// INTERRUPT ROUTINE
void __interrupt () ISR(void)
{ 
    // handle timer1 overflow interrupt
    if (PIR1bits.TMR1IF) {
        TMR1H           = TMR1H_VAL;        // reset timer1 count
        TMR1L           = TMR1L_VAL; 
        PIR1bits.TMR1IF = 0;                // clear timer1 interrupt flag
        ms_count++;                         // this overflows after 49 days,
                                            // so nbd- battery lasts hours :)
        // update potentiometer value
        if (!ADCON0bits.GO_nDONE) {
            potentiometer_val = ADRESH;
            ADCON0bits.GO_nDONE = 1;        // start the ADC conversion 
        }
        
        // update flow rate
        check_flowmeter();
    }
}

// MAINLOOP
void main(void) {
    
    initialize();
    
    // Mainloop
    while(1) {
        
        if (is_flow_sufficient()) {
            set_status_led(LED_ON);
            set_error_led(LED_OFF);
            compressor_set_power(pot_pct_to_cmp_power_table[get_potentiometer_pct()]);
            water_pump_set_power(100); // have to make sure to reset these two
            condenser_fan_set_power(50); // because pwm period changes
        } else {
            set_error_led(LED_ON);
            compressor_set_power(0);
            water_pump_set_power(100);
            condenser_fan_set_power(50);
        }
        
        // sleep for a bit
        for(int d = 0; d < MAINLOOP_DELAY >> 2; d++);
        set_status_led(LED_OFF);
        for(int d = 0; d < MAINLOOP_DELAY >> 2; d++);
    }
    
    return;
}
