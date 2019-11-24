
// ATMEL ATTINY 25/45/85 / ARDUINO
//
//                  +-\/-+
// Ain0 (D 5) PB5  1|    |8  Vcc
// Ain3 (D 3) PB3  2|    |7  PB2 (D 2) Ain1
// Ain2 (D 4) PB4  3|    |6  PB1 (D 1) pwm1
//            GND  4|    |5  PB0 (D 0) pwm0
//                  +----+

#include <NRFLite.h>
#include <avr/sleep.h> // Sleep Modes
#include <avr/power.h> // Power management
#include <avr/wdt.h>
#include <SendOnlySoftwareSerial.h>
#include <Arduino.h>

#define PWR_ON HIGH
#define PWR_OFF LOW

void disablePinChangeInterupt();
void enableLowLevelPinChangeInterupt();
void enableRisingEdgePinChangeInterupt();
void enableFallingEdgePinChangeInterupt();
void enableEdgeChangeInterupt();
void disableWatchdog(void);
void goToSleep();
void enableWatchdog();
void setupRadio();

#define PULSE_THRESHOLD 100

enum DetectorState
{
    DS_DETECTING_EDGE,
    DS_RADIO_TRANSMIT,
    DS_GOTO_SLEEP

};

struct RadioPacket
{
    uint32_t pulses;
    uint32_t pulsesTotal;
};

const static uint8_t PIN_RADIO_MOMI = 4;
const static uint8_t PIN_RADIO_SCK = 1;
const static uint8_t PIN_POWER_BUS = 0;
const static uint8_t PIN_DETECT = PB2;

const static uint8_t DESTINATION_RADIO_ID = 0;
const static uint8_t SENDER_RADIO_ID = 1;

volatile DetectorState state;
NRFLite _radio;
RadioPacket radioPacket;
volatile uint32_t pulses = 0;
SendOnlySoftwareSerial s(3);

ISR(PCINT0_vect)
{

    if (state == DS_DETECTING_EDGE) {
        pulses++;

        if(pulses < PULSE_THRESHOLD){
            state = DS_GOTO_SLEEP;
        } else {
            state = DS_RADIO_TRANSMIT;
        }
    } else {
        // Continue count pulses during transmit
        pulses++;
    }

}

void setup()
{
    // put your setup code here, to run once:

    // disableWatchdog();

    s.begin(9600);
    s.println("(Re)start xxxxxxxxxxxxxx");

    pinMode(PIN_DETECT, INPUT);
    pinMode(PIN_POWER_BUS, INPUT);

    state = DS_DETECTING_EDGE;
    enableEdgeChangeInterupt();

    //enableWatchdog();
}

void loop()
{
    // put your main code here, to run repeatedly:

    switch (state)
    {

    case DS_DETECTING_EDGE:
        /* Do nothing */

        break;


    case DS_RADIO_TRANSMIT:

        radioPacket.pulses = pulses;
        pulses = 0;
        radioPacket.pulsesTotal += radioPacket.pulses;
        

        s.print("Pulses:");
        s.println(radioPacket.pulses);
        s.print("Total pulses:");
        s.println(radioPacket.pulsesTotal);

        // Enable the power bus, setup radio and send
        //uint32_t timer = millis();
        pinMode(PIN_POWER_BUS, OUTPUT);
        digitalWrite(PIN_POWER_BUS, PWR_ON);
        setupRadio();

        _radio.send(DESTINATION_RADIO_ID, &radioPacket, sizeof(radioPacket));


        _radio.powerDown(); // Put the radio into a low power state.
        digitalWrite(PIN_POWER_BUS, PWR_OFF);
        //timer = millis() - timer;
        //s.println(timer);
        pinMode(PIN_POWER_BUS, INPUT);


        state = DS_GOTO_SLEEP;         

        break;

    case DS_GOTO_SLEEP:

        goToSleep();
        break;

    default:
        break;
    }
}

void setupRadio()
{
    if (!_radio.initTwoPin(SENDER_RADIO_ID, PIN_RADIO_MOMI, PIN_RADIO_SCK, NRFLite::BITRATE250KBPS))
    {
        while (1)
            ; // Cannot communicate with radio.
    }
}

void enableLowLevelPinChangeInterupt()
{
    // pin change interrupt
    PCMSK |= bit(PCINT2);                // want pin PB2 / pin 7
    MCUCR &= ~(bit(ISC01) | bit(ISC00)); // Low level triggers interrupt
    GIFR |= bit(PCIF);                   // clear any outstanding interrupts
    GIMSK |= bit(PCIE);                  // enable pin change interrupts
}

void enableEdgeChangeInterupt()
{
    // pin change interrupt
    PCMSK |= bit(PCINT2);                // want pin PB2 / pin 7
    MCUCR &= ~(bit(ISC01) | bit(ISC00)); // Any edge triggers interrupt
    GIFR |= bit(PCIF);                   // clear any outstanding interrupts
    GIMSK |= bit(PCIE);                  // enable pin change interrupts
}


void enableRisingEdgePinChangeInterupt()
{
    // pin change interrupt
    PCMSK |= bit(PCINT2);               // want pin PB2 / pin 7
    MCUCR |= (bit(ISC01) | bit(ISC00)); // Rising edge triggers interrupt
    GIFR |= bit(PCIF);                  // clear any outstanding interrupts
    GIMSK |= bit(PCIE);                 // enable pin change interrupts
}

void enableFallingEdgePinChangeInterupt()
{
    // pin change interrupt
    PCMSK |= bit(PCINT2);               // want pin PB2 / pin 7
    MCUCR |= (bit(ISC01) | ~bit(ISC00)); // Falling edge triggers interrupt
    GIFR |= bit(PCIF);                  // clear any outstanding interrupts
    GIMSK |= bit(PCIE);                 // enable pin change interrupts
}


void disablePinChangeInterupt()
{
    GIMSK &= ~bit(PCIE); // disable pin change interrupts
}

void disableWatchdog(void)
{
    /*  Clearing the watchdog reset flag before disabling the
     watchdog is required, according to the datasheet.*/

    MCUSR = 0;
    wdt_disable();
}

void goToSleep()
{

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
    if (digitalRead(PIN_DETECT) == HIGH)
    {
        
        ADCSRA &= ~_BV(ADEN); // Disable ADC to save power.

        state = DS_DETECTING_EDGE;
        s.print("Going to sleep  ");
        s.println(pulses);
        //enableEdgeChangeInterupt();
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();

    }
    sei();

} // end of goToSleep

void enableWatchdog()
{
    // clear various "reset" flags
    MCUSR = 0;

    // allow changes, disable reset, clear existing interrupt
    WDTCR = bit(WDCE) | bit(WDE) | bit(WDIF);

    // set interrupt mode and an interval (WDE must be changed from 1 to 0 here)
    WDTCR = _BV(WDIE) | _BV(WDP3) | _BV(WDP0); // Enable watchdog and set 8 second interrupt time.

    // pat the dog
    wdt_reset();
}
