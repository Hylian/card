/* Name: main.c
 * Project: EasyLogger
 * Author: Christian Starkjohann
 * Creation Date: 2006-04-23
 * Tabsize: 4
 * Copyright: (c) 2006 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: Proprietary, free under certain conditions. See Documentation.
 * This Revision: $Id$
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "usbdrv.h"
#include "oddebug.h"

#define UTIL_BIN4(x)        (uchar)((0##x & 01000)/64 + (0##x & 0100)/16 + (0##x & 010)/4 + (0##x & 1))
#define UTIL_BIN8(hi, lo)   (uchar)(UTIL_BIN4(hi) * 16 + UTIL_BIN4(lo))

#ifndef NULL
#define NULL    ((void *)0)
#endif

static uchar    reportBuffer[3];    /* buffer for HID reports */
static uchar    idleRate;           /* in 4 ms units */
static uint16_t btncal;
static uchar btnAPressed;
static uint16_t sample;

/* ------------------------------------------------------------------------- */

uint16_t touchMeasure(uint8_t channel);

/* ------------------------------------------------------------------------- */

static inline void touchInit()
{
    char i;

    /* Reference AVCC (5V) */
    ADMUX = 0;

    /* Clockdiv 64 , enable ADC */
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADEN);

    btncal = 0;
    for(i=0;i<10;i++)
       btncal += touchMeasure(1)/10;
}

static inline void adc_channel(uint8_t channel)
{
    /* Enable a single pin to sample, ground the rest */
    //ADMUX = (ADMUX & ~(0b111111)) | (0b111111 & channel);
    ADMUX = (ADMUX & ~(0b111111)) | channel;
}

static inline uint16_t adc_get()
{
    /* Start Conversion */
    ADCSRA |= (1<<ADSC);

    /* Block while conversion is ongoing */
    while(!(ADCSRA & (1<<ADIF)));

    /* Manually clear the interrupt flag */
    ADCSRA |= (1<<ADIF);
    return ADC;
}

uint16_t touchMeasure(uint8_t channel)
{
    uint8_t i;
    uint16_t retval;

    retval = 0;
    for (i = 0; i < 16; i++)
    {
        // Charge the capacitive pad w/ pullup
        PORTA |= channel;
        _delay_ms(1);
        PORTA &= ~channel;

        // Set ADC mux to AGND
        //adc_channel(0b100000);
        adc_channel(0);

        // Discharge sampling cap
        adc_get();
        _delay_ms(3);

        // Set mux to button channel
        adc_channel(channel);

        // Read button value
        retval += adc_get()/16;
    }

    return retval;



    /*
        // Charge the capacitive pad w/ pullup
        //PORTA |= channel;
        PORTA = channel;
        _delay_ms(1);
        //PORTA &= ~channel;
        PORTA = 0;

        // Set ADC mux to AGND
        //adc_channel(0b100000);
        adc_channel(0);

        // Discharge sampling cap
        adc_get();

        // Set mux to button channel
        adc_channel(channel);
        _delay_ms(3);

        // Read button value
        retval = adc_get();

        PORTA = channel;
        return retval;
        */



}

 
/* ------------------------------------------------------------------------- */

const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = { /* USB report descriptor */
   0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
   0x09, 0x05,                    // USAGE (Game Pad)
   0xa1, 0x01,                    // COLLECTION (Application)
   0xa1, 0x00,                    //   COLLECTION (Physical)
   0x05, 0x09,                    //     USAGE_PAGE (Button)
   0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
   0x29, 0x04,                    //     USAGE_MAXIMUM (Button 4)
   0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
   0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
   0x75, 0x01,                    //     REPORT_SIZE (1)
   0x95, 0x04,                    //     REPORT_COUNT (4)
   0x81, 0x02,                    //     INPUT (Data,Var,Abs)
   0x75, 0x04,                    //     REPORT_SIZE (4)
   0x95, 0x01,                    //     REPORT_COUNT (1)
   0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
   0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
   0x09, 0x30,                    //     USAGE (X)
   0x09, 0x31,                    //     USAGE (Y)
   0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
   0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
   0x75, 0x08,                    //     REPORT_SIZE (8)
   0x95, 0x02,                    //     REPORT_COUNT (2)
   0x81, 0x02,                    //     INPUT (Data,Var,Abs)
   0xc0,                          //   END_COLLECTION
   0xc0                           // END_COLLECTION
};

/* ------------------------------------------------------------------------- */

static void buildReport(void)
{
    reportBuffer[0] = !btnAPressed; // Buttons: 8b0000[Start][Select][B][A]
    reportBuffer[2] = (sample>>3); // X-axis (8-bit signed)
    reportBuffer[1] = 0; // Y-axis (8-bit signed)
}

/* ------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* ------------------------------------------------------------------------- */

uchar	usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    usbMsgPtr = reportBuffer;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
            buildReport();
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
	return 0;
}

/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
static void calibrateOscillator(void)
{
uchar       step = 128;
uchar       trialValue = 0, optimumValue;
int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

    /* do a binary search: */
    do{
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    /* proportional to current real frequency */
        if(x < targetValue)             /* frequency still too low */
            trialValue += step;
        step >>= 1;
    }while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; /* this is certainly far away from optimum */
    for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/

void    usbEventResetReady(void)
{
    /* Disable interrupts during oscillator calibration since
     * usbMeasureFrameLength() counts CPU cycles.
     */
    cli();
    calibrateOscillator();
    sei();
    eeprom_write_byte(0, OSCCAL);   /* store the calibrated value in EEPROM */
}

/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */

int main(void)
{
uchar   i;
uchar   calibrationValue;

    calibrationValue = eeprom_read_byte(0); // calibration value from last time
    if(calibrationValue != 0xff){
        OSCCAL = calibrationValue;
    }
    OSCCAL = eeprom_read_byte(0);
    odDebugInit();
    touchInit();
    usbDeviceDisconnect();
    for(i=0;i<20;i++){  // 300 ms disconnect 
        _delay_ms(15);
    }
    usbDeviceConnect();
    wdt_enable(WDTO_1S);
    usbInit();
    sei();
    for(;;){    /* main event loop */
        wdt_reset();
        
        sample = touchMeasure(1);

        btnAPressed = touchMeasure(1) > (2000);

        usbPoll();
        if(usbInterruptIsReady()){ /* we can send another key */
            buildReport();
            usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
        }
    }
    return 0;
}
