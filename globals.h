/* MIT License

Copyright (c) 2021 Advanced Design and Manufacturing, LLC

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

#ifndef GLOBALS_H
#define	GLOBALS_H

#ifdef	__cplusplus
extern "C" {
#endif

/*
  Section: Definitions
*/
#define ADC12BITS                       // Use 12-Bit ADC mode instead of 10-Bit mode
#define SYS_FREQ        32000000L
#define FCY             (SYS_FREQ/2)
#define NUMLEDS         64              // Set number of WS2812 LEDs in string here
#define WS2812_L        0xC0            // Crazy way of defining the Low signal for the WS2812B using SPI
#define WS2812_H        0xF0            // Crazy way of defining the High signal for the WS2812B using SPI
#define BLACK           0               // Indexes to 24-bit LED color value in color lookup table
#define WHITE           1
#define RED             2
#define GREEN           3
#define BLUE            4
#define YELLOW          5
#define AQUA            6
#define VIOLET          7
#define SAMPLES         10              // Number of rolling average samples to use
#define MINATTVAL       120               // Minimum allowed value of data sent to digital pot
#define MAXATTVAL       127             // Maximum allowed value of data sent to digital pot
#define ATTTHRESH       800            // Threshold where gain will be reduced in ADC counts
#define ATTHYST         400             // Hysteresis for threshold

#ifdef ADC12BITS
    #define SCALINGFACTOR   (4096/NUMLEDS)            // Scaling factor to match ADC resolution
#else
    #define SCALINGFACTOR   (1024/NUMLEDS)            // Scaling factor to match ADC resolution
#endif

    
/*
 Section: Globals
 */
extern unsigned char ledData[NUMLEDS];         // Array for holding separate 8-bit LED data
extern unsigned long colorLUT[8] = {0x000000, 0xFFFFFF, 0x00FF00, 0xFF0000, 0x0000FF, 0xFFFF00, 0xFF00FF, 0x00FFFF};
unsigned int averageTable[SAMPLES];     // Array of 10 or 12 bit samples to use in calculating average
unsigned int avgLvl;                    // Average value of incoming ADC data
extern unsigned int attVal;            // Set starting point for display sensitivity pot
extern unsigned char oldAttVal;



#ifdef	__cplusplus
}
#endif

#endif	/* GLOBALS_H */