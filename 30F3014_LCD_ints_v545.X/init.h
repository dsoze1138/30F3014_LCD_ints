/*
 * File:   init.h
 * Author: 
 * Target: dsPIC30F3014
 *
 */

#ifndef INIT_H
#define INIT_H
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

/* 
 * Define the target system clock frequency.
 * 
 * The initialization code MUST set the system clock to implement these definitions.
 * 
 */
/* Setup the clock to run at about 117.9648 MHz (29.4912 MIPS) from 7.3728MHz internal oscillator */
#define FSRC  (7372800ul)   /* nominal fast RC frequency */
#define PLL_MODE (16ul)     /* Use 16x PLL mode set in configuration words */
#define FSYS (FSRC*PLL_MODE)
#define FCY  (FSYS/4ul)     /* dsPIC30 uses a 4 clock instruction cycle */

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif
