/*
 * File:   lcd.h
 * Author: 
 *
 */
#ifndef LCD_H
#define LCD_H
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

/* Define the LCD interface and character size */
#define LCD_FORMAT (FOUR_BIT & LINES_5X7)

/* Define the LCD port pins */
#define E_PIN               LATBbits.LATB10
#define RW_PIN              LATBbits.LATB9
#define RS_PIN              LATBbits.LATB11
#define LCD_DATA_BITS       0x0F
#define LCD_PORT_IN         PORTD
#define LCD_PORT_OUT        LATD

#define E_PIN_DIR           TRISBbits.TRISB10
#define RW_PIN_DIR          TRISBbits.TRISB9
#define RS_PIN_DIR          TRISBbits.TRISB11
#define LCD_PORT_DIR        TRISD

/* Clear display command */
#define CLEAR_DISPLAY 0b00000001

/* Return home command */
#define RETURN_HOME 0b00000010

/* Display ON/OFF Control defines */
#define DON         0b00001111   /* Display on      */
#define DOFF        0b00001011   /* Display off     */
#define CURSOR_ON   0b00001111   /* Cursor on       */
#define CURSOR_OFF  0b00001101   /* Cursor off      */
#define BLINK_ON    0b00001111   /* Cursor Blink    */
#define BLINK_OFF   0b00001110   /* Cursor No Blink */

/* Cursor or Display Shift defines */
#define SHIFT_CUR_LEFT    0b00010011   /* Cursor shifts to the left   */
#define SHIFT_CUR_RIGHT   0b00010111   /* Cursor shifts to the right  */
#define SHIFT_DISP_LEFT   0b00011011   /* Display shifts to the left  */
#define SHIFT_DISP_RIGHT  0b00011111   /* Display shifts to the right */

/* Function Set defines */
#define FOUR_BIT   0b00101111   /* 4-bit Interface               */
#define EIGHT_BIT  0b00111111   /* 8-bit Interface               */
#define LINE_5X7   0b00110011   /* 5x7 characters, single line   */
#define LINE_5X10  0b00110111   /* 5x10 characters               */
#define LINES_5X7  0b00111011   /* 5x7 characters, multiple line */

/* Start address of each line */
#define LINE_ONE    0x00
#define LINE_TWO    0x40
#define LINE_THREE  0x14
#define LINE_FOUR   0x54

void LCD_Init(void);
void LCD_SetCGRamAddr(unsigned char data);
void LCD_SetDDRamAddr(unsigned char data);
void LCD_WriteCmd(unsigned char data);
void LCD_WriteData(unsigned char data);
void LCD_WriteString(const char * pString);
unsigned char LCD_GetBusyBit(void);

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif
