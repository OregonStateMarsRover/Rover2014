/*
 * XMegaMacros.h
 *
 * Created: 4/25/2014 5:51:17 AM
 *  Author: Nick
 */ 


#ifndef XMEGAMACROS_H_
#define XMEGAMACROS_H_

//Custom Defined Macros
#define STATUS1_SET(void) (PORTC.OUTSET = PIN6_bm)
#define STATUS1_CLR(void) (PORTC.OUTCLR = PIN6_bm)

#define STATUS2_SET(void) (PORTC.OUTSET = PIN5_bm)
#define STATUS2_CLR(void) (PORTC.OUTCLR = PIN5_bm)

#define ERROR_SET(void) (PORTC.OUTSET = PIN7_bm)
#define ERROR_CLR(void) (PORTC.OUTCLR = PIN7_bm)

//Motor 1 enable/disables
#define MD1_ENABLE(void) (PORTD.OUTCLR = PIN6_bm)
#define MD1_DISABLE(void) (PORTD.OUTSET = PIN6_bm)

#define MD2_ENABLE(void) (PORTD.OUTCLR = PIN2_bm)
#define MD2_DISABLE(void) (PORTD.OUTSET = PIN2_bm)

//Motor 1 M enable/disables
#define MD1_M0_SET(void) (PORTA.OUTSET = PIN5_bm)
#define MD1_M1_SET(void) (PORTA.OUTSET = PIN6_bm)
#define MD1_M2_SET(void) (PORTA.OUTSET = PIN7_bm)

#define MD1_M0_CLR(void) (PORTA.OUTCLR = PIN5_bm)
#define MD1_M1_CLR(void) (PORTA.OUTCLR = PIN6_bm)
#define MD1_M2_CLR(void) (PORTA.OUTCLR = PIN7_bm)

#define MD2_M0_SET(void) (PORTB.OUTSET = PIN0_bm)
#define MD2_M1_SET(void) (PORTB.OUTSET = PIN1_bm)
#define MD2_M2_SET(void) (PORTB.OUTSET = PIN2_bm)

#define MD2_M0_CLR(void) (PORTB.OUTCLR = PIN0_bm)
#define MD2_M1_CLR(void) (PORTB.OUTCLR = PIN1_bm)
#define MD2_M2_CLR(void) (PORTB.OUTCLR = PIN2_bm)

//Set direction macros
#define MD1_DIR_SET(void) (PORTD.OUTSET = PIN4_bm)
#define MD1_DIR_CLR(void) (PORTD.OUTCLR = PIN4_bm)

#define MD2_DIR_SET(void) (PORTD.OUTSET = PIN0_bm)
#define MD2_DIR_CLR(void) (PORTD.OUTCLR = PIN0_bm)

//Set step macros
#define MD1_STEP_SET(void) (PORTD.OUTSET = PIN5_bm)
#define MD1_STEP_CLR(void) (PORTD.OUTCLR = PIN5_bm)

#define MD2_STEP_SET(void) (PORTD.OUTSET = PIN1_bm)
#define MD2_STEP_CLR(void) (PORTD.OUTCLR = PIN1_bm)

//Limit switch macros
#define CHECK_GRIP_CLOSE(void) (!(PORTA.IN & PIN3_bm))  //Returns true if grip close
#define CHECK_GRIP_LIMIT(void) (PORTB.IN & PIN3_bm)     //Returns true if a limit switch has been pressed
#define CHECK_CAL(void) (PORTA.IN & PIN2_bm)            //Returns true if cal limit switch has been pressed

#define CHECK_ISROVING(void) (PORTA.IN & PIN4_bm)		//Returns true if rover's status is 'roving
														//Implied to be false during e-stop condition


//#define BASE_ROT_LIMIT



#endif /* XMEGAMACROS_H_ */