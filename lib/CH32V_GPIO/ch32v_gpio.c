/**
 *  CH32VX GPIO Library
 *
 *  file         : ch32v_gpio.c
 *  description  : ch32v gpio library main code
 *  Created on: Jan 12, 2023
 *  Author: larry
 *  https://github.com/bitbank2/CH32V_Experiments/tree/master
 *
 */

#include "ch32v_gpio.h"

// Function pointer for EXTI ISR function body, placeholder declaration
void (*f_body_exti0)(void);
void (*f_body_exti1)(void);
void (*f_body_exti2)(void);
void (*f_body_exti3)(void);
void (*f_body_exti4)(void);
void (*f_body_exti5)(void);
void (*f_body_exti6)(void);
void (*f_body_exti7)(void);
void (*f_body_exti8)(void);
void (*f_body_exti9)(void);
void (*f_body_exti10)(void);
void (*f_body_exti11)(void);
void (*f_body_exti12)(void);
void (*f_body_exti13)(void);
void (*f_body_exti14)(void);
void (*f_body_exti15)(void);
// EXTI ISR, placeholder declaration
void EXTI0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI9_5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI15_10_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

#ifdef BITBANG
uint8_t u8SDA_Pin, u8SCL_Pin;
int iDelay = 1;
#endif

void delay(int i)
{
	Delay_Ms(i);
}

// Arduino-like API defines and function wrappers for WCH MCUs

/*********************************************************************
 * @fn      pinMode
 *
 * @brief   Set pin mode, analog to Arduino API.
 * 
 * @param   u8Pin           Pin number with 2 x nibble numbering scheme (e.g 0xC7 = Pin C7)
 * @param   iMode           Pin Mode (can be either OUTPUT, INPUT, INPUT_PULLUP or INPUT_PULLDOWN)
 * 
 * @return  None
 */
void pinMode(uint8_t u8Pin, int iMode)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    if (u8Pin < 0xa0 || u8Pin > 0xdf) return; // invalid pin number

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 << (u8Pin & 0xf);
    if (iMode == OUTPUT)
    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    else if (iMode == INPUT)
    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    else if (iMode == INPUT_PULLUP)
    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    else if (iMode == INPUT_PULLDOWN)
    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    switch (u8Pin & 0xf0) {
    case 0xa0:
    	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    	break;
    case 0xb0:
    	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        GPIO_Init(GPIOB, &GPIO_InitStructure);
    	break;
    case 0xc0:
    	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        GPIO_Init(GPIOC, &GPIO_InitStructure);
    	break;
    case 0xd0:
    	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
        GPIO_Init(GPIOD, &GPIO_InitStructure);
    	break;
    }
} /* pinMode() */

/*********************************************************************
 * @fn      digitalRead
 *
 * @brief   Digital Read analog to Arduino API.
 * 
 * @param   u8Pin           Pin number with 2 x nibble numbering scheme (e.g 0xC7 = Pin C7)
 * 
 * @return  Binary value on digital input pin. 1 or 0
 */
uint8_t digitalRead(uint8_t u8Pin)
{
    uint32_t u32GPIO = GPIO_Pin_0 << (u8Pin & 0xf);
    uint8_t u8Value = 0;
    switch (u8Pin & 0xf0) {
    case 0xa0:
    	u8Value = GPIO_ReadInputDataBit(GPIOA, u32GPIO);
    	break;
    case 0xb0:
    	u8Value = GPIO_ReadInputDataBit(GPIOB, u32GPIO);
    	break;
    case 0xc0:
    	u8Value = GPIO_ReadInputDataBit(GPIOC, u32GPIO);
    	break;
    case 0xd0:
    	u8Value = GPIO_ReadInputDataBit(GPIOD, u32GPIO);
    	break;
    }
    return u8Value;
} /* digitalRead() */

/*********************************************************************
 * @fn      digitalWrite
 *
 * @brief   Digital Write analog to Arduino API.
 * 
 * @param   u8Pin           Pin number with 2 x nibble numbering scheme (e.g 0xC7 = Pin C7)
 * @param   u8Value         New value for digital output. Either Bit_SET or Bit_RESET (1 or 0). See @ref BitAction
 * 
 * @return  none
 */
void digitalWrite(uint8_t u8Pin, uint8_t u8Value)
{
	uint32_t u32GPIO = GPIO_Pin_0 << (u8Pin & 0xf);
	u8Value = (u8Value) ? Bit_SET : Bit_RESET;

	switch (u8Pin & 0xf0) {
	case 0xa0:
		GPIO_WriteBit(GPIOA, u32GPIO, u8Value);
		break;
	case 0xb0:
		GPIO_WriteBit(GPIOB, u32GPIO, u8Value);
		break;
	case 0xc0:
		GPIO_WriteBit(GPIOC, u32GPIO, u8Value);
		break;
	case 0xd0:
		GPIO_WriteBit(GPIOD, u32GPIO, u8Value);
		break;
	}
} /* digitalWrite() */

// TODO: Debounce example with button + LED + sysclock to detect bouncing
// TODO: EXTI function for other types of external interrupt (pvd, rtc, ...)
/*********************************************************************
 * @fn      pinInterrupt
 *
 * @brief   This function initializes the interrupt generation for external events on GPIO Pins.
 * 
 * @param   u8Pin           Pin number with 2 x nibble numbering scheme (e.g 0xC7 = Pin C7)
 * @param   iMode           Pin Mode (can be either INPUT, INPUT_PULLUP or INPUT_PULLDOWN)
 * @param   NewState        New state for interrupt generation after function call (either ENABLE or DISABLE) see @ref FunctionalState
 * @param   TriggerType     Trigger type (either EXTI_Trigger_Rising, EXTI_Trigger_Falling or EXTI_Trigger_Rising_Falling) see @ref EXTITrigger_TypeDef
 * @param   func            Pointer to function which shall be called upon interrupt, must be return type void and parameter void. Declare or define before passing it here as function name.
 * 
 * @return  none
 */
void pinInterrupt(uint8_t u8Pin, int iMode, FunctionalState NewState, EXTITrigger_TypeDef TriggerType, void (*func)(void))
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    if (u8Pin < 0xa0 || u8Pin > 0xdf) return; // invalid pin number
    // Get Pin number GPIO_Pin_X
    uint32_t u32GPIO = GPIO_Pin_0 << (u8Pin & 0xf);     // GPIO_Pin_X = EXTI_LineX
    // Convert GPIO_Pin_X to GPIO_PinSourceX
    uint32_t u32GPIO_temp = u32GPIO;
    uint8_t u8GPIO = 0;                                 // GPIO_PinSourceX
    while (u32GPIO_temp >>= 1) u8GPIO++;
    // Set pin to input
    GPIO_InitStructure.GPIO_Pin = u32GPIO;
    if (iMode == OUTPUT)
    	return; // invalid pin mode for interrupt
    else if (iMode == INPUT)
    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    else if (iMode == INPUT_PULLUP)
    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    else if (iMode == INPUT_PULLDOWN)
    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    // Port-specific enable
    switch (u8Pin & 0xf0) 
    {
        case 0xa0:
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, u8GPIO);
            break;
        case 0xb0:
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
            GPIO_Init(GPIOB, &GPIO_InitStructure);
            GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, u8GPIO);
            break;
        case 0xc0:
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, u8GPIO);
            break;
        case 0xd0:
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD, ENABLE);
            GPIO_Init(GPIOD, &GPIO_InitStructure);
            GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, u8GPIO);
            break;
    }
    // Configure EXTI Structure
    EXTI_InitStructure.EXTI_Line = u32GPIO;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = TriggerType;
    EXTI_InitStructure.EXTI_LineCmd = NewState;
    EXTI_Init(&EXTI_InitStructure);
    // Configure NVIC Structure
    if (u8GPIO == GPIO_PinSource0) { NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;   f_body_exti0 = func; }
    else if (u8GPIO == GPIO_PinSource1) { NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;   f_body_exti1 = func; }
    else if (u8GPIO == GPIO_PinSource2) { NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;   f_body_exti2 = func; }
    else if (u8GPIO == GPIO_PinSource3) { NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;   f_body_exti3 = func; }
    else if (u8GPIO == GPIO_PinSource4) { NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;   f_body_exti4 = func; }
    else if (u8GPIO == GPIO_PinSource5) { NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;   f_body_exti5 = func; }
    else if (u8GPIO == GPIO_PinSource6) { NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;   f_body_exti6 = func; }
    else if (u8GPIO == GPIO_PinSource7) { NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;   f_body_exti7 = func; }
    else if (u8GPIO == GPIO_PinSource8) { NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;   f_body_exti8 = func; }
    else if (u8GPIO == GPIO_PinSource9) { NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;   f_body_exti9 = func; }
    else if (u8GPIO == GPIO_PinSource10) { NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;   f_body_exti10 = func; }
    else if (u8GPIO == GPIO_PinSource11) { NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;   f_body_exti11 = func; }
    else if (u8GPIO == GPIO_PinSource12) { NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;   f_body_exti12 = func; }
    else if (u8GPIO == GPIO_PinSource13) { NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;   f_body_exti13 = func; }
    else if (u8GPIO == GPIO_PinSource14) { NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;   f_body_exti14 = func; }
    else if (u8GPIO == GPIO_PinSource15) { NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;   f_body_exti15 = func; }
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = NewState;
    NVIC_Init(&NVIC_InitStructure);
}

/*********************************************************************
 * @fn      EXTI0_IRQHandler
 *
 * @brief   This function handles EXTI0 ISR.
 *
 * @return  none
 */
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0)!=RESET)
    {
        (*f_body_exti0)();
        EXTI_ClearITPendingBit(EXTI_Line0);     /* Clear Flag */
    }
}

/*********************************************************************
 * @fn      EXTI1_IRQHandler
 *
 * @brief   This function handles EXTI1 ISR.
 *
 * @return  none
 */
void EXTI1_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line1)!=RESET)
    {
        (*f_body_exti1)();
        EXTI_ClearITPendingBit(EXTI_Line1);     /* Clear Flag */
    }
}

/*********************************************************************
 * @fn      EXTI2_IRQHandler
 *
 * @brief   This function handles EXTI2 ISR.
 *
 * @return  none
 */
void EXTI2_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line2)!=RESET)
    {
        (*f_body_exti2)();
        EXTI_ClearITPendingBit(EXTI_Line2);     /* Clear Flag */
    }
}

/*********************************************************************
 * @fn      EXTI3_IRQHandler
 *
 * @brief   This function handles EXTI3 ISR.
 *
 * @return  none
 */
void EXTI3_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line3)!=RESET)
    {
        (*f_body_exti3)();
        EXTI_ClearITPendingBit(EXTI_Line3);     /* Clear Flag */
    }
}

/*********************************************************************
 * @fn      EXTI4_IRQHandler
 *
 * @brief   This function handles EXTI4 ISR.
 *
 * @return  none
 */
void EXTI4_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line4)!=RESET)
    {
        (*f_body_exti4)();
        EXTI_ClearITPendingBit(EXTI_Line4);     /* Clear Flag */
    }
}

/*********************************************************************
 * @fn      EXTI9_5_IRQHandler
 *
 * @brief   This function handles EXTI9_5 ISR.
 *
 * @return  none
 */
void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line5)!=RESET)
    {
        (*f_body_exti5)();
        EXTI_ClearITPendingBit(EXTI_Line5);     /* Clear Flag */
    }
    else if(EXTI_GetITStatus(EXTI_Line6)!=RESET)
    {
        (*f_body_exti6)();
        EXTI_ClearITPendingBit(EXTI_Line6);     /* Clear Flag */
    }
    else if(EXTI_GetITStatus(EXTI_Line7)!=RESET)
    {
        (*f_body_exti7)();
        EXTI_ClearITPendingBit(EXTI_Line7);     /* Clear Flag */
    }
    else if(EXTI_GetITStatus(EXTI_Line8)!=RESET)
    {
        (*f_body_exti8)();
        EXTI_ClearITPendingBit(EXTI_Line8);     /* Clear Flag */
    }
    else if(EXTI_GetITStatus(EXTI_Line9)!=RESET)
    {
        (*f_body_exti9)();
        EXTI_ClearITPendingBit(EXTI_Line9);     /* Clear Flag */
    }
}

/*********************************************************************
 * @fn      EXTI15_10_IRQHandler
 *
 * @brief   This function handles EXTI15_10 ISR.
 *
 * @return  none
 */
void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line10)!=RESET)
    {
        (*f_body_exti10)();
        EXTI_ClearITPendingBit(EXTI_Line10);     /* Clear Flag */
    }
    else if(EXTI_GetITStatus(EXTI_Line11)!=RESET)
    {
        (*f_body_exti11)();
        EXTI_ClearITPendingBit(EXTI_Line11);     /* Clear Flag */
    }
    else if(EXTI_GetITStatus(EXTI_Line12)!=RESET)
    {
        (*f_body_exti12)();
        EXTI_ClearITPendingBit(EXTI_Line12);     /* Clear Flag */
    }
    else if(EXTI_GetITStatus(EXTI_Line13)!=RESET)
    {
        (*f_body_exti13)();
        EXTI_ClearITPendingBit(EXTI_Line13);     /* Clear Flag */
    }
    else if(EXTI_GetITStatus(EXTI_Line14)!=RESET)
    {
        (*f_body_exti14)();
        EXTI_ClearITPendingBit(EXTI_Line14);     /* Clear Flag */
    }
    else if(EXTI_GetITStatus(EXTI_Line15)!=RESET)
    {
        (*f_body_exti15)();
        EXTI_ClearITPendingBit(EXTI_Line15);     /* Clear Flag */
    }
}

#ifdef BITBANG
uint8_t SDA_READ(void)
{
	return digitalRead(u8SDA_Pin);
}
void SDA_HIGH(void)
{
	pinMode(u8SDA_Pin, INPUT);
}
void SDA_LOW(void)
{
	pinMode(u8SDA_Pin, OUTPUT);
	digitalWrite(u8SDA_Pin, 0);
}
void SCL_HIGH(void)
{
	pinMode(u8SCL_Pin, INPUT);
}
void SCL_LOW(void)
{
	pinMode(u8SCL_Pin, OUTPUT);
	digitalWrite(u8SCL_Pin, 0);
}
void I2CSetSpeed(int iSpeed)
{
	if (iSpeed >= 400000) iDelay = 1;
	else if (iSpeed >= 100000) iDelay = 10;
	else iDelay = 20;
}
void I2CInit(uint8_t u8SDA, uint8_t u8SCL, int iSpeed)
{
	u8SDA_Pin = u8SDA;
	u8SCL_Pin = u8SCL;
	if (iSpeed >= 400000) iDelay = 1;
	else if (iSpeed >= 100000) iDelay = 10;
	else iDelay = 20;
} /* I2CInit() */

void my_sleep_us(int iDelay)
{
	Delay_Us(iDelay);
}
// Transmit a byte and read the ack bit
// if we get a NACK (negative acknowledge) return 0
// otherwise return 1 for success
//

int i2cByteOut(uint8_t b)
{
uint8_t i, ack;

for (i=0; i<8; i++)
{
//    my_sleep_us(iDelay);
    if (b & 0x80)
      SDA_HIGH(); // set data line to 1
    else
      SDA_LOW(); // set data line to 0
    b <<= 1;
//    my_sleep_us(iDelay);
    SCL_HIGH(); // clock high (slave latches data)
    my_sleep_us(iDelay);
    SCL_LOW(); // clock low
    my_sleep_us(iDelay);
} // for i
//my_sleep_us(iDelay);
// read ack bit
SDA_HIGH(); // set data line for reading
//my_sleep_us(iDelay);
SCL_HIGH(); // clock line high
my_sleep_us(iDelay); // DEBUG - delay/2
ack = SDA_READ();
//my_sleep_us(iDelay);
SCL_LOW(); // clock low
my_sleep_us(iDelay); // DEBUG - delay/2
SDA_LOW(); // data low
return (ack == 0); // a low ACK bit means success
} /* i2cByteOut() */

//
// Receive a byte and read the ack bit
// if we get a NACK (negative acknowledge) return 0
// otherwise return 1 for success
//
uint8_t i2cByteIn(uint8_t bLast)
{
uint8_t i;
uint8_t b = 0;

     SDA_HIGH(); // set data line as input
     for (i=0; i<8; i++)
     {
         my_sleep_us(iDelay); // wait for data to settle
         SCL_HIGH(); // clock high (slave latches data)
         my_sleep_us(iDelay);
         b <<= 1;
         if (SDA_READ() != 0) // read the data bit
           b |= 1; // set data bit
         SCL_LOW(); // clock low
     } // for i
     if (bLast)
        SDA_HIGH(); // last byte sends a NACK
     else
        SDA_LOW();
//     my_sleep_us(iDelay);
     SCL_HIGH(); // clock high
     my_sleep_us(iDelay);
     SCL_LOW(); // clock low to send ack
     my_sleep_us(iDelay);
//     SDA_HIGH();
     SDA_LOW(); // data low
  return b;
} /* i2cByteIn() */
//
// Send I2C STOP condition
//
void i2cEnd(void)
{
   SDA_LOW(); // data line low
   my_sleep_us(iDelay);
   SCL_HIGH(); // clock high
   my_sleep_us(iDelay);
   SDA_HIGH(); // data high
   my_sleep_us(iDelay);
} /* i2cEnd() */

int i2cBegin(uint8_t addr, uint8_t bRead)
{
   int rc;
//   SCL_HIGH();
//   my_sleep_us(iDelay);
   SDA_LOW(); // data line low first
   my_sleep_us(iDelay);
   SCL_LOW(); // then clock line low is a START signal
   addr <<= 1;
   if (bRead)
      addr++; // set read bit
   rc = i2cByteOut(addr); // send the slave address and R/W bit
   return rc;
} /* i2cBegin() */

void I2CWrite(uint8_t addr, uint8_t *pData, int iLen)
{
uint8_t b;
int rc;

   i2cBegin(addr, 0);
   rc = 1;
   while (iLen && rc == 1)
   {
      b = *pData++;
      rc = i2cByteOut(b);
      if (rc == 1) // success
      {
         iLen--;
      }
   } // for each byte
   i2cEnd();
//return (rc == 1) ? (iOldLen - iLen) : 0; // 0 indicates bad ack from sending a byte
} /* I2CWrite() */

void I2CRead(uint8_t addr, uint8_t *pData, int iLen)
{
   i2cBegin(addr, 1);
   while (iLen--)
   {
      *pData++ = i2cByteIn(iLen == 0);
   } // for each byte
   i2cEnd();
} /* I2CRead() */

int I2CTest(uint8_t addr)
{
int response = 0;

   if (i2cBegin(addr, 0)) // try to write to the given address
   {
      response = 1;
   }
   i2cEnd();
return response;
} /* I2CTest() */

//
// Read N bytes starting at a specific I2C internal register
// returns 1 for success, 0 for error
//
int I2CReadRegister(uint8_t iAddr, uint8_t u8Register, uint8_t *pData, int iLen)
{
//  int rc;

  I2CWrite(iAddr, &u8Register, 1);
  Delay_Ms(5);
  I2CRead(iAddr, pData, iLen);

return 1; // returns 1 for success, 0 for error
} /* I2CReadRegister() */

#else // hardware I2C

void I2CInit(int iSpeed)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    I2C_InitTypeDef I2C_InitStructure={0};
    //I2C_DeInit(I2C1);

#ifdef __CH32V20x_H
    // Fixed to pins B6 = SCL, B7 = SDA on TSSOP20 (remapped)
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE );
    //GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );
    // i2c reset
//    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
//    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );
//    I2C_Cmd(I2C1, DISABLE);
//    AFIO->PCFR1 = AFIO_PCFR1_SWJ_CFG_DISABLE; // disable SWDIO to allow GPIO usage
//    Delay_Ms(5);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );
#else // must be CH32V003
    // Fixed to pins C1=SDA/C2=SCL
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );
#endif

    I2C_DeInit(I2C1);
 //   I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_ClockSpeed = iSpeed;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;
    I2C_InitStructure.I2C_OwnAddress1 = 0x02; //address; sender's unimportant address
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init( I2C1, &I2C_InitStructure );

    I2C_Cmd( I2C1, ENABLE );

    I2C_AcknowledgeConfig( I2C1, ENABLE );
//    while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );
} /* I2CInit() */

void I2CRead(uint8_t u8Addr, uint8_t *pData, int iLen)
{
    I2C_GenerateSTART( I2C1, ENABLE );
    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );

    I2C_Send7bitAddress( I2C1, u8Addr<<1, I2C_Direction_Receiver );

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) );

    while(iLen)
    {
        if( I2C_GetFlagStatus( I2C1, I2C_FLAG_RXNE ) !=  RESET )
        {
            pData[0] = I2C_ReceiveData( I2C1 );
            pData++;
            iLen--;
        }
    }

    I2C_GenerateSTOP( I2C1, ENABLE );


} /* I2CRead() */

void I2CWrite(uint8_t u8Addr, uint8_t *pData, int iLen)
{
    while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );
    I2C_GenerateSTART( I2C1, ENABLE );
    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );

    I2C_Send7bitAddress( I2C1, u8Addr<<1, I2C_Direction_Transmitter );

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );

    while(iLen)
    {
        if( I2C_GetFlagStatus( I2C1, I2C_FLAG_TXE ) !=  RESET )
        {
            I2C_SendData( I2C1, pData[0] );
            pData++;
            iLen--;
        }
    }

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
    I2C_GenerateSTOP( I2C1, ENABLE );

} /* I2CWrite() */

int I2CTest(uint8_t u8Addr)
{
    I2C_GenerateSTART( I2C1, ENABLE );
    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );

    I2C_Send7bitAddress( I2C1, u8Addr<<1, I2C_Direction_Transmitter );

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );
    I2C_GenerateSTOP( I2C1, ENABLE );
    return (I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE) != RESET); // 0 = fail, 1 = succeed

} /* I2CTest() */
#endif // BITBANG

// Put CPU into standby mode for a multiple of 82ms tick increments
// max ticks value is 63
void Standby82ms(uint8_t iTicks)
{
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // init external interrupts
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    EXTI_InitStructure.EXTI_Line = EXTI_Line9;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Init GPIOs
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;

    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // init wake up timer and enter standby mode
    RCC_LSICmd(ENABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
//    PWR_AWU_SetPrescaler(PWR_AWU_Prescaler_10240);
//    PWR_AWU_SetWindowValue(iTicks);
//    PWR_AutoWakeUpCmd(ENABLE);
//    PWR_EnterSTANDBYMode(PWR_STANDBYEntry_WFE);

    GPIO_DeInit(GPIOA);
    GPIO_DeInit(GPIOC);
    GPIO_DeInit(GPIOD);

} /* Standby82ms() */

//
// Ramp an LED brightness with PWM from 0 to 50%
// The period represents the total up+down time in milliseconds
//
void breatheLED(uint8_t u8Pin, int iPeriod)
{
	int i, j, iStep, iCount, iOnTime;

	pinMode(u8Pin, OUTPUT);
	// Use a pwm freq of 1000hz and 50 steps up then 50 steps down
	iStep = 10000/iPeriod; // us per step
	iCount = iPeriod / 20;
	// ramp up
	iOnTime = 0;
	for (i=0; i<iCount; i++) {
		for (j=0; j<20; j++) { // 20ms per step
			digitalWrite(u8Pin, 1); // on period
			Delay_Us(iOnTime);
			digitalWrite(u8Pin, 0); // off period
			Delay_Us(1000 - iOnTime);
		} // for j
		iOnTime += iStep;
	} // for i
	// ramp down
	iOnTime = 500;
	for (i=0; i<iCount; i++) {
		for (j=0; j<20; j++) { // 20ms per step
			digitalWrite(u8Pin, 1); // on period
			Delay_Us(iOnTime);
			digitalWrite(u8Pin, 0); // off period
			Delay_Us(1000 - iOnTime);
		} // for j
		iOnTime -= iStep;
	} // for i
} /* breatheLED() */

void SPI_begin(int iSpeed, int iMode)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    SPI_InitTypeDef SPI_InitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; // SPI1 CLK
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOA, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; // SPI1 MOSI
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOA, &GPIO_InitStructure );

    SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = (iMode & 2) ? SPI_CPOL_High : SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = (iMode & 1) ? SPI_CPHA_2Edge : SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    if (iSpeed >= (SystemCoreClock/2))
    	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    else if (iSpeed >= (SystemCoreClock/4))
    	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    else if (iSpeed >= (SystemCoreClock/8))
    	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    else if (iSpeed >= (SystemCoreClock/16))
    	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    else if (iSpeed >= (SystemCoreClock/32))
    	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    else if (iSpeed >= (SystemCoreClock/64))
    	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    else if (iSpeed >= (SystemCoreClock/128))
    	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    else
    	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init( SPI1, &SPI_InitStructure );

    SPI_Cmd( SPI1, ENABLE );

} /* SPI_begin() */

// polling write
void SPI_write(uint8_t *pData, int iLen)
{
	int i = 0;

    while (i < iLen)
    {
    	if ( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) != RESET )
          SPI_I2S_SendData( SPI1, pData[i++] );
    }
    // wait until transmit empty flag is true
    while (SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) == RESET)
    {};
    while (SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_BSY ) == SET)
    {}; // wait until it's not busy

} /* SPI_write() */