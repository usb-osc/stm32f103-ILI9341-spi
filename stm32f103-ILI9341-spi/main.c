#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h> // timer
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>
#include <misc.h> // interrupts
#include <math.h>
#include "lib/ILI9341/text.h"
#include "lib/tinystd/itoa.h"
#include "lib/encoder/encoder.h"


#define M_PI 3.14159

int hours = 11, minutes = 59, seconds = 56, mseconds = 0;

void InitializeTimer()
{
  // see ARM Timer tutorial https://visualgdb.com/tutorials/arm/stm32/timers/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseInitTypeDef timerInitStructure;
  timerInitStructure.TIM_Prescaler = 7200-1; // the main clock is 72,000,000 so we'll prescale to 72,000,000 / 7,200 = 10,000 Hz
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Period = 1000-1; // of the TIM_Prescaler/1000 scaled clock, we actually have (10 tick per second)
  timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &timerInitStructure);
  TIM_Cmd(TIM2, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // enable timter interrupt updates
}

void InitializeLED()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void EnableTimerInterrupt(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void timeToDigits()
{
  unsigned char lb_colon[16] = ":";
  unsigned char lb_night[16] = "Good night\n";
  unsigned char lb_morning[16] = "Good morning";
  unsigned char lb_day[16] = "Good day";
  unsigned char lb_evening[16] = "Good evening";
  unsigned char lb_empty[16] = {"             "};
  unsigned char times[16];

  LCD_setCursor(25, 90);
  LCD_setTextColor(YELLOW);
  LCD_setTextSize(4);
  itoa(hours/10, times, 10); LCD_writeString(times);
  itoa(hours%10, times, 10); LCD_writeString(times);
  LCD_writeString(lb_colon);
  LCD_setTextColor(WHITE);
  itoa(minutes/10, times, 10); LCD_writeString(times);
  itoa(minutes%10, times, 10); LCD_writeString(times);
  LCD_writeString(lb_colon);
  LCD_setTextColor(GREEN);
  itoa(seconds/10, times, 10); LCD_writeString(times);
  itoa(seconds%10, times, 10); LCD_writeString(times);
  LCD_setTextColor(WHITE);
  LCD_setTextSize(3);

  if(hours<4)
  {
    LCD_setCursor(10, 130);
    LCD_writeString(lb_night);
  }
  if(hours>=4 && hours<12)
  {
    LCD_setCursor(10, 130);
    LCD_writeString(lb_morning);
  }
  if(hours>=12 && hours<18)
  {
    LCD_setCursor(10, 130);
    LCD_writeString(lb_day);
  }
  if(hours>=18 && hours<=23)
  {
    LCD_setCursor(10, 130);
    LCD_writeString(lb_evening);
  }
  if(!minutes && !seconds && !mseconds)
  {
    LCD_setCursor(10, 130);
    LCD_writeString(lb_empty);
  }
}

void timeIncrement()
{
  mseconds++;
  if(mseconds >= 10)
  {
    mseconds=0;
    seconds++;
  }
  if (seconds >= 60)
  {
    seconds = 0; // the 60'th second is actually 0
    minutes++;
    if (minutes >= 60)
    {
      minutes = 0; // the 60'th minute is actually 0
      hours++;
      if (hours >= 24)
      {
        hours = 0; // the 24'th hour is actually 0
      }
    }
  }
}

volatile int ledStatus = 0;
//extern "C" void TIM2_IRQHandler() //Note that you only need extern C if you are building a C++ program.
void TIM2_IRQHandler() //Note that you only need extern C if you are building a C++ program.
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    // GPIO_ToggleBits(GPIOC, GPIO_Pin_13); // appears to be no ToggleBits for stm32f10x, so we'll do this a bit more manually with ledStatus
    if (ledStatus == 0)
    {
      ledStatus = 1;
      GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
    }
    else
    {
      ledStatus = 0;
      GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
    }
  }
  timeIncrement();
}

int main(void)
{
  InitializeLED();
  InitializeTimer();
  EnableTimerInterrupt();
  LCD_init();
  ENC_init();
  LCD_setOrientation(ORIENTATION_PORTRAIT_MIRROR); //ORIENTATION_PORTRAIT_MIRROR ORIENTATION_PORTRAIT
  LCD_fillScreen(BLACK);

  unsigned char lb_hello[16] = "hello\n";
  unsigned char lb_enc[16] = "encoder: ";

  unsigned char buf[16];

  LCD_setTextBgColor(BLACK);
  LCD_setTextSize(3);

  u16 ov = 111;
  u16 encVal;

  while(1)
  {
    LCD_setTextColor(WHITE);
    LCD_setCursor(20, 45);
    LCD_writeString(lb_hello);
    if (ov != (encVal = ENC_getValue()))
    {
      LCD_fillRect(0, 0, 120, 24, CYAN);
      LCD_fillRect(120, 0, 120, 24, MAGENTA);
      itoa(encVal, buf, 10);
      LCD_setTextColor(RED);
      LCD_setTextSize(2);
      LCD_setCursor(20, 200);
      LCD_writeString(lb_enc);
      LCD_setTextSize(3);
      LCD_setTextColor(BLUE);
      LCD_setCursor(120, 195);
      LCD_writeString(buf);

      for(uint16_t i = 5; i<275; i+=30)
      {
        LCD_drawLine( i, 5, i, 315, LGRAY);
      }

      for(uint16_t i = 5; i<315; i+=30)
      {
        LCD_drawLine( 5, i, 275, i, LGRAY);
      }

      for(uint16_t i = 0; i < 320; i++) // count point
      {
         //LCD_fillCircle(i, sin(i), 2, GREEN);
         LCD_fillCircle(i, (100*i/180)+120, 2, GREEN);
         //LCD_fillCircle(i, 80*sin(30*0.2*M_PI*i/180)+120, 2, GREEN);
      }
      
      ov = encVal;
    }
    timeToDigits();
  }
}
