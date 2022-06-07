void SetSysClock(void)
{
	uint32_t status = 0;

  /* Enable HSI */
  RCC->CR |= (uint32_t)1;

  /* Wait till HSI is ready and if Time out is reached exit */
 while(!(RCC->CR & (uint32_t)2)){} //CR bit 1 HSIRDY sets when HSI oscillator is stable. p141

  if ((RCC->CR & (uint32_t)2) != 0) //if CR bit 1 HSIDRY high when oscillator is stable.
  {
    status = 1;
  }
  else
  {
    status = 0;
  }

  if (status == 1)
  {
    /*  PLL configuration: PLLCLK = (HSI * 4)/2 = 32 MHz */
	  //PLLSCR bit 16, PLLMUL bits 18-21, PLLDIV bits 22,23.
	  //0000 0000 1111 1101 0000 0000 0000 0000 corresponding bits
	  //these bits PLLSRC, PLLMUL and PLLDIV cleared
	  RCC->CFGR &= ~(0x00FD0000);
	  //PLLMUL must be 0001 when multiplication=4 and PLLDIV must be 01 when division=2
	  //PLLSCR bit 16 must be 1 --> HSE oscillator clock selected as PLL input clock (done in previous line)
	  //0000 0000 0100 0100 0000 0000 0000 0000
	  RCC->CFGR |= 0x00440000;
  }

  else
  {
    /* If HSI fails to start-up, the application will have wrong clock
    configuration. User can add here some code to deal with this error */
  }
  
  /*64-bit access is configured by setting the ACC64 bit in the Flash access control register (FLASH_ACR).
   *This access mode accelerates the execution of program operations.*/
  FLASH->ACR |= (uint32_t)4; //ACC64 bit 2, 64-bit access. 64-bit access is used to improve the performance. p84
    /*Prefetch is enabled by setting the PRFTEN bit in the FLASH_ACR register.
    *This feature is useful if at least one wait state is needed to access the Flash memory.
	*Figure 5 shows the execution of sequential 32-bit instructions*/
  FLASH->ACR |= (uint32_t)2; //PRFTEN bit 1, prefetch enable. p84
  FLASH->ACR |= (uint32_t)1; //LATENCY one wait state bit 0. One wait state enabled. p84. p59

  RCC->APB1ENR |= 0x10000000; //bit 28 PWREN: Power interface clock enable. p158. p101
  PWR->CR = (uint32_t)(1<<11); //Bits 12:11 VOS[1:0]: Voltage scaling range selection, 01: 1.8 V (range 1). p121
  	/*A delay is required for the internal regulator to be ready after the voltage range is changed.
	The VOSF bit indicates that the regulator has reached the voltage level defined with bits VOS
	of PWR_CR register. p102 1.8V needed for 32 MHz clock and lower voltages to save power.

	0: Regulator is ready in the selected voltage range
	1: Regulator voltage output is changing to the required VOS level.
	*/
  while((PWR->CSR & (uint32_t)(1<<4)) != 0){} //bit 4 VOSF: Voltage Scaling select flag. p125
    
  RCC->CFGR &=(uint32_t)~(1<<7); //Bits 7:4 HPRE[3:0]: AHB prescaler. 0xxx: SYSCLK not divided. p144
  RCC->CFGR &=(uint32_t)~(1<<13); //Bits 13:11 PPRE2[2:0]: APB high-speed prescaler (APB2). p144
  RCC->CFGR &=(uint32_t)~(1<<10); //Bits 10:8 PPRE1[2:0]: APB low-speed prescaler (APB1)p. 144

  RCC->CR |= (1<<24); //Bit 24 PLLON: PLL enable. p140
  while((RCC->CR & (uint32_t)(1<<25)) == 0){} //Bit 25 PLLRDY: PLL clock ready flag. p140
    
  /* Select PLL as system clock source */
  RCC->CFGR |= (uint32_t)3; //Bits 1:0 SW[1:0]: System clock switch, 11: PLL used as system clock
    
  /*Bits 3:2 SWS[1:0]: System clock switch status, 11: PLL used as system clock.
   * These bits are set and cleared by hardware to indicate which clock source is used as
	system clock. 11: PLL used as system clock. p145.
   */
  while ((RCC->CFGR & (uint32_t)(3<<2))==0){}
}