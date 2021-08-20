/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include	"project_config.h"


/*_____ D E C L A R A T I O N S ____________________________________________*/
extern int IsDebugFifoEmpty(void);
/*_____ D E F I N I T I O N S ______________________________________________*/

volatile uint32_t BitFlag = 0;

uint32_t counter_tick = 0;
/*_____ M A C R O S ________________________________________________________*/

#define FMC_CUSTOM_APROM_BASE          		(0x00008000UL)

#define FMC_CUSTOM_STORE_APROM01_BASE       	(0x00010000UL)
#define FMC_CUSTOM_STORE_APROM02_BASE       	(0x00012000UL)

#define CONFIG0_ADDR                			(FMC_CONFIG_BASE)
#define CONFIG1_ADDR                			(CONFIG0_ADDR + 4)

#define USE_EMULATE_AUTO_JUMP
//#define USE_MANUAL_UPDATE_FW

/*_____ F U N C T I O N S __________________________________________________*/

void tick_counter(void)
{
	counter_tick++;
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void compare_buffer(uint8_t *src, uint8_t *des, int nBytes)
{
    uint16_t i = 0;	
	
    for (i = 0; i < nBytes; i++)
    {
        if (src[i] != des[i])
        {
            printf("error idx : %4d : 0x%2X , 0x%2X\r\n", i , src[i],des[i]);
			set_flag(flag_error , ENABLE);
        }
    }

	if (!is_flag_set(flag_error))
	{
    	printf("%s finish \r\n" , __FUNCTION__);	
		set_flag(flag_error , DISABLE);
	}

}

void reset_buffer(void *dest, unsigned int val, unsigned int size)
{
    uint8_t *pu8Dest;
//    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;

	#if 1
	while (size-- > 0)
		*pu8Dest++ = val;
	#else
	memset(pu8Dest, val, size * (sizeof(pu8Dest[0]) ));
	#endif
	
}

void copy_buffer(void *dest, void *src, unsigned int size)
{
    uint8_t *pu8Src, *pu8Dest;
    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;
    pu8Src  = (uint8_t *)src;


	#if 0
	  while (size--)
	    *pu8Dest++ = *pu8Src++;
	#else
    for (i = 0; i < size; i++)
        pu8Dest[i] = pu8Src[i];
	#endif
}

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void  dump_buffer_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02X ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}

void delay(uint16_t dly)
{
/*
	delay(100) : 14.84 us
	delay(200) : 29.37 us
	delay(300) : 43.97 us
	delay(400) : 58.5 us	
	delay(500) : 73.13 us	
	
	delay(1500) : 0.218 ms (218 us)
	delay(2000) : 0.291 ms (291 us)	
*/

	while( dly--);
}


void delay_ms(uint16_t ms)
{
	TIMER_Delay(TIMER0, 1000*ms);
}

void rtc_write_magic_tag(uint8_t tag)
{
    RTC_EnableSpareAccess();

    RTC->RWEN = RTC_WRITE_KEY;
    while(!(RTC->RWEN & RTC_RWEN_RWENF_Msk));
    
    RTC_WRITE_SPARE_REGISTER(0, tag);
    
    printf("Write MagicTag <0x%02X>\r\n", tag);
}

uint8_t rtc_read_magic_tag(void)
{
    uint8_t tag;
    
    RTC_EnableSpareAccess();

    RTC->RWEN = RTC_WRITE_KEY;
    while(!(RTC->RWEN & RTC_RWEN_RWENF_Msk));
    
    tag =  RTC_READ_SPARE_REGISTER(0);

    printf("Raed MagicTag <0x%02X>\r\n", tag);
    
    return tag;
}

uint8_t check_reset_source(void)
{
    uint8_t tag;
    uint32_t src = SYS_GetResetSrc();

    SYS->RSTSTS |= 0x1FF;
    printf("Reset Source <0x%08X>\r\n", src);
   
    tag = rtc_read_magic_tag();
    
    if (src & SYS_RSTSTS_PORF_Msk) 
	{
        SYS_ClearResetSrc(SYS_RSTSTS_PORF_Msk);
        
        if (tag == 0xA5) 
		{
            rtc_write_magic_tag(0);
            printf("Enter BOOTLOADER from APPLICATION\r\n");
            return TRUE;
        } 
		else if (tag == 0xBB) 
		{
            rtc_write_magic_tag(0);
            printf("from other source\r\n");
            return FALSE;
        } 
		else 
		{
            printf("Enter BOOTLOADER from POR\r\n");
            return FALSE;
        }
    } 
	else if (src & SYS_RSTSTS_PINRF_Msk)
	{
        SYS_ClearResetSrc(SYS_RSTSTS_PINRF_Msk);
        printf("Enter BOOTLOADER from nRESET pin\r\n");
        return FALSE;
    }
    
    printf("Enter BOOTLOADER from unhandle reset source\r\n");
    return FALSE;
}

void FMC_Enable(void)
{
    SYS_UnlockReg();                   /* Unlock register lock protect */

    FMC_Open();                        /* Enable FMC ISP function */
}

void FMC_Disable(void)
{
    FMC_Close();                       /* Disable FMC ISP function */

    SYS_LockReg();                     /* Lock protected registers */
}


int  load_image_to_flash(uint32_t image_base, uint32_t image_limit, uint32_t flash_addr, uint32_t max_size)
{
    uint32_t   i, j, u32Data, u32ImageSize, *pu32Loader;

    u32ImageSize = max_size;

    printf("image_base address 0x%2X...\r\n", image_base);
    printf("Program image to flash address 0x%2X...\r\n", flash_addr);

	FMC_Enable();

    /*
     * program the whole image to specified flash area
     */
    pu32Loader = (uint32_t *)image_base;
    for (i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {

        FMC_Erase(flash_addr + i);
        for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            FMC_Write(flash_addr + i + j, pu32Loader[(i + j) / 4]);
        }
    }
    printf("OK.\nVerify ...");

    /* Verify loader */
    for (i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            u32Data = FMC_Read(flash_addr + i + j);

            if (u32Data != pu32Loader[(i+j)/4])
            {
                printf("data mismatch on 0x%x, [0x%x], [0x%x]\n", flash_addr + i + j, u32Data, pu32Loader[(i+j)/4]);
                return -1;
            }

            if (i + j >= u32ImageSize)
                break;
        }
    }
    printf("OK.\n");

    printf("boot loader : %s\r\n", __FUNCTION__);

    while(IsDebugFifoEmpty() == 0);

	FMC_Disable();
	
    return 0;
}


int set_IAP_boot_mode(void)
{
    uint32_t  au32Config[2];

    if (FMC_ReadConfig(au32Config, 2) < 0)
    {
        printf("\nboot loader : Read User Config failed!\n");
        return -1;
    }

    /*
        CONFIG0[7:6]
        00 = Boot from LDROM with IAP mode.
        01 = Boot from LDROM without IAP mode.
        10 = Boot from APROM with IAP mode.
        11 = Boot from APROM without IAP mode.
    */
    if (au32Config[0] & 0x40)      /* Boot from APROM with IAP mode */
    {
        FMC_ENABLE_CFG_UPDATE();
        au32Config[0] &= ~0x40;
        FMC_WriteConfig(au32Config, 2);

        // Perform chip reset to make new User Config take effect
        SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;
    }
    return 0;
}

void Read_BS(void)
{
	uint32_t res = 0;


	printf("boot loader >>>>> make sure CONFIG set as [APROM with IAP]\r\n");
	printf("boot loader >>>>> APP_01 in KEIL linker option R/O base :0x00008000\r\n");
	printf("boot loader >>>>> APP_02 in KEIL linker option R/O base :0x00008000\r\n");



	res = FMC_GetBootSource();
	printf("boot loader : %s : %d\r\n" , __FUNCTION__ ,res);

    res = FMC_ReadCID();
    printf("boot loader : Company ID ............................ [0x%08X]\r\n", res);

    res = FMC_ReadPID();
    printf("boot loader : Product ID ............................ [0x%08X]\r\n", res);

    /* Read User Configuration */
    printf("boot loader : User Config 0 ......................... [0x%08X]\r\n", FMC_Read(CONFIG0_ADDR));
    /* Read User Configuration CONFIG1 */
    printf("boot loader : User Config 1 ......................... [0x%08X]\r\n", FMC_Read(CONFIG1_ADDR));

    while(IsDebugFifoEmpty() == 0);
	
}

//
// resetPeripherals
//
void resetPeripherals(void)
{
    /* Note: Remember disable all of peripherals interrupt used in the application 
        before jump to app.*/
	NVIC_DisableIRQ(UART0_IRQn);
	NVIC_DisableIRQ(TMR1_IRQn);


	 
    /* Note: Remember reset all of peripherals used in the application 
        before jump to app. */
    SYS_ResetModule(UART0_IRQn);
    SYS_ResetModule(TMR1_RST);
}

void Jump_To_Application(void)
{
    printf("boot loader : %s\r\n", __FUNCTION__);
    while(IsDebugFifoEmpty() == 0);

	resetPeripherals();

	FMC_Enable();
    FMC_SetVectorPageAddr(FMC_CUSTOM_APROM_BASE);

    /* Reset CPU only to reset to new vector page */
    SYS_ResetCPU();

    /* Reset System to reset to new vector page. */
    //NVIC_SystemReset();

	FMC_Disable();

    while (SYS->PDID) __WFI();

}


void set_FW_update_TimeOut(void)
{

    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;

	FMC_ENABLE_ISP();
    FMC_ENABLE_AP_UPDATE();//FMC->ISPCTL |= (FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk);


    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
//	SysTick_Config(300 * CyclesPerUs);


}

void Check_if_FW_update_TimeOut(void)
{
    if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
    {
	    printf("boot loader : %s\r\n", __FUNCTION__);
	    while(IsDebugFifoEmpty() == 0);
	
        Jump_To_Application();
    }

}

void Manual_update_FW(void)
{
    uint8_t u8Ch;

	printf("boot loader : selection option to manual update firmware or jump to application.\r\n");
	printf("boot loader >>>>> pre-progammming APP_01 to 0x10000\r\n");
	printf("boot loader >>>>> pre-progammming APP_02 to 0x12000\r\n");
	
	
	printf("boot loader : [1] , update APP_01 to 0x8000 ,  (APP_01 store in 0x10000)\r\n");
	printf("boot loader : [2] , update APP_02 to 0x8000 , (APP_02 store in 0x12000)\r\n");
	
	u8Ch = getchar();

	
}


void GPIO_Init (void)
{
    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);

    GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT);	
}


void TMR1_IRQHandler(void)
{
	static uint32_t LOG = 0;

	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 10000) == 0)
		{
        	printf("boot loader : %s : %4d\r\n",__FUNCTION__,LOG++);
		}

		if ((get_tick() % 250) == 0)
		{			
			PH0 ^= 1;
			PH4 ^= 1;
		}

		if ((get_tick() % 50) == 0)
		{

		}	
    }
}


void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res == 'x' || res == 'X')
	{
		NVIC_SystemReset();
	}

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{

			#if defined (USE_MANUAL_UPDATE_FW)
			case '1':
				load_image_to_flash(FMC_CUSTOM_STORE_APROM01_BASE,0x2000,FMC_CUSTOM_APROM_BASE,0x2000);		
				Jump_To_Application();
				break;

			case '2':
				load_image_to_flash(FMC_CUSTOM_STORE_APROM02_BASE,0x2000,FMC_CUSTOM_APROM_BASE,0x2000);			
				Jump_To_Application();
				break;	
			#endif


			case 'X':
			case 'x':
			case 'Z':
			case 'z':
				NVIC_SystemReset();		
				break;
		}
	}
}


void UART0_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
//	UART_SetTimeoutCnt(UART0, 20);

//	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
//	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);

	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	#endif
}

void Custom_Init(void)
{	
	//EVM LED
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);

	// EVM M483 LED
	GPIO_SetMode(PH,BIT4,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT5,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT6,GPIO_MODE_OUTPUT);	
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk|CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk|CLK_STATUS_HIRCSTB_Msk);

    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk|CLK_PWRCTL_LIRCEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk|CLK_STATUS_LIRCSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    CLK_EnableModuleClock(RTC_MODULE);
    CLK_SetModuleClock(RTC_MODULE, CLK_CLKSEL3_RTCSEL_LXT,  NULL);

	    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

	UART0_Init();
	Custom_Init();	
	TIMER1_Init();

	FMC_Enable();
	set_IAP_boot_mode();
	Read_BS();
	set_FW_update_TimeOut();
	FMC_Disable();
	
	#if defined (USE_MANUAL_UPDATE_FW)
	Manual_update_FW();
	#endif

	check_reset_source();
	
    while(1)
    {
		#if defined (USE_EMULATE_AUTO_JUMP)
		if (is_flag_set(flag_update_fw))
		{
			// process update firmwar flow
			break;
		}

		
		// if no need to update firmware , wait for timeout to app.
		Check_if_FW_update_TimeOut();
		#endif
		
    }

    /* Got no where to go, just loop forever */
    while(1)
    {
    	printf("boot loader : while(1)\r\n");
    }

	
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
