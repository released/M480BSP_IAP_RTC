# M480BSP_IAP_RTC
 M480BSP_IAP_RTC

update @ 2021/08/20

1. under APROM , with 2 project (boot loader and application)

	under application project , KEILC R/O base : 0x00008000

	under boot loader project , two define (USE_EMULATE_AUTO_JUMP , USE_MANUAL_UPDATE_FW) 

	enable USE_EMULATE_AUTO_JUMP to test auto jump , from boot loader to application 

2. use RTC spare register read/write in boot loader and application 

	under boot loader : check RTC spare register from different reset source
	
	under application code , write RTC spare register , and jump to boot loader

for application code , 

by press digit 1 (write RTC spare register , jump to boot loader ) , log message in boot loader as below , 

![image](https://github.com/released/M480BSP_IAP_RTC/blob/main/fromApplication.jpg)

from RESET pin , log message in boot loader as below , 

![image](https://github.com/released/M480BSP_IAP_RTC/blob/main/nRESET.jpg)

if press digit z , will re-start from application code , log message as below

![image](https://github.com/released/M480BSP_IAP_RTC/blob/main/NVIC_SystemReset.jpg)


