
        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        PUBLIC  __vector_table

        DATA
__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler             ; Reset Handler

        DCD     NMI_Handler               ; NMI Handler
        DCD     HardFault_Handler         ; Hard Fault Handler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     SVC_Handler               ; SVCall Handler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     PendSV_Handler            ; PendSV Handler
        DCD     SysTick_Handler           ; SysTick Handler

        ; External Interrupts
		DCD     WWDT_IRQHandler           ; 0:  WWDT 
		DCD     SVD_IRQHandler            ; 1:  SVD 	
		DCD     RTC_IRQHandler            ; 2:  RTC 	
		DCD     FLASH_IRQHandler          ; 3:  NVMIF	
		DCD     CMU_IRQHandler           ; 4:  CMU	
		DCD     ADC_IRQHandler            ; 5:  ADC	
		DCD     SPI0_IRQHandler           ; 6:  SPI0	
		DCD     SPI1_IRQHandler           ; 7:  SPI1 	
		DCD     SPI2_IRQHandler           ; 8:  SPI2	
		DCD     UART0_IRQHandler          ; 9:  UART0	
		DCD     UART1_IRQHandler          ; 10:  UART1	
		DCD     UART2_IRQHandler          ; 11:  UART2	
		DCD     UART3_IRQHandler          ; 12:  UART3	
		DCD     UART4_IRQHandler          ; 13:  UART4	
		DCD     UART5_IRQHandler          ; 14:  UART5	
		DCD     U7816_IRQHandler         ; 15:  U7816	
		DCD     LPUART0_IRQHandler         ; 16:  LPUART0
		DCD     I2Cx_IRQHandler            ; 17:  I2Cx	
		DCD     0                        ; 18:  NULL	
		DCD     CRYPTO_IRQHandler         	  ; 19:  CRYPTO	
		DCD     LPTIM_IRQHandler          ; 20:  LPTIM	
		DCD     DMA_IRQHandler            ; 21:  DMA	
		DCD     WKUPx_IRQHandler           ; 22:  WKUPx	
		DCD     COMP_IRQHandler           ; 23:  COMP	
		DCD     BTx_IRQHandler          ; 24:  BTx
		DCD     QSPI_IRQHandler          ; 25:  QSPI
		DCD     ETx_IRQHandler          ; 26:  ETx	
		DCD     BSTIM_IRQHandler          ; 27:  BSTIM	
		DCD     SPI3_IRQHandler          ; 28:  SPI3	
		DCD     SPI4_IRQHandler          ; 39:  SPI4	
        DCD     GPIO_IRQHandler           ; 30:  GPIO 
        DCD     LPUART1_IRQHandler        ; 31: LPUART1
                
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB
        PUBWEAK Reset_Handler
        SECTION .text:CODE:NOROOT:REORDER(2)
Reset_Handler
        IMPORT  SystemInit
        LDR     R0, =SystemInit
        BLX     R0		
        LDR     R0, =__iar_program_start
        BX      R0
        
        PUBWEAK NMI_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
NMI_Handler
        B NMI_Handler
        
        
        PUBWEAK HardFault_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
HardFault_Handler
        B HardFault_Handler
       
        
        PUBWEAK SVC_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
SVC_Handler
        B SVC_Handler
        
        
        PUBWEAK PendSV_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
PendSV_Handler  
        B PendSV_Handler
        
        
        PUBWEAK SysTick_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
SysTick_Handler
        B SysTick_Handler
        
        
        PUBWEAK WWDT_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
WWDT_IRQHandler
        B WWDT_IRQHandler
        
                
        PUBWEAK SVD_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
SVD_IRQHandler
        B SVD_IRQHandler
        
                
        PUBWEAK RTC_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
RTC_IRQHandler
        B RTC_IRQHandler
        
                
        PUBWEAK FLASH_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
FLASH_IRQHandler
        B FLASH_IRQHandler
        
                
        PUBWEAK CMU_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
CMU_IRQHandler
        B CMU_IRQHandler
        
                
        PUBWEAK ADC_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
ADC_IRQHandler
        B ADC_IRQHandler
        
                
        PUBWEAK SPI0_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
SPI0_IRQHandler
        B SPI0_IRQHandler
        

        PUBWEAK SPI1_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
SPI1_IRQHandler
        B SPI1_IRQHandler
        
                                
        PUBWEAK SPI2_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
SPI2_IRQHandler
        B SPI2_IRQHandler
        
                
        PUBWEAK UART0_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
UART0_IRQHandler
        B UART0_IRQHandler
        
                
        PUBWEAK UART1_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
UART1_IRQHandler
        B UART1_IRQHandler
        
                
        PUBWEAK UART2_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
UART2_IRQHandler
        B UART2_IRQHandler
        
                
        PUBWEAK UART3_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
UART3_IRQHandler
        B UART3_IRQHandler
        
                
        PUBWEAK UART4_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
UART4_IRQHandler
        B UART4_IRQHandler
        
                 
        PUBWEAK UART5_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
UART5_IRQHandler
        B UART5_IRQHandler
        
                 
        PUBWEAK U7816_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
U7816_IRQHandler
        B U7816_IRQHandler
        
                
        PUBWEAK LPUART0_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
LPUART0_IRQHandler
        B LPUART0_IRQHandler
        
                
        PUBWEAK I2Cx_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
I2Cx_IRQHandler
        B I2Cx_IRQHandler
                                                  
                
        PUBWEAK CRYPTO_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
CRYPTO_IRQHandler
        B CRYPTO_IRQHandler
        
        
        PUBWEAK LPTIM_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
LPTIM_IRQHandler
        B LPTIM_IRQHandler


        PUBWEAK DMA_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
DMA_IRQHandler
        B DMA_IRQHandler


        PUBWEAK WKUPx_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
WKUPx_IRQHandler
        B WKUPx_IRQHandler     


        PUBWEAK COMP_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
COMP_IRQHandler
        B COMP_IRQHandler
        
                
        PUBWEAK BTx_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
BTx_IRQHandler
        B BTx_IRQHandler
        
        
        PUBWEAK QSPI_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
QSPI_IRQHandler
        B QSPI_IRQHandler
        
        
        PUBWEAK ETx_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
ETx_IRQHandler
        B ETx_IRQHandler
         
         
        PUBWEAK BSTIM_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
BSTIM_IRQHandler
        B BSTIM_IRQHandler
        
        
         PUBWEAK SPI3_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
SPI3_IRQHandler
        B SPI3_IRQHandler
        
        
         PUBWEAK SPI4_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
SPI4_IRQHandler
        B SPI4_IRQHandler
        
        PUBWEAK GPIO_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
GPIO_IRQHandler
        B GPIO_IRQHandler
        
        PUBWEAK LPUART1_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
LPUART1_IRQHandler
        B LPUART1_IRQHandler
        END