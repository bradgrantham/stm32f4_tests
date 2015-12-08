#include "stm32f4xx_hal.h"

static GPIO_InitTypeDef  GPIO_InitStruct;
static UART_HandleTypeDef UartHandle;

static uint32_t ticks = 0;

void HAL_IncTick(void)
{
    ticks++;
}

// From Projects/STM32F411RE-Nucleo/Examples/GPIO/GPIO_IOToggle/Src/main.c
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
int ready_to_printf = 0;
static void Error_Handler(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);

    if(ready_to_printf)
        printf("Error Handler...\n");

    while(1) {
        /* delay */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(500);
    }
}

// From Projects/STM32F411RE-Nucleo/Examples/GPIO/GPIO_IOToggle/Src/main.c
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 400
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&UartHandle);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  /* Enable USART clock */
  __HAL_RCC_USART1_CLK_ENABLE(); 
  
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = GPIO_PIN_6;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* NVIC for USART */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
  * @brief UART MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  /*##-1- Reset peripherals ##################################################*/
    __HAL_RCC_USART1_FORCE_RESET();
    __HAL_RCC_USART1_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
  /* Configure UART Rx as alternate function  */
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);
}

// XXX Really need to enqueue and drain in interrupt handler
// Or double-buffer two arrays for DMA
void __io_putchar( char c )
{
    char tmp = c;

    uint32_t tmp1 = 0;
    do {
        tmp1 = UartHandle.State;
    } while ((tmp1 == HAL_UART_STATE_BUSY_TX) || (tmp1 == HAL_UART_STATE_BUSY_TX_RX));

    if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t *)&tmp, 1) != HAL_OK) {
        ready_to_printf = 0;
        Error_Handler();
    }
}

volatile int char_next = 0;
volatile char chars[1024];
volatile unsigned int error_code = 0;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    error_code = UartHandle.ErrorCode;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    int result;
    if(UartHandle.ErrorCode != HAL_UART_ERROR_NONE)
        error_code = UartHandle.ErrorCode;
    char_next++;
    if((result = HAL_UART_Receive_IT(&UartHandle, (uint8_t *)(chars + char_next), 1)) != HAL_OK) {
        error_code = UartHandle.State;
    }
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1); // toggles on callback, not on success
}

int main(int argc, char **argv)
{
    // From Projects/STM32F411RE-Nucleo/Examples/GPIO/GPIO_IOToggle/Src/main.c
    /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
    HAL_Init();

    /* Configure the system clock to 100 MHz */
    SystemClock_Config();

    /*##-1- Enable GPIOA Clock (to be able to program the configuration registers) */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // end of code from Projects/STM32F411RE-Nucleo/Examples/GPIO/GPIO_IOToggle/Src/main.c

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 

    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); 

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);

    /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* UART1 configured as follow:
        - Word Length = 8 Bits
        - Stop Bit = One Stop bit
        - Parity = ODD parity
        - BaudRate = 115200 baud
        - Hardware flow control disabled (RTS and CTS signals) */
    UartHandle.Instance          = USART1;
    
    UartHandle.Init.BaudRate     = 115200;
    UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits     = UART_STOPBITS_1;
    UartHandle.Init.Parity       = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode         = UART_MODE_TX_RX;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
      
    if(HAL_UART_Init(&UartHandle) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler(); 
    }

    ready_to_printf = 1;
  
    printf("UART test...\n");

    if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)&chars[0], 1) != HAL_OK)
    {
      /* Transfer error in reception process */
      Error_Handler();
    }

    int pin = 1;
    int chars_so_far = 0;
    int then = 0;
    int then2 = 0;

    while(1) {
        int now;
        int tmp;

        __disable_irq();
        now = ticks;
        __enable_irq();

        if(now - then > 500) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, pin);
            pin = pin ? 0 : 1;
            then = now;
        }

        if(now - then2 > 2000) {
            // printf("ping, %d\n", UartHandle.State);
            then2 = now;
        }

        __disable_irq();
        tmp = char_next;
        __enable_irq();

    
        while(tmp > chars_so_far) {
            printf("received %d: %02X (%c)\n", chars_so_far, chars[chars_so_far], chars[chars_so_far]);
            chars_so_far++;
        }

        if(error_code != HAL_UART_ERROR_NONE) {
            printf("UART error code 0x%04x\n", error_code);
            Error_Handler();
        }
    }
}
