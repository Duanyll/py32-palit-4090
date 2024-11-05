/**
  * Demo: Write Option Bytes
  * 
  * Board: PY32F003W1XS (SOP16)
  * 
  * This demo shows how to config reset pin as gpio output
  */

#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_ll_spi.h"
#include "py32f0xx_ll_tim.h"

#define APP_LED_MODE_COUNT 16
static int led_mode = 0;

static void APP_GPIOConfig(void);
static void APP_FlashSetOptionBytes(void);
static void APP_SetPwmDutyCycle(uint16_t dutyCycle);
static void APP_StepFanSpeed(void);
static void APP_ScanButtons(void);
static void APP_UpdateLed(void);

uint8_t SPI_TxRxByte(uint8_t data);

#define WS2812_NUM_LEDS 64
#define WS2812_SPI_HANDLE Spi1Handle

#define WS2812_RESET_PULSE 60
#define WS2812_BUFFER_SIZE (WS2812_NUM_LEDS * 24 + WS2812_RESET_PULSE)

void ws2812_init(void);
void ws2812_send_spi(void);
void ws2812_pixel(uint16_t led_no, uint8_t r, uint8_t g, uint8_t b);
void ws2812_pixel_all(uint8_t r, uint8_t g, uint8_t b);

int main(void)
{
  BSP_RCC_HSI_24MConfig();

  LL_mDelay(1000);

  if(READ_BIT(FLASH->OPTR, FLASH_OPTR_NRST_MODE) == OB_RESET_MODE_RESET)
  {
    /* This will reset the MCU */
    APP_FlashSetOptionBytes();
  }
  /* Don't config GPIO before changing the option bytes */
  APP_GPIOConfig();
  APP_SetPwmDutyCycle(1000);

  ws2812_pixel_all(0xFF, 0xFF, 0xFF);
  ws2812_send_spi();
  while (1)
  {
    APP_ScanButtons();
    APP_UpdateLed();
    LL_mDelay(20);
  }
}

static void APP_GPIOConfig(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  // PA1 -> FAN BTN1 (Input with pull-up)
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT);

  // PA2 -> LED BTN2 (Input with pull-up)
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_INPUT);

  // PA3 -> WS2812B DIN (SPI1 MOSI, AF10)
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_10;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_Enable(SPI1);

  // PA10 -> FAN PWM (TIM1 CH3, AF2)
  // To shutdown the fan at the beginning, make the pin as GPIO output low
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);

  LL_TIM_InitTypeDef TIM1CountInit = {0};
  TIM1CountInit.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM1CountInit.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM1CountInit.Prescaler = 2400 - 1;
  /* PWM period = 1000 */
  TIM1CountInit.Autoreload = 1000 - 1;
  TIM1CountInit.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM1CountInit);
  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_EnableCounter(TIM1);
}

static void APP_SetPwmDutyCycle(uint16_t dutyCycle)
{
  // Duty cycle = 0 -> Max speed
  // Duty cycle = 999 -> Min speed
  // Duty cycle >= 1000 -> Stop
  if (dutyCycle >= 1000)
  {
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);
    return;
  } else {
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    LL_TIM_OC_InitTypeDef TIM_OC_Initstruct = {0};
    TIM_OC_Initstruct.OCMode = LL_TIM_OCMODE_PWM2;
    TIM_OC_Initstruct.OCState = LL_TIM_OCSTATE_ENABLE;
    TIM_OC_Initstruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_Initstruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_Initstruct.CompareValue = dutyCycle;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_Initstruct);
  }
}

static void APP_FlashSetOptionBytes(void)
{
  FLASH_OBProgramInitTypeDef OBInitCfg;

  LL_FLASH_Unlock();
  LL_FLASH_OB_Unlock();

  OBInitCfg.OptionType = OPTIONBYTE_USER;
  OBInitCfg.USERType = OB_USER_BOR_EN | OB_USER_BOR_LEV | OB_USER_IWDG_SW  | OB_USER_NRST_MODE | OB_USER_nBOOT1;
  /*
   * The default value: OB_BOR_DISABLE | OB_BOR_LEVEL_3p1_3p2 | OB_IWDG_SW | OB_WWDG_SW | OB_RESET_MODE_RESET | OB_BOOT1_SYSTEM;
  */
  OBInitCfg.USERConfig = OB_BOR_DISABLE | OB_BOR_LEVEL_3p1_3p2 | OB_IWDG_SW  | OB_RESET_MODE_GPIO | OB_BOOT1_SYSTEM;
  LL_FLASH_OBProgram(&OBInitCfg);

  LL_FLASH_Lock();
  LL_FLASH_OB_Lock();
  /* Reload option bytes */
  LL_FLASH_OB_Launch();
}

static void APP_StepFanSpeed(void)
{
  static int fan_speed_level = 4;
  fan_speed_level = (fan_speed_level + 1) % 5;
  switch (fan_speed_level)
  {
  case 0:
    APP_SetPwmDutyCycle(0);
    break;
  case 1:
    APP_SetPwmDutyCycle(250);
    break;
  case 2:
    APP_SetPwmDutyCycle(500);
    break;
  case 3:
    APP_SetPwmDutyCycle(750);
    break;
  case 4:
    APP_SetPwmDutyCycle(1000); // Stop
    break;
  }
}

static void APP_ScanButtons(void) {
  static int last_btn1 = 1, last_btn2 = 1;
  int btn1 = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_1);
  int btn2 = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_2);
  if (btn1 == 0 && last_btn1 == 1) {
    APP_StepFanSpeed();
  }
  if (btn2 == 0 && last_btn2 == 1) {
    led_mode = (led_mode + 1) % APP_LED_MODE_COUNT;
  }
}

static void APP_UpdateLed(void) {
  static uint32_t state = 0;
  state++;
  switch (led_mode) {
  default:
    // All white
    ws2812_pixel_all(0xFF, 0xFF, 0xFF);
  }
  ws2812_send_spi();
}

uint8_t SPI_TxRxByte(uint8_t data) {
  uint8_t SPITimeout = 0xFF;
  /* Check the status of Transmit buffer Empty flag */
  while (READ_BIT(SPI1->SR, SPI_SR_TXE) == RESET) {
    if (SPITimeout-- == 0)
      return 0;
  }
  LL_SPI_TransmitData8(SPI1, data);
  SPITimeout = 0xFF;
  while (READ_BIT(SPI1->SR, SPI_SR_RXNE) == RESET) {
    if (SPITimeout-- == 0)
      return 0;
  }
  // Read from RX buffer
  return LL_SPI_ReceiveData8(SPI1);
}

#define WS2812_FILL_BUFFER(COLOR)                                              \
  for (uint8_t mask = 0x80; mask; mask >>= 1) {                                \
    if (COLOR & mask) {                                                        \
      *ptr++ = 0xfc;                                                           \
    } else {                                                                   \
      *ptr++ = 0x80;                                                           \
    }                                                                          \
  }

uint8_t ws2812_buffer[WS2812_BUFFER_SIZE];

void ws2812_init(void) {
  memset(ws2812_buffer, 0, WS2812_BUFFER_SIZE);
  ws2812_send_spi();
}

void ws2812_send_spi(void) {
  uint16_t i;
  for (i = 0; i < WS2812_BUFFER_SIZE; i++) {
    SPI_TxRxByte(*(ws2812_buffer + i));
  }
}

void ws2812_pixel(uint16_t led_no, uint8_t r, uint8_t g, uint8_t b) {
  uint8_t *ptr = &ws2812_buffer[24 * led_no];
  WS2812_FILL_BUFFER(g);
  WS2812_FILL_BUFFER(r);
  WS2812_FILL_BUFFER(b);
}

void ws2812_pixel_all(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t *ptr = ws2812_buffer;
  for (uint16_t i = 0; i < WS2812_NUM_LEDS; ++i) {
    WS2812_FILL_BUFFER(g);
    WS2812_FILL_BUFFER(r);
    WS2812_FILL_BUFFER(b);
  }
}

void APP_ErrorHandler(void)
{
  while (1);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  while (1);
}
#endif /* USE_FULL_ASSERT */
