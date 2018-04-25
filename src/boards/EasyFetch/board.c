/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"

/*!
 * Unique Devices IDs register set ( STM32L1xxx )
 */
#define         ID1                                 ( 0x1FF80050 )
#define         ID2                                 ( 0x1FF80054 )
#define         ID3                                 ( 0x1FF80064 )

/*!
 * LED GPIO pins objects
 */
Gpio_t Led1;
Gpio_t Led2;
Gpio_t Led3;
// modified
// Gpio_t GpsPowerEn;

/*
 * MCU objects
 */
Adc_t Adc;
I2c_t I2c;

#if defined( USE_DEBUG_PINS )
Gpio_t DbgPin1;
Gpio_t DbgPin2;
Gpio_t DbgPin3;
Gpio_t DbgPin4;
Gpio_t DbgPin5;
#endif

/*!
 * Initializes the unused GPIO to a know status
 */
static void BoardUnusedIoInit( void );

/*!
 * System Clock Configuration
 */
static void SystemClockConfig( void );

/*!
 * Used to measure and calibrate the system wake-up time from STOP mode
 */
//static void CalibrateSystemWakeupTime( void );

/*!
 * System Clock Re-Configuration when waking up from STOP mode
 */
static void SystemClockReConfig( void );

/*!
 * Timer used at first boot to calibrate the SystemWakeupTime
 */
//static TimerEvent_t CalibrateSystemWakeupTimeTimer;

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

/*!
 * Flag to indicate if the SystemWakeupTime is Calibrated
 */
//static bool SystemWakeupTimeCalibrated = false;

/*!
 * Flag to indicate the Status of LED3
 */
volatile uint8_t Led3Status = 1;

/*!
 * Callback indicating the end of the system wake-up time calibration
 */
/*static void OnCalibrateSystemWakeupTimeTimerEvent( void )
{
    SystemWakeupTimeCalibrated = true;
}*/

/*!
 * Nested interrupt counter.
 *
 * \remark Interrupt should only be fully disabled once the value is 0
 */
static uint8_t IrqNestLevel = 0;

void BoardDisableIrq( void )
{
    __disable_irq( );
    IrqNestLevel++;
}

void BoardEnableIrq( void )
{
    IrqNestLevel--;
    if( IrqNestLevel == 0 )
    {
        __enable_irq( );
    }
}

void BoardInitPeriph( void )
{

    GpioInit( &Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
	
#if defined( USE_DEBUG_PINS )
	GpioInit( &DbgPin1, CON_EXT_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
	GpioInit( &DbgPin2, CON_EXT_3, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
	GpioInit( &DbgPin3, CON_EXT_7, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
	GpioInit( &DbgPin4, CON_EXT_8, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
	GpioInit( &DbgPin5, CON_EXT_9, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif
	  // modified
    GpsInit( );
}

void BoardInitMcu( void )
{

    if( McuInitialized == false )
    {
        HAL_Init( );

        SystemClockConfig( );
		
#if defined( UART_PRINTF )
        printf_init();
#endif		
        RtcInit( );

        BoardUnusedIoInit( );

        I2cInit( &I2c, I2C_SCL, I2C_SDA );
    }
    else
    {
        SystemClockReConfig( );
    }

    SpiInit( &SX1276.Spi, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );
    SX1276IoInit( );

//    if( McuInitialized == false )
//    {
//        McuInitialized = true;
//        if( GetBoardPowerSource( ) == BATTERY_POWER )
//        {
//            CalibrateSystemWakeupTime( );
//        }
//    }
}

void BoardDeInitMcu( void )
{
    Gpio_t ioPin;

		UartDeInit( &Uart1 );
		I2cDeInit( &I2c );
    SpiDeInit( &SX1276.Spi );
    SX1276IoDeInit( );

    GpioInit( &ioPin, OSC_HSE_IN, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, OSC_HSE_OUT, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    GpioInit( &ioPin, OSC_LSE_IN, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, OSC_LSE_OUT, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

uint32_t BoardGetRandomSeed( void )
{
    return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

void BoardGetUniqueId( uint8_t *id )
{
    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID2 ) );
}

/*!
 * Factory power supply
 */
#define FACTORY_POWER_SUPPLY                        3300 // mV

/*!
 * VREF calibration value
 */
#define VREFINT_CAL                                 ( *( uint16_t* )0x1FF80078 )

/*!
 * ADC maximum value
 */
#define ADC_MAX_VALUE                               4095

/*!
 * Battery thresholds
 */
#define BATTERY_MAX_LEVEL                           4500 // mV
#define BATTERY_MIN_LEVEL                           1800 // mV
#define BATTERY_SHUTDOWN_LEVEL                      1900 // mV

uint16_t BoardGetPowerSupply( void )
{
    float vref = 0;
    float vdiv = 0;
    float batteryVoltage = 0;

//    AdcInit( &Adc, BAT_LEVEL );

//    vref = AdcMcuRead( &Adc, (uint8_t)ADC_CHANNEL_17 );
//    vdiv = AdcMcuRead( &Adc, (uint8_t)ADC_CHANNEL_8 );

    batteryVoltage = ( FACTORY_POWER_SUPPLY * VREFINT_CAL * vdiv ) / ( vref * ADC_MAX_VALUE );

    //                                vDiv
    // Divider bridge  VBAT <-> 1M -<--|-->- 1M <-> GND => vBat = 2 * vDiv
    batteryVoltage = 2 * batteryVoltage;

    return ( uint16_t )( batteryVoltage * 1000 );
}

uint8_t BoardGetBatteryLevel( void )
{
    volatile uint8_t batteryLevel = 0;
//    uint16_t batteryVoltage = 0;

////    if( GpioRead( &UsbDetect ) == 1 )
//    {
//        batteryLevel = 0;
//    }
//    else
//    {
//        batteryVoltage = BoardGetPowerSupply( );

//        if( batteryVoltage >= BATTERY_MAX_LEVEL )
//        {
//            batteryLevel = 254;
//        }
//        else if( ( batteryVoltage > BATTERY_MIN_LEVEL ) && ( batteryVoltage < BATTERY_MAX_LEVEL ) )
//        {
//            batteryLevel = ( ( 253 * ( batteryVoltage - BATTERY_MIN_LEVEL ) ) / ( BATTERY_MAX_LEVEL - BATTERY_MIN_LEVEL ) ) + 1;
//        }
//        else if( batteryVoltage <= BATTERY_SHUTDOWN_LEVEL )
//        {
//            batteryLevel = 255;
//            //GpioInit( &DcDcEnable, DC_DC_EN, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//            //GpioInit( &BoardPowerDown, BOARD_POWER_DOWN, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
//        }
//        else // BATTERY_MIN_LEVEL
//        {
//            batteryLevel = 1;
//        }
//    }
    return batteryLevel;
}

static void BoardUnusedIoInit( void )
{
    Gpio_t ioPin;

    if( GetBoardPowerSource( ) == BATTERY_POWER )
    {
        GpioInit( &ioPin, USB_DM, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioInit( &ioPin, USB_DP, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    }

    GpioInit( &ioPin, TEST_POINT1, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, TEST_POINT2, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, TEST_POINT3, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, TEST_POINT4, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

//    GpioInit( &ioPin, PIN_NC, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, BOOT_1, PIN_ANALOGIC, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 );

//    GpioInit( &ioPin, RF_RXTX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, WKUP1, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

#if defined( USE_DEBUGGER )
    HAL_DBGMCU_EnableDBGStopMode( );
    HAL_DBGMCU_EnableDBGSleepMode( );
    HAL_DBGMCU_EnableDBGStandbyMode( );
#else
    HAL_DBGMCU_DisableDBGSleepMode( );
    HAL_DBGMCU_DisableDBGStopMode( );
    HAL_DBGMCU_DisableDBGStandbyMode( );

    GpioInit( &ioPin, SWDIO, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, SWCLK, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif
}

void SystemClockConfig( void )
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    __HAL_RCC_PWR_CLK_ENABLE( );

    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
		RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
		RCC_OscInitStruct.HSIState = RCC_HSI_ON;
		RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
    RCC_OscInitStruct.HSICalibrationValue = 0x10;
    if( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
    {
        assert_param( FAIL );
    }

    RCC_ClkInitStruct.ClockType = ( RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
																		RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 );
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 ) != HAL_OK )
    {
        assert_param( FAIL );
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
    {
        assert_param( FAIL );
    }
		
    HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq( ) / 1000 );

    HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );

    /*    HAL_NVIC_GetPriorityGrouping*/
//    HAL_NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}

/*
void CalibrateSystemWakeupTime( void )
{
    if( SystemWakeupTimeCalibrated == false )
    {
        TimerInit( &CalibrateSystemWakeupTimeTimer, OnCalibrateSystemWakeupTimeTimerEvent );
        TimerSetValue( &CalibrateSystemWakeupTimeTimer, 1000 );
        TimerStart( &CalibrateSystemWakeupTimeTimer );
        while( SystemWakeupTimeCalibrated == false )
        {
            TimerLowPowerHandler( );
        }
    }
}
*/
void SystemClockReConfig( void )
{
    __HAL_RCC_PWR_CLK_ENABLE( );
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    /* Enable HSE */
    __HAL_RCC_HSE_CONFIG( RCC_HSE_ON );

    /* Wait till HSE is ready */
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSERDY ) == RESET )
    {
    }

    /* Enable PLL */
    __HAL_RCC_PLL_ENABLE( );

    /* Wait till PLL is ready */
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) == RESET )
    {
    }

    /* Select PLL as system clock source */
    __HAL_RCC_SYSCLK_CONFIG ( RCC_SYSCLKSOURCE_PLLCLK );

    /* Wait till PLL is used as system clock source */
    while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_PLLCLK )
    {
    }
}

void SysTick_Handler( void )
{
    HAL_IncTick( );
    HAL_SYSTICK_IRQHandler( );
}

uint8_t GetBoardPowerSource( void )
{
#if defined( USE_USB_CDC )
    if( GpioRead( &UsbDetect ) == 1 )
    {
        return BATTERY_POWER;
    }
    else
    {
        return USB_POWER;
    }
#else
    return BATTERY_POWER;
#endif
}

#ifdef USE_FULL_ASSERT
/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while( 1 )
    {
    }
}
#endif
