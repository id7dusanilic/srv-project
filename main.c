/**
 * @file    main.c
 * @brief   Samples ADC12 Channels 0 and 1 and calculates mean value.
 *
 * Samples ADC12 Channels 0 and 1 with 500ms sampling rate.
 * When a button is pressed, mean value of the last 8 samples of
 * the selected channel is transmitted via UART.
 *
 * Button S3 selects Channel 0 and button S4 selects Channel 1.
 *
 * @author  Dusan Ilic
 * @date    2021
 */

/* Standard includes. */
#include <ETF5529_HAL/hal_ETF_5529.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Hardware includes. */
#include "msp430.h"

/* Macro definitions */

/** @brief Macro for setting bits specified by the mask */
#define BIT_SET(reg, mask)              reg |=  (mask)
/** @brief Macro for clearing bits specified by the mask */
#define BIT_CLEAR(reg, mask)            reg &= ~(mask)
/** @brief Macro for toggling bits specified by the mask */
#define BIT_TOGGLE(reg, mask)           reg ^=  (mask)

/** @brief Macro for making an artificial delay using a loop */
#define ARTIFICIAL_DELAY(i, count)      for( i = 0; i < count; i++ )

/** @brief Maximum unsigned integer mask */
#define ULONG_MAX                       ( 0xffffffff )

/** @brief UCBRx for 9600 baud rate with 32,768Hz clock */
#define BR9600_UCBR                     ( 3 )

/** @brief UCBRSx for 9600 baud rate with 32,768Hz clock */
#define BR9600_UCBRS                    ( UCBRS_3 )

/** @brief xTask1 priority */
#define mainTASK1_PRIORITY              ( 2 )
/** @brief xTask2 priority */
#define mainTASK2_PRIORITY              ( 4 )
/** @brief xTask3 priority */
#define mainTASK3_PRIORITY              ( 3 )
/** @brief xTask4 priority */
#define mainTASK4_PRIORITY              ( 3 )
/** @brief UART Task priority */
#define mainUART_TASK_PRIORITY          ( 1 )

/** @brief ADC12 Sampling rate in milliseconds
 *
 * Used as a software timer period.
 */
#define mainADC_SAMPLING_RATE_MS        ( 500 )

/** @brief Queue length for the queue storing ADC Conversion results */
#define mainADC_DATA_QUEUE_LENGTH       ( 10 )
/** @brief Queue length for the queue storing data to be transmitted via UART */
#define mainUART_DATA_QUEUE_LENGTH      ( 10 )
/** @brief Queue length for the queue storing the selected channel */
#define mainCHANNEL_SELECT_QUEUE_LENGTH ( 10 )
/** @brief Queue length for the queue storing bridge information */
#define mainTASK1_BRIDGE_QUEUE_LENGTH   ( 10 )

/** @brief Notification value for events on S3 */
#define mainS3_NOTIFICATION_VALUE       ( 3 )
/** @brief Notification value for events on S4 */
#define mainS4_NOTIFICATION_VALUE       ( 4 )

/** @brief Number of last samples used for calculating the mean value */
#define mainNUM_OF_SAMPLES              ( 8 )

/* Data type definitions */

/** @brief Enum data type for identifying ADC Channels */
typedef enum {
    eADC_CHANNEL_0,
    eADC_CHANNEL_1
} adc_channel_t;

/**
 * @brief Enum data type for identifying event types
 *
 * Event can be either an end of ADC conversion or
 * a button pressed event.
 */
typedef enum {
    eADC_EVENT,
    eBUTTON_EVENT
} bridge_data_t;

/** @brief Stores information about ADC conversion value and the channel converted */
typedef struct {
    adc_channel_t   xChannel;
    uint8_t         ucData;
} adc_data_t;

/** @brief Stores information about ADC conversion value and the channel converted */
typedef struct {
    adc_channel_t   xChannel;
    uint8_t         ucData;
} uart_data_t;

/* Handle declarations */

/** @brief xTask1 Task Handle */
TaskHandle_t  xTask1Handle;
/** @brief xTask2 Task Handle */
TaskHandle_t  xTask2Handle;
/** @brief xTask3 Task Handle */
TaskHandle_t  xTask4Handle;
/** @brief xTask4 Task Handle */
TaskHandle_t  xTask3Handle;
/** @brief UART Task Handle */
TaskHandle_t  xUARTTaskHandle;

/** @brief ADC Sampling Timer Handle */
TimerHandle_t xADCSamplingTimer;

/**
 * @brief ADC Data Queue Handle
 *
 * Holds adc_data_t structures from last conversions
 */
QueueHandle_t xADCDataQueue;

/**
 * @brief UART Data Queue Handle
 *
 * Holds uart_data_t structures ready for sending via UART
 */
QueueHandle_t xUARTDataQueue;

/**
 * @brief Channel Select Queue Handle
 *
 * Holds last channel selected for reading
 */
QueueHandle_t xChannelSelectQueue;

/**
 * @brief xTask1 Bridge Queue Handle
 *
 * Holds bridge_data_t to signal which queues
 * are ready for reading by xTask1.
 */
QueueHandle_t xTask1BridgeQueue;

/* Function declarations */

/**
 * @brief Initializes required peripherals
 *
 * Peripherals initialized are ADC, USCI_A1 (UART) and ports (buttons).
 */
static inline void prvInitPeripherals( void );

/**
 * @brief Initializes ADC12
 *
 * Initializes ADC12 in single sequence of channels mode.
 * Conversion is started with a software start, and a single
 * start converts all selected channels.
 * Selected channels are 0 and 1.
 * End of conversion interrupts are enabled.
 * Channel 0 converted value is stored in ADC12MEM0.
 * Channel 1 converted value is stored in ADC12MEM1.
 * Pins P6.0 and P6.1 are configured for use with ADC.
 */
static inline void prvInitADC( void );

/**
 * @brief Initializes USCI_A1 in 9600 8N1 UART Mode
 *
 * Clock source is ACLK (32,768Hz).
 * 9600 8N1 UART Mode is selected.
 * Pins P4.4 (Tx) and P4.5 (Rx) are configured for use with UART.
 * Interrupts are disabled.
 */
static inline void prvInitUART( void );

/**
 * @brief Initializes pins for use with buttons S3 and S4.
 *
 * Pins P1.4 (button S3) and P1.5 (button S4) are configured
 * as inputs with falling edge interrupts enabled, and pull-up
 * resistors enabled.
 */
static inline void prvInitButtons( void );

/** @brief xTask1 Task Function
 *
 * Depending on the event that happened (ADC Conversion or Button press)
 * performs an action.
 *
 * If ADC Conversion ended, stores the value in the array that holds last
 * mainNUM_OF_SAMPLES samples for the corresponding channel.
 *
 * If button was pressed, calculates the mean of the samples from the samples
 * array and sends the information to one of tasks xTask3 or xTask4 depending
 * on the button pressed.
 */
static void prvTask1Function( void *pvParameters );

/**
 * @brief xTask2 Task Function
 *
 * Detects button presses and notifies xTask1 about it.
 */
static void prvTask2Function( void *pvParameters );

/** @brief Function for implementation of xTask3 and xTask4.
 *
 * Packages the mean samples value, and information about the channel and
 * sends it to the UART Task true a queue.
 *
 * The channel information is passed through pvParameters when creating the task
 */
static void prvPrepareDataTaskFunction( void *pvParameters );

/** @brief UART Task Function
 *
 * Reads data from a queue, forms a string from it, and sends it via UART.
 */
static void prvUARTTaskFunction( void *pvParameters );

/**
 * @brief Convert an integer to a string in decimal representation
 *
 * @param num Number to be converted to a string.
 * @param str Buffer where the result is written.
 */
static void prvDecimalToString( unsigned int num, char *str );

/**
 * @brief Transmits a string via UART
 *
 * @param str String to be sent via UART.
 */
static void prvTransmitStringViaUART( const char *str );

/**
 * @brief Calculates the mean value of the given array
 *
 * @param array Array whose mean value is calculated
 * @param length Length of array
 */
static inline uint8_t prvArrayMean( uint8_t *array, uint8_t length );

/**
 * @brief Timer Callback Function
 *
 * Starts the ADC Conversion each time the timer expires
 */
void prvTimerCallbackFunction( TimerHandle_t xTimer );

/**
 * @brief Main function
 *
 * Creates necessary tasks and queues, creates and starts the timer, and starts the scheduler.
 */
void main( void )
{
    /* Configure peripherals */
    prvInitPeripherals();
    BIT_SET(P1DIR, BIT0);
    BIT_CLEAR(P1OUT, BIT0);

    /* Create tasks */
    xTaskCreate( prvTask1Function,
                 "xTask1",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 mainTASK1_PRIORITY,
                 &xTask1Handle );

    xTaskCreate( prvTask2Function,
                 "xTask2",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 mainTASK2_PRIORITY,
                 &xTask2Handle );

    xTaskCreate( prvPrepareDataTaskFunction,
                 "xTask3",
                 configMINIMAL_STACK_SIZE,
                 ( void * ) eADC_CHANNEL_0,
                 mainTASK3_PRIORITY,
                 &xTask3Handle );

    xTaskCreate( prvPrepareDataTaskFunction,
                 "xTask4",
                 configMINIMAL_STACK_SIZE,
                 ( void * ) eADC_CHANNEL_1,
                 mainTASK4_PRIORITY,
                 &xTask4Handle );

    xTaskCreate( prvUARTTaskFunction,
                 "UART Task",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 mainUART_TASK_PRIORITY,
                 &xUARTTaskHandle );

    /* Create queues */
    xChannelSelectQueue = xQueueCreate(mainCHANNEL_SELECT_QUEUE_LENGTH, sizeof(adc_channel_t));
    xTask1BridgeQueue   = xQueueCreate(mainTASK1_BRIDGE_QUEUE_LENGTH,   sizeof(bridge_data_t));
    xUARTDataQueue      = xQueueCreate(mainUART_DATA_QUEUE_LENGTH,      sizeof(uart_data_t));
    xADCDataQueue       = xQueueCreate(mainADC_DATA_QUEUE_LENGTH,       sizeof(adc_data_t));

    /* Create timer */
    xADCSamplingTimer = xTimerCreate(
            "ADC Sampling Timer",
            pdMS_TO_TICKS(mainADC_SAMPLING_RATE_MS),
            pdTRUE,
            NULL,
            prvTimerCallbackFunction);

    /* Start the timer */
    xTimerStart(xADCSamplingTimer, 0);


    /* Start the scheduler. */
    vTaskStartScheduler();


    /* If all is well then this line will never be reached.  If it is reached
    then it is likely that there was insufficient (FreeRTOS) heap memory space
    to create the idle task.  This may have been trapped by the malloc() failed
    hook function, if one is configured. */	
    for( ;; );
}

static inline void prvInitPeripherals( void )
{
    taskDISABLE_INTERRUPTS();

    /* Disable the watchdog timer */
    WDTCTL = WDTPW + WDTHOLD;

    /* Configure clock */
    hal430SetSystemClock( configCPU_CLOCK_HZ, configLFXT_CLOCK_HZ );

    prvInitButtons();

    prvInitADC();

    prvInitUART();

    /* Enable global interrupts */
    taskENABLE_INTERRUPTS();
}

static inline void prvInitADC( void )
{
    /* Disable conversion before configuring */
    BIT_CLEAR(ADC12CTL0, ADC12ENC);

    /* 8 ADC12CLK clock cycles S/H time */
    BIT_SET(ADC12CTL0, ADC12SHT0_1);

    /* Convert all channels on single trigger */
    BIT_SET(ADC12CTL0, ADC12MSC);

    /* Turn ADC12 on */
    BIT_SET(ADC12CTL0, ADC12ON);

    /* Selecting Pulse Sample Mode */
    BIT_SET(ADC12CTL1, ADC12SHP);

    /* Converting sequence of channels */
    BIT_SET(ADC12CTL1, ADC12CONSEQ_1);

    /* Store channel A0 data to ADCMEM0 */
    BIT_SET(ADC12MCTL0, ADC12INCH_0);

    /* Store channel A1 data to ADCMEM1 */
    BIT_SET(ADC12MCTL1, ADC12INCH_1);

    /* Channel 1 is the last conversion in a sequence */
    BIT_SET(ADC12MCTL1, ADC12EOS);

    /* Configure P6.0 and P6.1 for use with ADC */
    BIT_SET(P6SEL, (BIT0 | BIT1));

    /* Enable interrupts */
    BIT_SET(ADC12IE, ADC12IE0);
    BIT_SET(ADC12IE, ADC12IE1);

    /* Enable conversion */
    BIT_SET(ADC12CTL0, ADC12ENC);
}

static inline void prvInitUART( void )
{
    /* Set the software reset bit before configuring */
    BIT_SET(UCA1CTL1, UCSWRST);

    /* USCI Registers Initialization
     * ACLK as clock source
     * 9600 8N1 Mode */
    UCA1CTL0 = 0;
    BIT_SET(UCA1CTL1, UCSSEL__ACLK);
    UCA1BRW = BR9600_UCBR;
    BIT_SET(UCA1MCTL, BR9600_UCBRS);

    /* Configure port secondary functions
     * P4.4 - UCA1TXD
     * P4.5 - UCA1RXD */
    BIT_SET(P4SEL, (BIT4 | BIT5));

    /* Release the software reset */
    BIT_CLEAR(UCA1CTL1, UCSWRST);
}

static inline void prvInitButtons( void )
{
    /* Buttons S3 and S4 configuration
     *
     * Ports are configured as inputs with pull-up resistors
     * enabled, and falling edge interrupts enabled */
    BIT_CLEAR(P1DIR, (BIT4 | BIT5));
    BIT_SET(P1REN, (BIT4 | BIT5));
    BIT_SET(P1OUT, (BIT4 | BIT5));
    BIT_SET(P1IES, (BIT4 | BIT5));
    BIT_SET(P1IE, (BIT4 | BIT5));
}

static void prvUARTTaskFunction( void *pvParameters )
{
uart_data_t xReceivedData;
char        cNumHolder[4];

    for( ;; )
    {
        xQueueReceive(xUARTDataQueue, &xReceivedData, portMAX_DELAY);
        prvTransmitStringViaUART("Channel ");
        prvDecimalToString(xReceivedData.xChannel, cNumHolder);
        prvTransmitStringViaUART(cNumHolder);
        prvTransmitStringViaUART(": ");
        prvDecimalToString(xReceivedData.ucData, cNumHolder);
        prvTransmitStringViaUART(cNumHolder);
        prvTransmitStringViaUART("\n\r");
    }
}

static void prvTask1Function( void *pvParameters )
{
bridge_data_t xBridgeData;
adc_channel_t xADCChannel;
adc_data_t    xADCData;
uint8_t       ucCh0Pos = 0;
uint8_t       ucCh1Pos = 0;
uint8_t       ucChannel0Samples[mainNUM_OF_SAMPLES] = {0};
uint8_t       ucChannel1Samples[mainNUM_OF_SAMPLES] = {0};
uint8_t       ucChannel0Mean = 0;
uint8_t       ucChannel1Mean = 0;

    for( ;; )
    {
        /* Wait for an event - ADC Conversion or Button press */
        xQueueReceive(xTask1BridgeQueue, &xBridgeData, portMAX_DELAY);

        switch( xBridgeData )
        {
        case eADC_EVENT:
            /* Read the sample values from the ADC data queue
             * and calculate the mean value for last 8 samples
             * for each channel */
            xQueueReceive(xADCDataQueue, &xADCData, 0);
            switch( xADCData.xChannel )
            {
            case eADC_CHANNEL_0:
                ucChannel0Samples[ucCh0Pos++] = xADCData.ucData;
                /* if( ucCh0Pos == mainNUM_OF_SAMPLES ) ucCh0Pos = 0; */
                ucCh0Pos &= 0x07; /* More efficient when mainNUM_OF_SAMPLES == 8 */
            break;
            case eADC_CHANNEL_1:
                ucChannel1Samples[ucCh1Pos++] = xADCData.ucData;
                /* if( ucCh1Pos == mainNUM_OF_SAMPLES ) ucCh1Pos = 0; */
                ucCh1Pos &= 0x07; /* More efficient when mainNUM_OF_SAMPLES == 8 */
            break;
            default: break;
            }
        break;
        case eBUTTON_EVENT:
            /* Read from the channel select queue
             * and send the mean value to the right task */
            xQueueReceive(xChannelSelectQueue, &xADCChannel, 0);
            switch( xADCChannel )
            {
            case eADC_CHANNEL_0:
                ucChannel0Mean = prvArrayMean(ucChannel0Samples, mainNUM_OF_SAMPLES);
                xTaskNotify(xTask3Handle, ucChannel0Mean, eSetValueWithOverwrite);
            break;
            case eADC_CHANNEL_1:
                ucChannel1Mean = prvArrayMean(ucChannel1Samples, mainNUM_OF_SAMPLES);
                xTaskNotify(xTask4Handle, ucChannel1Mean, eSetValueWithOverwrite);
            break;
            default: break;
            }
        break;
        default: break;
        }
    }
}

static void prvTask2Function( void *pvParameters )
{
uint16_t            i;
uint32_t            ulNotificationValue;
adc_channel_t       xADCChannel;
const bridge_data_t xBridgeData = eBUTTON_EVENT;

    for( ;; )
    {
        /* Wait for a notification from PORT1_ISR */
        xTaskNotifyWait(ULONG_MAX, ULONG_MAX, &ulNotificationValue, portMAX_DELAY);
        /* Delay and read the button state - "Debounce" */
        ARTIFICIAL_DELAY(i, 3000);
        /* Depending on the notification value read the button state */
        switch( ulNotificationValue )
        {
        case mainS3_NOTIFICATION_VALUE:
            /* Re-enable interrupts */
            BIT_SET(P1IE, BIT4);
            if ( ( P1IN & BIT4 ) == 0 )
            {
                /* Fill the channel select queue */
                xADCChannel = eADC_CHANNEL_0;
                xQueueSendToBack(xChannelSelectQueue, &xADCChannel, portMAX_DELAY);
                /* Fill the xTask1 bridge queue */
                xQueueSendToBack(xTask1BridgeQueue, &xBridgeData, portMAX_DELAY);
            }
        break;
        case mainS4_NOTIFICATION_VALUE:
            /* Re-enable interrupts */
            BIT_SET(P1IE, BIT5);
            if ( ( P1IN & BIT5 ) == 0 )
            {
                /* Fill the channel select queue */
                xADCChannel = eADC_CHANNEL_1;
                xQueueSendToBack(xChannelSelectQueue, &xADCChannel, portMAX_DELAY);
                /* Fill the xTask1 bridge queue */
                xQueueSendToBack(xTask1BridgeQueue, &xBridgeData, portMAX_DELAY);
            }
        break;
        default: break;
        }

    }
}

static void prvPrepareDataTaskFunction( void *pvParameters )
{
uart_data_t xUARTData;
uint32_t    ulNotificationValue;

    /* Channel is assigned to each task when creating it */
    xUARTData.xChannel = ( adc_channel_t )( pvParameters );

    for( ;; )
    {
        /* Wait for a notification from xTask1 */
        xTaskNotifyWait(ULONG_MAX, ULONG_MAX, &ulNotificationValue, portMAX_DELAY);

        xUARTData.ucData = ( uint8_t )( ulNotificationValue );
        xQueueSendToBack(xUARTDataQueue, &xUARTData, portMAX_DELAY);
    }
}

static void prvTransmitStringViaUART( const char *str )
{
    while( *str )
    {
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = *str;
        str++;
    }
}

static void prvDecimalToString( unsigned int num, char *str )
{
    int i = 0;

    int j;
    int len;
    int digit;
    char tmp;

    do
    {
        digit = num % 10;
        str[i++] = '0' + digit;
        num /= 10;
    }
    while ( num != 0 );

    len = i;

    for(i = 0, j = len - 1; i < j; i++, j--)
    {
        tmp = str[i];
        str[i] = str[j];
        str[j] = tmp;
    }

    str[len] = '\0';
}

static inline uint8_t prvArrayMean( uint8_t *array, uint8_t length )
{
uint8_t  i;
uint32_t sum = 0;

    for( i = 0; i < length; i++ )
    {
        sum += array[i];
    }

    return ( sum / length );
}

void prvTimerCallbackFunction( TimerHandle_t xTimer )
{
    BIT_CLEAR(ADC12CTL0, ADC12SC);
    BIT_SET(ADC12CTL0, ADC12SC);
    BIT_TOGGLE(P1OUT, BIT0);
}

/**
 * @brief ADC12 Interrupt Service Routine
 *
 * Each time a conversion finishes, packages the conversion result and
 * the channel converted and sends the information to the ADC Data Queue.
 */
void __attribute__ ( ( interrupt( ADC12_VECTOR  ) ) ) ADC12_ISR( void )
{
BaseType_t          xHigherPriorityTaskWoken_1 = pdFALSE;
BaseType_t          xHigherPriorityTaskWoken_2 = pdFALSE;
BaseType_t          xHigherPriorityTaskWoken   = pdFALSE;
adc_data_t          xConversionResult;
const bridge_data_t xBridgeData = eADC_EVENT;


    switch( ADC12IV )
    {
    case ADC12IV_ADC12IFG0:
        /* Package data from channel 0 and send to queue */
        xConversionResult.xChannel = eADC_CHANNEL_0;
        xConversionResult.ucData   = (ADC12MEM0 >> 4) & 0xff;
        xQueueSendToBackFromISR(xADCDataQueue, &xConversionResult, &xHigherPriorityTaskWoken_1);
        /* Fill the xTask1 bridge queue */
        xQueueSendToBackFromISR(xTask1BridgeQueue, &xBridgeData, &xHigherPriorityTaskWoken_2);
    break;
    case ADC12IV_ADC12IFG1:
        /* Package data from channel 1 and send to queue */
        xConversionResult.xChannel = eADC_CHANNEL_1;
        xConversionResult.ucData   = (ADC12MEM1 >> 4) & 0xff;
        xQueueSendToBackFromISR(xADCDataQueue, &xConversionResult, &xHigherPriorityTaskWoken_1);
        /* Fill the xTask1 bridge queue */
        xQueueSendToBackFromISR(xTask1BridgeQueue, &xBridgeData, &xHigherPriorityTaskWoken_2);
    break;
    default: break;
    }

    if ( xHigherPriorityTaskWoken_1 || xHigherPriorityTaskWoken_2 )
        xHigherPriorityTaskWoken = pdTRUE;

    /* trigger scheduler if higher priority task is woken */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/**
 * @brief PORT1 Interrupt Service Routine
 *
 * Whenever a falling edge occurs on one of the buttons S3 (P1.4) or S4 (P1.5),
 * notifies the xTask2 about it.
 */
void __attribute__ ( ( interrupt( PORT1_VECTOR  ) ) ) PORT1_ISR( void )
{
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch( P1IV )
    {
    case P1IV_P1IFG4:
        /* Notify the Button task that there is a potential button press */
        xTaskNotifyFromISR(xTask2Handle, mainS3_NOTIFICATION_VALUE, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
        /* Disable interrupts */
        BIT_CLEAR(P1IE, BIT4);
        /* Clear the interrupt flag */
        BIT_CLEAR(P1IFG, BIT4);
    break;
    case P1IV_P1IFG5:
        /* Notify the Button task that there is a potential button press */
        xTaskNotifyFromISR(xTask2Handle, mainS4_NOTIFICATION_VALUE, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
        /* Disable interrupts */
        BIT_CLEAR(P1IE, BIT5);
        /* Clear the interrupt flag */
        BIT_CLEAR(P1IFG, BIT5);
    break;
    default: break;
    }

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
