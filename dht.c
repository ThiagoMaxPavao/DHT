#include "dht.h"
#include "dhtConf.h"
#include "tim.h"
#if (_DHT_USE_FREERTOS==1)
#include "cmsis_os.h"
#define DHT_delayMs(x)     osDelay(x)
#else
#define DHT_delayMs(x)     HAL_Delay(x)
#endif

/**
 * @brief Callback function for pin change interrupt.
 *
 * This callback function should be called inside the HAL_GPIO_EXTI_Callback ISR,
 * after checking that the pin that caused the interrupt is dht.pin
 * The pin interruput should be configured for both Rising and Falling Edge.
 *
 * @param dht Pointer to the DHT structure.
 *
 * @retval None
 */
void DHT_pinChangeCallBack(DHT_t *dht)
{
    dht->time = HAL_GetTick();
    if (dht->cnt < sizeof(dht->data) - 1)
    {
        dht->data[dht->cnt] = dht->tim->Instance->CNT - dht->lastCNT;
        dht->lastCNT = dht->tim->Instance->CNT;
        dht->cnt++;
    }
}

/**
 * @brief Delay for the specified number of microseconds.
 *
 * This function provides a delay for the specified number of microseconds using the Timer associated with the DHT struct.
 *
 * @param dht Pointer to the DHT structure.
 * @param DelayUs Number of microseconds to delay.
 *
 * @retval None
 */
void DHT_delayUs(DHT_t *dht, uint16_t DelayUs)
{
    dht->tim->Instance->CNT = 0;
    while (dht->tim->Instance->CNT < DelayUs);
}

/**
 * @brief Configure the GPIO pin for output mode for DHT sensor communication.
 *
 * This function configures the GPIO pin connected to the DHT sensor for output mode.
 *
 * @param dht Pointer to the DHT structure.
 *
 * @retval None
 */
void DHT_output(DHT_t *dht)
{
    GPIO_InitTypeDef gpio;
    dht->gpio->BSRR = dht->pin;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin = dht->pin;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(dht->gpio, &gpio);
}

/**
 * @brief Configure the GPIO pin for input mode for DHT sensor communication.
 *
 * This function configures the GPIO pin connected to the DHT sensor for input mode.
 *
 * @param dht Pointer to the DHT structure.
 *
 * @retval None
 */
void DHT_input(DHT_t *dht)
{
    GPIO_InitTypeDef gpio;
    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin = dht->pin;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(dht->gpio, &gpio);
}

/**
 * @brief Decode the data received from the DHT sensor.
 *
 * This function decodes the raw data received from the DHT sensor into a byte array.
 *
 * @param dht Pointer to the DHT structure.
 * @param byteArray Pointer to the byte array to store the decoded data.
 *
 * @retval true if decoding is successful, false otherwise.
 */
bool DHT_decode(DHT_t *dht, uint8_t *byteArray)
{
    int8_t bit;

    uint16_t firstTimeUs, // first times at transmission beggining (low and high are the same)
            lowTimeUs, // during data bits, time spent low between bits
            highTimeZeroUs, // time on High for a zero
            highTimeOneUs; // time on High for a one

    uint16_t toleranceUs; // tolerance time for data transmission

    switch (dht->type)
    {
    case DHT_Type_DHT11:
    case DHT_Type_DHT22:
        firstTimeUs = 80;
        lowTimeUs = 50;
        highTimeZeroUs = 27;
        highTimeOneUs = 70;

        toleranceUs = 15;
        break;
    default:
        return false;
    }

    // check first time (high), ignore low time (index = 0)
    if ((dht->data[1] < firstTimeUs - toleranceUs)
            || (dht->data[1] > firstTimeUs + toleranceUs))
        return false;

    bit = 7;
    for (uint8_t i = 2; i < 82; i += 2)
    {
        // check low time
        if ((dht->data[i] < lowTimeUs - toleranceUs)
                || (dht->data[i] > lowTimeUs + toleranceUs))
            return false;

        // check high time, zero or one or error
        if ((dht->data[i + 1] >= highTimeZeroUs - toleranceUs)
                && (dht->data[i + 1] <= highTimeZeroUs + toleranceUs))
            *byteArray &= ~(1 << bit);
        else if ((dht->data[i + 1] >= highTimeOneUs - toleranceUs)
                && (dht->data[i + 1] <= highTimeOneUs + toleranceUs))
            *byteArray |= (1 << bit);
        else
            return false;

        bit--;
        if (bit == -1)
        {
            bit = 7;
            byteArray++;
        }

    }

    return true;
}

/**
 * @brief Initialize the DHT sensor.
 *
 * This function initializes the DHT sensor with the specified parameters.
 *
 * @param dht Pointer to the DHT structure.
 * @param type Type of the DHT sensor (DHT11, DHT22...).
 * @param tim Pointer to the Timer handle used for communication timing.
 * @param timerBusFrequencyMHz Timer bus frequency in MHz.
 * @param gpio GPIO port used for sensor communication.
 * @param pin GPIO pin used for sensor communication.
 *
 * @retval None
 */
void DHT_init(DHT_t *dht, DHT_Type_t type, TIM_HandleTypeDef *tim,
        uint16_t timerBusFrequencyMHz, GPIO_TypeDef *gpio, uint16_t pin)
{
    // configure struct parameters
    dht->tim = tim;
    dht->gpio = gpio;
    dht->pin = pin;
    dht->type = type;

    // set dht as output
    DHT_output(dht);

    // Configure Timer to 1us period and start
    dht->tim->Init.Prescaler = timerBusFrequencyMHz - 1;
    dht->tim->Init.CounterMode = TIM_COUNTERMODE_UP;
    dht->tim->Init.Period = 0xFFFF;
    dht->tim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(dht->tim);
    HAL_TIM_Base_Start(dht->tim);

    // Wait for DHT initialization
    while (HAL_GetTick() < 2000)
        DHT_delayMs(1);
    DHT_delayMs(20);
}

/**
 * @brief Read data from the DHT sensor.
 *
 * This function reads temperature and humidity data from the DHT sensor.
 *
 * @param dht Pointer to the DHT structure.
 * @param Temperature Pointer to store the temperature value.
 * @param Humidity Pointer to store the humidity value.
 *
 * @retval true if data read successfully, false otherwise.
 */
bool DHT_readData(DHT_t *dht, float *Temperature, float *Humidity)
{
    uint32_t startTime;

    uint32_t startCommunicationLowMs, // time to pull down DATA pin to start communication
            pullDownWaitUs, // time to wait so that DHT has pulled down pin
            maxReadingTimeMs, // time to cancel conversion and return error, taking too long
            timeoutTimeMs; // time with no pin change from DHT, to trigger conversion decoding

    // Configures times depending on sensor type
    switch (dht->type)
    {
    case DHT_Type_DHT11:
        startCommunicationLowMs = 20; // at least 18ms
        pullDownWaitUs = 45; // in range 20 to 40us, has to be more than that
        maxReadingTimeMs = 6; // should take at maximum 4.8 ms
        timeoutTimeMs = 1; // default, each change while sending data takes at maximum 70us so 1ms of no change signals end of transmission
        break;
    case DHT_Type_DHT22:
        startCommunicationLowMs = 1; // at least 800us
        pullDownWaitUs = 45;
        maxReadingTimeMs = 6; // should take at maximum 4.8 ms
        timeoutTimeMs = 1; // default, each change while sending data takes at maximum 70us so 1ms of no change signals end of transmission
        break;
    default:
        goto ERROR;
        // time not configured for given DHT type
        break;
    }

    // Communication Trigger
    DHT_output(dht);
    dht->gpio->BSRR = (dht->pin) << 16; // RESET PIN
    DHT_delayMs(startCommunicationLowMs);
    dht->gpio->BSRR = dht->pin; // SET PIN
    DHT_delayUs(dht, pullDownWaitUs);

    // Reset timer and auxiliary variables
    dht->cnt = 0;
    dht->lastCNT = 0;
    dht->tim->Instance->CNT = 0;
    startTime = HAL_GetTick();
    DHT_input(dht);

    // Loops checking for timeout and maxReading Times
    while (1)
    {
        if (HAL_GetTick() - startTime > maxReadingTimeMs)
            goto ERROR;
        if (HAL_GetTick() - dht->time > timeoutTimeMs)
            break;
    }

    // timeout, may have good data, try conversion

    uint8_t data[5];
    if (DHT_decode(dht, data) == false) // Try decoding, could go wrong
        goto ERROR;
    if (((data[0] + data[1] + data[2] + data[3]) & 0x00FF) != data[4]) // Checksum
        goto ERROR;

    // Decoding Success, convert to number
    switch (dht->type)
    {
    case DHT_Type_DHT11:
        dht->temperature = (float) (data[2]);
        dht->humidity = (float) (data[0]);
        break;
    case DHT_Type_DHT22:
        dht->temperature = (float) ((data[2] << 8) | data[3]) / 10.0f;
        dht->humidity = (float) ((data[0] << 8) | data[1]) / 10.0f;
        break;
    default:
        goto ERROR;
        // calculation not configured for given DHT type
        break;
    }

    // save data in output parameters
    if (Temperature != NULL)
        *Temperature = dht->temperature;
    if (Humidity != NULL)
        *Humidity = dht->humidity;

    // Success return
    dht->dataValid = true;
    DHT_output(dht);
    return true;

    ERROR:
    dht->dataValid = false;
    DHT_output(dht);
    return false;
}
