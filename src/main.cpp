#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <stdio.h>
#include <math.h>
#include <queue.h>
#include "pico/stdlib.h"
#include "pico/audio_i2s.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"


#include "max30105Utils.h"
#include "heartRate.h"
#include "heartbeat.h"

enum fadeDirection {rampUp, rampDown, turnOff};

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

struct audio_buffer_pool *init_audio();

#define ledcount 6
#define RED 20
#define BLUE 11
#define WHITERED 10
#define WHITEBLUE 8
#define YELLOW 12
#define GREEN 7

#define LED_STATUS 6

#define SAMPLES_PER_BUFFER 256
uint32_t uIReceivedValue;
struct ledState {
  int GpioPort;
  fadeDirection currentState;
  double brightnessAdjust;
  double currentBrightness;
};

ledState Leds[ledcount] = {
  {RED,    rampUp,     1.0, 0  },   
  {BLUE,     turnOff,    1.0, 0  }, 
  {WHITERED,    turnOff,    0.25, 0  },
  {YELLOW,     turnOff,    1.0, 0  },
  {GREEN,    turnOff,    1.0, 0  },
  {WHITEBLUE,    turnOff,    0.25, 0  }
};

static QueueHandle_t qPlayFile = NULL;
static QueueHandle_t qLightDown = NULL;
static QueueHandle_t qLightSpeed = NULL;
const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;
const int task_size = 128;
double rampUpAdjustHeartbeatOverride = 0.0;
double rampUpAdjust = 6.4;
int rampDownAdjust = 10;
uint32_t currentVolume = 128;
int currentLed = 0;
// called every 25 ms (0 - 255 in 8 increments) (the 128 to 0 every 25ms in jumps of 16 is the trailing led, so does not affect the period)
void vTimerCallback( TimerHandle_t xTimer )
{
//    uint32_t ulAdjust = ( uint32_t ) pvTimerGetTimerID( xTimer );
    uint32_t nextPlay = 1;
    double ulAdjust = rampUpAdjust;
    if (rampUpAdjustHeartbeatOverride > 0.0) {
        ulAdjust = rampUpAdjustHeartbeatOverride;
    }
    for (int l = 0 ; l < ledcount ; l++ ) {
        ledState& currentLed = Leds[l];
        ledState& nextLed = l==ledcount-1 ? Leds[0] : Leds[l+1];

        if (currentLed.currentState == fadeDirection::rampUp) {
            currentLed.currentBrightness += ulAdjust;
            if (currentLed.currentBrightness > 255.0) {
                currentLed.currentBrightness = 128.0;
                currentLed.currentState = fadeDirection::rampDown;
                nextLed.currentState = fadeDirection::rampUp;
                xQueueSendToBack(qPlayFile, &nextPlay, 0);    
            } 
            pwm_set_gpio_level(currentLed.GpioPort, 
                (int)((currentLed.currentBrightness * currentLed.currentBrightness) * currentLed.brightnessAdjust) );
        } 
        if (currentLed.currentState == fadeDirection::rampDown) {
            currentLed.currentBrightness -= (ulAdjust*2);
            if (currentLed.currentBrightness <= 0) {
                currentLed.currentBrightness = 0;
                currentLed.currentState = fadeDirection::turnOff;
            } 
            pwm_set_gpio_level(currentLed.GpioPort, 
                (int)((currentLed.currentBrightness * currentLed.currentBrightness) * currentLed.brightnessAdjust) );
        }
    }
}

void vTaskPlayer(void *pvParameters){

    struct audio_buffer_pool *ap = init_audio();
    uint32_t step = 0x200000;
    uint32_t pos = 0;

    bool beatComplete = false;
    uint32_t uIReceivedValue;
    while (1) {
        if (xQueueReceive(qPlayFile, &uIReceivedValue, 0) == pdTRUE) {
            pos = 0;
        }
        
        struct audio_buffer *buffer = take_audio_buffer(ap, true);
        int16_t *samples = (int16_t *) buffer->buffer->bytes;
        for (uint i = 0; i < buffer->max_sample_count; i++) {
            samples[i] = (currentVolume * WAV_DATA[pos]);
            pos += 1;
            if (pos >= WAV_DATA_LENGTH) {
                pos = 0;
            } 
//            printf ("%d .. ", pos);
            buffer->sample_count = buffer->max_sample_count;
            give_audio_buffer(ap, buffer);
        }
    }

}

void vTaskReadSpeed(void *pvParameters){
const TickType_t xpollingDelay = 200 / portTICK_PERIOD_MS;
uint16_t currentAdc = 0;
uint16_t lastAdc = 0;
uint32_t newRampUpAdjust = 0;
uint32_t lastRampUpAdjust = 0;

uint32_t newVolume = 0;
uint32_t lastVolume = 0;

    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    
    while (true) {
        adc_select_input(0);
        currentAdc = adc_read();
        if (currentAdc != lastAdc) {
            newRampUpAdjust = 4 + (int)(currentAdc/100);
            if (lastRampUpAdjust != newRampUpAdjust) {
                lastAdc = currentAdc;
//                xQueueSendToBack(qLightSpeed, &newRampUpAdjust, 0);
                rampUpAdjust = (double)newRampUpAdjust;
                lastRampUpAdjust = newRampUpAdjust;
            }
        }
        adc_select_input(1);
        newVolume = adc_read();
        currentVolume = (uint32_t)((newVolume / 1000.0) * 150.0);
//        printf ("new volume %d  ", currentVolume);
        vTaskDelay(xpollingDelay);
    }
}

void vTaskLightController(void *pvParameters){
    StaticTimer_t xTimerBuffer;
    TimerHandle_t xTimer = xTimerCreate ("Timer", pdMS_TO_TICKS(25), pdTRUE, ( void * ) 0, vTimerCallback);
    vTimerSetTimerID( xTimer, ( void * ) &rampUpAdjust );
    xTimerStart(xTimer, 0);

    while (true) {
//        vTaskDelay( pdMS_TO_TICKS(25) );
  //      printf ("wait for new speed\n");
        xQueueReceive(qLightSpeed, &uIReceivedValue, portMAX_DELAY);
    //    printf ("new spseed %d\n", uIReceivedValue);
//        vTimerSetTimerID( xTimer, ( void * ) uIReceivedValue );
    }
}
 
struct audio_buffer_pool *init_audio() {

    static audio_format_t audio_format = {
            .sample_freq = 11000,
            .format = AUDIO_BUFFER_FORMAT_PCM_U8,
            .channel_count = 1,
    };

    static struct audio_buffer_format producer_format = {
            .format = &audio_format,
            .sample_stride = 2
    };

    struct audio_buffer_pool *producer_pool = audio_new_producer_pool(&producer_format, 3,
                                                                      SAMPLES_PER_BUFFER); // todo correct size
    bool __unused ok;
    const struct audio_format *output_format;
    struct audio_i2s_config config = {
            .data_pin = PICO_AUDIO_I2S_DATA_PIN,
            .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
            .dma_channel = 0,
            .pio_sm = 0,
    };

    output_format = audio_i2s_setup(&audio_format, &config);
    if (!output_format) {
        panic("PicoAudio: Unable to open audio device.\n");
    }

    ok = audio_i2s_connect(producer_pool);
    assert(ok);
    audio_i2s_set_enabled(true);
    return producer_pool;
}

void vTaskHeart(void *pvParameters){
    static const uint8_t MAX30105_Address = 0x57;

    const u_int8_t RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
    u_int8_t rates[RATE_SIZE];    // Array of heart rates
    u_int8_t rateSpot = 0;
    long lastBeat = 0; // Time at which the last beat occurred
    bool finger = false;
    float beatsPerMinute;
    long rampOverrideTimeout = 0;
    int beatAvg;

    vTaskDelay(5000);
printf("i2c_init\n");
    i2c_init(I2C_PORT, 100 * 1000);
printf("i2c_init done\n");
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    max30105Utils particleSensor = max30105Utils();
    bool isFound = particleSensor.begin(MAX30105_Address);
    vTaskDelay(2000);
    particleSensor.setup(0x1f, 4, 3, 400, 411, 4096);
    float temp = particleSensor.readTemperature();
    while (true)
    {
//        vTaskDelay(10);
        long irValue = particleSensor.getIR();

        if (irValue > 50000) {
            if (!finger) {
                printf("found finger %d\n", irValue);
                if (rampUpAdjustHeartbeatOverride == 0.0)
                    pwm_set_gpio_level(LED_STATUS, 65000);
                finger = true;
            }
        } else {
            if (finger) {
                printf("de fingered %d override in place %f\n", irValue, rampUpAdjustHeartbeatOverride);
                finger = false;
                pwm_set_gpio_level(LED_STATUS, 0);
                for (u_int8_t x = 0; x < RATE_SIZE; x++)
                    rates[x] = 0;
//                rampOverrideTimeout = to_ms_since_boot(get_absolute_time()) + 20000;
            }
        }
        if (rampUpAdjustHeartbeatOverride > 0.0) {
            long now = to_ms_since_boot(get_absolute_time());
            if (now > rampOverrideTimeout) {
                printf("reset override\n");
                rampUpAdjustHeartbeatOverride = 0.0;
            }
        }
        if (checkForBeat(irValue) == true)
        {
            // We sensed a beat!
            long delta = to_ms_since_boot(get_absolute_time()) - lastBeat;
            lastBeat = to_ms_since_boot(get_absolute_time());

            beatsPerMinute = 60 / (delta / 1000.0);

            if (beatsPerMinute < 155 && beatsPerMinute > 40)
            {
                int validBeats = 0;
                rates[rateSpot++] = (u_int8_t)beatsPerMinute; // Store this reading in the array
                rateSpot %= RATE_SIZE;                    // Wrap variable

                // Take average of readings
                beatAvg = 0;
                for (u_int8_t x = 0; x < RATE_SIZE; x++) {
                    beatAvg += rates[x];
                    if (rates[x] > 0) validBeats++;
                }
                
                beatAvg /= RATE_SIZE;
                if (validBeats == RATE_SIZE) {
                    rampUpAdjustHeartbeatOverride = ((double)beatAvg / 60) * 6.4;
                    printf("Lets use=%d override to %f\n", beatAvg, rampUpAdjustHeartbeatOverride);
                    rampOverrideTimeout = to_ms_since_boot(get_absolute_time()) + 20000;
                    pwm_set_gpio_level(LED_STATUS, 0);
                }
                printf("IR=%d, BPM=%f, Avg BPM=%d \n", irValue, beatsPerMinute, beatAvg);    
            } else {
                printf("Ignoring BPM=%f\n", beatsPerMinute);    
            }
        }
    } 
} 

int main(){

    bi_decl(bi_3pins_with_names(PICO_AUDIO_I2S_DATA_PIN, "I2S DIN", PICO_AUDIO_I2S_CLOCK_PIN_BASE, "I2S BCK", PICO_AUDIO_I2S_CLOCK_PIN_BASE+1, "I2S LRCK"));
 
    stdio_init_all();
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 8.0);

    for (int iled = 0 ; iled < ledcount ; iled++) {
        gpio_set_function(Leds[iled].GpioPort, GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(Leds[iled].GpioPort);
        pwm_init(slice, &config, true);
        pwm_set_enabled(slice, true);
    }
    gpio_set_function(LED_STATUS, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(LED_STATUS);
    pwm_init(slice, &config, true);
    pwm_set_enabled(slice, true);

    TaskHandle_t hAudio;
    TaskHandle_t hControl;
    TaskHandle_t hHeartRate;
    qPlayFile = xQueueCreate(100, sizeof(uint32_t));
    qLightSpeed = xQueueCreate(100, sizeof(uint32_t));

    xTaskCreate(vTaskReadSpeed, "speed", task_size, NULL, 1, NULL);
    xTaskCreate(vTaskLightController, "control", task_size, NULL, 1, &hControl);
    xTaskCreate(vTaskPlayer, "player", task_size*5, NULL, 1, &hAudio);
    xTaskCreate(vTaskHeart, "heartbeat", task_size*5, NULL, 1, &hHeartRate);
    vTaskStartScheduler();
    while (true) {
        sleep_ms(1);
    }
    return 0;
}


