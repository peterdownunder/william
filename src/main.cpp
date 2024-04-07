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
#include "pico/binary_info.h"


#include "heartbeat.h"

enum fadeDirection {rampUp, rampDown, turnOff};


struct audio_buffer_pool *init_audio();

#define ledcount 6
#define RED 20
#define BLUE 11
#define WHITERED 10
#define WHITEBLUE 8
#define YELLOW 12
#define GREEN 7

#define SAMPLES_PER_BUFFER 256
uint32_t uIReceivedValue;
struct ledState {
  int GpioPort;
  fadeDirection currentState;
  double brightnessAdjust;
  int currentBrightness;
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
uint32_t rampUpAdjust = 4;
int rampDownAdjust = 10;
uint32_t currentVolume = 128;
int currentLed = 0;

void vTimerCallback( TimerHandle_t xTimer )
{
    uint32_t ulAdjust = ( uint32_t ) pvTimerGetTimerID( xTimer );
    uint32_t nextPlay = ulAdjust;
    ulAdjust = rampUpAdjust;
    for (int l = 0 ; l < ledcount ; l++ ) {
        ledState& currentLed = Leds[l];
        ledState& nextLed = l==ledcount-1 ? Leds[0] : Leds[l+1];

        if (currentLed.currentState == fadeDirection::rampUp) {
            currentLed.currentBrightness += ulAdjust;
            if (currentLed.currentBrightness > 255) {
                currentLed.currentBrightness = 128;
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
                rampUpAdjust = newRampUpAdjust;
                lastRampUpAdjust = newRampUpAdjust;
            }
        }
        adc_select_input(1);
        newVolume = adc_read();
        currentVolume = (uint32_t)((newVolume / 1000.0) * 150.0);
        printf ("new volume %d  ", currentVolume);
        vTaskDelay(xpollingDelay);
    }
}

void vTaskLightController(void *pvParameters){
    StaticTimer_t xTimerBuffer;
    TimerHandle_t xTimer = xTimerCreate ("Timer", pdMS_TO_TICKS(25), pdTRUE, ( void * ) 0, vTimerCallback);
    vTimerSetTimerID( xTimer, ( void * ) rampUpAdjust );
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
    TaskHandle_t hAudio;
    TaskHandle_t hControl;
    qPlayFile = xQueueCreate(100, sizeof(uint32_t));
    qLightSpeed = xQueueCreate(100, sizeof(uint32_t));

    xTaskCreate(vTaskReadSpeed, "speed", task_size, NULL, 1, NULL);
    xTaskCreate(vTaskLightController, "control", task_size, NULL, 1, &hControl);
    xTaskCreate(vTaskPlayer, "player", task_size, NULL, 1, &hAudio);
   
    vTaskStartScheduler();
    return 0;
}


