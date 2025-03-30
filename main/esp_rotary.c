#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "esp_check.h"

static const char *logtag = "rotary";

#define GPIO_ROT_A	4
#define GPIO_ROT_B	5
#define GPIO_SWITCH	6

#define GPIO_BLINK	2


enum event_type {
	EVENT_TURN_CLOCKW,
	EVENT_TURN_COUNTW,
	EVENT_SWITCH_CLOSE,
	EVENT_SWITCH_OPEN,
};


#define EVENT_QUEUE_SIZ	10
static QueueHandle_t _event_queue = NULL;

void
isr_rotary_a(void *arg)
{ 

#if 0
	int	rotary_idx;

	rotary_idx = (uint32_t) arg;
#endif


	uint32_t b_value;
	uint32_t ev;


	/* Since we are in an ISR, we are checking the B value by accessing
	 * the GPIO input register directly */
	b_value = (REG_READ(GPIO_IN_REG) >> GPIO_ROT_B) & 1;

	if(b_value)
		ev = EVENT_TURN_CLOCKW;
	else
		ev = EVENT_TURN_COUNTW;

	xQueueSendFromISR(_event_queue, &ev, NULL);
}


void
isr_rotary_switch(void *arg)
{
#if 0
	int	rotary_idx;

	rotary_idx = (uint32_t) arg;
#endif
}
  

int
config_gpio(int pinnr, int isinput, gpio_isr_t isr, void *isr_arg)
{
	gpio_config_t	config;
	int		ret;
	int		err;

	err = ESP_OK;

	memset(&config, 0, sizeof(gpio_config_t));

	config.mode = isinput ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT;
	config.intr_type = isr ? GPIO_INTR_POSEDGE : GPIO_INTR_DISABLE;
    	config.pin_bit_mask = 1ULL << pinnr;
	config.pull_down_en = 1;
	return gpio_config(&config);

	if(isr) {
		ret = gpio_isr_handler_add(pinnr, isr, isr_arg);
		printf("Could not install interrupt handler\n");
		err = ret;
	}

	return err;
}

#define EVENT_LOOP_PRIORITY	10

static void event_loop(void *arg)
{
	uint32_t	event;

	while(1) {
		if(xQueueReceive(_event_queue, &event, portMAX_DELAY)) {
			printf("Event: %"PRIu32"\n", event);
		}
		printf("high watermark: %d\n",
		    uxTaskGetStackHighWaterMark(NULL));
	}
}


void
app_main(void)
{
	esp_err_t	ret;

	ret = ESP_OK;

	ESP_GOTO_ON_ERROR(gpio_install_isr_service(0), err_label, logtag,
	    "Could not install gpio ISR Service");

	/* Rotary A: input, interrupt */
	ESP_GOTO_ON_ERROR(config_gpio(GPIO_ROT_A, 1, isr_rotary_a, NULL),
	    err_label, logtag, "Could not configure pin %d", GPIO_ROT_A);

	/* Rotary B: input, no interrupt */
	ESP_GOTO_ON_ERROR(config_gpio(GPIO_ROT_B, 1, NULL, NULL), err_label,
	    logtag, "Could not configure pin %d", GPIO_ROT_B);

	/* Rotary Switch: input, interrupt */
	ESP_GOTO_ON_ERROR(config_gpio(GPIO_SWITCH, 1, isr_rotary_switch, NULL),
	    err_label, logtag, "Could not configure pin %d", GPIO_SWITCH);

	/* LED to blink: output, no interrupt */
	ESP_GOTO_ON_ERROR(config_gpio(GPIO_BLINK, 0, NULL, NULL), err_label,
	    logtag, "Could not configure pin %d", GPIO_BLINK);

	_event_queue = xQueueCreate(EVENT_QUEUE_SIZ, sizeof(uint32_t));
	if(_event_queue == NULL) {
		printf("Could not create _event_queue");
		goto err_label;
	}

	ret = xTaskCreate(event_loop, "event_loop", 2048, NULL,
	    EVENT_LOOP_PRIORITY, NULL);
	if(ret != pdPASS) {
		printf("Could not create event_loop task");
		goto err_label;
	}


err_label:

	printf("Error: %s\n", esp_err_to_name(ret));

	while (1) {
       		gpio_set_level(GPIO_BLINK, 0);
		vTaskDelay(pdMS_TO_TICKS(100));
		gpio_set_level(GPIO_BLINK, 1);
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
