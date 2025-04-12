#include <stdio.h>
#include <string.h>
#include "esp_rotary.h"
#include "driver/gpio.h"
#include "esp_check.h"


/*
 * This example uses the rotary event queue to receive events
 */

int
config_led_gpio(int pinnr)
{
	gpio_config_t	config;
	int		ret;

	memset(&config, 0, sizeof(gpio_config_t));

	config.mode = GPIO_MODE_OUTPUT;
	config.intr_type = GPIO_INTR_DISABLE;
	config.pin_bit_mask = 1ULL << pinnr;
	ret = gpio_config(&config);
	if(ret != ESP_OK) {
		printf("Could not config gpio\n");
		return ret;
		
	}

	return 0;
}


static const char *logtag = "rotary test";

#define GPIO_BLINK	2

void
app_main(void)
{
	esp_err_t	ret;
	rotary_event_t	rev;
	rotary_config_t	rconf[2];

	/* Set up onboard LED (will blink on error) */
	ESP_GOTO_ON_ERROR(config_led_gpio(GPIO_BLINK), err_label,
	    logtag, "Could not configure LED on pin %d", GPIO_BLINK);

	/* Rotary encoders have to be configured all at once. Here we set up
         * two encoders */
	memset(rconf, 0, sizeof(rotary_config_t) * 2);
	
	rconf[0].rc_pin_a = 4;
	rconf[0].rc_pin_b = 5;
	rconf[0].rc_pin_button = 6;
	rconf[0].rc_style = ROT_STYLE_BOUND; /* Will not go over min and max */
	rconf[0].rc_min = 0;
	rconf[0].rc_max = 100000;
	rconf[0].rc_start = 0;
	rconf[0].rc_enable_speed_boost = 1;

	rconf[1].rc_pin_a = 7;
	rconf[1].rc_pin_b = 8;
	rconf[1].rc_pin_button = 9;
	rconf[1].rc_style = ROT_STYLE_WRAPAROUND; /* Value will wrap around */
	rconf[1].rc_min = -10;
	rconf[1].rc_max = 30;
	rconf[1].rc_start = 10;
	
	ret = rotary_config(rconf, 2);
	if(ret != ESP_OK) {
		printf("Could not configure rotary encoders\n");
		goto err_label;
	}	

	while(1) {
		if(xQueueReceive(rotary_event_queue, &rev, portMAX_DELAY) !=
		   pdPASS)
			continue;

		switch(rev.re_type) {
		case ROT_EVENT_INCREMENT:
			printf("Rotary %d value incremented to %"PRId32"\n",
				rev.re_idx, rev.re_value);
			break;
		case ROT_EVENT_DECREMENT:
			printf("Rotary %d value decremented to %"PRId32"\n",
				rev.re_idx, rev.re_value);
			break;
		case ROT_EVENT_BUTTON_PRESS:
			printf("Rotary %d switch pressed\n", rev.re_idx);
			break;
		case ROT_EVENT_BUTTON_RELEASE:
			printf("Rotary %d switch released\n", rev.re_idx);
			break;
		default:
			printf("Rotary %d unknown event\n", rev.re_idx);
			break;
		}
	}


err_label:

	printf("Error: %s\n", esp_err_to_name(ret));

	/* Blink onboard LED */

	while (1) {
       		gpio_set_level(GPIO_BLINK, 0);
		vTaskDelay(pdMS_TO_TICKS(100));
		gpio_set_level(GPIO_BLINK, 1);
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
