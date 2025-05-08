#ifndef ESP_ROTARY
#define ESP_ROTARY

#include "freertos/FreeRTOS.h"

typedef enum rotary_style {
	ROT_STYLE_BOUND,
	ROT_STYLE_WRAPAROUND
} esp_rotary_style_t;


typedef struct rotary_config {

	esp_rotary_style_t	rc_style;

	int32_t			rc_max;
	int32_t			rc_min;
	int32_t			rc_start;
	uint8_t			rc_step_value;
	uint8_t			rc_enable_speed_boost;

	uint8_t			rc_pin_a;
	uint8_t			rc_pin_b;
	uint8_t			rc_pin_button;


} rotary_config_t;

esp_err_t rotary_config(rotary_config_t *, uint8_t);
esp_err_t rotary_reconfig(rotary_config_t *, uint8_t);


typedef enum rotary_button_state {
	BUTTON_NOTPRESSED,
	BUTTON_PRESSED
} rotary_button_state_t;

int32_t	rotary_get_value(uint8_t);
rotary_button_state_t rotary_get_button_state(uint8_t);


typedef enum rotary_event_type {
	ROT_EVENT_INCREMENT,
	ROT_EVENT_DECREMENT,
	ROT_EVENT_BUTTON_PRESS,
	ROT_EVENT_BUTTON_RELEASE
} rotary_event_type_t;


typedef struct rotary_event {
	rotary_event_type_t	re_type;
	uint8_t			re_idx;
	int32_t			re_value;
} rotary_event_t;

extern QueueHandle_t	rotary_event_queue;

void rotary_event_queue_reset(void);


#endif
