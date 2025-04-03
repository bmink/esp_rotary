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
	unsigned char		enable_speed_boost;

	unsigned char		rc_pin_a;
	unsigned char		rc_pin_b;
	unsigned char		rc_pin_switch;


} rotary_config_t;

esp_err_t rotary_config(rotary_config_t *, unsigned char);


typedef enum rotary_switch_state {
	SWITCH_NOTPRESSED,
	SWITCH_PRESSED
} rotary_switch_state_t;

int32_t	rotary_get_value(unsigned char);
rotary_switch_state_t rotary_get_switch_state(unsigned char);


typedef enum rotary_event_type {
	ROT_EVENT_INCREMENT,
	ROT_EVENT_DECREMENT,
	ROT_EVENT_SWITCH_PRESS,
	ROT_EVENT_SWITCH_RELEASE
} rotary_event_type_t;


typedef struct rotary_event {
	rotary_event_type_t	re_type;
	unsigned char		re_idx;
	int32_t			re_value;
} rotary_event_t;

extern QueueHandle_t	rotary_event_queue;



#endif
