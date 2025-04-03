#ifndef ESP_ROTARY_PARAMS
#define ESP_ROTARY_PARAMS

/*
 * Modifiable parameters
 */

#define ROTARY_CALL_GPIO_INSTALL_ISR_SERVICE 1
	/* If this is nonzero, rotary_config() will call
	 * gpio_install_isr_service(). Set to 0 or the application will call
	 * it (make sure it is called before rotary_config() */

#define ROTARY_GPIO_INSTALL_ISR_SERVICE_FLAGS 0
	/* Flags to call gpio_install_isr_service() with. */

#define ROTARY_ISR_EVENT_TASK_PRI 10
	/* Priority of esp_rotary's internal event management task. */

#define ROTARY_SWITCH_DEBOUNCE_MS 50
	/* Milliseconds delay for the switch debounce logic */

#define ROTARY_SPEED_BOOST_MEDIUM_MS 20
	/* Milliseconds between value changes under which the medium boost
	 * mode will engage */

#define ROTARY_SPEED_BOOST_MEDIUM_VALUE_CHANGE 2
	/* Value change to apply in medium boost mode  */

#define ROTARY_SPEED_BOOST_FAST_MS 20
	/* Milliseconds between value changes under which the fast boost
	 * mode will engage */

#define ROTARY_SPEED_BOOST_FAST_VALUE_CHANGE 4
	/* Value change to apply in fast boost mode  */

#define ROTARY_EVENT_QUEUE_SIZE 10
	/* Size of rotary_event_queue  */

#define ROTARY_ISR_QUEUE_SIZE 10
	/* Size of esp_rotary's internal ISR event queue  */


#endif
