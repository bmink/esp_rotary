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

#define ROTARY_SPEED_BOOST_MEDIUM_TICKS pdMS_TO_TICKS(20)
	/* Ticks between value changes under which the medium boost
	 * mode will engage */

#define ROTARY_SPEED_BOOST_MEDIUM_VALUE_CHANGE 5
	/* Value change to apply in medium boost mode  */

#define ROTARY_SPEED_BOOST_FAST_TICKS pdMS_TO_TICKS(10)
	/* Ticks between value changes under which the fast boost
	 * mode will engage */

#define ROTARY_SPEED_BOOST_FAST_VALUE_CHANGE 10
	/* Value change to apply in fast boost mode  */

#define ROTARY_EVENT_QUEUE_SIZE 10
	/* Size of rotary_event_queue  */

#define ROTARY_ISR_QUEUE_SIZE 10
	/* Size of esp_rotary's internal ISR event queue  */

#define ROTARY_ISR_EVENT_TASK_PRIORITY 10
	/* Priority of esp_rotary's internal event management task. */

#define ROTARY_ISR_EVENT_TASK_HEAP 4096
	/* Size of the internal ISR event task heap  */

#define ROTARY_BUTTON_TASK_PRIORITY 10
	/* Priority of esp_rotary's internal button task heap. */

#define ROTARY_BUTTON_TASK_HEAP 4096
	/* Size of the button task heap  */

#define ROTARY_BUTTON_DEBOUNCE_SAMPLE_DELAY_TICKS pdMS_TO_TICKS(10)
	/* Ticks between sampling button state for debounce. This is *not*
         * the time for a button state change to be counted. It is the time
         * between two samplings of the button states. Multiple consecutive
	 * readings of the same value is required for a button to change
	 * state. See the button loop for more info. */


#endif
