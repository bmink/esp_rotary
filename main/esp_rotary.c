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


/* A naive implementation would simply look at B when A goes active (ie. is
 * falling): if B is active (=low) then it's a step clockwise; if B is not
 * active (=high) then it's a step counter-clockwise. The issue with this is
 * signal bounce as contact is made within the encoder. This will result in
 * the counting of unpredictable extra steps in either direction. 
 *
 * We could do a time-based debounce, which could work but at higher spin
 * speeds would start to miss steps.
 *
 * A better idea (not mine) is to take advantage of the fact that we *do* have
 * two inputs that go active/inactive, offset from one another, and in a
 * pattern that's unique for the CW / CCW directions. This means that we can
 * implement a state machine to go through the pin value stages of CW or CCW
 * rotation in order.
 * 
 * This will essentially eliminate bounce, since bounce will manifest as
 * the state machine harmlessly advancing / reverting steps. However, once
 * we complete an *entire* go around in state changes, we can be sure that
 * this was no coincidence and count it as a rotary increment or decrement.
 * 
 * The following graphic illustrates the states. The "Pins" line shows the
 * state of pins A and B. For example, "HH" means both A and B are high
 * (inactive) and "HL" means A is high and B is low.
 *
 * State      CCW3  CCW2  CCW1  IDLE   CW1   CW2   CW3  
 * Pins     /- LH <- LL <- HL <- HH -> LH -> LL -> HL -\
 *          |                    ||                    |
 *          v                    ^^                    v
 *	    |      (decrement)   ||    (increment)     |
 *          \------->------->----/\-----<-------<------/
 *
 * We start in IDLE, when both pins are inactive. When the rotary encoder is
 * turned, the state machine will start advancing in the direction indicated by 
 * the pin values. Due to bounce, during each state change it may oscillate
 * back and forth between the states a few times.
 * 
 * When a full go-around has been completed, the step increment
 * or decrement is noted. What's most important to understand is that the steps
 * CW3->IDLE and CCW3->IDLE *only* result in an increment / decrement 
 * respectively, if a *whole* loop has been completed. Temporary bounce back
 * and forth between IDLE and CW3/CCW3 will not result in the counter changing.
 * 
 * If at any point in the state machine we encounter an input that's not
 * allowed for the state we are in, we ignore that input except for HH, in
 * which case we reset the state machine to IDLE. This is a failsafe to get
 * back to a known state if somehow the steps or signals got messed up.
 */

enum rotary_state {
	STATE_IDLE,
	CW_1,
	CW_2,
	CW_3,
	CCW_1,
	CCW_2,
	CCW_3
}


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
	 * the GPIO input register directly
	 * Alternatively could set CONFIG_GPIO_CTRL_FUNC_IN_IRAM in the config,
	 * then presumably we could use gpio_get_level() (? need to verify) */
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

	memset(&config, 0, sizeof(gpio_config_t));

	config.mode = isinput ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT;
	config.intr_type = isr ? GPIO_INTR_NEGEDGE : GPIO_INTR_DISABLE;
    	config.pin_bit_mask = 1ULL << pinnr;
	config.pull_up_en = 1;
	ret = gpio_config(&config);
	if(ret != ESP_OK) {
		printf("Could not config gpio\n");
		return ret;
		
	}

	if(isr) {
		ret = gpio_isr_handler_add(pinnr, isr, isr_arg);
		if(ret != ESP_OK) {
			printf("Could not install interrupt handler\n");
			return ret;
		}
	}

	return 0;
}

#define EVENT_LOOP_PRIORITY	10

static uint32_t	eventcnt;
static int	val;

static void event_loop(void *arg)
{
	uint32_t	event;

printf("Entering event loop\n");
	while(1) {
		if(xQueueReceive(_event_queue, &event, portMAX_DELAY)) {
			//printf("Event: %"PRIu32"\n", event);
			//printf("eventcnt = %"PRIu32"\n", eventcnt++);
			if(event)
				--val;
			else
				++val;
			printf("val = %d\n", val);
			
		}
		//printf("high watermark: %d\n",
		//    uxTaskGetStackHighWaterMark(NULL));
	}
}


void
app_main(void)
{
	esp_err_t	ret;

	ret = ESP_OK;

printf("here\n");

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

printf("here2\n");

	_event_queue = xQueueCreate(EVENT_QUEUE_SIZ, sizeof(uint32_t));
	if(_event_queue == NULL) {
		printf("Could not create _event_queue");
		goto err_label;
	}

printf("here3\n");

	ret = xTaskCreate(event_loop, "event_loop", 4096, NULL,
	    EVENT_LOOP_PRIORITY, NULL);
	if(ret != pdPASS) {
		printf("Could not create event_loop task");
		goto err_label;
	} else
		ret = ESP_OK;

printf("here4\n");

err_label:

	printf("Error: %s\n", esp_err_to_name(ret));

	while (1) {
       		gpio_set_level(GPIO_BLINK, 0);
		vTaskDelay(pdMS_TO_TICKS(100));
		gpio_set_level(GPIO_BLINK, 1);
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
