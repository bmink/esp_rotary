#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "esp_check.h"
#include "esp_rotary.h"


typedef struct rotary {
	rotary_config_t	ro_config;
	unsigned char	ro_state;
	
	QueueHandle_t 	ro_value_mailb;
	QueueHandle_t 	ro_switch_state_mailb;
	
} rotary_t;

static rotary_t	*rotary = NULL;
static unsigned char rotary_cnt = 0;	

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
 * A better idea is to take advantage of the fact that we *do* have two
 * inputs that go active/inactive, offset from one another, and in a
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
 * back and forth between the states a number of times.
 * 
 * When a full go-around has been completed, the step increment
 * or decrement is noted. What's most important to understand is that the steps
 * CW3->IDLE and CCW3->IDLE *only* result in an increment / decrement 
 * respectively, if a *whole* loop has been completed. Temporary bounce back
 * and forth between IDLE and CW1/CCW1 will not result in the counter changing.
 * 
 * If at any point in the state machine we encounter an input that's not
 * allowed for the state we are in, we ignore that input except for HH, in
 * which case we reset the state machine to IDLE. This is a failsafe to get
 * back to a known state if somehow the steps or signals got messed up.
 */

#define STATE_IDLE	0
#define STATE_CW1	1
#define STATE_CW2	2
#define STATE_CW3	3
#define STATE_CCW1	4
#define STATE_CCW2	5
#define STATE_CCW3	6
#define STATE_CNT	7

static unsigned char rot_state = STATE_IDLE;


/* rot_state_change contains the new state for each possible input for each
 * possible state. Each row represents the corresponding state value.
 * The columns in each state represent the new state based on the value of
 * A and B in the format: A << 1 + B. So the first column is the new state if
 * the input is LOW/LOW (ie. both A and B active) and the fourth column
 * represents HIGH/HIGH (both inactive). The format of the new state byte
 * is as follows:
 *  7 6 5 4 3 2 1 0
 *  | | | | | | | +- bits 0-4 represent the index of the new state
 *  | | | | | | +--- see above
 *  | | | | | +----- see above
 *  | | | | +------- see above
 *  | | | +--------- not used
 *  | | +----------- not used
 *  | +------------- decrement counter
 *  +--------------- increment counter
 */

#define COUNT_INCR	1 << 7
#define COUNT_DECR	1 << 6

static char rot_state_change[STATE_CNT][4] = {

       /* STATE_IDLE 
        * LL            LH              HL              HH                  */
	{ STATE_IDLE,	STATE_CW1, 	STATE_CCW1,	STATE_IDLE },

       /* STATE_CW1 
        * LL            LH              HL              HH                  */
	{ STATE_CW2,	STATE_CW1, 	STATE_CW1,	STATE_IDLE },

       /* STATE_CW2 
        * LL            LH              HL              HH                  */
	{ STATE_CW2,	STATE_CW1, 	STATE_CW3,	STATE_IDLE },

       /* STATE_CW3 
        * LL            LH              HL              HH                  */
	{ STATE_CW2,	STATE_CW3, 	STATE_CW3,	STATE_IDLE|COUNT_INCR },

       /* STATE_CCW1 
        * LL            LH              HL              HH                  */
	{ STATE_CCW2,	STATE_CCW1, 	STATE_CCW1,	STATE_IDLE },

       /* STATE_CCW2 
        * LL            LH              HL              HH                  */
	{ STATE_CCW2,	STATE_CCW3, 	STATE_CCW1,	STATE_IDLE },

       /* STATE_CCW3 
        * LL            LH              HL              HH                  */
	{ STATE_CCW2,	STATE_CCW3, 	STATE_CCW3,	STATE_IDLE|COUNT_DECR }
};


/* Internal event queue. Used by the ISRs to record events as they see them
 *
 * event is a single byte of the following format:
 *  7 6 5 4 3 2 1 0
 *  | | | | | | | +- status of rotary B
 *  | | | | | | +--- status of rotary A
 *  | | | | | +----- status of switch (0=open, 1=closed)
 *  | | | | +------- not used
 *  | | | +--------- not used
 *  | | +----------- not used
 *  | +------------- not used
 *  +--------------- event type (1 - rotary, 0 - switch)
 */
#define EVENT_QUEUE_SIZ	10
static QueueHandle_t _event_queue = NULL;

static int rot_value;


void
isr_rotary(void *arg)
{ 
	uint32_t a_value;
	uint32_t b_value;
	uint32_t regs;
	char event;

	/* Since we are in an ISR, we are checking the pin value by accessing
	 * the GPIO input register directly */
	regs = (REG_READ(GPIO_IN_REG));
	a_value = (regs >> GPIO_ROT_A) & 1;
	b_value = (regs >> GPIO_ROT_B) & 1;

	event = (a_value << 1) | b_value | (1 << 7);

	xQueueSendFromISR(_event_queue, &event, NULL);
}


void
isr_rotary_switch(void *arg)
{
#if 0
	int	rotary_idx;

	rotary_idx = (uint32_t) arg;
#endif
}


static void event_loop(void *arg)
{
	char	event;
	char	statech;

printf("Rotary size: %d\n", sizeof(esp_rotary_t));
printf("Entering event loop\n");
	while(1) {
		if(xQueueReceive(_event_queue, &event, portMAX_DELAY)) {
			if(event & (1<<7)) {
				/* Rotary event */

				
				statech =
				    rot_state_change[rot_state][event & 0xf];

				rot_state = statech & 0xf;
				if(statech & COUNT_INCR)
					++rot_value;
				else
				if(statech & COUNT_DECR)
					--rot_value;

				if(statech > 0xf)
					printf("rot_value = %d\n", rot_value);
			}

			
		}
		//printf("high watermark: %d\n",
		//    uxTaskGetStackHighWaterMark(NULL));
	}
}


  

int
config_gpio(int pinnr, int isinput, gpio_isr_t isr, void *isr_arg)
{
	gpio_config_t	config;
	int		ret;

	memset(&config, 0, sizeof(gpio_config_t));

	config.mode = isinput ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT;
	config.intr_type = isr ? GPIO_INTR_ANYEDGE : GPIO_INTR_DISABLE;
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

void
app_main(void)
{
	esp_err_t	ret;

	ret = ESP_OK;

printf("here\n");

	ESP_GOTO_ON_ERROR(gpio_install_isr_service(0), err_label, logtag,
	    "Could not install gpio ISR Service");

	/* Rotary A: input, interrupt */
	ESP_GOTO_ON_ERROR(config_gpio(GPIO_ROT_A, 1, isr_rotary, NULL),
	    err_label, logtag, "Could not configure pin %d", GPIO_ROT_A);

	/* Rotary B: input, no interrupt */
	ESP_GOTO_ON_ERROR(config_gpio(GPIO_ROT_B, 1, isr_rotary, NULL),
	    err_label, logtag, "Could not configure pin %d", GPIO_ROT_B);

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
