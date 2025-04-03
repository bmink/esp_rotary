#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "esp_check.h"
#include "esp_rotary.h"
#include "esp_rotary_params.h"


typedef struct rotary {
	rotary_config_t	ro_conf;

	unsigned char	ro_state;
	
	QueueHandle_t 	ro_value_mailb;
	QueueHandle_t 	ro_switch_state_mailb;
	
} rotary_t;

static rotary_t	*rotary = NULL;
static unsigned char rotary_cnt = 0;	



#define GPIO_BLINK	2



esp_err_t
rot_set_val(rotary_t *rot, const int32_t val)
{
	int	ret;
	if(rot == NULL)
		return ESP_ERR_INVALID_ARG;

	ret = xQueueOverwrite(rot->ro_value_mailb, &val);
	if(ret != pdPASS)
		return ESP_FAIL;
	else
		return ESP_OK;
}


esp_err_t
rot_get_val(rotary_t *rot, int32_t *val)
{
	if(rot == NULL)
		return ESP_ERR_INVALID_ARG;

	return xQueuePeek(rot->ro_value_mailb, val, 0);
}


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
 * turned, the state machine will 100start advancing in the direction indicated by 
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


/* Internal event queue. Used by the ISRs to record events as they see them.
 * These events are then processed by the event loop.
 *
 * event is a word of the following format:
 * 
 * [Byte1][Byte2]
 * 
 * Byte 1 represents the index of the rotary generating the event
 * 
 * Byte 2:
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
static QueueHandle_t isr_event_queue = NULL;

void
isr_rotary(void *arg)
{ 
	uint32_t 	a_value;
	uint32_t 	b_value;
	uint32_t	regs;
	unsigned char	idx;
	uint16_t	event;

	/* Since we are in an ISR, we are checking the pin value by accessing
	 * the GPIO input register directly */
	idx = (uint32_t) arg;

	regs = (REG_READ(GPIO_IN_REG));
	a_value = (regs >> rotary[idx].ro_conf.rc_pin_a) & 1;
	b_value = (regs >> rotary[idx].ro_conf.rc_pin_b) & 1;

	event = ((uint16_t)idx) << 8 | (a_value << 1) | b_value | (1 << 7);

	xQueueSendFromISR(isr_event_queue, &event, NULL);
}


void
isr_switch(void *arg)
{
#if 0
	int	rotary_idx;

	rotary_idx = (uint32_t) arg;
#endif
}



static void
event_loop(void *arg)
{
	uint16_t	event;
	char		statech;
	unsigned char	idx;
	char		evbyte;
	rotary_t	*rot;
	int32_t		val;
	int		ret;
	int		changed;
	int		i;

printf("Rotary size: %d\n", sizeof(rotary_t));
printf("Entering event loop\n");

	while(1) {
		/* Wait for events from ISRr */
		if(xQueueReceive(isr_event_queue, &event, portMAX_DELAY) !=
		   pdPASS)
			continue;


		idx = event >> 8;
		evbyte = event & 0xff;


		if(idx >= rotary_cnt)
			continue;

		rot = &rotary[idx];
	
		if(evbyte & (1<<7)) {
			/* Rotary event */
			changed = 0;
			statech = rot_state_change[rot->ro_state][evbyte & 0xf];

			rot->ro_state = statech & 0xf;

			if(statech & (COUNT_INCR | COUNT_DECR)) {
				ret  = rot_get_val(rot, &val);
				if(ret != pdPASS)
					continue;

				if(statech & COUNT_INCR) {
					switch(rot->ro_conf.rc_style) {
					case ROT_STYLE_BOUND:
						if(val < rot->ro_conf.rc_max) {
							++val;
							++changed;
						}
						break;

					case ROT_STYLE_WRAPAROUND:
					default:
						if(val == rot->ro_conf.rc_max)
						    val = rot->ro_conf.rc_min;
						else
						    ++val;
						++changed;
						break;
					}
				}
				else { /* COUNT_DECR */	
					switch(rot->ro_conf.rc_style) {
					case ROT_STYLE_BOUND:
						if(val > rot->ro_conf.rc_min) {
							--val;
							++changed;
						}
						break;

					case ROT_STYLE_WRAPAROUND:
					default:
						if(val == rot->ro_conf.rc_min)
						    val = rot->ro_conf.rc_max;
						else
						    --val;
						++changed;
						break;
					}
				}
				if(changed)
					rot_set_val(rot, val);
			}

			if(changed) {
				for(i = 0; i < rotary_cnt; ++i) {
					rot_get_val(&rotary[i], &val);
					printf("%2d = %4"PRId32" ", i, val);
				}
				printf("\n");	
			}
			
		} else
		if(evbyte & (1<<6)) {
			/* Switch event */
		}

		//printf("high watermark: %d\n",
		//    uxTaskGetStackHighWaterMark(NULL));
	}
}
	  

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


#define ROT_EVENT_LOOP_PRIORITY	10

esp_err_t
rotary_config(rotary_config_t *rconf, unsigned char cnt)
{
	int		err;
	int		ret;
	uint32_t	i;
	rotary_t	*rot;
	rotary_config_t	*conf;
	gpio_config_t	config;

	if(rotary != 0)
		return ESP_ERR_NOT_ALLOWED; /* already configured */

	if(rconf == NULL || cnt == 0)
		return ESP_ERR_INVALID_ARG;

	err = ESP_OK;


	if(ROTARY_CALL_GPIO_INSTALL_ISR_SERVICE) {
		ret = gpio_install_isr_service(
		    ROTARY_GPIO_INSTALL_ISR_SERVICE_FLAGS);
		if(ret != ESP_OK) {
			err = ret;
			goto end_label;
		}
	}

	rotary = malloc(sizeof(rotary_t) * cnt);
	if(rotary == NULL) {
		err = ESP_ERR_NO_MEM;
		goto end_label;
	}

	memset(rotary, 0, sizeof(rotary_t) * cnt);

	memset(&config, 0, sizeof(gpio_config_t));
	config.mode = GPIO_MODE_INPUT;
	config.intr_type = GPIO_INTR_ANYEDGE;
	config.pull_up_en = 1;

	for(i = 0; i < cnt; ++i) {
		rot = &rotary[i];
		conf = &rconf[i];

		config.pin_bit_mask |= 1ULL << conf->rc_pin_a;
		config.pin_bit_mask |= 1ULL << conf->rc_pin_b;
		config.pin_bit_mask |= 1ULL << conf->rc_pin_switch;

		ret = gpio_isr_handler_add(conf->rc_pin_a, isr_rotary,
		    (void *) i);
		if(ret != ESP_OK) {
			err = ret;
			goto end_label;
		}

		ret = gpio_isr_handler_add(conf->rc_pin_b, isr_rotary,
		    (void *) i);
		if(ret != ESP_OK) {
			err = ret;
			goto end_label;
		}

#if 0
		ret = gpio_isr_handler_add(conf->rc_pin_switch, isr_switch,
		    (void *) i);
		if(ret != ESP_OK) {
			err = ret;
			goto end_label;
		}
#endif

		rot->ro_value_mailb = xQueueCreate(1, sizeof(int32_t));
		ret = rot_set_val(rot, conf->rc_start);
		if(ret != ESP_OK) {
			err = ret;
			goto end_label;
		}

		rot->ro_conf = *conf;
		rot->ro_state = STATE_IDLE;
	}

	ret = gpio_config(&config);
	if(ret != ESP_OK) {
		err = ret;
		goto end_label;
	}

	isr_event_queue = xQueueCreate(EVENT_QUEUE_SIZ, sizeof(uint16_t));
	if(isr_event_queue == NULL) {
		err = ESP_FAIL;
		goto end_label;
	}

	rotary_cnt = cnt;

	ret = xTaskCreate(event_loop, "event_loop", 4096, NULL,
	    ROT_EVENT_LOOP_PRIORITY, NULL);
	if(ret != pdPASS) {
		goto end_label;
	} else
		ret = ESP_OK;

end_label:

	if(err != ESP_OK && rotary != NULL) {
		free(rotary);
		rotary_cnt = 0;
	}

	return err;
}






static const char *logtag = "rotary test";


void
app_main(void)
{
	esp_err_t	ret;




	/* All rotary encoders have to be configured at once. Here we set up
         * two encoders */

	rotary_config_t	rconf[2];
	memset(rconf, 0, sizeof(rotary_config_t) * 2);
	
	rconf[0].rc_pin_a = 7;
	rconf[0].rc_pin_b = 8;
	rconf[0].rc_pin_switch = 9;
	rconf[0].rc_style = ROT_STYLE_BOUND; /* Will not go over min and max */
	rconf[0].rc_max = 0;
	rconf[0].rc_min = 100;
	rconf[0].rc_start = 0;

	rconf[1].rc_pin_a = 4;
	rconf[1].rc_pin_b = 5;
	rconf[1].rc_pin_switch = 6;
	rconf[1].rc_style = ROT_STYLE_WRAPAROUND; /* Value will wrap around */
	rconf[1].rc_max = 30;
	rconf[1].rc_min = -10;
	rconf[1].rc_start = 10;
	
	ret = rotary_config(rconf, 2);
	if(ret != ESP_OK) {
		printf("Could not configure rotary encoders\n");
		goto err_label;
	}	

	/* LED to blink: output, no interrupt */
	ESP_GOTO_ON_ERROR(config_led_gpio(GPIO_BLINK), err_label,
	    logtag, "Could not configure LED on pin %d", GPIO_BLINK);


err_label:

	printf("Error: %s\n", esp_err_to_name(ret));

	while (1) {
       		gpio_set_level(GPIO_BLINK, 0);
		vTaskDelay(pdMS_TO_TICKS(100));
		gpio_set_level(GPIO_BLINK, 1);
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
