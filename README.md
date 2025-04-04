# esp_rotary

Accurate & feature-rich rotary encoder driver for ESP32. Will not
miss or count extra steps even when turned very fast.

Features:
* Value counting is done with a state machine to ensure reliable increment /
decrement counting (see "Implementation Notes" below)
* Multiple rotary count modes: bound, wraparound
* Optional "speed boost" mode: if the knob is turned fast, a multiplier is
applied to the increments / decrements
* Callers receive turn and switch events via a queue *and* also can read
count value and switch status at any time
* Switch has software debounce

# Using the driver

## Setup
Applications should `#include "esp_rotary.h"`.

Encoders are set up and configured via `rotary_config()` which takes an array
of `rotary_config_t` structs. All encoders have to be configured at once,
repeated calls to `rotary_config()` are not allowed.

`rotary_config()` will configure the gpio pins, interrupts, internal event
queues and state machines (see "Implementation Notes" below).

Example configuration code:

```C
/* All rotary encoders have to be configured at once. Here we set up
 * two encoders: */

rotary_config_t rconf[2];
memset(rconf, 0, sizeof(rotary_config_t) * 2);

rconf[0].rc_pin_a = 4;
rconf[0].rc_pin_b = 5;
rconf[0].rc_pin_switch = 6;
rconf[0].rc_style = ROT_STYLE_BOUND; /* Will not go over min and max */
rconf[0].rc_min = 0;
rconf[0].rc_max = 100000;
rconf[0].rc_start = 0;
rconf[0].rc_enable_speed_boost = 1; /* Enable speed boost mode */

rconf[1].rc_pin_a = 7;
rconf[1].rc_pin_b = 8;
rconf[1].rc_pin_switch = 9;
rconf[1].rc_style = ROT_STYLE_WRAPAROUND; /* Value will wrap around */
rconf[1].rc_max = 30;
rconf[1].rc_min = -10;
rconf[1].rc_start = 10;

if(rotary_config(rconf, 2) != ESP_OK) {
	printf("Could not configure rotary encoders\n");
	goto error_label;
}
```

The encoders are now up and running and ready to be used. There are two ways
applications can use encoders: using the event queue or reading values
directly.

## Rotary encoder event queue

Upon successful initialization, a queue named `rotary_event_queue` becomes
available. The application can wait on this queue to receive events of type
`rotary_event_t`. Each event contains the index of the encoder that is
sending the event, the type of the event (value increment/decrement, button
press or release, as well as the encoder's value.

Example usage of the rotary encoder event queue:

```C
rotary_event_t	rev;


while(1) {
	if(xQueueReceive(rotary_event_queue, &rev, portMAX_DELAY) != pdPASS)
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
		printf("Rotary %d button pressed\n", rev.re_idx);
		break;
	case ROT_EVENT_BUTTON_RELEASE:
		printf("Rotary %d button released\n", rev.re_idx);
		break;
	default:
		/* Not reached */
		break;
	}
}
```

## Reading values directly

Values can also be read using the function `rotary_get_value()`. Similarly,
button state can be read with `rotary_get_button_state()`.

These calls are nonblocking and can be used without any further synchronization
(all that is handled internally).

Example of reading rotary value and button state directly:

```
TODO
```


## Compile-time configuration options

`rotary_params.h` contains constants that can be modified to the application's
needs. Most of these are fine to leave alone. The only one to pay attention to
is `ROTARY_CALL_GPIO_INSTALL_ISR_SERVICE`. gpio_install_isr_service() must
only be called once, so if you application already calls it somewhere else,
make sure to set this to 0.

The following constants are defined:

```
#define ROTARY_CALL_GPIO_INSTALL_ISR_SERVICE 1
        /* If this is nonzero, rotary_config() will call
         * gpio_install_isr_service(). Set to 0 or the application will call
         * it (make sure it is called before rotary_config() */

#define ROTARY_GPIO_INSTALL_ISR_SERVICE_FLAGS 0
        /* Flags to call gpio_install_isr_service() with. */

#define ROTARY_ISR_EVENT_TASK_PRI 10
        /* Priority of esp_rotary's internal event management task. */

#define ROTARY_SWITCH_DEBOUNCE_TICKS pdMS_TO_TICKS(50)
        /* Ticks delay for the switch debounce logic */

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
```


# Implementation notes

## Rotary encoder state machine

To determine rotary motion, a naive implementation would simply look at B when
A goes active (ie. goes low): if B is active (=low) then it's a step clockwise;
if B is not active (=high) then it's a step counter-clockwise. The issue with
this is signal bounce as contact is made within the encoder. This will result
in the counting of unpredictable extra steps in either direction.

A delay-based debounce might work, but already at moderately high turn speeds
it would start to miss steps.

A better idea is to take advantage of the fact that we *do* have two inputs
that go active/inactive, offset from one another, and in a pattern that's
unique for the clockwise / counterclockwise directions. This means that we can
implement a state machine to go through the pin value stages of CW or CCW
rotation in order.

This will eliminate bounce, since bounce will manifest as the state machine
harmlessly advancing / reverting steps. However, once we complete an *entire*
go around in state changes, we can be sure that this was no coincidence and
count it as a rotary increment or decrement.

The following graphic illustrates the states. The "Pins" line shows the state
of pins A and B. For example, "HH" means both A and B are high (inactive) and
"HL" means A is high and B is low.

```
State      CCW3  CCW2  CCW1  IDLE   CW1   CW2   CW3
Pins     /- LH <- LL <- HL <- HH -> LH -> LL -> HL -\
         |                    ||                    |
         v                    ^^                    v
         |      (decrement)   ||    (increment)     |
         \------->------->----/\-----<-------<------/
```

We start in IDLE, when both pins are inactive. When the rotary encoder is
turned, the state machine will start advancing in the direction indicated by
the pin values. Due to bounce, during each state change it may oscillate back
and forth between the states a number of times.

When a full go-around has been completed, the step increment or decrement is
noted. What's most important to understand is that the steps CW3->IDLE and
CCW3->IDLE *only* result in an increment / decrement respectively, if a *whole*
loop has been completed. Temporary bounce back and forth between IDLE and
CW1/CCW1 will not result in the counter changing.

If at any point in the state machine we encounter an input that's not allowed
for the state we are in, we ignore that input except for HH, in which case we
reset the state machine to IDLE. This is a failsafe to get back to a known
state if somehow the steps or signals got messed up.

