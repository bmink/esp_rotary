# esp_rotary

Rock solid, accurate & efficient rotary encoder driver for ESP32. Will not
miss or count extra steps even when turned very fast.

Features:
* Value counting is done with a state machine to ensure reliable increment /
decrement counting
* Multiple rotary count modes: "infinite", bound, wraparound
* Optional "speed boost" mode: if the knob is turned fast, a multiplier is
applied to the increments / decrements
* Callers receive turn and switch events via a queue *and* also can read
count value and switch status at any time
* Switch has software debounce

## Using the driver

### Setup
The public interface is defined in `esp_rotary.h`.

Encoders are set up and configured via the `rotary_config()` call which takes
an array of `rotary_config_t` structs. All encoders have to be configured at
once, repeated calls to `rotary_config()` are not allowed.

`rotary_config()` will configure the gpio pins, interrupts, internal event
queues and state machines (see "Implementation Notes" below).

The application needs to ensure that `gpio_install_isr_service()` has
been called before attempting to call `rotary_config()`.

Example configuration:

```
rotary_config_t rconf[2];
memset(rconf, 0, sizeof(rotary_config_t) * 2);

/* All rotary encoders have to be configured at once. Here we set up
 * two encoders: */

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

/* gpio_install_isr_service() must be called before rotary_config().
 * Since it should only be called once it is left to the application to
 * decide when it's best to do it. */

if(gpio_install_isr_service(0) != ESP_OK) {
	printf("Could not install ISR service\n");
	goto error_label;
}

if(rotary_config(rconf, 2) != ESP_OK) {
	printf("Could not configure rotary encoders\n");
	goto error_label;
}
```

The encoders are now up and running and ready to be used. There are two ways
applications can use encoders: using the event queue or reading values directly

### Rotary encoder event queue

Upon successful initialization, a queue called `rot_event_queue` becomes
available. The application can wait on this queue to receive events of type
`rot_event_t`. Each event comes with the index of the encoder that is sending
the event, the tupe of event (value increment/decrement, switch press or
release, as well as the encoder's value.

Example usage of the rotary encoder event queue:

```
TODO
```


### Reading values directly

Values can also be read using the function `rotary_get_value()`. Similarly,
button state can be read with `rotary_get_switch_state()`.

These calls are nonblocking and can be used without needing any synchronization
mechanism (that is all handled internally).

Example of reading rotary value and switch state directly:

```
TODO
```


## Implementation notes

### Rotary encoder state machine


