# esp_rotary

Rock solid, accurate & efficient rotary encoder driver for ESP32. Will not
miss or count extra steps even when turned very fast.

Features:
* Counting is done with a state machine to ensure reliable increment /
decrement counting
* Multiple rotary count modes: "infinite", bound, wraparound
* Optional "speed boost" mode: if the knob is turned fast, a multiplier is
applied to the increments / decrements
* Callers receive turn and switch events via a queue *and* also can read
count value and switch status at any time
* Switch implements software debounce

## Using the driver

### Setup

### Encoder events and reading values


## Implementation notes

### Rotary encoder state machine


