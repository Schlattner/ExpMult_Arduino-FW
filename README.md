# ExpMult_Arduino-FW
Arduino Micro Firmware Exp. Multiplexer

/*
 * Expression Multiplexer Firmware
 * Lucas H. 07.04.2020
 * v0.1
 * 
 * Exp Multiplexer has 4 Output Channels and 3 Operating Modes.
 * Each Output Channel can have different Potentiometer Taper settings applied - either
 * lin, inv. lin, log or inv. log can be set for each channel separately.
 * In the "normal" operating mode, or "channel mode", the select button steps through the 4 channels
 * and the on/off button activates/deactivates the selected channel. Each channel can have the pot taper assigned
 * via the selector knob.
 * 
 * In "Patch Mode", activated/deactivated by pressing both buttons at once, the Select Button steps
 * through 4 patches (indicated by the "Mode" Leds), where each patch has the on/off and Mode info for each
 * channel saved. The patch is activated with the on/off button. Logically, only one patch can be active at
 * a time. It is still possible to deactivate all patches.
 * 
 * The third mode, "Save Mode" can only be entered from normal (channel) mode, not Patch mode. It is used
 * to set up the patches for patch mode. This is done by first setting up the channels on/off and modes as
 * desired and then holding the Select buton for ~2s. Then the desired saving spot for the patch can be selected
 * via the selector knob and by pressing the select button again for ~2s the patch is saved to that spot, overwriting
 * any previosly saved patch.
 * 
 */
 
Normal Mode:
Button0 (On/Off) switches selected channel on/off
Button1 (Select) cycles through channels
Leds0-3 are signaling channel unselected on, unselected off, selected on and selected off
Unselected on: Led On
Unselected off: Led Off
Selected on: fast blink
Selected off: slow blink
Leds4-7 are signaling the mode of the selected channel
Led of the NVM-saved mode is on, others are off
If selector wheel is changed, led of selected mode is on, others off and NVM is updated

Patch Mode:
Button0 (On/Off) switches selected patch on/off
Button1 (Select) cycles through patches
Leds0-3 are signaling channel on/off
Leds4-7 are signaling patch 
Unselected on: Led On
Unselected off: Led Off
Selected on: fast blink
Selected off: slow blink
Selector wheel can also be used to select patch, but is recognized only on change - default patch mode starts at patch 0.

Switching between Normal and Patch Mode: Pushing both Buttons simultaneously.

Save Mode:
Used for saving patches
Activated by holding Button1 (Select) for >2s (only from normal mode!)
Patch selected via Selector Knob -> selected Patch Led fast blink
Channel Leds represent channel state (on/off)
Patch then is saved on selected spot
