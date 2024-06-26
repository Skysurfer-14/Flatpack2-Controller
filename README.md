## Flatpack2 Controller ##
Connection board with controller, display and rotary encoder for controlling an Eltek Flatpack2 HE 48V/3000W power supply.
The Arduino program of the controller is maintained in this repository. Extensions and improvements are welcome!
![flatpack2-controller](https://github.com/Skysurfer-14/Flatpack2-Controller/assets/26379759/a0ce48e2-12ed-4c86-9bd7-830b1249368d)
### Quick guide ###
- voltage and current are displayed after switching on (readings)
- to protect the OLED display, it is switched off after 60s if the rotary encoder is not used. It can be switched on again by pressing the Rotary Encoder.
- briefly pressing the Rotary Encoder switches the OLED display to the settings for the voltage. Turning the rotary encoder increases or decreases the voltage. A short press switches to the settings for the current. If the rotary encoder is pressed and held, the set value is also written to the non-volatile memory.
- in the settings for the current, the maximum current can be increased or decreased by turning the rotary encoder. A short press switches to the readings. If the rotary encoder is pressed and held, the set value is also written to the non-volatile memory.
