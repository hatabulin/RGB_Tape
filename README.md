# RGB_Tape
stm32 rgb controller

MCU: stm32f103c8t6

HAL-Driver,
Build tools: CubeMX 4.27 & Atollic TrueStudio 9.3

Controlled via USB port with scripts:

Command 1:
======= 
send: EFFECT_CONFIG:01.1.1.0.FF.80.77.00.00
              
-activate effect 01
- use channel 1: 1 (true)
- use channel 2: 1 (true)
- use channel 3: 0 (false)
- data PWM - red = 	0xFF
- data PWM - green = 	0x80
- data PWM - blue = 	0x77
- data delay/freq blink... - 0x00
- data delay between changes (for effect 2)


Effect 1 - change color.
Usage:
EFFECT_CONFIG:01.<use red chnl>.<use green chnl>.<use blue chnl>.<red value>.<green value>.<blue value>.<pwm delay>.<pause delay>

EFFECT-2 - change color and restore (flash)
Usage:
EFFECT_CONFIG:02.<use red chnl>.<use green chnl>.<use blue chnl>.<red value>.<green value>.<blue value>.<pwm delay>.<pause delay>

Command 2:
======= 
send: x.y.zzz

x: 0 - off, 1 -on, 2 - blink
y: 0..2 - virtual channel
zzz: blink delay 
 
Command 3: (used for config virtual channel <cmd2>) channel pallette
======= 
 send: CH0:XXYYZZ
  
  set channel 0 with Red:XX, Green:YY, Blue:ZZ 
  
