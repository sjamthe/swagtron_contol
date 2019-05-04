# swagtron_contol

This code is for Teensy 3.2 that can control hacked swagatron hoverboard.

It hacks the protocol (UART SERIAL_9N1) that is sent by the foot sensors to control the motors

It is able to move left, right, forward and backward based on following commands sent on serial of teensy
* 'on' power on
* 'off' power off
* 'l N' turn left N steps (16*N is roughly a turn)
* 'r N' turn right N steps
* 'f N' move forward N steps
* 'b N' move backward N steps
