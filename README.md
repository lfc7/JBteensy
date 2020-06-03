# JBteensy

```  
     ___  _______    _______  _______  _______  __    _  _______  __   __   
    |   ||  _    |  |       ||       ||       ||  |  | ||       ||  | |  |  
    |   || |_|   |  |_     _||    ___||    ___||   |_| ||  _____||  |_|  |  
    |   ||       |    |   |  |   |___ |   |___ |       || |_____ |       |  
 ___|   ||  _   |     |   |  |    ___||    ___||  _    ||_____  ||_     _|  
|       || |_|   |    |   |  |   |___ |   |___ | | |   | _____| |  |   |    
|_______||_______|    |___|  |_______||_______||_|  |__||_______|  |___|    
```

Read midi from usb or from MIDI-In socket or from a MIDIfile on SD card to action physical devices such as selenoides, micro-motors and stepper motors

Designed for teensy 3.6

Stepper-motors controls boards A4988 are controlled from a mcp23017 ( extender GPIO )  
Selenoides and micro motors are controlled from a IRF540 board connected to a pca9685 board ( PWM controller )  

