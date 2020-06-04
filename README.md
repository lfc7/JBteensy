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

Stepper-motors controls boards A4988 are controlled from a mcp23017 ( I2C extender GPIO )  
Selenoides and micro motors are controlled with IRF540 boards connected to a pca9685 board ( I2C PWM controller )  

MD_MIDIFile lib has been sligthy modified to permit teensy builtin-SD reading

