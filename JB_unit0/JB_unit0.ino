// Test playing a succession of MIDI files from the SD card.
// Example program to demonstrate the use of the MIDFile library
// Just for fun light up a LED in time to the music.
//
// Hardware required:
//  SD card interface - change SD_SELECT for SPI comms
//  3 LEDs (optional) - to display current status and beat.
//  Change pin definitions for specific hardware setup - defined below.


// pwm output 2, 3, 4, 5, 6, 7, 8, 9, 10, 14, 16, 17, 20, 21, 22, 23, 29, 30, 35, 36, 37, 38

//

#include <SPI.h>
#include <SD.h>
#include <MIDI.h>
#include <Bounce2.h>

#include <MD_MIDIFile.h> //!!! mod version for teensy 3.6 !!!

#include <Wire.h>

#include <Adafruit_MCP23017.h> //need for GPIO mcp23017
#include "AccelStepper.h"      //needed for steppers
#include "MCP3017AccelStepper.h" //needed for steppers on mcp23017

#include <PCA9685.h> // I2C PWM

#include <SerialCommand.h> // parse serial command

//instanciate objects

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1,  MIDI);

SerialCommand sCmd;

MD_MIDIFile SMF;

#define STEPPER_COUNT 8
Adafruit_MCP23017 mcpSTEPPERS;
MCP3017AccelStepper steppers[STEPPER_COUNT] = {
    // interface, step, dir, en
    MCP3017AccelStepper(AccelStepper::DRIVER, 0, 0xff),
    MCP3017AccelStepper(AccelStepper::DRIVER, 1, 0xff),
    MCP3017AccelStepper(AccelStepper::DRIVER, 2, 0xff),
    MCP3017AccelStepper(AccelStepper::DRIVER, 3, 0xff),
    MCP3017AccelStepper(AccelStepper::DRIVER, 4, 0xff),
    MCP3017AccelStepper(AccelStepper::DRIVER, 5, 0xff),
    MCP3017AccelStepper(AccelStepper::DRIVER, 6, 0xff),
    MCP3017AccelStepper(AccelStepper::DRIVER, 7, 0xff)
};

PCA9685 pwmSelenoides; 
#define MAX_SELENOIDES_OUTPUTS 16

PCA9685 pwmMicroMotors; 
#define MAX_MICROMOTORS_OUTPUTS 16

#define USE_TEENSY3_OPTIMIZED_CODE

#define USE_SDIO          1

// HardWare pin for buttons & selector & led & sync out
#define STARTPAUSE_PIN    24
#define STOP_PIN          25
#define LOOP_PIN          26
#define PLAY_NEXT_PIN     27

#define LEDPLAY_PIN       28

#define SYNC_OUT_PIN      29

#define SELECTOR_PIN      32
#define  BANK_PIN         33

// SETTINGS *****************
//timings
#define BLINKTIME         500 //ms
#define BOUNCE_MS         50 //ms

//midi file reader
#define MAXBNK           12 //number of bnks max
#define MAXSMF           12 //number of tracks max
#define FILENAMEFORMAT  "B%dS%d.MID" // 8.1 upper name ; where %d is 1-12

//MIDI stuff
#define MIDICH_SELENOIDES   0
#define MIDICH_MICROMOTORS  1
#define MIDICH_STEPPERS     2

// Use these with the Teensy 3.5 & 3.6 SD card
#define SDCARD_CS_PIN    BUILTIN_SDCARD
#define SDCARD_MOSI_PIN  11  // not actually used
#define SDCARD_SCK_PIN   13  // not actually used

//useful
#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))

//PWM outputs assignations
#define MAX_ANALOG_OUTPUTS 12
const uint8_t PWM_OUTPUT_PINS[MAX_ANALOG_OUTPUTS] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 23, 22, 21};

//Stepper motor assignations
#define MAX_STEPPER_MOTORS 	8
#define STEPPER_STEPDEG		15	//assume x degrees by step
#define MAX_STEPPER_SPEED	900
#define MAX_STEPPER_ACCEL	360
#define MAX_STEPPER_STEPS  512


//debounce&buttons
#define NUM_BUTTONS 4
const uint8_t BUTTONS_PINS[NUM_BUTTONS] = {STARTPAUSE_PIN, STOP_PIN, LOOP_PIN, PLAY_NEXT_PIN};
Bounce * buttons = new Bounce[NUM_BUTTONS];

void  playPauseFile();
void  stopFile();
void  loopFile();
void  playNextFile();

typedef void (*buttonFellActionList[])();
buttonFellActionList buttonFellAction = { playPauseFile, stopFile, loopFile, playNextFile };

void  unLoopFile();
void  unPlayNextFile();
void  doNothing();

typedef void (*buttonRoseActionList[])();
buttonRoseActionList buttonRoseAction = { doNothing, doNothing, unLoopFile, unPlayNextFile };


//LED global var
elapsedMillis elapsedBeat;
elapsedMillis ledTimer;
uint8_t       ledStatus = 0;

//smf filename
const size_t bufferLen = 13;
char filename[bufferLen];
uint8_t songSelector = 1;
uint8_t bankSelector = 1;
uint8_t	selected_BNK = 1;
uint8_t	selected_SNG = 1;


boolean flag_newFileSelector=false;

boolean flag_playNextFile = false;
boolean flag_playLoopFile = false;
boolean flag_pause = false;



static const char* smfErrors_liste[] =
{
   "Blank file name",
   "Can't open file specified",
   "File is not MIDI format",
   "MIDI header size incorrect",
   "File format type not 0 or 1",
   "File format 0 but more than 1 track",
   "More than MIDI_MAX_TRACKS required",
   "n0 = Track n track chunk not found",
   "n1 = Track n chunk size past end of file"
};

//ROUTINES ****************************************

void midiCallback(midi_event *pev)
// Called by the MIDIFile library when a file event needs to be processed
// thru the midi communications interface.
// This callback is set up in the setup() function.
{
  uint8_t note;
  uint8_t velocity;

  Serial.print("CH");Serial.print(pev->channel);  Serial.print(", ");Serial.print(pev->data[0], HEX); Serial.print(", "); Serial.print(pev->data[1]); Serial.print(", "); Serial.println(pev->data[2]);
  //if ( pev->channel != MIDICH ) return;
   if ( pev->channel != MIDICH_SELENOIDES )
   {
      if ((pev->data[0] >= 0x80) && (pev->data[0] <= 0xe0) && (pev->size == 3))
        {
      
          // NOTE OFF
          if (pev->data[0] == 0x80 ) // note off
          {
            noteOnSelenoide( pev->data[1] , 0 );
          }
      
          // NOTE ON
          if (pev->data[0] == 0x90 ) // note on
          {
            if (pev->data[2] != 0 ) //vel==0 -> note off
            {
              noteOnSelenoide( pev->data[1] , pev->data[2] );
            } else {
              noteOffSelenoide( pev->data[1] , 0 );
            }
          }
          
        }  
   }
   if ( pev->channel != MIDICH_MICROMOTORS )
   {
      if ((pev->data[0] >= 0x80) && (pev->data[0] <= 0xe0) && (pev->size == 3))
        {
      
          // NOTE OFF
          if (pev->data[0] == 0x80 ) // note off
          {
            noteOffPwmMotor( pev->data[1] , 0 );
          }
      
          // NOTE ON
          if (pev->data[0] == 0x90 ) // note on
          {
            if (pev->data[2] != 0 ) //vel==0 -> note off
            {
              noteOnPwmMotor( pev->data[1] , pev->data[2] );
            } else {
              noteOffPwmMotor( pev->data[1] , 0 );
            }
          }
          
        }  
   }

   if ( pev->channel != MIDICH_STEPPERS )
   {
      if ((pev->data[0] >= 0x80) && (pev->data[0] <= 0xe0) && (pev->size == 3))
        {
      
          // NOTE OFF
          if (pev->data[0] == 0x80 ) // note off
          {
            noteOffStepper( pev->data[1] , 0 );
          }
      
          // NOTE ON
          if (pev->data[0] == 0x90 ) // note on
          {
            if (pev->data[2] != 0 ) //vel==0 -> note off
            {
              noteOnStepper( pev->data[1] , pev->data[2] );
            } else {
              noteOffStepper( pev->data[1] , 0 );
            }
          }
          
        }  
   }


  

}

void sysexCallback(sysex_event *pev)
// Called by the MIDIFile library when a system Exclusive (sysex) file event needs
// to be processed through the midi communications interface. Most sysex events cannot
// really be processed, so we just ignore it here.
// This callback is set up in the setup() function.
{
  return;
}

void midiSilence(void)
// Turn everything off on every channel.
// Some midi files are badly behaved and leave notes hanging, so between songs turn
// off all the notes and sound
{
  // All sound off
  /*
  for ( int note = 0; note < MAX_ANALOG_OUTPUTS; note++)
  {
    noteOffPwm( note , 0);
  }
  */

  pwmSelenoides.setAllChannelsPWM(0);

  pwmMicroMotors.setAllChannelsPWM(0);

  for ( int stepperNb = 0; stepperNb < STEPPER_COUNT; stepperNb++)
  {
    steppers[stepperNb].stop();
    steppers[stepperNb].setCurrentPosition(0);  
    //steppers[stepperNb].moveTo(0); //reset to absolute position
  }
  
}

/*
void noteOnPwm( uint8_t note, uint8_t velocity )
{
  int velocityMapped;
  velocityMapped = map( velocity, 0, 127, 0, 256 );
  if ( note <= MAX_ANALOG_OUTPUTS )
  {
    analogWrite( PWM_OUTPUT_PINS[note],  velocityMapped);
  }
}

void noteOffPwm( uint8_t note, uint8_t velocity )
{
  if ( note <= MAX_ANALOG_OUTPUTS )
  {
    analogWrite( PWM_OUTPUT_PINS[note],  0);
  }
}
*/

//MIDI *******************************************
void handleControlChange(byte channel, byte number, byte value)
{
  if (value == 0)
  {
    handleNoteOff(channel, number, 0);
  } else {
    handleNoteOn( channel, number, value);
  }

}

void handleNoteOn(byte channel, byte number, byte value)
{
  if ( channel == MIDICH_SELENOIDES )
  {
     if (value == 0)
     {
       noteOffSelenoide( number, 0 );
     } else {
       noteOnSelenoide( number, value );
     }
  }
  
  if ( channel == MIDICH_MICROMOTORS )
  {
     if (value == 0)
     {
       noteOffPwmMotor( number, 0 );
     } else {
       noteOnPwmMotor( number, value );
     }
  }

  if ( channel == MIDICH_STEPPERS )
  {
     if (value == 0)
     {
       noteOffStepper( number, 0 );
     } else {
       noteOnStepper( number, value );
     }
  }
  
}

void handleNoteOff(byte channel, byte number, byte value)
{
  switch (channel)
    {
      case MIDICH_SELENOIDES:    
          noteOffSelenoide( number, 0 );
      break;
      
      case MIDICH_MICROMOTORS:    
          noteOffPwmMotor( number, 0 );
      break;   
      
      case MIDICH_STEPPERS:    
          noteOffStepper( number, 0 );
      break;
            
    }

}

void noteOnSelenoide( uint8_t note, uint8_t velocity )
{
  int velocityMapped;
  velocityMapped = map( velocity, 0, 127, 0, 4096 );
  if ( note <= MAX_SELENOIDES_OUTPUTS )
  {
    //analogWrite( PWM_OUTPUT_PINS[note],  velocityMapped);
    pwmSelenoides.setChannelPWM(note, velocityMapped);
  }
}

void noteOffSelenoide( uint8_t note, uint8_t velocity )
{
  if ( note <= MAX_SELENOIDES_OUTPUTS )
  {
    //analogWrite( PWM_OUTPUT_PINS[note],  0);
      pwmSelenoides.setChannelOff(note);
  }
}

void noteOnPwmMotor( uint8_t note, uint8_t velocity )
{
  int velocityMapped;
  velocityMapped = map( velocity, 0, 127, 0, 4096 );
  if ( note <= MAX_MICROMOTORS_OUTPUTS )
  {
    //analogWrite( PWM_OUTPUT_PINS[note],  velocityMapped);
    pwmMicroMotors.setChannelPWM(note, velocityMapped);
  }
}

void noteOffPwmMotor( uint8_t note, uint8_t velocity )
{
  if ( note <= MAX_MICROMOTORS_OUTPUTS )
  {
    //analogWrite( PWM_OUTPUT_PINS[note],  0);
    pwmMicroMotors.setChannelOff(note);
  }
}

void noteOnStepper( uint8_t note, uint8_t velocity )
{
  long velocityMapped;
  int stepperNb = note >> 2;
  int stepperfct = note %4 ;
  int stepDirection = 1;

   if ( note >= STEPPER_COUNT * 4 )return;
  
  (note % 2 == 0) ? stepDirection = 1 : stepDirection = -1;
  
  if( stepperfct < 2) //move x step 
  {
      velocityMapped = map( velocity, 0, 127, 0, ( stepDirection * MAX_STEPPER_STEPS ) );
      steppers[stepperNb].setSpeedMode(false);
      steppers[stepperNb].moveTo(velocityMapped); 
  }else{
      velocityMapped = map( velocity, 0, 127, 0, ( stepDirection * MAX_STEPPER_SPEED ) );
      steppers[stepperNb].setSpeedMode(true);
      steppers[stepperNb].setSpeed( (float)velocityMapped ); 
  }

}

void noteOffStepper( uint8_t note, uint8_t velocity )
{
  int stepperNb = note >> 2;
  int stepperfct = note %4 ;
  int stepDirection = 1;
    if ( note >= STEPPER_COUNT * 4 )return;
  
  (note % 2 == 0) ? stepDirection = 1 : stepDirection = -1;
  
  if( stepperfct < 2) //
  {
      //steppers[stepperNb].setSpeedMode(false);
      //steppers[stepperNb].moveTo(stepDirection);
      steppers[stepperNb].stop(); 
      steppers[stepperNb].setCurrentPosition(0);  
  }else{
      steppers[stepperNb].setSpeedMode(false);
      steppers[stepperNb].stop();
      steppers[stepperNb].setCurrentPosition(0);  
      steppers[stepperNb].setSpeed( 0.0 ); // not need indeed
  }

}


//BUTTONS ACTIONS************
void playPauseFile()
{

  int err;

  //if( ! flag_readyToPlay )return;

  if ( ! SMF.isEOF() )
  {
    if ( ! flag_pause )
    {
      SMF.pause(true);
      flag_pause = true;
      LedPlayBLINK();
      return;
    } else {
      SMF.pause(false);
      flag_pause = false;
      LedPlayON();
      return;
    }
  }

  // Start playing the file.  This sketch continues to
  // run while the file plays.
  SMF.setFilename(filename); // filenames are always uppercase 8.3 format
  err = SMF.load();
  if (err != -1)
  {
    LedPlayERROR();
    Serial.print ("Erreur smf: ");
    Serial.println(smfErrors_liste[err]);
    return;
  }
	Serial.println("playing :");
	Serial.println(filename);
  // Simply wait for the file to finish playing.
  LedPlayON();
}

void stopFile()
{
  SMF.pause(false);
  SMF.close();
  midiSilence();
  LedPlayOFF();
}

void  loopFile()
{
  SMF.looping( true );
  flag_playLoopFile = true;
}

void  unLoopFile()
{
  SMF.looping( false );
  flag_playLoopFile = false;
}

void playNextFile()
{
  flag_playNextFile = true;
}

void unPlayNextFile()
{
  flag_playNextFile = false;
}

// FILE ***************
void selectFile(int banknb, int songnb )
{
  if (songnb > MAXSMF)songnb = MAXSMF;
  if (banknb > MAXBNK)banknb = MAXBNK;
  sprintf(filename, FILENAMEFORMAT, banknb, songnb);
}

// flash a LED to the beat
void tickMetronome(void)
{
  static boolean  inBeat = false;
  uint16_t  beatTime;

  beatTime = 60000 / SMF.getTempo();  // msec/beat = ((60sec/min)*(1000 ms/sec))/(beats/min)

    if (elapsedBeat >= beatTime)
    {
      elapsedBeat = 0;
      digitalWriteFast(SYNC_OUT_PIN, true);
      LedPlayON();
	  inBeat = true;
      Serial.println("tick");
    }
    
    if (elapsedBeat >= 100) // keep the flash on for 100ms only
    {
      LedPlayOFF();
      // inBeat = false;
    }else if (inBeat && elapsedBeat >= 20) // keep the SYNC_OUT ON for 20ms only
    {
      digitalWriteFast(SYNC_OUT_PIN, false);
      inBeat = false;
    }
  
}

//deBOUNCE *******************
void updateBounce()
{
  int actualSongSelector;
  int actualBankSelector;
  bool new_settings;

  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    // Update the Bounce instance :
    buttons[i].update();
    if (buttons[i].fell())buttonFellAction[i]();
    if (buttons[i].rose())buttonRoseAction[i]();
  }
  
  actualBankSelector = map(analogRead( BANK_PIN ), 0, 1023, 1, MAXBNK);
  if ( actualBankSelector != bankSelector )
  {
    new_settings = true;
    bankSelector = actualBankSelector;
  }

  actualSongSelector = map(analogRead( SELECTOR_PIN ), 0, 1023, 1, MAXSMF);
  if ( actualSongSelector != songSelector )
  {
    new_settings = true;
    songSelector = actualSongSelector;
  }
  
	if( new_settings )
	{
	   selected_BNK=bankSelector;
	   selected_SNG=songSelector;
	   flag_newFileSelector=true;
	   selectFile(bankSelector,songSelector);
	}
}

//MIDI FILE READER ***********
void updateSMF()
{
  if (!SMF.isEOF())
  {
    if (SMF.getNextEvent())tickMetronome();
  } else {
    // done with this one
    //Serial.println("stop");
    stopFile();

    if (flag_playNextFile)
    {
		if( ! flag_newFileSelector ) // no new file already selected?
		{
			selected_SNG++;
			if (selected_SNG > MAXSMF)
			{
				  selected_SNG = 1;
				  
				  selected_BNK++;
				  if (selected_BNK > MAXBNK)
				  {
					  selected_BNK = 1;
				  }
			} 
			selectFile(selected_BNK, selected_SNG); 
		}else{
			flag_newFileSelector=false; //reset flag
		}
		
		playPauseFile();
    }
  }
}

//LEDs ***********************
void updateLeds()
{
  switch (ledStatus)
  {
    case 0:
      digitalWrite(LEDPLAY_PIN, false);
      break;
    case 1:
      digitalWrite(LEDPLAY_PIN, true);
      break;
    case 2:
      if ( ledTimer > BLINKTIME )
      {
        digitalWrite(LEDPLAY_PIN, true);
        ledTimer = 0;
      }
      if ( ledTimer > ( BLINKTIME / 2 ) )
      {
        digitalWrite(LEDPLAY_PIN, false);
      }
      break;

    case 3:
      if ( ledTimer > BLINKTIME / 2 )
      {
        digitalWrite(LEDPLAY_PIN, true);
        ledTimer = 0;
      }
      if ( ledTimer > ( BLINKTIME / 4 ) )
      {
        digitalWrite(LEDPLAY_PIN, false);
      }
      break;

    default:
      // if nothing else matches, do the default
      // default is optional
      break;
  }

}

void LedPlayON()
{
  ledStatus = 1;
}

void LedPlayOFF()
{
  ledStatus = 0;
}

void LedPlayBLINK()
{
  ledStatus = 2;
}

void LedPlayERROR()
{
  ledStatus = 3;
}

void doNothing()
{
  return;
}

// serial command ***************************
void setup_sCmd()
{
  // Setup callbacks for SerialCommand commands
  sCmd.addCommand("P", d_play);  //
  sCmd.addCommand("T", d_select_song);  //1-12
  sCmd.addCommand("B", d_select_bank);  //1-12
  sCmd.addCommand("S", d_stop);  //
  sCmd.addCommand("L", d_loop);  //
  sCmd.addCommand("N", d_next);  //
  
  sCmd.addCommand("M", d_midi);  //mimic midi noteOn
  
  sCmd.setDefaultHandler(unrecognized); // Handler for command that isn't matched  (says "What?")
}

void unrecognized(const char *command)
{
  Serial.print("erreur: ");
  Serial.println( command );
  Serial.println("--------------------------------");
  Serial.println(" \"P\" play/pause");
  Serial.println(" \"S\" stop");
  Serial.println(" \"L\" loop");
  Serial.println(" \"N\" next");
  Serial.println(" \"B n\" select bank n (n=1-12)");
  Serial.println(" \"T n\" select track n (n=1-12)");
  Serial.println(" \"M n\" send MidiNoteOn: M channel note velocity ( M 0 12 127)");
  Serial.println("");
  Serial.print("selected file: ");
  Serial.println(filename);
  Serial.print("loop: ");
  Serial.println(flag_playLoopFile);
  Serial.print("next: ");
  Serial.println(flag_playNextFile);
  Serial.println("--------------------------------");
  Serial.println("");
  return;
}

void d_play()
{
  playPauseFile();
  if ( ! flag_pause)
  {
    Serial.print("play ");
    Serial.println(filename);
  } else {
    Serial.println("pause");
  }
}

void d_stop()
{
  Serial.println("stop");
  stopFile();
}

void d_loop()
{
  flag_playLoopFile = ! flag_playLoopFile;
  
  if( flag_playLoopFile )
  {
    Serial.println("loop");
    SMF.looping( true );
    //flag_playLoopFile = true;
  }else{
    Serial.println("no loop");
    SMF.looping( false );
    //flag_playLoopFile = false;
  }
}

void d_next()
{
  flag_playNextFile = ! flag_playNextFile;
  
  if( flag_playNextFile )
  {
    Serial.println("next at end");
    flag_playNextFile = true;
  }else{
    Serial.println("stop at end");
    flag_playNextFile = false;
  }
}

void d_select_bank()
{
  int aNum;
  char *arg;
  arg = sCmd.next();
  if (arg != NULL)
  {
    aNum = constrain(atoi(arg), 1, 12); // Converts a char string to an integer
    if( selected_BNK != aNum )
    {
		selected_BNK=aNum;
		selectFile(selected_BNK, selected_SNG);
		flag_newFileSelector = true;
	}
    Serial.print("selected bank: ");
    Serial.println(selected_BNK);
    Serial.print("selected file: ");
    Serial.println(filename);
  } else {
    Serial.println("error need 1 arguments, B nb (1-12) ex: B 10");
  }
}

void d_select_song()
{
  int aNum;
  char *arg;
  arg = sCmd.next();
  if (arg != NULL)
  {
    aNum = constrain(atoi(arg), 1, 12); // Converts a char string to an integer
    if( selected_SNG != aNum )
    {
		selected_SNG=aNum;
		selectFile(selected_BNK, selected_SNG);
		flag_newFileSelector = true;
	}
    Serial.print("selected song: ");
    Serial.println(selected_SNG);
    Serial.print("selected file: ");
    Serial.println(filename);
  } else {
    Serial.println("error need 1 arguments, S nb (1-12) ex: S 10");
  }
}

void d_midi()
{
  int aNum;
  char *arg;
  uint8_t	chan, note, vel;
  arg = sCmd.next();
  if (arg != NULL)
  {
    aNum = constrain(atoi(arg), 0, 15); // Converts a char string to an integer
  	chan = aNum;
  } else {
    Serial.println("error: need 3 arguments, M channel note velocity ( M 0 15 67 )");
    return;
  }
  
  arg = sCmd.next();
  if (arg != NULL)
  {
	aNum = constrain(atoi(arg), 0, 127); // Converts a char string to an integer
	note = aNum;
  } else {
    Serial.println("error: need 3 arguments, M channel note velocity ( M 0 15 67 )");
    return;
  }
  
  arg = sCmd.next();
  if (arg != NULL)
  {
    aNum = constrain(atoi(arg), 0, 127); // Converts a char string to an integer
	vel = aNum;
  } else {
    Serial.println("error: need 3 arguments, M channel note velocity ( M 0 15 67 )");
    return;
  }
  
  Serial.print("NoteOn : ");
  Serial.print("channel ");  Serial.print(chan);
  Serial.print("  note ");  Serial.print(note);
  Serial.print("  velocity ");  Serial.println(vel);
  
  handleNoteOn(chan, note, vel);
  
}

//STEPPERS *********************************
void updateStepper()
{
  for (int i = 0; i < STEPPER_COUNT; i++)
  {
    steppers[i].update_run();
  } 
}

//INITS **************************************
void init_debounce()
{
	for (int i = 0; i < NUM_BUTTONS; i++)
	{
		// Setup the first button with an internal pull-up :
		pinMode(BUTTONS_PINS[i], INPUT_PULLUP);
		// After setting up the button, setup the Bounce instance :
		buttons[i].attach(BUTTONS_PINS[i]);
		buttons[i].interval(BOUNCE_MS); // interval in ms
	 }
}

void init_steppers()
{
	mcpSTEPPERS.begin();
	for (int i = 0; i < STEPPER_COUNT; i++) 
	{
		steppers[i].setMcp(mcpSTEPPERS);
		steppers[i].enableOutputs();
		steppers[i].setMaxSpeed(500.0);
		steppers[i].setAcceleration(100.0);
		steppers[i].moveTo(200);
	}
}

void init_pwm()
{
	  //pwmSelenoide.resetDevices();       // Software resets all PCA9685 devices on Wire line
	pwmSelenoides.init(B000000); 
	pwmMicroMotors.init(B000001); 
}

void init_SD()
{
	Serial.print("Try to init SD... ");

	if (! SD.begin(SDCARD_CS_PIN))
	{
		LedPlayERROR();
		Serial.println("SD error !!!");
		
		while (true)
		{
			updateLeds();
		}
		
	}
	Serial.println("SD ok");

}

void initMIDISerial() // old fashion MIDI hardware INPUT
{
  // // initialize MIDI ***********************************************************
  // midi input is setup in MIDI.begin (pin 2 serialRX)
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  //MIDI.setHandlePitchBend(handlePitchBend);
  MIDI.setHandleControlChange(handleControlChange);
  //MIDI.setHandleSystemReset(handleSystemReset);
  MIDI.begin(MIDI_CHANNEL_OMNI);
}

void initMIDIUsb() //USB MIDI IN
{
  // initialize MIDI usb ***********************************************************
  //
  usbMIDI.setHandleNoteOn(handleNoteOn);
  usbMIDI.setHandleNoteOff(handleNoteOff);
  // MIDI.setHandleNoteOff(handleNoteOff);
  //MIDI.setHandlePitchBend(handlePitchBend);
  //MIDI.setHandleControlChange(handleControlChange);
  //MIDI.setHandleSystemReset(handleSystemReset);
  //MIDI.begin(MIDI_CHANNEL_OMNI);
}


//SETUP **************************************
void setup(void)
{
	Serial.begin(115200);
	delay(500);
	
	Serial.println("**** MIDI File reader to PWM & Steppers ****");
	Serial.println("Starting");

	// Set hardware in / out
	pinMode(LEDPLAY_PIN, OUTPUT);
	pinMode(SYNC_OUT_PIN, OUTPUT);

	pinMode(SELECTOR_PIN, INPUT); //analog input (not useful for convinience)

	//init  pwm outputs (not useful for convinience)
	for (int i = 0; i < MAX_ANALOG_OUTPUTS; i++)
	{
		pinMode(PWM_OUTPUT_PINS[i], OUTPUT);
	}

	//init debounce
	init_debounce();

	//init steppers
	init_steppers();

	// init pwm outputs
	init_pwm();

	//init SD
//	init_SD();
	
	//init serial(usb) commands
	setup_sCmd();

	// Initialize MIDIFile reader
//	SMF.begin();
//	SMF.setMidiHandler(midiCallback);
//	SMF.setSysexHandler(sysexCallback);

	//init old fashion midi
	initMIDISerial();
	
	//init usb midi
	initMIDIUsb();
	
	Serial.println("Init all done !");
	Serial.println("*********************************");
	
	//select first SD file
//	selectFile(1,1);

}


//INFINITE LOOP *****************************
void loop(void)
{
  MIDI.read();
  usbMIDI.read();
  sCmd.readSerial();
  //updateBounce();
 // updateSMF();
  updateStepper();
  updateLeds();
}
