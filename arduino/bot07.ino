// sample with documentation
// http://punchthrough.com/bean/examples/lightblue-demo-sketch/

// thomas' example:
// https://github.com/pebble-hacks/devretreat14/blob/master/pebblebots/lightblue-bean/pebblebot/pebblebot.ino

#include <Bean.h>
#include <Servo.h> 

// globals
//uint16_t LED_BLUE_VALUE_SCRATCH_NUMBER = 1;
uint16_t LEFT_SERVO_SCRATCH_NUMBER = 1;
uint16_t RIGHT_SERVO_SCRATCH_NUMBER = 2;
uint16_t SPECIAL_ACTION_ID_SCRATCH_NUMBER = 3;
Servo leftServo;
Servo rightServo;

uint16_t servoStill = 90;  
uint16_t previousLeftServoSpeed = 0;  
uint16_t previousRightServoSpeed = 0;
int specialActionId = 0;
uint16_t opr_mode=0;


// qualification: 
// interpretation
// if value < 90: 0
// if value == 90: 90
// if value > 90: 180
uint16_t adjustValue (uint16_t value) {
  return value;
  // if (value  > 90) {
  //  return 180;
  //}
  //return 0;
}

void left ()
{
  leftServo.write(180);
  rightServo.write(180);
}

void right ()
{
  leftServo.write(180);
  rightServo.write(0);
}

void straight () {
  leftServo.write(0);
  rightServo.write(0);
}
      

// main entry point
void setup() 
{
  // initialize serial communication at 57600 bits per second:
  // Serial.begin(57600);
  // Serial.setTimeout(25);
  
  // when starting up, blink to know we are up, and leave it green to know we are running
  // http://punchthrough.com/bean/arduino-reference/setled/
  {
    int sleep = 250;
//    setLed (255, 0, 0);
    //Bean.sleep (sleep);
    
//    setLed (0, 255, 0);
    //Bean.sleep (sleep);
    
//    setLed (0, 0, 255);
    //Bean.sleep (sleep);
    
//    setLed (0, 255, 0);
  }
  
  // attach to both motors
  leftServo.attach (1);
  rightServo.attach (0);

  // initialize to stopped
  Bean.setScratchNumber(LEFT_SERVO_SCRATCH_NUMBER, servoStill);
  Bean.setScratchNumber(RIGHT_SERVO_SCRATCH_NUMBER, servoStill);
//  Bean.setScratchNumber(SPECIAL_ACTION_ID_SCRATCH_NUMBER, servoStill);
}

// the loop routine runs over and over again forever:
// http://punchthrough.com/bean/arduino-reference/readscratchdata/
void loop() 
{
unsigned long StartMillis;
unsigned long currentMillis;  // sleep and sync up
  currentMillis = millis();
  Bean.sleep(100);
  int8_t state;
  // read scratch #1 as an integer that controls the intensity of the blue light
  // so that the pebble can toggle it and we know we are talking
  // http://punchthrough.com/bean/arduino-reference/readscratchnumber/
  {
//    long value = Bean.readScratchNumber(LED_BLUE_VALUE_SCRATCH_NUMBER);
//    setLedBlue (value);
  }
  
 {
    long value = Bean.readScratchNumber(SPECIAL_ACTION_ID_SCRATCH_NUMBER);
	if ((value >0)&&(opr_mode==0))
	{
		StartMillis = millis();
		opr_mode=value;
		state=0;
		Bean.setScratchNumber(SPECIAL_ACTION_ID_SCRATCH_NUMBER, 0);
	}
    if (opr_mode == 1)
    {
		if (state==0)
		{
			left ();
			state=1;
		}
		if (state==1)
			if (currentMillis-StartMillis>500)
		{
			straight ();
			state=2;
		}	
		if (state==2)
			if (currentMillis-StartMillis>1500)
		{
			right ();
			state=3;
			
		}	
		if (state==3)
			if (currentMillis-StartMillis>2000)

	{
			opr_mode=0;
			
		}	

	

    }
    else if (opr_mode == 2)
    {
      // turn 180/270 degrees
      left ();
      left ();
      left ();
      left ();
      left ();
      left ();
      
      // wiggle to steal the ball
     // left ();
     // right ();
     // left ();
     // right ();
      
      // turn 180 minus a little bit to be in front of the ball
      right ();
      right ();
      right ();
      right ();
      opr_mode=0;
      // reset
      leftServo.write(previousLeftServoSpeed);
      rightServo.write(previousRightServoSpeed); 
    }
    else if (opr_mode == 3)
    {
      // go left, fast, and turn around
      left ();
      left ();
      left ();
      straight ();
      straight ();
      straight ();
      right ();
      right ();
      right ();
      right ();
      right ();
      opr_mode=0;
      // reset
      leftServo.write(previousLeftServoSpeed);
      rightServo.write(previousRightServoSpeed); 
    }
    else if (opr_mode == 4)
    {
      // go right, fast, and turn around
      right ();
      right ();
      right ();
      straight ();
      straight ();
      straight ();
      left ();
      left ();
      left ();
      left ();
      left ();
      opr_mode=0;
      // reset
      leftServo.write(previousLeftServoSpeed);
      rightServo.write(previousRightServoSpeed); 
    }
  }

  // read buffers 2 and 3 for left and right wheel speeds
  // http://punchthrough.com/bean/arduino-reference/readscratchnumber/
  if (opr_mode==0)
  {
    uint16_t leftServoSpeed = Bean.readScratchNumber(LEFT_SERVO_SCRATCH_NUMBER);
    leftServoSpeed = adjustValue (leftServoSpeed);
    if (leftServoSpeed != previousLeftServoSpeed) leftServo.write(leftServoSpeed); 
    previousLeftServoSpeed = leftServoSpeed;
  }
  if (opr_mode==0)
  {
    uint16_t rightServoSpeed = Bean.readScratchNumber(RIGHT_SERVO_SCRATCH_NUMBER);
    rightServoSpeed = adjustValue (rightServoSpeed);

    if (rightServoSpeed != previousRightServoSpeed) rightServo.write(rightServoSpeed); 
    previousRightServoSpeed = rightServoSpeed;
  }
}

