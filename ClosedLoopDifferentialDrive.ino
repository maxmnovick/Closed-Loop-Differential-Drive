/* Given translational and rotational velocities,
 * determine left and right motor commands.
 */
//======Advisor======
//===arbitration struct===
typedef struct layer LAYER; // C struct for subsumption task output

struct layer
{
  volatile int cmd,   // assertion command
                arg,  // assertion argument
                flag; // subsumption flag (instead of layer state?)
};

// the robot has 2 subsumption behaviors, so define 2 LAYERS:
LAYER user,
  halt; // the default layer

const int job1Size = 2;

LAYER *job1[job1Size] = { &user, &halt };

volatile LAYER *thisLayer = &halt; // output, layer chosen by arbitrator; global pointer to highest priority task asserting its flag, which is maintained by the "arbitration winner" signal

LAYER **job; // pointer to job priority list

int jobSize, // number of tasks in priority list
  arbitrate, // global flag to enable subsumption
  haltBot; // global flag to halt robot
  
volatile int userInvert;

//======Interface======
volatile String inputString = "";

volatile boolean stringComplete = false;  // whether the string is complete

//======Motor Driver======
const byte mSigPins[] = { 4, 7 };
const byte mEnablePins[] = { 5, 6 };

//======Controller======
int reverse, // note: could be bool
  sensorsTmrCtr;

volatile int setTanVel, // 0-100%
  setRotVel, // 0-100%
  botVel; // global, current requested robot velocity
  
const int numMtrs = 2;

volatile int pubMtrCmds[numMtrs],
  signs[numMtrs],
  mtrOutAccums[numMtrs];
  
void setup() // robot startup
{
  initSystem(); // e.g. interrupts

  initBehaviors();
}

int initSystem()
{
  initNode("OpenLoopDifferentialDrive");

  initSubscribers();

  initPublishers();
  
  return 0;
}

void initNode(String id)
{
  Serial.begin(9600);

  while(!Serial);
  
  Serial.print("Starting ");
  Serial.print(id);
  Serial.println(".ino\n");
}

void initSubscribers()
{
  // pulse count
  //attachInterrupt(digitalPinToInterrupt(esPins[0]), encoder1Callback, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(esPins[1]), encoder2Callback, CHANGE);
}

void initPublishers()
{
  /* Start Motor Channel 1 */
  pinMode(mEnablePins[0], OUTPUT);
  pinMode(mSigPins[0], OUTPUT);

  /* Start Motor Channel 2 */
  pinMode(mEnablePins[1], OUTPUT);
  pinMode(mSigPins[1], OUTPUT);

  initSensorsTimer(); // ADD: publish rotational (and later translational) rotVel
}

void initSensorsTimer()
{
  noInterrupts();           // disable all interrupts
  
  TCCR1A = 0;
  TCCR1B = 0;
  sensorsTmrCtr = 59286;   // preload timer 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)
  
  TCNT1 = sensorsTmrCtr;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  
  interrupts();             // enable all interrupts
}

void initBehaviors()
{
  initVars();

  setParams();
  
  // set job1 as active job at startup time
  initJob1();
}

void initVars()
{
  setTanVel = 0; // [%]

  setRotVel = 0; // [%]

  pubMtrCmds[0] = 0;
  pubMtrCmds[1] = 0;

  signs[0] = 1;
  signs[1] = 1;

  mtrOutAccums[0] = 0;
  mtrOutAccums[1] = 0;
}

void setParams()
{
  userInvert = 0;

  reverse = 0;

  arbitrate = 1;
}

int initJob1() // make job1 the active job
{
  job = &job1[0]; // global job priority list pointer

  jobSize = job1Size; // no. tasks in job1 list

  return 0;
}

void loop() // scheduler
{
}

//======Interrupt Service Routines======
/* Run Sensor Loop at x (maybe 10-20) Hz, 
 * interrupt service routine - tick every 0.1 s (10 Hz)
 */
ISR(TIMER1_OVF_vect) // sensors loop!
{
  TCNT1 = sensorsTmrCtr; // set timer

  userTask(); // accelerate forward and maintain top speed

  arbitrator(); // send highest priority to motors
}

/* Like common cruise behavior,
 * but allows user to request tan and rot vels,
 * rather than assuming top speed or 0.
 */
int userTask()
{
  extern LAYER user; // C structure for task output

  readUserInput(); // read: get requested tan and rot velocities. in this case, received by uio, which is a separate task(?)

  if(userInvert) // if inverted
  {
    user.cmd = 0; // request drive speed 0
    user.arg = 0; // request turn speed 0
  }
  else 
  {
    if(reverse) 
    {
      user.cmd = -setTanVel; // request -drive
      user.arg = -setRotVel; // request -turn
    }
    else
    {
      user.cmd = setTanVel; // request drive
      user.arg = setRotVel; // request turn
    }
  }

  user.flag = true; // always signal arbitrator we want control, unless disabled (would need to add cruiseEnable var)
}

void readUserInput()
{
  if(stringComplete)
  {
    Serial.print("inputString: ");

    // receive command from user
    if(inputString.substring(0,1) == "g")
    {
      Serial.println("go");

      userInvert = 0;
    }
    else if(inputString.substring(0,1) == "s")
    {
      Serial.println("stop");

      userInvert = 1;
    }
    else if(inputString.substring(0,1) == "t") // 0-100% duty cycle
    {
      setTanVel = inputString.substring(1, inputString.length()).toInt(); // get string after 't'

      Serial.print("vt = ");
      Serial.print(setTanVel);
      Serial.println("%\n");
    }
    else if(inputString.substring(0,1) == "r") // 0-100% duty cycle
    {
      setRotVel = inputString.substring(1, inputString.length()).toInt(); // get string after 'r'
    
      Serial.print("vr = ");
      Serial.print(setRotVel);
      Serial.println("%\n");
    }

    // clear string:
    inputString = ""; //note: in code below, inputString will not become blank, inputString is blank until '\n' is received

    stringComplete = false;
  }

  if(Serial.available())
    serialEvent();
}

void serialEvent()
{
  while (Serial.available()) 
  {
    // get the new byte:
    char inChar = (char) Serial.read();
    
    // add it to the inputString:
    inputString += inChar;
    
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n')
      stringComplete = true;
  }
}

/* "winnerId" feedback line 
 * from Arbitrate back to tasks:
 * Essentially global var containing ID of task 
 * that "won" this round of arbitration.
 * It can be used by the individual tasks
 * to determine if they have been subsumed.
 */
void arbitrator()
{
  int i = 0;

  if(arbitrate)
  {
    for(i=0; i < jobSize - 1; i++) // step through tasks
    {
      if(job[i]->flag) break; // subsume
    }

    thisLayer = job[i]; // global output winner
  }

  mtrCmd(thisLayer); // send command to motors; execute, given pointer to winning layer
}

void mtrCmd(LAYER *l)
{
  int setVels[numMtrs];
  
  botVel = l->cmd; // current requested velocity
  
  // setVels as duty cycle (next version will allow set actual speed)
  setVels[0] = botVel + l->arg; // // left motor = velocity + rotation, (int) round( 1000 * setRotVel / ( minAngularRes * pubVelRate ) ); // convert [deg/s] to [pulses/(1/pubVelRate)s]
  setVels[0] = clip(setVels[0], 100, -100); // don't overflow +/- 100% full speed
  
  setVels[1] = botVel - l->arg; // right motor = velocity - rotation
  setVels[1] = clip(setVels[1], 100, -100); // don't overflow +/- 100% full speed

  modulatePulseWidths(setVels);
}

int clip(int a, int maximum, int minimum)
{
  //Serial.print("Computed val: ");
  //Serial.print(a);
    
  if(a > maximum) 
    a = maximum;
  else if(a < minimum) 
    a = minimum;

  //Serial.print(", Clipped val: ");
  //Serial.println(a);

  return a;
}

/* The PWM code drives the hardware H-bridge, 
 * which actually control the motor.
 * This routine takes a signed value, 
 * -100 < signedVal < 100,
 * sets the sign variable used by the speedometer code,
 * sets the forward/backward (i.e. direct/reverse) bits 
 * on the H-bridge, and
 * uses abs(signedVal) as an index into a 100 entry table 
 * of linear PWM values.
 * This function uses a timer interrupt to generate 
 * a x Hz (maybe 120 Hz) variable pulse-width output.
 */
void modulatePulseWidths(int signedVals[]) // take signed value, b/t -100 and 100
{
  int i;
  
  for(i=0; i < 2; i++)
  {
    //setSpeedometerSign(i, signedVals[i]); // set sign variable used by speedometer code

    setHBridgeDirectionBit(i, signedVals[i]);
  
    setPWMValueFromEntryTable(i, abs(signedVals[i])); // use abs(signedVal) as an index into a 100 entry table of linear PWM values
  }
  
  for(i=0; i < 2; i++)
    analogWrite(mEnablePins[i], pubMtrCmds[i]); // generate variable pulse-width output
}

/* The sign variable represents the direction of rotation
 * of the motor (1 for forward and -1 for backward).
 * With more expensive quadrature encoders this info is
 * read directly from the encoders.
 * In this implementation I have only simple encoders so
 * the direction of rotation is taken from the sign of the
 * most recent command issued by the PID to the PWM code.
 */
//void setSpeedometerSign(int mid, int signedVal) // signedVal should be most recent cmd issued by PID to PWM code
//{
//  if(signedVal < 0) // {motor direction of rotation} = backward
//    signs[mid] = -1;
//  else if(signedVal >= 0)
//    signs[mid] = 1; // {motor direction of rotation} = {forward | resting}
//  else
//    Serial.println("Invalid command issued by PID to PWM code!\n");
//}

void setHBridgeDirectionBit(int mid, int signedVal)
{
  if(signedVal < 0) // {motor direction of rotation} = backward
  { 
    if(mid == 0) digitalWrite(mSigPins[mid], HIGH);
    else if(mid == 1) digitalWrite(mSigPins[mid], LOW);
  }
  else if(signedVal >= 0) // {motor direction of rotation} = {forward | resting}
  { 
    if(mid == 0) digitalWrite(mSigPins[mid], LOW);
    else if(mid == 1) digitalWrite(mSigPins[mid], HIGH);
  }
  else
    Serial.println("Invalid command issued by PID to PWM code!\n");
}

// use magnitude as an index into a 100 entry table of linear PWM values
void setPWMValueFromEntryTable(int mid, int magnitude)
{
  pubMtrCmds[mid] = (int) round( magnitude * 255 / 100 ); // cruise outputs
}
