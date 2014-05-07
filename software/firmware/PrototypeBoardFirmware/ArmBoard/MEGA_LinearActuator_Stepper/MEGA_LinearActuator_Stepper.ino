
/*

 ---------------------------------------------

 Nick McComb
 mccombn@onid.orst.edu
 January / Febuary / March 2014
 Code Originally Developed for Mars Rover 2014
 
 ---------------------------------------------
 
 This code was developed to control a roboitc arm that has two stepper motors and
 two linear actuators. The linear actuators control joints in the arm, and one stepper
 motor controls the base rotation, and the other stepper controls the gripping mechanism.
 
 The system also has a GUI that was written in python, and can send packets over serial that
 will work with the implementation in this file. The Serial Packet is described below. 
 
 There is a handshake implementation, such that the microcontroller transmitts an 'r' over serial
 once it has completed all of its actions, and it will not recieve new information until it has
 proccessed its current task.
 
 The overall operating procedure for the code is as follows:
 
 INIT
  - Initialize Serial 0 and 1
  - Send 170 byte over serial to Sabertooth to init Sabertooth
  - Set a serial timeout delay of 1 second for the Sabertooth  //Might be changed in the future
  - Move the linear actuators to a default position that facilitates rotation
  - Calibrate the stepper
    - To calibrate the stepper, the base rotates clockwise, until it recieves a claibration switch input,
      then rotates counter-clockwise until it recieves another input. 
    - NOTE: The algorithm will assume that it has moved 360 degrees between both button presses, even if it 
      hasn't. This means that you can (for testing, for example) only have it move 180 degrees, then send it
      commands that are proportional to what it thinks will be 0/360 but in reality will only be 0/180
  - Moves the arms to a beginning position, currently 180 degrees  //Might change in the future
 MAIN
  - Check for incoming serial byte
    - If a byte has been detected, add it to the current buffer
    - If a 'complete buffer' has been recieved, then process the buffer
      - Processing the buffer involves converting all of the information recieved into the types that are 
        required for the program to operate
      - Processing also enables all of the motors that should be enabled (that changed values), with the 
        exception of the linear actuators, which are enabled everytime
      - After a byte has been processed, the motors are moved in the following order:
	- Rotate Base
	- Move Linear Actuators (simultaneously)
	- Process Grip
 
 

 
 Description of the Serial Packet. Each of the lines represents a byte that is recieved over serial
 HEADER - 255
 COMMAND - See Google Doc
 BASE ROT VALUE 1 - Lower 255 bytes of the degrees of the base rotation
 BASE ROT VALUE 2 - Upper (360-255) of the degrees of the base rotation
 ACT 1 VAL 1 - Lower 2.55 inches of lower (elbow) actuator extension (multiplied by 100)
 ACT 1 VAL 2 - Upper (3.75 - 2.55) inches of lower (elbow) actuator extension (multiplied by 100)
 ACT 2 VAL 1 - Lower 2.55 inches of upper (shoulder) actuator extension (multiplied by 100)
 ACT 2 VAL 2 - Upper (3.75 - 2.55) inches of upper (shoulder) actuator extension (multiplied by 100)
 CHECKSUM: command ^ baseRotVal1 ^ baseRotVal2 ^ act1Val1 ^ act1Val2 ^ act2Val1 ^ act2Val2
 TAIL - 255
 
 End desciption.
 
 The following is reference information.
 
 TODO:
 Implement e-stop

 
 ARDUINO PINOUT: 
 8 - Button

 Analog Inputs
 A0 - Lower Arm
 A1 - Upper Arm
 
 Sabertooth Outputs
 Lower Arm is motor A on the Sabertooth
 Upper arm is motor B on the Sabertooth
 
 How to Wire the Potentiometers in the Linear Actuators
 White pot is ground
 Yellow is 5v
 Blue is pot reading
 
 Wire Extension Comparison
 -Blue - Blue
 -White/Blue - White
 -Orange - Yellow
 -Browm - Black
 Green - Red
 
 
 Sabertooth Connections
 M1A - red
 M1B - black
 
 
 Notes: 
 
 default = 2 in
 % = %/twentyeight
 
 motor 1 hardcoded limits  //Might not be accurate
 1 - 3.75
 motor 2 hardcoded limits  //Might not be accurate
 .9 - 3.6
 
 When motor 2 is at default, //Hard, mechanical limits
 motor 1 can vary between 30 and 105 '%' 
 
 When motor 1 is at default,
 motor 2 can vary between 25 and 100  //Hard, mechanical limits
 
 READY BYTE
 This program sends back a ready byte of 'r' once it has proccessed a command
 
 Arm Motor
 -High is opening
 -Low is gripping
 
 */


int led = 13;

unsigned int ADDRESS = 128;  //Address for Sabertooth motor controller

int maxPot = 1009;
int minPot = 75;

int motorSpeed = 20;

float desiredPosition = 2; //Desired position in inches

float motorPosition = 0;

int motor1Pot = 0;
int motor2Pot = 1;

float acceptableError = .05;  //Used to determine if the motor should move
int acceptableErrorMax = 5;  //Number of acceptable errors 


int alternate = 0;  //Temporary code

//PACKET STRUCTURE variables

const int packetSize = 10;
int numPackets = 0;
bool canAcceptPackets = true;
float f  = 0; //Multipurpose float

unsigned char recieveBuffer[packetSize];
unsigned char bufferPos = 0;

//This probably should be an enum...
const int header = 0;
const int command = 1;
const int baseRotVal1 = 2;
const int baseRotVal2 = 3;
const int act1Val1 = 4;
const int act1Val2 = 5;
const int act2Val1 = 6;
const int act2Val2 = 7;
const int checksum = 8;
const int tail = 9;

//// END PACKET

////LIQUID CRYSTAL 
#include <LiquidCrystal.h>
LiquidCrystal lcd(2,3,4,5,6,7);

////END LIQUID

//Hardcoded limits
const float motor1LowerLimit = 0.2;
const float motor1UpperLimit = 3.75;
const float motor2LowerLimit = .05;  //.9
const float motor2UpperLimit = 3.6;

const int motorSlowMult = 5;  //How many times the acceptable error that the motors will half their speedint 

int rotValue = 180;

//Struct to hold the motor information
struct motor {
  motor();
  float desiredPos;
  float motorPos;
  int motorSpeed;
  int enable; //Used to determine whether it has hit acceptable error [5] times
  int acceptableCount;
};

//Default motor values (about half stroke)
motor::motor(){
  desiredPos = 3.5; //Default motor position
  motorPos = 0;
  motorSpeed = 80;
  enable = 1;
  acceptableCount = 0;
}

motor motor1;
motor motor2;


//STEPPER MOTOR VARIABLES
struct stepperData{
  stepperData();
  stepperData(int id); //True (nonzero) id for gripper motor
  int stepPin;
  int dirPin;
  int calButton;

  float absPosition;
  int maxPosition;  //The maximum number of degrees in the program;
  int moveDelay;

  int baseMaxRot;
  int baseMinRot;

  int count;
  bool stepperCalibrated;
  double mult;

  //Used only for second stepper (in gripper)
  int grippingPin;
  int openPin;

  int limitsPin;
  int grabPin;
  int enabled;

  int currentState;
  int previousState;  //1 represents gripped, 0 represents opened
  int init;
};

//Setup struct with default valuess
stepperData::stepperData(){
  stepPin = 11;
  dirPin = 12;
  calButton = 8;
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(calButton, INPUT);

  absPosition = 360;
  maxPosition = 360;
  moveDelay = 1000; //Delay between step instructions (in MICROSECONDS)

  baseMaxRot = 330;  //Ensure the base can intially calibrate correctly (ADJUSTMENT NEEDED)
  baseMinRot = 30;   //These variables provide hard coded limits to the stepper rotation

  count = 0;  // Count variable used for calibration
  stepperCalibrated = false;
  mult = 0;  //The converter from steps to degrees
}

stepperData stepper;  //Stepper 1 (the base)

//This constructor will only be called if there is an id supplied to the struct
//And because there are only two motors, the first has no id, and the second is non-zero
stepperData::stepperData(int id){
  if(id){  
    init = 1;
    stepPin = 22;
    dirPin = 40;
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

    limitsPin = 36;
    grabPin = 38;
    pinMode(limitsPin, INPUT);
    pinMode(grabPin, INPUT);

    moveDelay = 250;  //Sufficient at 250

      //Switches
    grippingPin = 26; //Detects that the gripper has an object
    openPin = 28;  //Detects both ends of movement

    enabled = 1;  //Begin enabled
    currentState = 0;  //Begin opened
    previousState = 0;  //Begin opened
  }
}

stepperData stepper2(2);  //Stepper 2 (the gripper)

/*
int mStep = 11;
 int mDir = 12;
 int calButton = 8; 
 
 float absStepperPosition = 360;
 
 int moveDelay = 50; //Delay between step instructions (in MICROSECONDS)
 int count = 0;  // Count variable used for calibration
 bool stepperCalibrated = false;
 double stepperMultiplier = 0;  //The converter from steps to degrees
 */


//Variables for testing
int testMin = 1000; //Not necessary
int testMax = 0;  //Will be changed to match max  //Not necessary

float twentyeight = 28;

// the setup routine runs once when you press reset:
void setup() {
  lcd.begin(16,2);
  lcd.clear();
  lcd.print("Init");

  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial1.write(170);
  sendCommand(14,10);  //Set a serial timeout delay of 1 second


  //Motor 1 = lower act
  //Motor 2 = upper act
  //Moves the arm to a high position initially
  motor1.desiredPos = 3.7;  //extend motor 1 by default
  motor2.desiredPos = 3;  //Retract motor 2 by default
  verifyLimits();
  motorEnable(1);
  motorEnable(2);
  moveMotors();

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Step Calibrate");
  calibrateStepper();  //Calibrates the stepper initially
  rotateBase(180);
}

// the loop routine runs over and over again forever:
void loop() {



  if(Serial.available()){
    recieveBuffer[bufferPos] = Serial.read();
    ++bufferPos;

    if(bufferPos == packetSize){  //We have read a complete packet
      lcd.setCursor(0,0);
      lcd.write("Read packet");
      digitalWrite(13, LOW);
      canAcceptPackets = false; //Enables the return packet
      bufferPos = 0;  //Reset the buffer position
      ++numPackets;
      if(recieveBuffer[checksum] == (recieveBuffer[command] ^ recieveBuffer[baseRotVal1] ^ recieveBuffer[baseRotVal2] ^ recieveBuffer[act1Val1] ^ recieveBuffer[act1Val2] ^ recieveBuffer[act2Val1] ^ recieveBuffer[act2Val2])){
        motor1.desiredPos = (recieveBuffer[act1Val1] + recieveBuffer[act1Val2]) / 100.0;
        motor2.desiredPos = (recieveBuffer[act2Val1] + recieveBuffer[act2Val2]) / 100.0;
        rotValue = (recieveBuffer[baseRotVal1] + recieveBuffer[baseRotVal2]);

        lcd.setCursor(5,1);
        lcd.print(motor1.desiredPos);
        lcd.setCursor(2,1);

        if(recieveBuffer[command] & 2){  //Check for a grip command
          lcd.print("G");
          if(stepper2.currentState != 1){
            stepper2.currentState = 1;
            enableGrip();
          }

        }
        else { 
          lcd.print("O");
          if(stepper2.currentState != 0){ 
            stepper2.currentState = 0;
            enableGrip();
          }
        }
        delay(1000);
        motorEnable(1);
        motorEnable(2);
        //Move to after moveMotors()
        rotateBase(rotValue);  //Rotates the base
      } 
      else{  //Checksum failed
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Checksum failed!");
        delay(5000);
      }     
    }

  } 




  verifyLimits();  //Checks to ensure that the motor values aren't passing the hardcoded stopping points
  //Essentially functions as a coerce for both actuators and the stepper (eventually)
  moveMotors();

  processGrip();  //Checks if a grip or open command has been issued, and if it has, proccess it.
   

  if(!canAcceptPackets){ //Handles sending a return packed, and opens the program up to more commands
    Serial.flush();  //Probably not necessary, but it doesn't hurt 
    Serial.write('r');
    digitalWrite(13, HIGH);  //Turns on the LED on the mega
    canAcceptPackets = true;
  }

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Waiting");
  lcd.setCursor(10,0);
  lcd.print(motor1.motorPos);
  lcd.setCursor(10,1);
  //  lcd.print(motor1.desiredPos);
  lcd.print(rotValue);
  lcd.setCursor(0,1);
  lcd.print(numPackets);
  delay(10);



}


//Moves the linear actuators
//This function is a main function that calls other commands to move the linear actuators
void moveMotors(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Moving");
  while(!(motor1.acceptableCount >= acceptableErrorMax && motor2.acceptableCount >= acceptableErrorMax)){  //Move untill both motors are withing acceptable errors
    sendCommand(chkMotorDir(1),chkMotorSpeed(1));  //Move the lower arm to the correct position
    sendCommand(chkMotorDir(2),chkMotorSpeed(2));  //Move the upper arm to the correct position
    checkPosition();
    delay(50);  //INNACCURATE
  }

}

//This fucntion acts as a coerce for the values passed to the motors
//This function limits the values as to the variables that are declared in the header
int verifyLimits(){
  if(motor1.desiredPos > motor1UpperLimit)
    motor1.desiredPos = motor1UpperLimit;
  if(motor1.desiredPos < motor1LowerLimit)
    motor1.desiredPos = motor1LowerLimit;
  if(motor2.desiredPos > motor2UpperLimit)
    motor2.desiredPos = motor2UpperLimit;
  if(motor2.desiredPos < motor2LowerLimit)
    motor2.desiredPos = motor2LowerLimit;
}

//Determines the motor's direction
//Motor options are 1 or 2
int chkMotorDir(int motor){
  if(motor == 1){
    if(motor1.motorPos > motor1.desiredPos)
      return 0;  //Move 'backward'
    else
      return 1;  //Move 'forward'
  }
  if(motor == 2) {
    if(motor2.motorPos > motor2.desiredPos){
      return 5;  //Move 'backward'
    }
    else {
      return 4;  //Move 'forward'
    }
  }
}


//Determines if the motor actually needs to move
int chkMotorSpeed(int motor){
  if(motor == 1){
    if(abs(motor1.motorPos - motor1.desiredPos) > acceptableError && motor1.acceptableCount < acceptableErrorMax){
      motor1.acceptableCount = 0;
      if(abs(motor1.motorPos - motor1.desiredPos) < (acceptableError*motorSlowMult))
        return motor1.motorSpeed/2;
      else
        return motor1.motorSpeed;
    }
    else {
      ////Serial.print(" Acceptable Error ");
      ++motor1.acceptableCount;
      return 0;
    }
  }
  else if(motor == 2){
    if(abs(motor2.motorPos - motor2.desiredPos) > acceptableError && motor2.acceptableCount < acceptableErrorMax){
      motor2.acceptableCount = 0;
      if(abs(motor2.motorPos - motor2.desiredPos) < (acceptableError*motorSlowMult))
        return motor2.motorSpeed/2;
      else
        return motor2.motorSpeed;
    }
    else {
      //      //Serial.println("Acceptable Error");
      ++motor2.acceptableCount;
      return 0;
    }
  }
}


//Enables the linear actuators, so that they can move
void motorEnable(int motor){
  if(motor == 1)
    motor1.acceptableCount = 0;
  if(motor == 2)
    motor2.acceptableCount = 0;   
}

//Checks the linear actuator's current posistion (essentially reads and processes the potentiometers)
void checkPosition(){
  motor1.motorPos = readPot(1) / 248; //248 is bits per inch
  motor2.motorPos = readPot(2) / 248; 

  //Debugging
  lcd.setCursor(10,0);
  lcd.print(motor1.motorPos);
  lcd.setCursor(10,1);
  lcd.print(motor1.desiredPos);
}

//Sends a command to the Sabertooth Motor Controller
void sendCommand(unsigned int command, unsigned int data){
  Serial1.write(ADDRESS);
  Serial1.write(command);
  Serial1.write(data);
  Serial1.write(sabertoothChecksum(command, data));
}

//Calculates the checksm that is required to use the Sabertooth
int sabertoothChecksum(unsigned int command, unsigned int data){
  int sum = ADDRESS + command + data;
  return sum & 127;
}

//Reads the analog sensor over 5 ms in an attempt to average values a little bit
float readPot(int motor){
  int sum = 0;
  if(motor == 1){  
    for(int i = 0; i < 5; ++i){
      sum += analogRead(motor1Pot);
      delay(1); //An attempt to average the analog feedback
    }
  }
  else if(motor == 2){
    for(int i = 0; i < 5; ++i){
      sum += analogRead(1);
      delay(1);
    }
  }
  return sum/5;
}


//FUNCTIONS FOR STEPPER MOTOR 1 (BASE) 

//This function is used by the interface (absolute desured position)
int rotateBase(int desiredPosition){
  if(desiredPosition < stepper.baseMinRot)   //Input Protection
    desiredPosition = stepper.baseMinRot;
  if(desiredPosition > stepper.baseMaxRot)
    desiredPosition = stepper.baseMaxRot;

  moveBase(desiredPosition - stepper.absPosition);
  stepper.absPosition = stepper.absPosition + (desiredPosition-stepper.absPosition);
}

//Moves the base a certian number of degrees
//HELPER FUNCTION (this function is only used by another funtion)
int moveBase(int degreesToMove){
  float steps = degreesToMove * stepper.mult;

  ////Serial.println(steps); //Debugging

  if(degreesToMove < 0){
    digitalWrite(stepper.dirPin, LOW);
  }
  else {
    digitalWrite(stepper.dirPin, HIGH);
  }
  steps = abs(steps);
  for(int i = 0; i < steps+1; ++i){  //+1 because we want to move the exact amount of steps (not 1 less)
    //////Serial.println("Step");
    digitalWrite(stepper.stepPin, HIGH);
    delayMicroseconds(stepper.moveDelay);
    digitalWrite(stepper.stepPin, LOW);
    delayMicroseconds(stepper.moveDelay);
  }
}

//Calibrates the stepper motor
//This function rotates the arm clockwise until it recieves an input from a switch, then rotates counter-clockwise
//This sweep is used to calulate what 360 degrees is, or at least what it will perceve 360 degrees to be. 
int calibrateStepper(){

  bool calInProcess = true;
  bool calFirstPress = false;
  bool calSecondPress = false;
  int calButtonState;

  int stepCount = 0; //Counts the number of steps in a 360 degree turn

  digitalWrite(stepper.dirPin, LOW); //Reverses first (this should turn the base clockwise)

  while(calInProcess){
    calButtonState = digitalRead(stepper.calButton);
    if(calButtonState == HIGH && calFirstPress == false){
      calFirstPress = true;
      digitalWrite(stepper.dirPin, HIGH); //This should turn the motor counter-clockwise
      delay(100); //Makes the stepper somewhat more graceful
    }

    if(calFirstPress == true)
      ++stepCount;

    if(calFirstPress == true && stepCount > 150 && calButtonState == HIGH) //Middle statement debounces the switch
      calInProcess = false;

    //Serial.println(stepCount);

    digitalWrite(stepper.stepPin, HIGH);
    delay(1);
    digitalWrite(stepper.stepPin, LOW);
    delay(2.5);
  }
  //Determine degrees / step multiplier
  stepper.mult = stepCount / stepper.maxPosition;
  //Serial.println(stepper.mult);
  return 0;
}


//FUNCTIONS FOR STEPPER 2 (Gripper)

//Enables the gripper motor
int enableGrip(){
  stepper2.enabled = 1;
}


//Processes the grip functionality
void processGrip(){
  
  lcd.setCursor(0,0);
   lcd.print("Stepping_");
   
   
   
   if(stepper2.enabled){
   
   //Set the dirction of the stepper
   if (!stepper2.currentState){  //If grip command
     digitalWrite(stepper2.dirPin, HIGH);
     alternate = 0;  //Not necessary
     //lcd.setCursor(9,0);
     //lcd.print("Release");
   }
   else {  //If open command
     alternate = 1;  //Not necessary
     digitalWrite(stepper2.dirPin, LOW);
     //lcd.setCursor(10,0);
     //lcd.print("Grip");
   }
   
   //Force the stepper to move beyond the throw of the switch
   //This portion is designed such that the motor doesn't isn't forced for the first command
   //TODO: Check and see if this will nullify the first command sent
   //TODO: Implement the moveDelay in the stepper2 struct
   if(!stepper2.init){
   
    for(int i = 0; i < 1500; ++i){   //Forces the gripper past the throw of the switch
      digitalWrite(stepper2.stepPin, HIGH);
      delayMicroseconds(250);
      //delayMicroseconds(stepper2.moveDelay);
      digitalWrite(stepper2.stepPin, LOW);
      delayMicroseconds(250);
      //delayMicroseconds(stepper2.moveDelay);
      
      lcd.setCursor(0,1);
      lcd.print((!digitalRead(stepper2.limitsPin) && digitalRead(stepper2.grabPin)));
    }
   
   }
   else
     stepper2.init = 0;
   
   
   
   int increment = 0;  //Debugging
   
   while(!digitalRead(stepper2.limitsPin) && digitalRead(stepper2.grabPin)){
     digitalWrite(stepper2.stepPin, HIGH);
     delayMicroseconds(200);
     //delayMicroseconds(stepper2.moveDelay);
     digitalWrite(stepper2.stepPin, LOW);
     delayMicroseconds(200);
     //delayMicroseconds(stepper2.moveDelay);    
   lcd.setCursor(0,1);
   lcd.print((!digitalRead(stepper2.limitsPin) && digitalRead(stepper2.grabPin)));
   
     ++increment;
   }
   
   stepper2.enabled = 0;

   }
}
