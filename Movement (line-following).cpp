/* Initial set up */
/* define pins for motors */ 
/* checked */
#define AIN1 8
#define AIN2 12
#define PWMA 5
#define BIN1 7
#define BIN2 6
#define PWMB 11

/* define pins for ultrasensor*/
/* checked */
#define triggerPin = 2;
#define echoPin = 4;

float duration, distance;

/* define pins for on-off switch */
int outPin = ___;

/* define pins for IR sensor*/
/* checked */
#define leftIR A2
#define rightIR A3

int white = 0;
int black = 1;

/* define reference variables */
/* checked */
double refTime;
int refBlackzone = 0;

/* define graph data variables*/
int graphnum = 0;

/* define variables to store accelerometer & gyroscope data*/
/* checked */
double previuseTime;

double previuseAccelX=0,previuseAccelY=0,previuseAccelZ=0;
double prevVelocityX=0,prevVelocityY=0,prevVelocityZ=0;
float prevCoordinateX=0,prevCoordinateY=0,prevCoordinateZ=0;
float prevAngleX=0,prevAngleY=0,prevAngleZ=0;

float VAngleX=0,VAngleY=0,VAngleZ=0;
float AccX=0,AccY=0,AccZ=0;

int i=1;

float AngleX, AngleY, AngleZ;
double CoordinateX,CoordinateY,CoordinateZ;

float initialAngle=0;
float autoAngle=0;

/* define variables to store IR sensor data*/
/* checked */
int leftIRvaluetotal = 0;
int rightIRvaluetotal = 0;
int leftIRvalue = 0;
int rightIRvalue = 0;

/* define variables to store color sensor data*/
float red = 0;
float green = 0;
float blue = 0;
float clearcolor = 0;

/* import accelerometer libraries*/
/* checked */
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

/* import light sensor libraries*/
/* checked */
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* import servo libraries*/
/* checked */
#include <Servo.h>
Servo myservo;

/* import ultrasensor libraries*/
/* checked */
#include <HCSR04.h>

/* import graph plotter libraries*/
#include <EEPROM.h>
#include <MegunoLink.h>



/* Setups for on-off switch */
void onoffswitch () {
	digitalWrite(outPin, HIGH);
	if (digitalRead(inPin) == LOW) {
		Serial.println(“The button is now OFF”);
while (1) {
			if (digitalRead(inPin) == HIGH) {
				break;
			}
		}
	}
	
	Serial.println(“The button is now ON”);
}
/* Setups for measurement */
/* calculate coordinate based on acceleration & time, calculate angle based on angular velocity & time */
/* checked */
/* need to check the unit for coordinate (should be m? - then maybe fixAcceleration has a too large range) & angle (should be rotation rate? - if radian, need to change angle conversion unit)*/
/* need to check VAngleX/Y/Z - I didn’t define them above, but I guess they are just dummy variables for me to put stuff inside? Just like currCoordinateX/Y/Z? */
void getCoorAngle(){
double currentTime;
currentTime=(millis()-previuseTime)/1000;
previuseTime=millis();

/*obtain data for calculation */
/* checked */
sensors_event_t acc, gyr, temp;
mpu.getEvent(&acc, &gyr, &temp);
VAngleX = gyr.gyro.x;
VAngleY = gyr.gyro.y;
VAngleZ = gyr.gyro.z;

AccX=acc.acceleration.x;
AccY=acc.acceleration.y;
AccZ=acc.acceleration.z;

/*remove data noise for calculation */
if (VAngleX < 1) {
	VAngleX = 0;
}

if (VAngleY < 1) {
	VAngleY = 0;
}

if (VAngleZ < 1) {
	VAngleZ = 0;
}

if (AccX < 0.02) {
	AccX= 0;
}

if (AccY < 0.02) {
	AccY= 0;
}

if (AccZ < 0.02) {
	AccZ= 0;
}

/*calculate X coordinate*/
CoordinateX=prevCoordinateX+(prevVelocityX*currentTime)+(AccX*currentTime*currentTime*0.5);
prevVelocityX=prevVelocityX+(AccX*currentTime);
prevCoordinateX=CoordinateX;

/*calculate Y coordinate*/
CoordinateY=prevCoordinateY+(prevVelocityY*currentTime)+(AccY*currentTime*currentTime*0.5);
prevVelocityY=prevVelocityY+(AccY*currentTime);
prevCoordinateY=CoordinateY;

/*calculate Z coordinate*/
CoordinateZ=prevCoordinateZ+(prevVelocityZ*currentTime)+(AccZ*currentTime*currentTime*0.5);
  	prevVelocityZ=prevVelocityZ+(AccelZ*currentTime);
  	prevCoordinateZ=CoordinateZ;

	/*calculate X-axis angle*/
AngleX=prevAngleX+(VAngleX*currentTime); 
 /* when angle has rotated for 1 cycle -> reset */
if (AngleX > 360) {
AngleX = AngleX - 360;
}

else if (AngleX < 0) {
AngleX = AngleX + 360;
}
	
prevAngleX = AngleX;

	/*calculate Y-axis angle*/
AngleY=prevAngleY+(VAngleY*currentTime*180/3.14159);
if (AngleY > 360) {
AngleY = AngleY - 360;
}

else if (AngleY < 0) {
AngleY = AngleY + 360;
}
	
prevAngleY = AngleY;

/*calculate Z-axis angle*/
AngleZ=prevAngleZ+(VAngleZ*currentTime*180/3.14159);
AngleZ = AngleZ - 360;

if (AngleZ > 360) {
AngleZ = AngleZ - 360;
}

else if (AngleZ < 0) {
AngleZ = AngleZ + 360;
}
	
prevAngleZ = AngleZ;

	/*print coordinates*/
	Serial.print("Coordinate X: ");
 Serial.print(CoordinateX*1000);
 Serial.print(", Y: ");
 Serial.print(CoordinateY*1000);
 Serial.print(", Z: ");
 Serial.println(CoordinateZ*1000);

	Serial.print("Velocity X: ");
Serial.print(prevVelocityX*1000);
Serial.print(", Y: ");
Serial.print(prevVelocityY*1000);
Serial.print(", Z: ");
Serial.println(prevVelocityZ*1000);

Serial.print("Acceleration X: ");
Serial.print(AccX*1000);
Serial.print(", Y: ");
Serial.print(AccY*1000);
Serial.print(", Z: ");
Serial.println(AccZ*1000);

/*print angles*/
Serial.print("Angle X: ");
Serial.print(AngleX);
Serial.print(", Y: ");
Serial.print(AngleY);
Serial.print(", Z: ");
Serial.println(AngleZ);

Serial.print("Angular speed X: ");
Serial.print(VAngleX);
Serial.print(", Y: ");
Serial.print(VAngleY);
Serial.print(", Z: ");
Serial.println(VAngleZ);
}

/*obtain light sensor data */
void getcolordata(){
	/*obtain light sensor data */
	uint16_t r, g, b, c, colorTemp, lux;
tcs.getRawData(&r, &g, &b, &c);
colorTemp = tcs.calculateColorTemperature(r, g, b);
lux = tcs.calculateLux(r, g, b);
red = r;
green = g;
blue = b;
clearcolor = c;
/*print light sensor data */
Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
Serial.println(" ");
}

/*obtain distance data for obstacle*/
void getobstacledistancedata(){
	digitalWrite(triggerPin, LOW);
delay(2);
digitalWrite(triggerPin, HIGH);
delay(10);
digitalWrite(triggerPin, LOW);

duration = pulseIn(echoPin, HIGH);
distance = (duration*.0343)/2;
Serial.print("Distance: ");
Serial.println(distance);
delay(100);
}

/*obtain IR sensor data */
void getIRsensordata() {
	/*reset total IR value */
	leftIRvaluetotal = 0;
	rightIRvaluetotal = 0;

	/*obtain IR sensor data */
for (int j = 0; j < 5; j++) {
	leftIRvaluetotal += analogRead(leftIR);
		rightIRvaluetotal += analogRead(rightIR);
	}
	
	/*take average of IR sensor data */
	leftIRvalue = leftIRvaluetotal / 5;
	rightIRvalue = rightIRvaluetotal / 5;

/*print result */	
Serial.print("\nLeft Analogue:");
Serial.print(leftIRvalue );
Serial.print("\nRight Analogue:");
Serial.println(rightIRvalue);

/*turn IR sensor data into digital*/
	if (leftIRvalue > 600) {
		leftIRvalue = black;
	}

else {
		leftIRvalue = white;
	}

	if (rightIRvalue > 600) {
		rightIRvalue = black;
	}

else {
		rightIRvalue = white;
	}

	Serial.print("\nLeft Digital:");
Serial.print(leftIRvalue );
Serial.print("\nRight Digital:");
Serial.println(rightIRvalue);
}

/* Setups for plotting graphs */
void EEPROM.clear() {
for (int i = 0; i < 512; i++) {
   		EEPROM.write(i, 0);
	}
}

/* store coordinates in EEPROM */
/* give up including the token we might get along the black line */
void storegraphdata() {
	/* turn double into integer */
	int graphx = CoordinateX;
	int graphy = CoordinateY;
	
EEPROM.put(graphnum, graphx);
EEPROM.put(graphnum + 1, graphy);
	
	/* indicate coordinate is stored */
	Serial.println(“One set of coordinate is stored”)

	/* increase plot number counter */
	graphnum += 2;
}

/* Setups for movement */
/* motor stops moving*/
void motorstop(){
digitalWrite(AIN1,HIGH);
digitalWrite(AIN2,HIGH);
analogWrite(PWMA,0);
digitalWrite(BIN1,HIGH);
digitalWrite(BIN2,HIGH);
analogWrite(PWMB,0);
}

/* motor moves forward fast */
void motorforwardfast(){
	
/* measure location*/
getCoorAngle();

/* motor moves forward */
digitalWrite(AIN1,HIGH);
digitalWrite(AIN2,LOW);
analogWrite(PWMA,200);
digitalWrite(BIN1,HIGH);
digitalWrite(BIN2,LOW);
analogWrite(PWMB,235);
}

/* motor moves backward fast*/
void motorbackwardslow(){

	/* measure location*/
getCoorAngle();

/* motor moves backward*/
digitalWrite(AIN1,LOW);
digitalWrite(AIN2,HIGH);
analogWrite(PWMA,200);
digitalWrite(BIN1,LOW);
digitalWrite(BIN2,HIGH);
analogWrite(PWMB,235);
}

/* motor moves forward slow */
void motorforwardslow(){
	
/* measure location*/
getCoorAngle();

/* motor moves forward  */
digitalWrite(AIN1,HIGH);
digitalWrite(AIN2,LOW);
analogWrite(PWMA,100);
digitalWrite(BIN1,HIGH);
digitalWrite(BIN2,LOW);
analogWrite(PWMB,118);
}

/* motor moves backward slow */
void motorbackwardfast(){

	/* measure location*/
getCoorAngle();

/* motor moves backward*/
digitalWrite(AIN1,LOW);
digitalWrite(AIN2,HIGH);
analogWrite(PWMA,100);
digitalWrite(BIN1,LOW);
digitalWrite(BIN2,HIGH);
analogWrite(PWMB,118);
}

/* motor turns anticlockwise*/
/* switch motor rotating direction to make the right side move backward and left side move forward, so as to turn anticlockwise*/
void motoranticlockwise(){
	/* measure location*/
getCoorAngle( );

/* motor turns anticlockwise*/
digitalWrite(AIN1,LOW);
digitalWrite(AIN2,HIGH); 
analogWrite(PWMA,255);
digitalWrite(BIN1,HIGH);
digitalWrite(BIN2,LOW);
analogWrite(PWMB, 255);
}

/* motor turns clockwise*/
/* switch motor rotating direction to make the right side move forward and left side move backward, so as to turn clockwise*/
void motorclockwise(){
	/* measure location*/
getCoorAngle();

/* motor turns clockwise*/
digitalWrite(AIN1,HIGH);
digitalWrite(AIN2,LOW); 
analogWrite(PWMA,255);
digitalWrite(BIN1,LOW);
digitalWrite(BIN2,HIGH);
analogWrite(PWMB,255);
}

/* motor turns left and moves forward a bit - anchor on left wheel*/
void motorleftforward(){
	/* measure location*/
getCoorAngle();

/* motor turns left*/
digitalWrite(AIN1,LOW);
digitalWrite(AIN2,HIGH); 
analogWrite(PWMA,100);
digitalWrite(BIN1,HIGH);
digitalWrite(BIN2,LOW);
analogWrite(PWMB,255);
}

/* motor turns right and moves forward a bit - anchor on right wheel*/
void motorrightforward(){
	/* measure location*/
getCoorAngle();

/* motor turns right*/
digitalWrite(AIN1,HIGH);
digitalWrite(AIN2,LOW); 
analogWrite(PWMA,255);
digitalWrite(BIN1,LOW);
digitalWrite(BIN2,HIGH);
analogWrite(PWMB,100);
}

/* motor turns left and moves backward a bit - anchor on right wheel*/
void motorleftbackward(){

	/* measure location*/
getCoorAngle();

/* motor turns right*/
digitalWrite(AIN1,HIGH);
digitalWrite(AIN2,LOW); 
analogWrite(PWMA,100);
digitalWrite(BIN1,LOW);
digitalWrite(BIN2,HIGH);
analogWrite(PWMB,255);

}

/* motor turns right and moves backwarda bit - anchor on right wheel*/
void motorrightbackward(){

	/* measure location*/
getCoorAngle();

/* motor turns left*/
digitalWrite(AIN1,LOW);
digitalWrite(AIN2,HIGH); 
analogWrite(PWMA,255);
digitalWrite(BIN1,HIGH);
digitalWrite(BIN2,LOW);
analogWrite(PWMB,100);

}

/* motor follows black line, moving forward*/
void motorfollowlineforward(){
	/* measure whether we are on the line*/
getIRsensordata();

/* change motor movement to follow the line */
/* too left -> turn right */
if(leftIRvalue == black&& rightIRvalue == white) {
	motorclockwise();
	delay(100);
}

/* too right -> turn left*/
else if(leftIRvalue == white&& rightIRvalue == black) {
	motoranticlockwise();
	delay(100);
}

/* on the line -> go straight */
else if(leftIRvalue == black && rightIRvalue == black) {
	motorforwardslow();
}

else if(leftIRvalue == white && rightIRvalue == white) {
	motorforwardslow();
}

}

/* motor follows black line, moving backward*/
void motorfollowlinebackward(){
	/* measure whether we are on the line*/
getIRsensordata();

/* change motor movement to follow the line */
/* too left -> turn right */
if(leftIRvalue == black&& rightIRvalue == white) {
	motorclockwise();
delay(100);
}

/* too right -> turn left*/
else if(leftIRvalue == white&& rightIRvalue == black) {
	motoranticlockwise();
	delay(100);
}

/* on the line -> go straight */
else if(leftIRvalue == black && rightIRvalue == black) {
	motorbackwardslow();
}

else if(leftIRvalue == white && rightIRvalue == white) {
	motorbackwardslow();
}

}

/* grabber opens */
void servoopen() {
	myservo.write(30);
}

/* grabber leaves a bit gap to allows possible blocks in front to enter*/
void servopartiallyclose() {
	myservo.write(60);
}

/* grabber closes to grab*/
void servofullyclose() {
	myservo.write(95);
}

/* Setups for preventing collision */
/*stop for a while if seeing opponents in the front */
/* need to check distance of opponent */
/* need to check how long we should stop */
void preventcollision() {	
	/*measure distance of opponents */
	getobstacledistancedata();
	
	/*stop for a while if seeing opponents in the front */
	if (distance < 1000) {
		/*increase reference time for time-dependent movement during the delay */
/* for non time-dependent movement, the change in refTime has no effect on its movement, and the refTime will be reset everytime before each time dependent movement so it doesn’t matter */
		refTime += 3000;

/*reset accelerometer & gyrosope timer so that the noise acceleration/velocity won’t be amplified by 3000*/
previuseTime=millis();

		/*stop moving for 3 seconds */
		motorstop();
		delay(3000);
	}
}

/* Void setup */
void setup() {
	/* INITIALIZATION */
/* initialize communication */
/* checked */
	Serial.begin(9600); 
	while (!Serial)
 		delay(10);
	Serial.println("Serial console is opened");

/* initialize token grabber (servo)*/
	/* checked */
  myservo.attach(3); 
  myservo.write(30);

/* initialize ultrasensor*/
/* checked */
pinMode(triggerPin, OUTPUT);
pinMode(echoPin, INPUT);
	
	/* initialize IR sensor*/
pinMode(leftIR, INPUT);
pinMode(rightIR, INPUT);	


/* initialize wheels (motor)*/
/* checked */
pinMode(AIN1, OUTPUT);
pinMode(AIN2, OUTPUT);
pinMode(PWMA, OUTPUT);
pinMode(BIN1, OUTPUT);
pinMode(BIN2, OUTPUT);
pinMode(PWMB, OUTPUT);

/* initialize on-off switch*/
pinMode(inPin, INPUT);
pinMode(outPIN, OUTPUT);

/* initialize MPU6050 - accelerometer and gyroscope*/
/* checked */
if (!mpu.begin()) {
Serial.println("Failed to find MPU6050 chip");
while (1) {
  	delay(10);
}
}
Serial.println("Accelerometer and gyroscope found!");

mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
mpu.setGyroRange(MPU6050_RANGE_500_DEG);
mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

/* initialize TCS34725 - color sensor*/
Adafruit_TCS34725 tcs = Adafruit_TCS34725();

if (tcs.begin()) {
Serial.println("Found sensor");
} else {
Serial.println("No TCS34725 found ... check your connections");
while (1);
}
	
/* reset EEPROM memory for the next operation */
EEPROM.clear();

/*START UNTIL THE BUTTON IS ON*/
onoffswitch();
}
/* Void loop */
void loop(){
/* FIRST TIME MOVING FORWARD */
	/*store initial position in graph as reference */
storegraphdata();

/*reset accelerometer & gyrosope timer*/
previuseTime=millis();

/*set reference time as now*/
refTime = millis();

/* set initial angle location */
initialAngle=AngleZ;

/* bot moves forward until reaching the middle */
while(1) {
/* motor moves forward */
motorforwardfast();
		
		/* measure line data */
		getIRsensordata();

/*start detecting if it enters the white zone when it moves for a certain time */
/* need to check what’s the best time */
if (((millis()-refTime) > 6000) && (leftIRvalue == white) && (rightIRvalue == white)) {
	refBlackzone = 1;
}

/*start detecting if it enters the middle zone */
if ((refBlackzone = 1) && (rightIRvalue == white)) {
	/* stop moving */
motorstop();
delay(500);

	/*quit loop */
	break;
}
}

/* grab token */
servofullyclose();

/*reset accelerometer & gyrosope timer*/
previuseTime=millis();

/*reset reference time as now*/
refTime = millis();

/* bot rotates anticlockwise until rotated 180 degrees - somewhat facing our scoring zone*/
while(1) {
/* motor rotates anticlockwise*/
motoranticlockwise();
		
/*stop rotating when certain time has passed - assumed facing out scoring zone*/
/* need to check what’s the best time */
if ((millis()-refTime) > 2000) {
	/* stop moving */
motorstop();
delay(500);

	/*quit loop */
	break;
}
}

/*endless loop to keep on going to the opponent’s side to sweep away their tokens */
while (1) {
/*grabber opens partially to leave gap*/
servopartiallyclose();

/*reset accelerometer & gyrosope timer*/
previuseTime=millis();

/* bot follows black line until detecting blue color - i.e. reaching scoring area*/
while(1) {
/* stop for a while if seeing opponents in the front */
preventcollision();

/* motor follow lines, moving forward*/
motorfollowlineforward();

/* measure color data */
getcolordata();


/* stop moving when detect blue color */
/* need to check data */
if((red<50)&&(red>0)&&(green<50)&&(green>0)&&(blue<255)&&(blue>0)&&(clearcolor<50)&&(clearcolor>0)){
/* stop rotating*/
motorstop();
delay(500);

/* quit program */
break;
}
		}

	/*grabber opens fully to release token */
	servoopen();

	/*reset accelerometer & gyrosope timer*/
previuseTime=millis();

	/*set reference time as now*/
refTime = millis();

/* bot follows black line to move backward for certain time, until the token is no longer in the grabber */
/* no need to prevent collision as it can’t detect opponents backward anyways */
while(1) {
/* motor follow lines, moving forward*/
motorfollowlinebackward();
		
/*stop moving when token is no longer in the grabber, assume 6 seconds, timing should be similar to the time needed to enter the white zone*/
/* need to check what’s the best time */
if ((millis()-refTime) > 6000) {
	/* stop moving */
motorstop();
delay(500);

	/*quit loop */
	break;
}
}

/*reset accelerometer & gyrosope timer*/
previuseTime=millis();

/*set reference time as now*/
refTime = millis();

/* bot rotates anticlockwise until rotated 180 degrees - somewhat facing opponent’s scoring zone*/
while(1) {
/* stop for a while if seeing opponents in the front */
preventcollision();

/* motor rotates anticlockwise*/
motoranticlockwise();
		
/*stop rotating when certain time has passed - assumed facing opponent’s scoring zone*/
/* need to check what’s the best time */
if ((millis()-refTime) > 2000) {
	/* stop moving */
motorstop();
delay(500);

	/*quit loop */
	break;
}
}

/*reset accelerometer & gyrosope timer*/
previuseTime=millis();

/* bot follows black line to move forward until reaching opponent’s scoring zone*/
while(1) {
/* stop for a while if seeing opponents in the front */
preventcollision();

/* motor follow lines, moving forward*/
motorfollowlineforward();
		
/* stop moving when detect blue color */
/* need to check data */
if((red<50)&&(red>0)&&(green<50)&&(green>0)&&(blue<255)&&(blue>0)&&(clearcolor<50)&&(clearcolor>0)){
/* stop rotating*/
motorstop();
delay(500);

/* quit program */
break;
}
		}

	/*grabber closes fully to grab token */
	servofullyclose();

/*set reference time as now*/
refTime = millis();

/*reset accelerometer & gyrosope timer*/
previuseTime=millis();

/* bot rotates anticlockwise until rotated 180 degrees - somewhat facing our scoring zone*/
while(1) {
/* stop for a while if seeing opponents in the front */
preventcollision();

/* motor rotates anticlockwise*/
motoranticlockwise();
		
/*stop rotating when certain time has passed - assumed facing opponent’s scoring zone*/
/* need to check what’s the best time */
if ((millis()-refTime) > 2000) {
	/* stop moving */
motorstop();
delay(500);

	/*quit loop */
	break;
}
}
	}

}
