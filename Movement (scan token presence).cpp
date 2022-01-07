/* Initial set up */
/* define pins for motors */ 
/* checked */
#define AIN1 8
#define AIN2 9
#define PWMA 10
#define BIN1 7
#define BIN2 6
#define PWMB 11

/* define pins for ultrasensor*/
/* checked */
const int triggerPin = 2;
const int echoPin = 4;

float duration, distance;

/* define variables to store accelerometer & gyroscope data*/
/* checked */
float previuseAccelX=0,previuseAccelY=0,previuseAccelZ=0;
double previuseTime;
double prevVelocityX=0,prevVelocityY=0,prevVelocityZ=0;
float prevCoordinateX=0,prevCoordinateY=0,prevCoordinateZ=0;
float prevAngleX=0,prevAngleY=0,prevAngleZ=0;

int i=1;

float AccelX, AccelY,AccelZ;
float AngleX, AngleY, AngleZ;
double CoordinateX,CoordinateY,CoordinateZ;

float initialAngle=0;
float autoAngle=0;

/* import accelerometer libraries*/
/* checked */
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

/* import light sensor libraries*/
/* checked */
#include "Adafruit_TCS34725.h"

/* import sevro libraries*/
/* checked */
#include <Servo.h>
Servo myservo;

/* import ultrasensor libraries*/
/* checked */
#include <HCSR04.h>

/* import libraries for calculating time*/
/* checked */
#include <Chrono.h>

/* initialization */
void setup() {
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
pinMode(triggerPin, OUTPUT);curr
pinMode(echoPin, INPUT);

/* initialize wheels (motor)*/
/* checked */
pinMode(AIN1, OUTPUT);
pinMode(AIN2, OUTPUT);
pinMode(PWMA, OUTPUT);
pinMode(BIN1, OUTPUT);
pinMode(BIN2, OUTPUT);
pinMode(PWMB, OUTPUT);

/* initialize MPU6050 - accelerometer and gyroscope*/
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

}


/* Setups for measurement */
/* fix acceleration as 0 if value is negligible */
void fixAccelaration(float *currAccel){
if ((*currAccel>-0.07)&&(*currAccel<0.07))
    		*currAccel=0;
}

void convertAcceleration(float *AccelX,float *AccelY,float *AccelZ,float currAccelX,float currAccelY,float currAccelZ){
while (i){
previuseAccelX=currAccelX;
previuseAccelY=currAccelY;
previuseAccelZ=currAccelZ;
i=0;
}
*AccelX= currAccelX-previuseAccelX;
previuseAccelX=currAccelX;
*AccelY= currAccelY-previuseAccelY;
previuseAccelY=currAccelY;
*AccelZ= currAccelZ-previuseAccelZ;
previuseAccelZ=currAccelZ;
fixAccelaration(AccelX);  fixAccelaration(AccelY);  fixAccelaration(AccelZ);
}

/* calculate coordinate based on acceleration & time, calculate angle based on angular velocity & time */
/* need to check the unit for coordinate (should be 9.81m? - then maybe fixAcceleration has a too large range) & angle (should be rotation rate? - if radian, need to change angle conversion unit)*/
/* need to check VAngleX/Y/Z - I didn’t define them above, but I guess they are just dummy variables for me to put stuff inside? Just like currCoordinateX/Y/Z? */
void getCoorAngle(double *currCoordinateX,double *currCoordinateY,double *currCoordinateZ,float AccelX,float AccelY,float AccelZ,float *currAngleX,float *currAngleY,float *currAngleZ,float VAnglelX,float VAngleY,float VAngleZ){
double currentTime;
currentTime=(millis()-previuseTime)/1000;
previuseTime=millis();

/*obtain data for calculation */
sensors_event_t acc, gyr, temp;
mpu.getEvent(&acc, &gyr, &temp);
convertAcceleration(&AccelX,&AccelY,&AccelZ,acc.acceleration.x,acc.acceleration.y,acc.acceleration.z)

/*calculate X coordinate*/
*currCoordinateX=prevCoordinateX+(prevVelocityX*currentTime)+(9.81*1000*AccelX*currentTime*currentTime*0.5); // 9.81*1000 is for converting acceleration to ms^-2
prevVelocityX=prevVelocityX+(AccelX*currentTime);
prevCoordinateX=*currCoordinateX;

/*calculate Y coordinate*/
*currCoordinateY=prevCoordinateY+(prevVelocityY*currentTime)+(9.81*1000*AccelY*currentTime*currentTime*0.5);
prevVelocityY=prevVelocityY+(AccelY*currentTime);
prevCoordinateY=*currCoordinateY;

/*calculate Z coordinate*/
*currCoordinateZ=prevCoordinateZ+(prevVelocityZ*currentTime)+(9.81*1000*AccelZ*currentTime*currentTime*0.5);
  	prevVelocityZ=prevVelocityZ+(AccelZ*currentTime);
  	prevCoordinateZ=*currCoordinateZ;

	/*calculate X-axis angle*/
*currAngleX=prevAngleX+(VAngleX*currentTime*90); // 90 is for converting rotation rate to angle in degree
if (*currAngleX > 360) {
*currAngleX = *currAngleX - 360;
} // when angle has rotated for 1 cycle -> reset
else if (*currAngleX < 0) {
*currAngleX = *currAngleX + 360;
}
	prevAngleX = *currAngleX;

	/*calculate Y-axis angle*/
*currAngleY=prevAngleY+(VAngleY*currentTime*90);
if (*currAngleY > 360) {
*currAngleY = *currAngleY - 360;
}
else if (*currAngleY < 0) {
*currAngleY = *currAngleY + 360;
}
	prevAngleY = *currAngleY;

/*calculate Z-axis angle*/
*currAngleZ=prevAngleZ+(VAngleZ*currentTime*90);
if (*currAngleZ > 360) {
*currAngleZ = *currAngleZ - 360;
}
else if (*currAngleZ < 0) {
*currAngleZ = *currAngleZ + 360;
}
	prevAngleZ = *currAngleZ;

	/*print location*/
	Serial.print("Coordinate X: ");
 Serial.print(CoordinateX);
 Serial.print(", Y: ");
 Serial.print(CoordinateY);
 Serial.print(", Z: ");
 Serial.println(CoordinateZ);

 Serial.print("Angle X: ");
 Serial.print(AngleX);
 Serial.print(", Y: ");
 Serial.print(AngleY);
 Serial.print(", Z: ");
 Serial.println(AngleZ);
}

/*obtain light sensor data */
void getcolordata(){
	/*obtain light sensor data */
	uint16_t r, g, b, c, colorTemp, lux;
tcs.getRawData(&r, &g, &b, &c);
colorTemp = tcs.calculateColorTemperature(r, g, b);
lux = tcs.calculateLux(r, g, b);

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
digitalWrite(trigPin, LOW);

duration = pulseIn(echoPin, HIGH);
distance = (duration*.0343)/2;
Serial.print("Distance: ");
Serial.println(distance);
delay(100);
}



/* Setups for safety */
void backtoscoringarea(){
/* ROTATE TO FACE THE MIDPOINT OF THE ARENA*/
/*reset accelerometer & gyrosope timer*/
previuseTime=millis();

/* check if need to face leftward or rightward */
if (CoordinateX<0) {
	autoAngle = 90; // on the left side -> need to turn right
}

else {
	autoAngle = 270; // on the right side -> need to turn left
}

while(1) {
/* motor turns anticlockwise*/
motoranticlockwise();
		
/* need to check the exact rotation data during trial */
if((AngleZ<autoAngle+0.1)&&(AngleZ>autoAngle-0.1)){
		/* stop rotating*/
motorstop();

/* quit loop*/
break;
}

/* wait for a short while */
delay(1000);

	/* MOVE TO MIDPOINT*/
/*reset accelerometer & gyrosope timer*/
previuseTime=millis();
	
	while(1) {
/* motor moves forward*/
motorforward();
		
/* need to check the exact movement data during trial */
if((CoordinateX <0.1)&&(CoordinateX > -0.1)){
		/* stop moving*/
motorstop();

/* quit loop*/
break;
}
	
	/* ROTATE TO FACE THE SCORING LOCATION*/
/*reset accelerometer & gyrosope timer*/
previuseTime=millis();

while(1) {
/* motor rotates clockwise*/
motorclockwise();
		
/* need to check the exact movement data during trial */
if((AngleZ<180.1)&&(AngleZ > 179.9)){
		/* stop moving*/
motorstop();

/* quit loop*/
break;
}

/* wait for a short while */
delay(1000);

	/* MOVE TO SCORING LOCATION*/
/*reset accelerometer & gyrosope timer*/
previuseTime=millis();
	
	while(1) {
/* motor moves forward*/
motorforward();
		
/* need to check the exact movement data during trial */
if((CoordinateY <10)){
		/* stop moving*/
motorstop();

/* quit loop*/
break;
}

}

void safety () {
if((CoordinateX<DATA)||(CoordinateX>DATA)||(CoordinateY<DATA)||(CoordinateY>DATA)){
		/* stop moving*/
motorstop();

/* move back to starting position*/
backtoscoringarea();

/* go back to the very beginning of the function */
goto BEGINNING;
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

/* motor moves forward */
void motorforward(){
	
/* measure location*/
getCoorAngle(&CoordinateX,&CoordinateY,&CoordinateZ,AccelX,AccelY,AccelZ,&AngleX,&AngleY,&AngleZ,gyr.gyro.x,gyr.gyro.y,gyr.gyro.z);

/* motor moves forward */
digitalWrite(AIN1,HIGH);
digitalWrite(AIN2,LOW);
analogWrite(PWMA,100);
digitalWrite(BIN1,HIGH);
digitalWrite(BIN2,LOW);
analogWrite(PWMB,100);

/* prevent any forward motion exceeding the boundary*/
safety();
}

/* motor moves backward*/
void motorbackward(){

	/* measure location*/
getCoorAngle(&CoordinateX,&CoordinateY,&CoordinateZ,AccelX,AccelY,AccelZ,&AngleX,&AngleY,&AngleZ,gyr.gyro.x,gyr.gyro.y,gyr.gyro.z);

/* motor moves backward*/
digitalWrite(AIN1,LOW);
digitalWrite(AIN2,HIGH);
analogWrite(PWMA,100);
digitalWrite(BIN1,LOW);
digitalWrite(BIN2,HIGH);
analogWrite(PWMB,100);

/* prevent any backward motion exceeding the boundary*/
safety();
}

/* motor turns anticlockwise*/
/* switch motor rotating direction to make the right side move backward and left side move forward, so as to turn anticlockwise*/
/* need to check whether A or B is the right one for 90 degree clockwise (assume A is right) */
void motoranticlockwise(){
	/* measure location*/
getCoorAngle(&CoordinateX,&CoordinateY,&CoordinateZ,AccelX,AccelY,AccelZ,&AngleX,&AngleY,&AngleZ,gyr.gyro.x,gyr.gyro.y,gyr.gyro.z);

/* motor turns anticlockwise*/
digitalWrite(AIN1,LOW);
digitalWrite(AIN2,HIGH); 
analogWrite(PWMA,100);
digitalWrite(BIN1,HIGH);
digitalWrite(BIN2,LOW);
analogWrite(PWMB,100);
}

/* motor turns clockwise*/
/* switch motor rotating direction to make the right side move forward and left side move backward, so as to turn clockwise*/
/* need to check whether A or B is the right one for 90 degree clockwise (assume A is right) */
void motorclockwise(){
	/* measure location*/
getCoorAngle(&CoordinateX,&CoordinateY,&CoordinateZ,AccelX,AccelY,AccelZ,&AngleX,&AngleY,&AngleZ,gyr.gyro.x,gyr.gyro.y,gyr.gyro.z);

/* motor turns clockwise*/
digitalWrite(AIN1,HIGH);
digitalWrite(AIN2,LOW); 
analogWrite(PWMA,100);
digitalWrite(BIN1,LOW);
digitalWrite(BIN2,HIGH);
analogWrite(PWMB,100);
}

/* Setups with specific functions */
/* auto mode - it starts from leftward and scan clockwise till right*/
void AutoMode(){
	while(1){
		/* ‘go to’ label for safety */
BEGINNING:

/* SCAN FOR TOKENS */
/*reset accelerometer & gyrosope timer*/
previuseTime=millis();

/* set initial angle location */
initialAngle=AngleZ;

while(1){
/* motor turns clockwise*/
motorclockwise();

/* detect color from color sensor */
getcolordata();

/* can detect color from color sensor */
if((r<DATA)&&(r>DATA)&&(g<DATA)&&(g>DATA)&&(b<DATA)&&(b>DATA)&&(c<DATA)&&(c>DATA){
	/* stop rotating*/
motorstop();

/* quit program */
break;
	}
	
	/* if rotated for 180 degree, and still can’t detect color*/
/* need to check the exact rotation data during trial */
if((AngleZ>=initialAngle+180)||(AngleZ>=initialAngle-180)){
/* stop rotating*/
motorstop();

/* rotate 90 degree anticlockwise*/
/*reset accelerometer & gyrosope timer*/
previuseTime=millis();

/* set initial angle location */
initialAngle=AngleZ;
while(1) {
/* motor turns anticlockwise*/
motoranticlockwise();

/* if reach 90 degree*/
/* need to check the exact rotation data during trial */
if((AngleZ>=initialAngle+90)||(AngleZ>=initialAngle-90)){
	/* stop rotating*/
motorstop();

/* quit loop for turning 90 degree*/
break;
}

/* move forward a bit and then re-enter search mode */
/* need to check how long should it move forward */
/* start loop timer */
std::chrono::steady_clock::timepoint start = std::chrono::steady_clock::now();

/*reset accelerometer & gyrosope timer*/
previuseTime=millis();
while(1) {
/* command motor to move forward*/
motorforward();

/* if time’s up i.e. 5 second */
if(std::chrono::steady_clock::now() - start > std::chrono::seconds(5)){
	/* stop moving*/
motorstop();

/* quit loop for moving forward*/
break;

/* rotate 90 degree anticlockwise to restart searching mode */
/*reset accelerometer & gyrosope timer*/
previuseTime=millis();

/* set initial angle location */
initialAngle=AngleZ;

while(1) {
/* motor turns anticlockwise*/
motorclockwise();

/* if reach 90 degree*/
/* need to check the exact rotation data during trial */
if((AngleZ>=initialAngle+90)||(AngleZ>=initialAngle-90)){
	/* stop rotating*/
motorstop();

/* quit loop for turning 90 degree*/
break;
}

/* return to the second while loop in the program (i.e. moving 180 degree to scan color*/
break;
}
}
	/*MOVE TO GRAB TOKEN*/
/*reset accelerometer & gyrosope timer*/
previuseTime=millis();

/* set initial angle location */
initialAngle=AngleZ;

while(1) {
/* motor moves forward */
motorforward();
		
		/* measure token color lux*/
		getcolordata();

		/* if the vehicle is close enough to the token*/
/* need to check the distance needed for grabbing the token */
if(lux > DATA){
/* stop rotating*/
motorstop();

/* quit moving loop */
break;
}
}

/* GRAB TOKEN */
myservo.write(85);  

/* RELEASE TOKEN */
/* need to check whether 180 degree is really giving us a desirable degree of movement for the token grib */
myservo.write(0);  
	
/* GO BACK A BIT FOR ONE SECOND */
/* start loop timer */
std::chrono::steady_clock::timepoint start = std::chrono::steady_clock::now();

/*reset accelerometer & gyrosope timer*/
previuseTime=millis();
while(1) {
/* command motor to move backward*/
motorbackward();

/* if time’s up i.e. 1 second */
if(std::chrono::steady_clock::now() - start > std::chrono::seconds(1)){
/* stop moving */
motorstop();

/* quit loop */
break;
}
	
	/* wait for a while */
delay(1000);

/*TURN CLOCKWISE TO FACE THE LEFT SIDE*/
while(1) {
/* motor turns clockwise*/
motorclockwise();

/* if reach 90 degree*/
/* need to check the exact rotation data during trial */
if((AngleZ<270.1)&&(AngleZ>269.9)){
/* stop moving */
motorstop();

/* quit loop */
break;
}

/* wait for a short while */
	delay(1000);
}
}
/* Looping setup */
void loop(){
	/* FIRST TIME MOVING FORWARD */
	/*reset accelerometer & gyrosope timer*/
previuseTime=millis();

/* set initial angle location */
initialAngle=AngleZ;

while(1) {
/* motor moves forward */
motorforward();
		
		/* measure token color lux*/
		getcolordata();

/* if the vehicle is close enough to the token*/
/* need to check the distance needed for grabbing the token */
if(lux > DATA){
/* stop rotating*/
motorstop();

/* quit moving loop */
break;
}

/* grab token */
myservo.write(85);  

/* rotate 180 degree clockwise to face the scoring location*/
/*reset accelerometer & gyrosope timer*/
previuseTime=millis();

/* set initial angle location */
initialAngle=AngleZ;
while(1) {
/* motor turns anticlockwise*/
motoranticlockwise();

/* if reach 180 degree*/
/* need to check the exact rotation data during trial */
if((AngleZ>=initialAngle+180)||(AngleZ>=initialAngle-180)){
	/* stop rotating*/
motorstop();

/* quit loop*/
break;
}

/* Measure whether it slips */
/* detect color from color sensor */
getcolordata();

/* if slips */
if((r<DATA)&&(r>DATA)&&(g<DATA)&&(g>DATA)&&(b<DATA)&&(b>DATA)&&(c<DATA)&&(c>DATA){
/* open token grabber */
	myservo.write(0);  

	/* rotate 180 degree clockwise to face the scoring location*/
/*reset accelerometer & gyrosope timer*/
previuseTime=millis();

/* set initial angle location */
initialAngle=AngleZ;
while(1) {
/* motor turns clockwise*/
motorclockwise();

/* if reach 90 degree*/
/* need to check the exact rotation data during trial */
if((AngleZ>=initialAngle+90)||(AngleZ>=initialAngle-90)){
	/* stop rotating*/
motorstop();

/* quit loop*/
break;
}
/* directly enter auto mode as we already don’t know where the initial token has been to*/
AutoMode();

/* if didn’t slips */
else {
	/*reset accelerometer & gyrosope timer*/
previuseTime=millis();

	/* need to check y location - maybe need to be slightly in front */
while(1) {

/* command motor to move forward, towards scoring position*/
motorforward();

/* if returned to initial position */
if(CoordinateY<0.1) {
	/* stop moving */
/* need to check: here I assume the robot will not continue to move so I think pausing the location tracking and letting time run doesn’t matter? And I think before each time it starts moving we have to start calculating coordinates again, but since it hasn’t moved so acceleration & velocity = 0 so the coordinate should be right?*/
motorstop();
	
	/* quit loop */
break;
}
}
	
	/* wait for a short while */
delay(1000);

/* RELEASE TOKEN */
	/* release token when headed back to scoring zone */
myservo.write(0);  
	
/* go back a bit for 1 second */
/* start loop timer */
std::chrono::steady_clock::timepoint start = std::chrono::steady_clock::now();

/*reset accelerometer & gyrosope timer*/
previuseTime=millis();
while(1) {
/* command motor to move backward*/
motorbackward();

/* if time’s up i.e. 1 second */
if(std::chrono::steady_clock::now() - start > std::chrono::seconds(1)){
/* stop moving */
motorstop();

/* quit loop */
break;
}
	
	/* wait for a while */
delay(1000);

/* turn clockwise to face the left side */ 
/* need to check whether A or B is the right one for 90 degree clockwise (assume A is right) */

while(1) {
/* motor turns clockwise*/
motorclockwise();

/* if reach 90 degree*/
/* need to check the exact rotation data during trial */
if((AngleZ<270.1)&&(AngleZ>269.9)){
/* stop moving */
motorstop();

/* quit loop */
break;
}

/* wait for a short while */
	delay(1000);

/* enter auto mode */
AutoMode();
}