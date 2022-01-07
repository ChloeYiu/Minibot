/* Initial set up */
/* define pins for motors */
#define AIN1 8
#define AIN2 12
#define PWMA 5
#define BIN1 7
#define BIN2 6
#define PWMB 11


/* define pins for on-off switch */
int inPIN = A2;

/* define reference variables */
double refTime;

/* define graph data variables*/
int graphnum = 0;

/* define variables to store accelerometer & gyroscope data*/
double previuseTime;

double previuseAccelX=0,previuseAccelY=0,previuseAccelZ=0;
double prevVelocityX=0,prevVelocityY=0,prevVelocityZ=0;
float prevCoordinateX=0,prevCoordinateY=0,prevCoordinateZ=0;
float prevAngleX=0,prevAngleY=0,prevAngleZ=0;

float VAngleX=0,VAngleY=0,VAngleZ=0;
float AccX=0,AccY=0,AccZ=0;

int i=1;

float AngleX=0, AngleY=0, AngleZ=0;
double CoordinateX=0,CoordinateY=0,CoordinateZ=0;

/* define variables to store color sensor data*/
float red = 0;
float green = 0;
float blue = 0;
float clearcolor = 0;

/* import accelerometer libraries*/
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

/* import light sensor libraries*/
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* import servo libraries*/
#include <Servo.h>
Servo myservo;

/* import graph plotter libraries*/
#include <EEPROM.h>
#include <MegunoLink.h>
/* Setups for on-off switch */
void onoffswitch () {
	if (digitalRead(inPIN) == LOW) {
		Serial.println("The button is now OFF");
while (1) {
			if (digitalRead(inPIN) == HIGH) {
				break;
			}
		}
	}
	
	Serial.println("The button is now ON");
}
/* Setups for measurement */
/* calculate coordinate based on acceleration & time, calculate angle based on angular velocity & time */
void getCoorAngle(){
double currentTime;
currentTime=(millis()-previuseTime)/1000;
previuseTime=millis();

/*obtain data for calculation */
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

if ((AccX < 1)&&(AccX > -1)) {
	AccX= 0;
}

if ((AccY < 1)&&(AccY > -1)) {
	AccY= 0;
}

if ((AccZ < 1)&&(AccZ > -1)) {
	AccZ= 0;
}

/*calculate X coordinate*/
CoordinateX=prevCoordinateX+(prevVelocityX*currentTime)+(AccX*currentTime*currentTime*0.5);
prevVelocityX=(AccX*currentTime);
prevCoordinateX=CoordinateX;

/*calculate Y coordinate*/
CoordinateY=prevCoordinateY+(prevVelocityY*currentTime)+(AccY*currentTime*currentTime*0.5);
prevVelocityY=(AccY*currentTime);
prevCoordinateY=CoordinateY;

/*calculate Z coordinate*/
CoordinateZ=prevCoordinateZ+(prevVelocityZ*currentTime)+(AccZ*currentTime*currentTime*0.5);
  	prevVelocityZ=(AccZ*currentTime);
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
Serial.print(CoordinateZ*1000);
Serial.println("mm");

	Serial.print("Velocity X: ");
Serial.print(prevVelocityX);
Serial.print(", Y: ");
Serial.print(prevVelocityY);
Serial.print(", Z: ");
Serial.print(prevVelocityZ);
Serial.println("ms-1");

Serial.print("Acceleration X: ");
Serial.print(AccX);
Serial.print(", Y: ");
Serial.print(AccY);
Serial.print(", Z: ");
Serial.print(AccZ);
Serial.println("ms-2");

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

Serial.println("");
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




/* Setups for plotting graphs */
void EEPROMclear() {
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
EEPROM.put(graphnum + 2, graphy);

int eex, eey;

EEPROM.get(graphnum, eex);
EEPROM.get(graphnum + 2, eey);
	
	/* indicate coordinate is stored */
	Serial.print("One set of coordinate is stored: ");
	Serial.print(eex);
	Serial.print(", ");
	Serial.println(eey);

	/* increase plot number counter */
	graphnum += 4;
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
analogWrite(PWMA,255);
digitalWrite(BIN1,HIGH);
digitalWrite(BIN2,LOW);
analogWrite(PWMB,250);
}

/* motor moves backward fast*/
void motorbackwardfast(){

	/* measure location*/
getCoorAngle();

/* motor moves backward*/
digitalWrite(AIN1,LOW);
digitalWrite(AIN2,HIGH);
analogWrite(PWMA,255);
digitalWrite(BIN1,LOW);
digitalWrite(BIN2,HIGH);
analogWrite(PWMB,250);
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

/* grabber opens */
void servoopen() {
	myservo.write(0);
}

/* grabber closes to grab*/
void servofullyclose() {
	myservo.write(30);
	storegraphdata();
}

/* Void setup */
void setup() {
	/* INITIALIZATION */
/* initialize communication */
	Serial.begin(9600); 
	while (!Serial)
 		delay(10);
	Serial.println("Serial console is opened");

/* initialize token grabber (servo)*/
  myservo.attach(3); 
  servoopen();


/* initialize wheels (motor)*/
pinMode(AIN1, OUTPUT);
pinMode(AIN2, OUTPUT);
pinMode(PWMA, OUTPUT);
pinMode(BIN1, OUTPUT);
pinMode(BIN2, OUTPUT);
pinMode(PWMB, OUTPUT);

/* initialize on-off switch*/
pinMode(inPIN, INPUT);

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

/*START UNTIL THE BUTTON IS ON*/
onoffswitch();

/* reset EEPROM memory for the next operation */
EEPROMclear();
}

/* Void loop  - attempt to grab token*/
void loop () {
	/*reset accelerometer & gyrosope timer*/
previuseTime=millis();	

	/*set reference time as now*/
refTime = millis();

	/* start moving forward for 2s till probably it leaves our scoring zone */
	while(1) {
/* motor follow lines, moving forward*/
motorforwardfast();
		
/*leaves non-detecting-color mode when 2s has passed */
/* need adjustment */
if ((millis()-refTime) > 2000) {

	/*quit loop */
	break;
}
}


/*reset accelerometer & gyrosope timer*/
previuseTime=millis();	
	
	/* move to the middle */
	while (1) {
		/* move forward */
		motorforwardfast();
		
		/* collect color sensor data */
		getcolordata();

		/* stop moving when detect white color i.e. reaching the mid point */
if(clearcolor>70){
/* stop moving*/
motorstop();

/* quit loop*/
break;
}
	}

	/* grab token regardless of whether a token exists or not */
	servofullyclose();
	delay(500);

/*reset accelerometer & gyrosope timer*/
previuseTime=millis();	

	/*set reference time as now*/
refTime = millis();

/* start moving backward for 3.4s - without data collection - till probably it passes through the two white lines */
	while(1) {

/* motor follow lines, moving forward*/
motorbackwardfast();
		
/*leaves non-detecting-color mode when 4.2s has passed */
/* need adjustment */
if ((millis()-refTime) > 4200) {

	/*quit loop */
	break;
}
}

/* move away from our scoring arena backward - with data collection */
	while (1) {
		/* move backward*/
		motorbackwardfast();
		
		/* collect color sensor data */
		getcolordata();

		/* stop moving when detect blue color i.e. reaching the scoring arena*/
if((clearcolor<65)&&(clearcolor>20)){
	/* move for 0.1s more - till it isn’t touching the blue tape */
delay(100);

/* stop moving*/
motorstop();
delay(500);

/* quit loop*/
break;
}
	}

	/* rotate for 180 degree */
	motorclockwise();
	delay(1200);

	/* release token */
	servoopen();
	delay(500);
	
/*reset accelerometer & gyrosope timer*/
previuseTime=millis();	

	/*set reference time as now*/
refTime = millis();

	/* move backward for 2s - until probably leaving the arena */
	while(1) {

/* motor follow lines, moving forward*/
motorbackwardfast();
		
/*stop moving when 2s has passed */
/* need adjustment */
if ((millis()-refTime) > 2000) {
	/* stop moving */
	motorstop();
	delay(500);

	/*quit loop */
	break;
}
}

	/* rotate for 180 degree */
	motorclockwise();
	delay(1200);


/* Void loop  - messing up*/

/*reset accelerometer & gyrosope timer*/
previuseTime=millis();	

/*set reference time as now*/
refTime = millis();

	/* move forward for 10s till probably it is close to opponent’s scoring zone */
	while(1) {

/* motor moving forward*/
motorforwardfast();
		
/*leaves non-detecting-color mode when 10s has passed */
/* need adjustment */
if ((millis()-refTime) > 10000) {

	/*quit loop */
	break;
}
}

	/* move and detect color sensor simutaneously */
	while (1) {

/* move forward */		
motorforwardfast();
		
		/* measure color data */
getcolordata();

/* stop moving when detect blue color */
/* need to check data */
if((clearcolor<65)&&(clearcolor>20)){
/* stop moving*/
motorstop();
delay(500);

/* quit loop*/
break;
}
}

	/* grab tokens */
	servofullyclose();
	delay(500);
	
/*reset accelerometer & gyrosope timer*/
previuseTime=millis();	
	
	
	/*set reference time as now*/
refTime = millis();

	/* move forward for 1s till probably midway in the scoring zone */
	while(1) {

/* motor follow lines, moving forward*/
motorforwardfast();
		
/*stop moving when 1s has passed */
if ((millis()-refTime) > 2000) {
		/*quit loop */
	break;
}
}


/*reset accelerometer & gyrosope timer*/
previuseTime=millis();	

	/*set reference time as now*/
refTime = millis();

	/* move and detect color sensor simutaneously */
	while (1) {
/* move forward */		
motorforwardfast();
		
		/* measure color data */
getcolordata();

/* stop moving when detect blue color */
/* need to check data */
if((clearcolor<65)&&(clearcolor>20)){
/* stop moving*/
motorstop();
delay(500);

/* quit loop*/
break;
}
}

/* dump tokens */
	servofullyclose();
delay(500);

/*reset accelerometer & gyrosope timer*/
previuseTime=millis();	

	/*set reference time as now*/
refTime = millis();

	/* move backward for 1s till probably it is midway in the scoring zone */
	while(1) {

/* motor follow lines, moving forward*/
motorbackwardfast();
		
/*stop moving when 1s has passed */
/* need adjustment */
if ((millis()-refTime) > 1000) {
	/* stop moving */
motorstop();
delay(500);

	/*quit loop */
	break;
}
}

	/*set reference time as now*/
refTime = millis();
	
	/* spin crazily*/
	while(1) {

/* motor spins crazily*/
motorclockwise();
		
}

}