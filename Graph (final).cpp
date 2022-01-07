#include <MegunoLink.h>
#include <EEPROM.h>
 
XYPlot MyPlot;

int XDataValue=0, YDataValue;
 
void plotgraph() {
  for (int i = 0; i < 255; i = i + 4) {
    /* retrieve coordinate information from EEPROM */
EEPROM.get(i, XDataValue);
   	EEPROM.get(i+2, YDataValue);
 
    /* send coordinate*/
    MyPlot.SendData("Position", XDataValue, YDataValue);

    /* print coordinate*/
    Serial.print(XDataValue);
    Serial.print(', ');
    Serial.println(YDataValue);

    delay(200);
 
  }
}

void setup()
{
  Serial.begin(9600);
 
  MyPlot.SetTitle("Location of the tokens we attempted to grab and the initial position");
  MyPlot.SetXlabel("X");
  MyPlot.SetYlabel("Y");
  MyPlot.SetSeriesProperties("Position", Plot::Red, Plot::Solid, 3, Plot::Square);

  plotgraph();
}
 
void loop() {
 
}