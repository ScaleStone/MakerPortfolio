#include <SPI.h>
#include <SD.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Si115X.h>;
#include <Wire.h>

#define DHTTYPE DHT11
#define DHTPIN 8
#define ONE_WIRE_BUS 7
#define RELAY_ON 2
#define RELAY_OFF 3
#define RELAY 5

File myFile;
DHT dht(DHTPIN, DHTTYPE);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Si115X si1151;

unsigned long seconds = millis();
boolean relayOverride = false;
float data[8] = {0};

//=======================================================================
// Initalizes the SD card to accept data
void SDInit()
{
  Serial.print("Initializing SD card...");

  if (!SD.begin(10))
  {
    Serial.println("INITIALIZATION FAILED");
    while (1);
  }
  Serial.println("Initialization done.");
}

//=======================================================================
//opens a file in the sd card
void openFile(char fileName[])
{
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open(fileName, FILE_WRITE);
}

//=======================================================================
//prints names of the columns of the data types
void initData()
{
  String dataName[8] = {"Secs", "Moist", "Temp", "Humid", "SoTemp", "Ir", "Visible", "Uv"};
  openFile("Data.csv");
  // fence-posting data for csv file
  Serial.print("Initializing data...");
  myFile.print(dataName[0]);
  Serial.print(dataName[0]);
  for (int i = 1; i < (sizeof(dataName) / sizeof(String)); i++)
  {
    //names for data columns beig printed
    myFile.print(", ");
    myFile.print(dataName[i]);
    Serial.print(", ");
    Serial.print(dataName[i]);
  }
  //closing the file
  myFile.println();
  Serial.println(" done");
  myFile.close();
}

//=======================================================================
// takes an integer array as an input and prints it to the .csv file
void dataFileWrite()
{
  openFile("Data.csv");
  Serial.print("Recording Data...");
  // fence-posting for the CSV file
  myFile.print(data[0]);
  Serial.print(data[0]);
  for (int i = 1; i < sizeof(data) / sizeof(float); i++)
  {
    // prints data with a comma seperating it ", 1, 2, 3..."
    myFile.print(", ");
    myFile.print(data[i]);
    Serial.print(", ");
    Serial.print(data[i]);
  }
  // moves to next line
  myFile.println();
  Serial.println(" done\n");
  // close the file:
  myFile.close();
}

//=======================================================================
//get the data from teh capacitive moisture sensor through pin A0
void capacitiveMoistureSensor()
{
  int sensorValue = analogRead(A0);
  Serial.print("Capacitive moisture sensor : ");
  Serial.println(sensorValue);

  data[1] = sensorValue;
}

//=======================================================================
//get the tempeature and humidity through the DHT
void tempHumSensor()
{
  float tempHumValue[2] = {0};
  dht.readTempAndHumidity(tempHumValue);
  
  data[2] = tempHumValue[1];
  data[3] = tempHumValue[0];

  Serial.print("Temp : ");
  Serial.print(data[2]);
  Serial.print(" | ");
  Serial.print("Moisture : ");
  Serial.println(data[3]);
}

//=======================================================================
//get the soil'd temperature throuhg the one wire bus
void soilTempSensor()
{
  sensors.requestTemperatures(); // Send the command to get temperatures

  float tempC = sensors.getTempCByIndex(0);

  data[4] = tempC;
  Serial.print("Soil Temp : ");
  Serial.println(tempC);
}

//=======================================================================
//get the data for the sulight sensor from the I2C port
void sunlightSensor()
{
  float ir = si1151.ReadHalfWord();
  float visible = si1151.ReadHalfWord_VISIBLE();
  float uv = si1151.ReadHalfWord_UV();

  Serial.print("IR : ");
  Serial.print(ir);
  Serial.print(" | ");
  Serial.print("Visible : ");
  Serial.print(visible);
  Serial.print(" | ");
  Serial.print("UV : ");
  Serial.println(uv);
  
  data[5] = ir;
  data[6] = visible;
  data[7] = uv; 
}

//=======================================================================
//sets the data for the data array using the methds to collect data
void setData()
{
  capacitiveMoistureSensor();
  tempHumSensor();
  soilTempSensor();
  sunlightSensor();
  seconds = millis();
  data[0] = seconds/1000;
}

//=======================================================================
//sets up all of the sensors and serial monitor
void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  //while (!Serial) {;}
   // wait for serial port to connect. Needed for native USB port only

  SDInit();
  initData(); 

  dht.begin();
  Wire.begin();

  si1151.Begin();

  pinMode(RELAY_ON, INPUT_PULLUP);
  pinMode(RELAY_OFF, INPUT_PULLUP);
  pinMode(RELAY, OUTPUT);
  setData();
  dataFileWrite();
}

//=======================================================================
//every seconds a new data measurement is made
//the relay is turned on and off by tehe buttons (blue for on white for off)
//uses the millis() function to parallel process both functionalitiies of the code
void loop()
{
  seconds = millis();
  capacitiveMoistureSensor();
  
  if (seconds % 30000 == 0)
  {
    setData();
    dataFileWrite();
  }

  if (digitalRead(RELAY_ON) == LOW)
  {
    digitalWrite(RELAY, HIGH);
    Serial.println("Relay on");
    relayOverride = true;
  }
  if (digitalRead(RELAY_OFF) == LOW)
  {
    digitalWrite(RELAY, LOW);
    Serial.println("Relay off");
    relayOverride = false;
  }
  if (data[1] > 650 and !relayOverride)
  {
    digitalWrite(RELAY, HIGH);
  }
  if (data[1] < 650 and !relayOverride)
  {
    digitalWrite(RELAY, LOW);
  }
}
