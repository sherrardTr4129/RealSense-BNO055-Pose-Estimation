#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// define constants needed for the duration of execution
#define BNO055_SAMPLERATE_DELAY_MS    20
#define BNO055_I2C_ADDR               55
#define TRUNC_VAL                      4
#define CE_PIN                        10
#define CSN_PIN                        9

// setup BNO055 object
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_I2C_ADDR);

// create NRF24L01 radio object
RF24 radio(CE_PIN, CSN_PIN); // CE, CSN

// set up radio address
const byte address[5] = {'R', 'x', 'A', 'A', 'A'};

// setup buffer of text to send
char textToSend[32] = "";

void setup() {
  // init serial connection
  Serial.begin(9600);

  // try to initialize BNO005 Sensor
  if (!bno.begin())
  {
    // sensor could not be found, loop forever
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while (true);
  }

  // configure BNO055 to use external crystal
  delay(1000);
  bno.setExtCrystalUse(true);

  // setup NRF24L01 radio module
  bool statusVal = radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate( RF24_250KBPS );
  radio.setRetries(3, 5); // delay, count
  radio.openWritingPipe(address);

  // if we can't set up the radio, loop forever.
  if (!statusVal)
  {
    Serial.println("Could not set up radio hardware!!");
    while(true);
  }
  else
  {
    Serial.println("Sucessfully set up radio hardware!!");
  }
}

void loop() {
  // get sensor information
  sensors_event_t event;
  bno.getEvent(&event);

  // extract roll, pitch, yaw from BNO sensor
  float qR = (float)event.orientation.x;
  float qP = (float)event.orientation.y;
  float qY = (float)event.orientation.z;

  // construct messages to send
  String textString = "R" + String(qR, TRUNC_VAL) + "P" + String(qP, TRUNC_VAL) +
                      "Y" + String(qY, TRUNC_VAL);

  // convert string object to char array
  textString.toCharArray(textToSend, sizeof(textToSend));

  // send data over radio
  radio.write(&textToSend, sizeof(textToSend));

  // refresh delay
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
