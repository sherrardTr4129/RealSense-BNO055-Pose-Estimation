#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// define constants needed throughout program execution
#define BUFFER_SIZE         32
#define CE_PIN              10
#define CSN_PIN             9

// instantiate radio object
RF24 radio(CSN_PIN, CE_PIN); // CE, CSN

// setup radio address array
const byte address[5] = {'R','x','A','A','A'};

//create buffer to store incomming data
char textData[BUFFER_SIZE];

void setup() {
  // start serial communication
  Serial.begin(115200);

  // setup NRF24L01 as reciever
  bool statusVal = radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.openReadingPipe(1, address);
  radio.startListening();

  // if we can't communicate with radio, loop forever.
  if(!statusVal)
  {
    Serial.println("Can't communicate with radio hardware!");
    while(true);
  }
  else
  {
    Serial.println("Sucessfully initialized radio hardware!");
  }
}
void loop() {
  if(radio.available())
  {
    // unload radio payload into buffer
    radio.read(&textData, sizeof(textData));

    // print out data from radio
    Serial.println(textData);
  }
}
