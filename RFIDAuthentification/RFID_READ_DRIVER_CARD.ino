/*
 * Initial Author: ryand1011 (https://github.com/ryand1011)
 *
 * Reads data written by a program such as "rfid_write_personal_data.ino"
 *
 * See: https://github.com/miguelbalboa/rfid/tree/master/examples/rfid_write_personal_data
 *
 * Uses MIFARE RFID card using RFID-RC522 reader
 * Uses MFRC522 - Library
 * -----------------------------------------------------------------------------------------
 *             MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
 *             Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
 * Signal      Pin          Pin           Pin       Pin        Pin              Pin
 * -----------------------------------------------------------------------------------------
 * RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
 * SPI SS      SDA(SS)      10            53        D10        10               10
 * SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
 * SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
 * SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
*/


// Include the Servo library 
#include <Servo.h> 
// Declare the Servo pin 
int servoPin = 6; 
// Create a servo object 
Servo Servo1;


#include <SPI.h>
#include <MFRC522.h>


#define FIRST_NAME_LENGTH 20
#define FAMILY_NAME_LEGTH 30
#define DRIVER_ID_LENGTH 10
#define SIGNATURE_SIZE 32
#define BUFFER_SIZE 32

int redpin = 2; // select the pin for the red LED
int bluepin = 4; // select the pin for the blue LED
int greenpin = 3 ;// select the pin for the green LED




#define RST_PIN         5           // Configurable, see typical pin layout above
#define SS_PIN          53          // Configurable, see typical pin layout above

MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance

bool driverDone = false;
bool vehicleDone = false;

//*****************************************************************************************//
void setup() {
  Serial.begin(9600);                                           // Initialize serial communications with the PC
  SPI.begin();                                                  // Init SPI bus
  mfrc522.PCD_Init();                                              // Init MFRC522 card
  //Serial.println(F("Read personal data on a MIFARE PICC:"));    //shows in serial that it is ready to read
  pinMode (redpin, OUTPUT);
  pinMode (bluepin, OUTPUT);
  pinMode (greenpin, OUTPUT);

  
  analogWrite (redpin, 255);
  analogWrite (bluepin, 0);
  analogWrite (greenpin, 0);

   Servo1.attach(servoPin); 
      Servo1.write(130); 
 

}

//*****************************************************************************************//
void loop() {

  // Prepare key - all keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
  MFRC522::MIFARE_Key key;
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

  //some variables we need
  byte block;
  byte len;
  MFRC522::StatusCode status;

  //-------------------------------------------

  // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
  if ( ! mfrc522.PICC_IsNewCardPresent()) {
    return;
  }

  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial()) {
    return;
  }

  len = BUFFER_SIZE;
  byte buffer[BUFFER_SIZE];
  uint8_t response[30];
   Serial.setTimeout(1200000L) ; 

  if(!driverDone)
  {
    analogWrite (redpin, 0);
    analogWrite (bluepin, 0);
    analogWrite (greenpin, 0);
    
    char message[] = {'D','r','i','v','e', 'r'/*,' '*/,'C','a','r','d',/*' ',*/'D','e','t','e','c','t','e','d','\n'};
      Serial.write(message, sizeof(message));
      Serial.flush();
  
    //-------------------------------------------
  
    //mfrc522.PICC_DumpDetailsToSerial(&(mfrc522.uid)); //dump some details about the card
  
    //mfrc522.PICC_DumpToSerial(&(mfrc522.uid));      //uncomment this to see all blocks in hex
  
    //-------------------------------------------
  
    
  
  
    //------------------------------------------- GET FIRST NAME
  //   Serial.print(F("First Name: "));
    
    block = 44;
    
    status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("Authentication failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
      return;
    }
  
    status = mfrc522.MIFARE_Read(block, buffer, &len);
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("Reading failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
      return;
    }
  
  /*
    
    block = 45;
  
    status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid));
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("Authentication failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
      return;
    }
  
    status = mfrc522.MIFARE_Read(block, &buffer[16], &len);
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("Reading failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
      return;
    }
  */
    int i = 0;
    for(i = 0; BUFFER_SIZE;i++)
      if(buffer[i]==' ')
        break;//Found end.
  
    buffer[i] = '\n';
    Serial.write(buffer, i+1);//Sent First Name.
    Serial.flush();
  
    //---------------------------------------- GET LAST NAME
  
  //  Serial.print(F("Last Name: "));
  
    block = 40;
    
    status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("Authentication failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
      return;
    }
  
    status = mfrc522.MIFARE_Read(block, buffer, &len);
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("Reading failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
      return;
    }
  
   /* 
    block = 41;
  
    status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid));
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("Authentication failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
      return;
    }
  
    status = mfrc522.MIFARE_Read(block, &buffer[16], &len);
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("Reading failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
      return;
    }
  */
    i = 0;
    for(i = 0; BUFFER_SIZE;i++)
      if(buffer[i]==' ')
        break;//Found end.
  
    buffer[i] = '\n';
    Serial.write(buffer, i+1);//Sent Last Name.
    Serial.flush();
  
  
   //---------------------------------------- GET DRIVER_ID
  
   //Serial.print(F("DRIVER_ID: "));
  
    block = 36;
    
    status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("Authentication failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
      return;
    }
  
    status = mfrc522.MIFARE_Read(block, buffer, &len);
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("Reading failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
      return;
    }
  
    /*
    block = 37;
  
    status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid));
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("Authentication failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
      return;
    }
  
    status = mfrc522.MIFARE_Read(block, &buffer[16], &len);
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("Reading failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
      return;
    }
    */
  
    i = 0;
    for(i = 0; BUFFER_SIZE;i++)
      if(buffer[i]==' ')
        break;//Found end.
    buffer[i] = '\n';
    Serial.write(buffer, i+1);//Sent Driver ID.
    //Serial.flush();

    char messageEnd[] = {'E','n','d','R','e','a','d','i','n','g','\n'};
    Serial.write(messageEnd, sizeof(messageEnd));
    //----------------------------------------
    Serial.setTimeout(1200000L) ; 
    int len = Serial.readBytesUntil('\n', (char *) response, 30) ; // read family name from serial

    response[len] = '\0';
    if(strcmp(response, "DriverAuthentified")==0)
     {
          //Yello LIGHT
          analogWrite (redpin, 255);
          analogWrite (bluepin, 0);
          analogWrite (greenpin, 155);
          driverDone = true;
     }
       
  }
  else
  {    
  //---------------------------------------- GET Vehicle_ID

     Serial.println("VehicleCardDetected");
     //Serial.print(F("Vehicle_ID: "));
      block = 1;
      len = 20;
    
    status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("Authentication failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
      return;
    }
  
    status = mfrc522.MIFARE_Read(block, buffer, &len);
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("Reading failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
      return;
    }
  
 //* 
    block = 2;
  
    status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid));
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("Authentication failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
      return;
    }
  
    status = mfrc522.MIFARE_Read(block, &buffer[16], &len);
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("Reading failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
      return;
    }
//*/
    int i = 0;
    for(i = 0; BUFFER_SIZE;i++)
      if(buffer[i]==' ')
        break;//Found end.
  
    buffer[i] = '\n';
    Serial.write(buffer, i+1);//Sent Last Name.
    Serial.flush();

      char messageEnd[] = {'E','n','d','R','e','a','d','i','n','g','\n'};
    Serial.write(messageEnd, sizeof(messageEnd));

      int len = Serial.readBytesUntil('\n', (char *) response, 30) ; // read family name from serial
  
      response[len] = '\0';
      if(strcmp(response, "VehicleAuthentified")==0)
        {
            //Green 
            analogWrite (redpin, 0);
            analogWrite (bluepin, 0);
            analogWrite (greenpin, 255);
            //Pull Barrier
            Servo1.write(230);
        }
        else
        {
             analogWrite (redpin, 255);
            analogWrite (bluepin, 0);
            analogWrite (greenpin, 0);
            driverDone = false;
        }
        
  } 

  
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  
  delay(1000); //change value if you want to read cards faster
}


//*****************************************************************************************//
