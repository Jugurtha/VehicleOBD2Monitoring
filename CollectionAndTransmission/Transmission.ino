#include <ESP8266WiFi.h>
#include "ELMduino.h"
#include <SPI.h>
#include <SD.h>
#include <ACrypto.h>  
#include <base64.hpp>
#include <pins_arduino.h>
#include <vector>

#define PASSWD_SIZE 10
#define SAMPLING_PERIOD 1000

const uint8_t chipSelect = D8;

const char* ssid = "LG K8 LTE_9930";
const char* password = "1234567890";

/*
const uint8_t vehicle_passwd[] = "1234567890";
const uint8_t parc_passwd[] =    "0987654321";
*/
const uint8_t vehicle_passwd[PASSWD_SIZE] = {'1', '2', '3', '4', '5', '6', '7', '8', '9', '0'};
const uint8_t parc_passwd[PASSWD_SIZE] = {'0', '9', '8', '7', '6', '5', '4', '3', '2', '1'};

const uint8_t MAX_SIZE_BLOCK = 128;


//IP Adress of your ELM327 Dongle
IPAddress server(192, 168, 43, 115);
WiFiServer wifiServer(1234);
WiFiClient clientELM;

ELM327 myELM327;

uint8_t Speed = 0;
bool started = false;
uint8_t startPin = D2;
uint32_t clocktime = 0;
bool newMission = true;


#define HMAC_KEY_LENGTH 16
#define AES_KEY_LENGTH 16

uint8_t* keyEncrypt;
uint8_t* keyHmac;
uint8_t keyHash[SHA256_SIZE];
uint8_t key[AES_KEY_LENGTH] = { 0x1C,0x3E,0x4B,0xAF,0x13,0x4A,0x89,0xC3,0xF3,0x87,0x4F,0xBC,0xD7,0xF3, 0x31, 0x31 };
uint8_t iv[AES_KEY_LENGTH];

SHA256 sha256;

// prints given block of given length in HEX
void printBlock(uint8_t* block, int length) {
  Serial.print(" { ");
  for (int i=0; i<length; i++) {
    if(block[i] <= 15)
      Serial.print(0, HEX);
      Serial.print(block[i], HEX);
    Serial.print(" ");
  }
  Serial.println("}");
}

void setup()
{
  pinMode(startPin, INPUT);
  Serial.begin(115200);

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  while(!SD.begin(chipSelect))
    Serial.println("Card failed, or not present");
  Serial.println("SD card initialized.");


  if(newMission){
    if(SD.exists("Vitesses.bin"))
    {
      SD.remove("Vitesses.bin");
      SD.remove("Vitesses.txt");
      File dataFile = SD.open("Vitesses.bin", FILE_WRITE);
      if (dataFile)
        dataFile.close();
      dataFile = SD.open("Vitesses.txt", FILE_WRITE);
      if (dataFile)
        dataFile.close();
    }

      // With deaufault keys
      // get SHA-256 hash of our secret key to create 256 bits of "key material"
      sha256.doUpdate(key, AES_KEY_LENGTH); 
      sha256.doFinal(keyHash);
    
      // keyEncrypt is a pointer pointing to the first 128 bits bits of "key material" stored in keyHash
      // keyHmac is a pointer poinging to the second 128 bits of "key material" stored in keyHashMAC
      keyEncrypt = keyHash;
      keyHmac = keyHash + AES_KEY_LENGTH;

      newMission = false;
  }


  // Connecting to ELM327 WiFi
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

   while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Connected to Wifi");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  if (clientELM.connect(server, 23))
    Serial.println("Connected to ELM327.");
  else
  {
    Serial.println("Connection to ELM327 failed.");
    ESP.reset();
  }

  myELM327.begin(clientELM);

  Serial.printf("\n\n");
}


void loop()
{
  started  = digitalRead(startPin)==HIGH;
  if(started){
      int32_t tempSpeed = myELM327.kph();
    
      if (myELM327.status == ELM_SUCCESS)
      {
        Speed = (uint8_t)tempSpeed;
        Serial.println(String(clocktime)+" - SPEED : "+String(Speed));
        
        File dataFile = SD.open("Vitesses.bin", FILE_WRITE);
        if (dataFile) {
          dataFile.write(&Speed, sizeof(Speed));
          dataFile.close();
        }
        else {// if the file isn't open, pop up an error:
          Serial.println("error opening Vitesses.txt");
        }
        dataFile = SD.open("Vitesses.txt", FILE_WRITE);
        if (dataFile) {
          dataFile.printf("%d, ", Speed);
          dataFile.close();
        }
        else {// if the file isn't open, pop up an error:
          Serial.println("error opening Vitesses.txt");
        }
      }
      else
        printError();
      clocktime += 1;
      delay(SAMPLING_PERIOD);
    }
    else {
      Serial.println("Waiting for parc server...");
    		wifiServer.begin();
    		delay(500);
    		WiFiClient clientParc = wifiServer.available();
         		
          if (clientParc){
      		  if (clientParc.connected()){

                if(!authentification(clientParc))
                  return;
               
               /*if(newMission) 
                 {
                    //Get Keys.
                  /*  
                    Serial.printf("\n\n");
                    // get SHA-256 hash of our secret key to create 256 bits of "key material"
                    sha256.doUpdate(key, AES_KEY_LENGTH); 
                    sha256.doFinal(keyHash);
                  
                    // keyEncrypt is a pointer pointing to the first 128 bits bits of "key material" stored in keyHash
                    // keyHmac is a pointer poinging to the second 128 bits of "key material" stored in keyHashMAC
                    keyEncrypt = keyHash;
                    keyHmac = keyHash + AES_KEY_LENGTH;
                  */
                  /*
                    if(SD.exists("Vitesses.bin"))
                    {
                      SD.remove("Vitesses.bin");
                      SD.remove("Vitesses.txt");
                      File dataFile = SD.open("Vitesses.bin", FILE_WRITE);
                      if (dataFile)
                        dataFile.close();
                      dataFile = SD.open("Vitesses.txt", FILE_WRITE);
                      if (dataFile)
                        dataFile.close();
                    }
                    
                    newMission = false;
                    return;
                 }
                */
                
                std::vector<uint8_t> data;
      			    
      			    File dataFile = SD.open("Vitesses.bin");
                if (!dataFile){// if the file isn't open, pop up an error:
                  Serial.println("error opening Vitesses.bin");
                  clientParc.stop();
                  return;
                }
          			while (dataFile.available()) {
                       data.push_back(dataFile.read());
          			}
                dataFile.close();
                
                Serial.println("data bin : ");
                  for (int i = 0; i < data.size(); i++) {
                    Serial.print(data.data()[i]);Serial.print(" ");
                  }
                Serial.println("");

                dataFile = SD.open("Vitesses.txt");
                if (!dataFile){// if the file isn't open, pop up an error:
                  Serial.println("error opening Vitesses.txt");
                  clientParc.stop();
                  return;
                }
                
                Serial.println("data txt : ");
                while (dataFile.available()) {
                       Serial.print((char)dataFile.read());
                }
                dataFile.close();
                
                Serial.println("");
                

               
               //Slice in 128 bytes blocks     
                uint8_t n = MAX_SIZE_BLOCK - 1;
                // determine number of blocks
                uint8_t nbr = (data.size() - 1) / n + 1;

                //Send number of blocks to send to parc
                clientParc.write(&nbr, sizeof(nbr));
                   Serial.println("Sending number of blocks to send to parc .......");
              
                // each iteration of this loop process next set of n elements
                for (uint8_t k = 0; k < nbr; ++k)
                {
                  // get range for next set of n elements
                  auto start_itr = std::next(data.cbegin(), k*n);
                  auto end_itr = std::next(data.cbegin(), k*n + n);
              
                  // allocate memory for the sub-vector
                  std::vector<uint8_t> packet(n);
                                
                  // code to handle the last sub-vector as it might
                  // contain less elements
                  if (k*n + n > data.size()) {
                    end_itr = data.cend();
                    packet.resize(data.size() - k*n);
                  }
              
                  // copy elements from the input range to the sub-vector
                  std::copy(start_itr, end_itr, packet.begin());

                  // print the sub-vectors
                  Serial.printf("\nPacket %d : \n", k);
                  for (int i = 0; i < packet.size(); i++) {
                    Serial.print(packet.data()[i]);Serial.print(" ");
                  }
                  Serial.println("");

                  //Encrypt sub-vector

                  uint8_t encryptedMessage[256];
                  int encryptedMessageSize;
                  encryptData(packet.data(), packet.size(), &encryptedMessageSize, encryptedMessage);

                  // print the sub-vectors
                  Serial.printf("encryptedMessage (%d bytes): ",encryptedMessageSize);
                  for (int i = 0; i < encryptedMessageSize; i++) {
                    Serial.print(encryptedMessage[i]);Serial.print(" ");
                  }
                  Serial.println("");
                  
                  clientParc.write(encryptedMessage, encryptedMessageSize);
                  Serial.println("Sending data to parc .......");
//*
                  int decryptedSize;
                  uint8_t decrypted[256];
                  decryptData(encryptedMessage, encryptedMessageSize, &decryptedSize, decrypted);

                  Serial.println("Packet : ");
                  for (int i = 0; i < decrypted[0]; i++) {
                    Serial.print(decrypted[i]);Serial.print(" ");
                  }
                  Serial.println("");
//*/
                  //Waiting for permition
                  String suivant;
                  suivant = clientParc.readStringUntil('\n');
                  Serial.println(suivant.c_str());
                  if(suivant.equals("suivant"))
                    Serial.println("Sending next block.");
                  else if( suivant.equals("finished") )
                  {
                      Serial.println("Finished sending data.");
                      newMission = true;
                  }
                  else
                  {
                    Serial.printf("Data transmission aborted after sending block n°%d.\n", k);
                    break;
                  }
                  delay(100);
                  Serial.print('\n');
                }
                Serial.println("---------------------------------------------\n\n");
      		  }
           clientParc.stop();
    		}
       delay(1000);
    }   
}


void printError()
{
  Serial.print("Received: ");
  for (byte i = 0; i < myELM327.recBytes; i++)
    Serial.write(myELM327.payload[i]);
  Serial.println();
  
  if (myELM327.status == ELM_SUCCESS)
    Serial.println(F("\tELM_SUCCESS"));
  else if (myELM327.status == ELM_NO_RESPONSE)
    Serial.println(F("\tERROR: ELM_NO_RESPONSE"));
  else if (myELM327.status == ELM_BUFFER_OVERFLOW)
    Serial.println(F("\tERROR: ELM_BUFFER_OVERFLOW"));
  else if (myELM327.status == ELM_UNABLE_TO_CONNECT)
    Serial.println(F("\tERROR: ELM_UNABLE_TO_CONNECT"));
  else if (myELM327.status == ELM_NO_DATA)
    Serial.println(F("\tERROR: ELM_NO_DATA"));
  else if (myELM327.status == ELM_STOPPED)
    Serial.println(F("\tERROR: ELM_STOPPED"));
  else if (myELM327.status == ELM_TIMEOUT)
    Serial.println(F("\tERROR: ELM_TIMEOUT"));
  else if (myELM327.status == ELM_TIMEOUT)
    Serial.println(F("\tERROR: ELM_GENERAL_ERROR"));

  //delay(100);
}


void encryptData(const uint8_t *packet, int packetSize, int *encryptedMessageSize, uint8_t *encryptedMessage)
{
  Serial.println("\nEncrypting :");

  //int packetSize = sizeof(packet)/sizeof(packet[0]);//strlen(packet);
  Serial.printf("Packet (%d bytes):\n", packetSize);
  Serial.print("{ ");
  for(int i = 0; i<packetSize;i++)
    {Serial.print(packet[i]);Serial.print(' ');}
  Serial.println('}');
  
  Serial.print("Packet HEX");
  printBlock((uint8_t*)packet, packetSize);//+1);  //+1 to add null termination

  // random initialization vector
  RNG::fill(iv, AES_KEY_LENGTH);

  Serial.printf("Random IV (%d bytes)", AES_KEY_LENGTH);
  printBlock(iv, AES_KEY_LENGTH);

  AES aes(keyEncrypt, iv, AES::AES_MODE_128, AES::CIPHER_ENCRYPT);

  // create buffer for final message which will contain IV, encrypted message, and HMAC
  int encryptedSize = aes.calcSizeAndPad(packetSize);
  int ivEncryptedSize = encryptedSize + AES_KEY_LENGTH;
  int ivEncryptedHmacSize = ivEncryptedSize + SHA256HMAC_SIZE;
  uint8_t ivEncryptedHmac[ivEncryptedHmacSize];

  // copy IV to our final message buffer
  memcpy(ivEncryptedHmac, iv, AES_KEY_LENGTH);

  // encrypted is a pointer that points to the encypted messages position in our final message buffer
  uint8_t* encrypted = ivEncryptedHmac + AES_KEY_LENGTH;

  // AES 128 CBC and pkcs7 padding
  aes.process((uint8_t*)packet, encrypted, packetSize);

  Serial.printf("Encrypted (%d bytes)", encryptedSize);
  printBlock(encrypted, encryptedSize);

  // computedHmac is a pointer which points to the HMAC position in our final message buffer
  uint8_t* computedHmac = encrypted + encryptedSize;

  // compute HMAC/SHA-256 with keyHmac
  SHA256HMAC hmac(keyHmac, HMAC_KEY_LENGTH);
  hmac.doUpdate(ivEncryptedHmac, ivEncryptedSize);
  hmac.doFinal(computedHmac);

  Serial.printf("Computed HMAC (%d bytes)", SHA256HMAC_SIZE);
  printBlock(computedHmac, SHA256HMAC_SIZE);

  Serial.printf("IV | encrypted | HMAC (%d bytes)", ivEncryptedHmacSize);
  printBlock(ivEncryptedHmac, ivEncryptedHmacSize);

  *encryptedMessageSize = ivEncryptedHmacSize;
  memcpy(encryptedMessage, ivEncryptedHmac, *encryptedMessageSize);
  
 /* 
  // base64 encode
  *encodedSize = encode_base64_length(ivEncryptedHmacSize); // get size needed for base64 encoded output
  //uint8_t encoded[encodedSize];
  encode_base64(ivEncryptedHmac, ivEncryptedHmacSize, encoded);

  Serial.printf("Base64 encoded to %d bytes\n", *encodedSize);
  */
}

void decryptData(uint8_t *encryptedMessage, int encryptedMessageSize, int *decryptedSize, uint8_t *decrypted)
{
  Serial.printf("\nDecrypting :\n");
  /*
  // base64 decode
  int decodedSize = decode_base64_length(encoded);
  uint8_t decoded[decodedSize];
  decode_base64(encoded, decoded);
  */
  int decodedSize = encryptedMessageSize;
  uint8_t *decoded = encryptedMessage;
//  Serial.printf("Received %d bytes\n", encodedSize);
  Serial.printf("Base64 decoded IV | encrypted | HMAC (%d bytes)", decodedSize);
  printBlock(decoded, decodedSize);
 
  // receivedHmac is a pointer which points to the received HMAC in our decoded message
  uint8_t* receivedHmac = decoded+decodedSize-SHA256HMAC_SIZE;

  Serial.printf("Received HMAC (%d bytes)", SHA256HMAC_SIZE);
  printBlock(receivedHmac, SHA256HMAC_SIZE); 

  // compute HMAC/SHA-256 with keyHmac
  uint8_t remote_computedHmac[SHA256HMAC_SIZE];  
  SHA256HMAC remote_hmac(keyHmac, HMAC_KEY_LENGTH);
  remote_hmac.doUpdate(decoded, decodedSize-SHA256HMAC_SIZE);
  remote_hmac.doFinal(remote_computedHmac);

  Serial.printf("Computed HMAC (%d bytes)", SHA256HMAC_SIZE);
  printBlock(remote_computedHmac, SHA256HMAC_SIZE);

  if (*receivedHmac == *remote_computedHmac) {
      // extract IV
      memcpy(iv, decoded, AES_KEY_LENGTH);
  
      Serial.printf("Received IV (%d bytes)", AES_KEY_LENGTH);
      printBlock(iv, AES_KEY_LENGTH);
    
      // decrypt 
      *decryptedSize = decodedSize - AES_KEY_LENGTH - SHA256HMAC_SIZE;
      //uint8_t decrypted[decryptedSize];
      AES aesDecryptor(keyEncrypt, iv, AES::AES_MODE_128, AES::CIPHER_DECRYPT);
      aesDecryptor.process((uint8_t*)decoded + AES_KEY_LENGTH, decrypted, *decryptedSize);  
      
      Serial.printf("Decrypted Packet HEX (%d bytes)", *decryptedSize);
      printBlock((uint8_t*)decrypted, *decryptedSize);
    }
}

bool authentification(WiFiClient clientParc)
{
  //Parc authentification
        Serial.println("Wait for parc passwd.");
        uint8_t rec_parc_passwd[64] = "";
        if (clientParc.available()) {
          clientParc.read(rec_parc_passwd, 64);
       
          int decryptedSize;
          uint8_t decrypted[PASSWD_SIZE];
          decryptData(rec_parc_passwd, 64, &decryptedSize, decrypted);
          
          Serial.printf("rec_parc_passwd (%d bytes) : ", decryptedSize - decrypted[decryptedSize-1]);
          printBlock(decrypted, decryptedSize - decrypted[decryptedSize-1]);
          
          if(memcmp(decrypted, parc_passwd, decryptedSize - decrypted[decryptedSize-1])==0)
          {
            Serial.println("Parc authentificated.");
          }
          else
          {
            Serial.println("Parc authentification failed.");
            return false;
          }
        }
        else
          return false;
        
        //Vehicle authentification
        uint8_t encryptedVehiclePasswd[64];
        int encryptedVehiclePasswdSize;
        encryptData(vehicle_passwd, PASSWD_SIZE, &encryptedVehiclePasswdSize, encryptedVehiclePasswd);
        Serial.printf("vehicle_passwd (%d bytes) : ", PASSWD_SIZE);
        printBlock((uint8_t*)vehicle_passwd, PASSWD_SIZE);
/*
        int dS;
        uint8_t d[16];
        decryptData(encryptedVehiclePasswd, 64, &dS, d);
        Serial.printf("d (%d bytes) : ", dS - d[dS-1]);
        printBlock(d, dS - d[dS-1]);
*/        
        clientParc.write(encryptedVehiclePasswd, encryptedVehiclePasswdSize);
        
        //Waiting for confirmation
        Serial.println("");
        String confirm;
        confirm = clientParc.readStringUntil('\n');
        Serial.println(confirm.c_str());
        if(confirm.equals("confirm"))
          Serial.println("Received Confimation.");
        else
          return false;            
        
        Serial.print('\n');
        return true;
}
