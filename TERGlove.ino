#include <I2Cdev.h>
#include "MPU6050_9Axis_MotionApps41.h"
#include <WiFi.h>
//#include <ros.h>
//#include <std_msgs/String.h>
//#include <sensor_msgs/Imu.h>
//#include <std_msgs/Header.h>
//#include <geometry_msgs/Vector3.h>
//#include <geometry_msgs/Quaternion.h>
//#include <geometry_msgs/Pose.h>
/// WARNING Be careful to choose the right pins for I2C , depending on the hardware version, wire.begin ( a,b)

#include <EEPROM_Utils.h>


typedef union {
 float floatingPoint[4];
 byte binary[16];
} binary4Float;

typedef union{
  int8_t integerPoint;
  byte binary;
} binaryInt;
  
typedef union{
  int16_t integerPoint[3];
  byte binary[6];
} binary3Int;

struct s_module{
  MPU6050 sensor;
  bool dmpReady;

  Quaternion q;
  VectorInt16 accel;
  VectorInt16 accelReal;
  VectorInt16 accelWorld;
  VectorInt16 gyro;
  VectorInt16 mag;
  VectorFloat gravity;

  uint8_t fifoBuffer[48];
  uint16_t packetSize;
};

s_module s_arrays[13];

//MPU6050 imu[12]; 

const uint8_t MPU_ADD0 = 0x68; 
const uint8_t MPU_ADD1 = 0x69; 
const uint8_t ADDRESSES[2] = {MPU_ADD0 , MPU_ADD1};

const int num_imus = 13 ; // 13,1 for the cube 
const bool cubeFlag = false;

const int MUX_ADDRESS = 0x6A; // here should be the address of the first MUX, the second will be incremented by 1

//uint16_t packetSize;
//bool dmpReady[12] = {false};

uint16_t fifoCount; 
//uint8_t fifoBuffer[num_imus][48]; // was 64

char * ssid = "EmaroLab-WiFi"; //
char * password = "walkingicub"; //
//declared in EEPROM_UTils
//IPAddress server(192,168,0,100); //(130, 251, 13,104);// fixed ip:36//113)//195;//(192,168,43,94);//// ip of your ROS server
IPAddress ip;  
IPAddress server_local ; //this variable is created because we had a problem accessing the variable server inside the send data function.

int8_t P[14] = {0,1,2,3,4,5,15,16,17,18,19,20,21,22};
int8_t channels[14] = {0x00,0x00,0x01,0x01,0x03,0x03,0x02,0x02,0x11,0x11,0x12,0x12,0x10,0x10}; // channels for the PCA9545B MX //hardware design reference FMA101P3L01 3.21

WiFiClient client;
unsigned int localPort = 2390;
WiFiClient clientDebug;
unsigned int debugPort = 2495;
WiFiUDP udpDebug; 
bool onoff=true;

// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(MUX_ADDRESS);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void tcaselect(uint8_t i,uint8_t mux_address) {
  if (i > 7) return;
  Wire.beginTransmission(mux_address);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void sendData(VectorInt16 accel, VectorInt16 gyro, VectorInt16 mag, Quaternion q, int8_t id){
  binaryInt ID;
  ID.integerPoint = id;
  WiFiUDP Udp;
  binary4Float quaternion;
  quaternion.floatingPoint[0] = q.x;
  quaternion.floatingPoint[1] = q.y;
  quaternion.floatingPoint[2] = q.z;
  quaternion.floatingPoint[3] = q.w;

  binary3Int accelerometer;
  accelerometer.integerPoint[0] = accel.x;
  accelerometer.integerPoint[1] = accel.y;
  accelerometer.integerPoint[2] = accel.z;

  binary3Int gyroscope;
  gyroscope.integerPoint[0] = gyro.x;
  gyroscope.integerPoint[1] = gyro.y;
  gyroscope.integerPoint[2] = gyro.z;

  binary3Int magnetometer;
  magnetometer.integerPoint[0] = mag.x;
  magnetometer.integerPoint[1] = mag.y;
  magnetometer.integerPoint[2] = mag.z;


  byte packetBuffer[29+6];
  memset(packetBuffer, 0, 29+6);

  packetBuffer[28+6] = ID.binary;
 // packetBuffer[16] = ID.binary;
  
  for(int i=0; i<16; i++){
    packetBuffer[i] = quaternion.binary[i];
  }
  
  for(int i=16; i<22; i++){
    packetBuffer[i] = accelerometer.binary[i-16];
  }

  for(int i=22; i<28; i++){
    packetBuffer[i] = gyroscope.binary[i-22];
  }


  for(int i=28; i<34; i++){
    packetBuffer[i] = magnetometer.binary[i-28];
  }
  
  
  int begin = Udp.beginPacket(server_local, localPort);
  Serial.println("SENDING ON UDP");
  Serial.println(server_local);
  int write = Udp.write(packetBuffer, 29+6);
  int end = Udp.endPacket();  
  Serial.println ("begin ");
  Serial.print(begin); 
  Serial.println ("write ");
  Serial.print(write); 
  Serial.println ("end ");
  Serial.print(end); 
}

void setupWiFi(){ 
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  //Print to serial to find out IP address and debugging
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) 
  {
    delay(500);
    onoff=!onoff;
   // digitalWrite (LED_BUILTIN, onoff);
  //  digitalWrite (D9, onoff);
    Serial.print(".....");

  }
  delay (1000);
  Serial.print ("ROS IP"); Serial.println(server);
  //digitalWrite (LED_BUILTIN, HIGH);
  //digitalWrite (D9, HIGH);
  delay(500);
  if(i == 21){
    Serial.print("Could not connect to"); Serial.println(ssid);
    while(1) delay(500);
  }
  Serial.print("Ready! Use ");
  ip = WiFi.localIP();
  Serial.print(ip);
  Serial.println(" to access client");
}

void i2cTest(){
  byte error, address;
  int nDevices;
  delay(500);
  Serial.println("Scanning...");
  nDevices = 0;
  
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }else {
   // Serial.println("found ");Serial.print(nDevices);
  }
  delay(1000);          
}

void setup() {
  Serial.begin(115200);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH); // To enable the multiplexer

//  pinMode(D9,OUTPUT);
//  digitalWrite(D9,HIGH); // To turn on a led non present on the board

  SerialBT.begin("ESP32 Test Device"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  
  int  ret = EEPROM.begin(512);
  if (ret!=1){
    Serial.println ("error initializing EEPROM");
  }

  getIPfromSerial();
  server = getIP();
  Serial.println ("saved IP");
  for (int i = 0 ;i<4; i ++){  
    Serial.print (server[i]); 
    Serial.print (".");
  }
  
  server_local = server; 
  Serial.println("server_local");Serial.print (server_local);
  Serial.print("saved SSID ");Serial.println(ssid);
  ssid = new char[ssidLen+1];
  getSSID().toCharArray(ssid, ssidLen+1);
  Serial.print("saved SSID eeprom ");Serial.println(ssid);
  Serial.print("getSSID ");Serial.println(getSSID());

  Serial.print("saved Password ");Serial.println(password);
  password = new char[pwdLen+1];
  getPWD().toCharArray(password, pwdLen+1);
  Serial.print("saved Password eeprom ");Serial.println(password);
  Serial.write (password);
    
  setupWiFi();
  
  int devStatus; 
  if (cubeFlag){
    Wire.begin();
  }else{
    Wire.begin(27,14); // (SDA,SCL) (33,25) and (  27,14)// please check if you use the single multiplexer or two
  }
  i2cTest();

  Wire.setClock(1000000); // super fast mode 1MHZ

  //TESTING ALL channels, not necessary everytime
  for (int j = 0; j <2; j++){
    for (int i =0 ; i <4 ; i ++){
      Serial.print("MUX ");  Serial.print(j); Serial.print(" Channel ");  Serial.println(i); 
      onoff=!onoff;
     // digitalWrite(D9,onoff);
      tcaselect (i,MUX_ADDRESS+j);
      i2cTest();
      tcaselect (5,MUX_ADDRESS+j); // Since two Mux are on the same i2c, you should disable the first mux, before enabling the other
    }
  }
    
  for (int i = 0  ; i <num_imus  ; i++){
    //tcaselect(channels[i]);
    tcaselect((channels[i] ) & 0x0F, (channels[i] >>4) + MUX_ADDRESS);

    s_arrays[i].sensor = MPU6050(ADDRESSES[i%2]);
    s_arrays[i].sensor.initialize();
    bool cnctd = s_arrays[i].sensor.testConnection();
  
    Serial.print("connecting to address " );Serial.print(ADDRESSES[i%2]);
    Serial.print(" i " ); Serial.print (i);
    Serial.print(" MUX " ); Serial.print ((channels[i] >> 4));
    Serial.print(" Channel " ); Serial.println ((channels[i] ) & 0x0F);
    Serial.println(cnctd ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    if (cnctd){
      s_arrays[i].sensor.resetFIFO();
    }

    devStatus= s_arrays[i].sensor.dmpInitialize();
    Serial.println ("initializing MPU");
    Serial.println(devStatus);
    if (devStatus == 0 ){
      Serial.println ("MPU initialized");
      s_arrays[i].sensor.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      s_arrays[i].sensor.setDMPEnabled(true);
         
      //set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready!"));
      s_arrays[i].dmpReady = true;
      s_arrays[i].packetSize = s_arrays[i].sensor.dmpGetFIFOPacketSize();
      fifoCount = s_arrays[i].sensor.getFIFOCount();
      Serial.print ("fifocount"); Serial.println(fifoCount);
      Serial.print("PacketSize");Serial.println(s_arrays[i].packetSize); 
    }else{
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }    
    tcaselect(5, (channels[i] >>4) + MUX_ADDRESS); // deselect channel
  }
 
  for (int i = 0 ;i <num_imus ;i ++){
    Serial.print ("DMP ready " ); 
    Serial.print (i);Serial.print (" ");
    Serial.print(s_arrays[i].dmpReady);
  }
}


/* ================================================================================================ *
 | Default MotionApps v4.1 48-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][MAG X ][MAG Y ][MAG Z ][ACC X ][ACC X ][ACC Y ][ACC Y ][ACC Z ][ACC Z ][      ] |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41  42  43  44  45  46  47  |
 * ================================================================================================ */
void  getIPfromSerial(){
  Serial.println("************************************************************************************"); 
  Serial.println("Enter the new server address if you want to change it, format IP;000;000;000;000;IP"); 
  Serial.println("************************************************************************************"); 

  delay (4000);

  if (!SerialBT.available()){
    delay (1000);
  }
  while (SerialBT.available()) {
    String msg = SerialBT.readString();
    Serial.print(msg);
    parseMy(msg);
  }

  while (Serial.available()) {
    String msg = Serial.readString();
    //Serial.print(msg);
    parseMy(msg);
  }
}



void loop() {
  
  char bigBuf[100] = "";
  //acc.x = 0;  acc.y = 0;  acc.z =0; gyr.x = 0;  gyr.y =0;  gyr.z = 0;
  
  //Quaternion q;           // [w, x, y, z]         quaternion container // moved here to avoid sending readings of another sensor when not getting data
 
  for (int i = 0;i <num_imus ;i++){
    if(s_arrays[i].dmpReady){
    //if(dmpReady[i]){
      tcaselect((channels[i] ) & 0x0F, (channels[i] >>4) + MUX_ADDRESS);

      int res = s_arrays[i].sensor.GetCurrentFIFOPacket(s_arrays[i].fifoBuffer,s_arrays[i].packetSize);
      
      if (res ==1){ // Get the Latest packet 
        s_arrays[i].sensor.dmpGetQuaternion(&s_arrays[i].q, s_arrays[i].fifoBuffer);
        s_arrays[i].sensor.dmpGetAccel(&s_arrays[i].accel,s_arrays[i].fifoBuffer);
        s_arrays[i].sensor.dmpGetGyro(&s_arrays[i].gyro,s_arrays[i].fifoBuffer);
        s_arrays[i].sensor.dmpGetGravity(&s_arrays[i].gravity, &s_arrays[i].q);
        s_arrays[i].sensor.dmpGetLinearAccel(&s_arrays[i].accelReal, &s_arrays[i].accel, &s_arrays[i].gravity);
        s_arrays[i].sensor.dmpGetMag(&s_arrays[i].mag, s_arrays[i].fifoBuffer);
      
  /*          
        gyr.x = gyro[0];
        gyr.y = gyro[1];
        gyr.z = gyro[2];

        acc.x = aaReal.x;
        acc.y = aaReal.y;
        acc.z = aaReal.z;

        mag_vec.x = mag[0];
        mag_vec.y = mag[1];
        mag_vec.z = mag[2];
   */         
        if (cubeFlag){
          sendData(s_arrays[i].accel,s_arrays[i].gyro,s_arrays[i].mag,s_arrays[i].q,22);
          //sendData(acc,gyr,mag_vec,q,22); 
        }else{
          sendData(s_arrays[i].accel,s_arrays[i].gyro,s_arrays[i].mag,s_arrays[i].q,P[i]);
          //sendData(acc,gyr,mag_vec,q,P[i]);   
        }
        onoff = !onoff;
//        digitalWrite(D9,onoff);
      }
    }
    tcaselect(5, (channels[i] >>4) + MUX_ADDRESS); // deselect channel
  }
}


///////// It seems the bufer reset is making this problem 
//minimising the number of buffer resets leads to more chaotic readings 
// when having more sensors, implies less readings, the buffer is full and more frequently 
// resetting the buffer when it is full (512), leads to less topic freq. because when we read from the buffer , we are reading the last packet and removing the others
// making the FIFO clock divider larger MPU6050_DMP_FIFO_RATE_DIVISOR (in _MPU6050_9AXIS_MOTIONAPPS41_H_),  solved partially the problem , 00 is OK for 6 sensors 50 HZ
// sending 11 topics with dummy data , the topics frequency reached 65-67 HZ, each.
//MPU6050_DMP_FIFO_RATE_DIVISOR =0x04  , for 11 sensors ==> 20 HZ each
//MPU6050_DMP_FIFO_RATE_DIVISOR =0x05  , for 11 sensors ==> 25 HZ each
//MPU6050_DMP_FIFO_RATE_DIVISOR =0x06  , for 11 sensors ==> 29 HZ each
//MPU6050_DMP_FIFO_RATE_DIVISOR =0x07  , for 11 sensors ==> 25 HZ each
//These two params needs to be tuned carefully , or to find the way where FIFO can store only one packet  
