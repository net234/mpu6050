/******
  Exemple of BetaEvent lib for the  MPU6050 class using DMP (MotionApps v6.12)
  This file is part of betaEvents.

    betaEvents is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    betaEvents is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with betaEvents.  If not, see <https://www.gnu.org/licenses/lglp.txt>.


  History
    V1.0 event_pushbutton 10/12/2020 NET234


    TODO: add virtual method to seve and restore offsets

 *************************************************/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050_9Axis_MotionApps41.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

//// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
//// is used in I2Cdev.h
//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//#include "Wire.h"
//#endif

#define MPU_RTCMEMORY_OFFSET 0
typedef int16_t MPUOffsets_t[6];

// controle de la lib
#ifndef CLOCK_RATIO
#error  nececite la lib MP6050 adaptee : "https://github.com/net234/mpu6050/tree/dev-NET234"
#endif
#if CLOCK_RATIO != 4
#error  dans la lib MP6050.h CLOCK_RATIO doit etre a 4 
#endif

// MPU_6050_Event is an interface classe to deal with MPU60_50
//on event driven environement and with lesser harware consideration
class MPU6050_EVENT : public MPU6050
{
  public:
    MPU6050_EVENT(const byte i2cadresse = 0x68) : MPU6050{i2cadresse}  {};
    void begin();  // force a reset and a full calibrate
    virtual void  readSavedOffsets();   // read the offsets from RTC memory   (override this if you have saved elsewhere)
    virtual void  writeSavedOffsets();  // write the offsets to RTC memory    (override this if you want to save elsewhere)
    bool calibrate(const bool autocalibrate = true); // full init
    bool asyncroneRead();  // get  angularPosQFloat & aaMesureInt  true if one read done false if no data ready;
    bool isOK();
    uint8_t status;         // return status after each device operation (0 = success, !0 = error)
    // orientation/motion vars
    Quaternion  angularPosQFloat;           // [w, x, y, z]         position angulaire quaternion
    VectorInt16 aaMesureInt;                // [x, y, z]            accel sensor measurements
    MPUOffsets_t offsets;

  private:
    uint16_t packetSize;    // expected DMP packet size (default is 28 bytes)
    #define MAXSIZE_FIFOBUFFER 64
};






// minimal init just to know if MPU6050 is here
void MPU6050_EVENT::begin() {
  this->reset();  // must reset before testConnection()
  delay(100);
  this->status = 99;  // mpu not initialized
  this->initialize();
  if (this->testConnection()) this->status = 0;
}


void  MPU6050_EVENT::readSavedOffsets() {
  struct {
    float checkPI = 0;
    MPUOffsets_t offsets;
  } savedRTCmemory;

  ESP.rtcUserMemoryRead(MPU_RTCMEMORY_OFFSET, (uint32_t*)&savedRTCmemory, sizeof(savedRTCmemory));
  if (savedRTCmemory.checkPI == float(PI + 234)) {
    Serial.println(F("MPU6050 got MPU offsets directly from RTC"));
    memcpy(this->offsets, savedRTCmemory.offsets, sizeof(offsets));
  }
}

void  MPU6050_EVENT::writeSavedOffsets() {
  struct {
    float checkPI = PI + 234;
    MPUOffsets_t offsets;
  } savedRTCmemory;

  Serial.println(F("MPU6050 save offsets directly to RTC"));
  memcpy(savedRTCmemory.offsets, offsets, sizeof(savedRTCmemory.offsets));
  ESP.rtcUserMemoryWrite(MPU_RTCMEMORY_OFFSET, (uint32_t*)&savedRTCmemory, sizeof(savedRTCmemory));
}


bool MPU6050_EVENT::calibrate(const bool autocalibrate) {

  //  struct  {
  //    float     checkPI;              // initialised to PI value to check POWER_ON Boot
  //    MPUOffsets_t offsets;
  //  } savedRTCmemory;

  if (!this->testConnection() ) {
    // Serial.println(F("MPU6050 Reset"));
    this->reset();
    delay(50);
    if (!this->testConnection() ) {
      Serial.println(F("MPU6050 connection failed"));
    }
    this->initialize();
  }

  // load and configure the DMP
  this->status = this->dmpInitialize();  //RAZ giro offset
  if (this->status == 0) {
    //setup saved Offset if any
    if (offsets[0] == 0) this->readSavedOffsets();
    //      MyDeepSleepManager.restoreRTCData(RTC_DATA(offsets));
    //      //      ESP.rtcUserMemoryRead(MPU_RTCMEMORY_OFFSET, (uint32_t*)&savedRTCmemory, sizeof(savedRTCmemory));
    //      //      if (savedRTCmemory.checkPI == float(PI + 1)) {
    //      Serial.println(F("MPU6050 get offset from RTC"));
    //      //        memcpy(offsets, savedRTCmemory.offsets, sizeof(offsets));
    //      //      }
    //    }

    if (offsets[0] != 0) {

      this->setActiveOffsets(offsets);
      //Serial.println(F("MPU6050 set offset"));
      this->PrintActiveOffsets();
    }
    Serial.println(F("Start initializing DMP..."));
    if (autocalibrate || offsets[3] == 0) { // offset 3 4 5 is at 0 on power on
      this->CalibrateAccel(40);
      Serial.println();
      this->CalibrateGyro(40);
      Serial.println();
      this->PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      this->getActiveOffsets(offsets);
      // save offset somewhere
      this->writeSavedOffsets();
      //      Serial.println(F("MPU6050 save offsets to RTC"));
      //      this->
      //
      //      MyDeepSleepManager.saveRTCData(RTC_DATA(offsets));
      //      //      savedRTCmemory.checkPI = PI + 1;
      //      //      memcpy(savedRTCmemory.offsets, offsets, sizeof(savedRTCmemory.offsets));
      //      //      ESP.rtcUserMemoryWrite(MPU_RTCMEMORY_OFFSET, (uint32_t*)&savedRTCmemory, sizeof(savedRTCmemory));
    }


    Serial.println(F("Enabling DMP..."));
    this->setDMPEnabled(true);

    // get expected DMP packet size for later comparison
    packetSize = this->dmpGetFIFOPacketSize();
    if (packetSize > MAXSIZE_FIFOBUFFER) {
      this->status = 98;
      Serial.print(F("Packet size = "));
      Serial.println(packetSize);
      return false;
    }
  }
  Serial.println(F("End initializing DMP..."));
  this->PrintActiveOffsets();
  return (this->status == 0);
}

bool MPU6050_EVENT::isOK() {
  return (this->status == 0);
}


bool MPU6050_EVENT::asyncroneRead() {
//      uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[MAXSIZE_FIFOBUFFER]; // FIFO storage buffer for 1 set of data

  uint16_t fifoCount = this->getFIFOCount();
  if (fifoCount >= 1024) {
    this->resetFIFO();
    Serial.print("!");
    // status = 100;
    fifoCount = this->getFIFOCount();
  }
  //Serial.print(F("FifoCount "));
  //Serial.println(fifoCount);
  if (fifoCount >= packetSize) {
    this->getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    this->dmpGetAccel(&aaMesureInt, fifoBuffer);          // acceleration instantanée -> aaMesureInt
    this->dmpGetQuaternion(&angularPosQFloat, fifoBuffer);
    return (true);
  }
  return (false);
}
