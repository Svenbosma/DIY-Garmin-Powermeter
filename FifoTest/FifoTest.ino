#include <LSM6DS3.h>
#include <Wire.h>

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

// Timestamp state must persist across FIFO drain cycles.
static bool gHasPrevTimestamp = false;
static uint32_t gPrevTimestampTicks = 0;

static float fifoDeltaSeconds(uint32_t currentTicks, uint32_t previousTicks) {
  // FIFO timestamp tick = 25 us.
  uint32_t deltaTicks = currentTicks - previousTicks; // unsigned subtraction handles wrap-around
  return (float)deltaTicks * 0.000025f;
}

static void integrateGyroDegrees(float gxDps, float gyDps, float gzDps, float dtSeconds,
                                 float &degX, float &degY, float &degZ) {
  degX += gxDps * dtSeconds;
  degY += gyDps * dtSeconds;
  degZ += gzDps * dtSeconds;
}

void setup( void ) {
  //Over-ride default settings if desired
  myIMU.settings.gyroEnabled = 1;  //Can be 0 or 1
  myIMU.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  myIMU.settings.gyroSampleRate = 208;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  myIMU.settings.gyroBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;
  myIMU.settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
  myIMU.settings.gyroFifoDecimation = 1;  //set 1 for on /1

  myIMU.settings.accelEnabled = 0;
  myIMU.settings.accelFifoEnabled = 0;  //Set to include accelerometer in the FIFO
  myIMU.settings.tempEnabled = 0;
  
  //Non-basic mode settings
  myIMU.settings.commMode = 1;

  myIMU.settings.timestampEnabled=1;    // 1: enable timestamp ; 0: disable timestamp
  myIMU.settings.timestampFifoEnabled=1;// 1: enable write timestamp into fifo ; 0: disable
  myIMU.settings.timestampResolution=1; // 1: Set timestamp resolution ; 0: 6.4ms  1: 25us  

  //FIFO control settings
  myIMU.settings.fifoThreshold = 100;  //Can be 0 to 4096 (16 bit bytes)
  myIMU.settings.fifoSampleRate = 50;  //Hz.  Can be: 10, 25, 50, 100, 200, 400, 800, 1600, 3300, 6600
  myIMU.settings.fifoModeWord = 6;  //FIFO mode.
  //FIFO mode.  Can be:
  //  0 (Bypass mode, FIFO off)
  //  1 (Stop when full)
  //  3 (Continuous during trigger)
  //  4 (Bypass until trigger)
  //  6 (Continous mode)
  

  Serial.begin(115200);  // start serial for output
  while (!Serial) delay(10);

  Serial.println("Processor came out of reset.\n");
  
  //Call .begin() to configure the IMUs
  if( myIMU.begin() != 0 )
  {
	  Serial.println("Problem starting the sensor with CS @ Pin 10.");
  }
  else
  {
	  Serial.println("Sensor with CS @ Pin 10 started.");
  }
  delay(1000);
  Serial.print("Configuring FIFO with no error checking...");
  
  delay(1000);
  myIMU.fifoBegin();
  Serial.print("Done!\n");
  
  Serial.print("Clearing out the FIFO...");
  
  delay(1000);
  //myIMU.fifoClear();
  Serial.print("Done!\n");
  
}


void loop()
{
  static unsigned long reportWindowStartMs = millis();
  static uint32_t samplesThisSecond = 0;
  static float integratedDegX = 0.0f;
  static float integratedDegY = 0.0f;
  static float integratedDegZ = 0.0f;
  static float accumulatedDtSeconds = 0.0f;

  // Drain all available FIFO packets.
  while( ( myIMU.fifoGetStatus() & 0x1000 ) == 0 ) {
    float gx = myIMU.calcGyro(myIMU.fifoRead());
    float gy = myIMU.calcGyro(myIMU.fifoRead());
    float gz = myIMU.calcGyro(myIMU.fifoRead());

    // Read and discard external sensor dataset before timestamp.
    (void)myIMU.fifoTimestamp();
    uint32_t fifoTimestamp = myIMU.fifoTimestamp();

    if (gHasPrevTimestamp) {
      float dtSeconds = (fifoTimestamp - gPrevTimestampTicks) * 0.000025f;
      accumulatedDtSeconds += dtSeconds;
      integratedDegZ += gz * dtSeconds;
    } else {
      gHasPrevTimestamp = true;
    }
    gPrevTimestampTicks = fifoTimestamp;
    samplesThisSecond++;
  }

  unsigned long nowMs = millis();
  if (nowMs - reportWindowStartMs >= 1000UL) {
    Serial.print("samples_per_second: ");
    Serial.print(samplesThisSecond);
    Serial.print(", accumulated_dt_s: ");
    Serial.print(accumulatedDtSeconds, 6);
    Serial.print(", integrated_deg_x: ");
    Serial.print(integratedDegX, 6);
    Serial.print(", integrated_deg_y: ");
    Serial.print(integratedDegY, 6);
    Serial.print(", integrated_deg_z: ");
    Serial.println(integratedDegZ, 6);

    samplesThisSecond = 0;
    integratedDegX = 0.0f;
    integratedDegY = 0.0f;
    integratedDegZ = 0.0f;
    accumulatedDtSeconds = 0.0f;
    reportWindowStartMs = nowMs;
  }
}
