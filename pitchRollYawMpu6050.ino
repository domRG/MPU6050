#include<Wire.h>

#define AC_NUM_TO_AVG 3  // add minor smoothing to accelerometer readings - high numbers increase lag and make for slower measurements - better for slow systems
#define MPU_addr 0x68  // default MPU6050 address

float acReadings[3][AC_NUM_TO_AVG + 1];  // [x,y,z],[readings,sum]
int acAvgIndex = 0;  // track which index of array is next to be updated
float acInst[3] = {0,0,0};  // x,y,z
float acAvg[3] = {0,0,0};  // x,y,z
float acPR[2] = {0,0};  // accel pitch and roll
float gyDeltaPRY[3] = {0,0,0};  // gyro pitch roll and yaw
float acGyOffset[6] = {-247.69, -133.50, -68.48, 905.85, -151.32, -110.68};  // accel: x,y,z and gyro: x,y,z offsets - paste from calibration results
float gyAcMix = 0.95;  // adjust mix of gyro and accel in data combination, 0 = all accel, 1 = all gyro
float pry[3] = {0,0,0};  // final pitch roll and yaw

bool calibrateMpuBool = false;  // set "true" to autorun calibration
int calibrateTime = 5000;  // calibrate for 5000 milliseconds

float timeNow = 0;
float timePrev = 0;
float timeDelta = 0;

float rad2deg(float rad){
  return (rad*180.0)/PI;
}

void updateAcReadings(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);
  int16_t xAcRaw = (Wire.read() << 8 | Wire.read());
  int16_t yAcRaw = (Wire.read() << 8 | Wire.read());
  int16_t zAcRaw = (Wire.read() << 8 | Wire.read());

  acInst[0] = (float)xAcRaw + acGyOffset[0];
  acInst[1] = (float)yAcRaw + acGyOffset[1];
  acInst[2] = (float)zAcRaw + acGyOffset[2];
}

void updateAcAvg(){
  for(int i = 0; i < 3; i++){
    acReadings[i][AC_NUM_TO_AVG] -= acReadings[i][acAvgIndex];
    acReadings[i][acAvgIndex] = acInst[i];
    acReadings[i][AC_NUM_TO_AVG] += acReadings[i][acAvgIndex];
    acAvg[i] = acReadings[i][AC_NUM_TO_AVG] / AC_NUM_TO_AVG;
  }
  acAvgIndex++;
  acAvgIndex = acAvgIndex % AC_NUM_TO_AVG;
}

void calculateAcRP(){
  acPR[0] = rad2deg(atan(acAvg[0]/acAvg[2]));
  acPR[1] = rad2deg(atan(acAvg[1]/acAvg[2]));
}

void updateGyReadings(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);
  int16_t xGyRaw = Wire.read() << 8 | Wire.read();
  int16_t yGyRaw = Wire.read() << 8 | Wire.read();
  int16_t zGyRaw = Wire.read() << 8 | Wire.read();
  
  gyDeltaPRY[1] = ((float)xGyRaw + acGyOffset[3])/65.5;
  gyDeltaPRY[0] = -((float)yGyRaw + acGyOffset[4])/65.5;  // negative to match orientations of accel +ve and gyro +ve
  gyDeltaPRY[2] = ((float)zGyRaw + acGyOffset[5])/65.5;
}

//////////MPU Calibration//////////
void calibrateMpu(){
  Serial.print("MPU calibration will begin in a few seconds, place on a level surface and do not move");
  for(int i = 0; i < 10000; i++){  // discard readings for 10000 milliseconds
    updateAcReadings();
    updateGyReadings();
    if((i%1000)==0){
      Serial.print(".");  // notify of ongoing action every 1000 milliseconds
    }
  }
  Serial.println();
  Serial.print("Calibrating");  // notify of calibration beginning
  long long int xAcSum = 0;
  long long int yAcSum = 0;
  long long int zAcSum = 0;
  long long int xGySum = 0;
  long long int yGySum = 0;
  long long int zGySum = 0;
  long int counter = 0;
  uint64_t timeNow = millis();
  while(millis()<(timeNow+calibrateTime)){
    updateAcReadings();
    updateGyReadings();
    xAcSum += acInst[0];
    yAcSum += acInst[1];
    zAcSum += acInst[2];
    xGySum += (int)(gyDeltaPRY[1]*65.5);
    yGySum += (int)(gyDeltaPRY[0]*65.5);
    zGySum += (int)(gyDeltaPRY[2]*65.5);
    counter++;
    if((counter%500)==0){
      Serial.print(".");  // notify of ongoing action every 500 milliseconds
    }
  }
  Serial.println();
  acGyOffset[0] += -((float)xAcSum/(float)counter);
  acGyOffset[1] += -((float)yAcSum/(float)counter);
  acGyOffset[2] += -((float)zAcSum/(float)counter) + 4100;  // 1g offset
  acGyOffset[3] += -((float)xGySum/(float)counter);
  acGyOffset[4] += -((float)yGySum/(float)counter);
  acGyOffset[5] += -((float)zGySum/(float)counter) - 11; //experimental addon to compensate gyro drift for short-term use
  Serial.print("Offsets = {"); Serial.print(acGyOffset[0]); Serial.print(", "); Serial.print(acGyOffset[1]); Serial.print(", "); Serial.print(acGyOffset[2]); Serial.print(", "); Serial.print(acGyOffset[3]); Serial.print(", "); Serial.print(acGyOffset[4]); Serial.print(", "); Serial.print(acGyOffset[5]); Serial.print("};\t Count = "); Serial.println(counter);  // output readable and in copy/paste format
  calibrateMpuBool = false;  // ensure script does not rerun instantly
  delay(500);
}

void setup() {
  Serial.begin(38400);
  delay(1000);
  Wire.begin();

  Wire.beginTransmission(MPU_addr);

  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(50);
  // Configuration of gyroscope
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);  // address of gyro config register
  Wire.write(0b00001000);  // gyro config - 500d/s
  Wire.endTransmission(true);

  // Configuration of the accelerometer
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);  // address of accel config register
  Wire.write(0b00010000);  // accel config - 4g
  Wire.endTransmission(true);

  for(int x = 0; x < 3; x++){  // initialise accel average array (done like this to be entirely variable in length
    for(int y = 0; y < AC_NUM_TO_AVG + 1; y++){
      acReadings[x][y] = 0;
    }
  }
  Serial.println("Press any key now to calibrate MPU.");
  delay(3000);  // allow time for input (gets stored in buffer)
  if(Serial.available() != 0){  // if something is in the buffer
    calibrateMpuBool = true;  // set calibration bool true
  }
}

void loop() {

  if(calibrateMpuBool){  // run calibration if requested (note: will autorun if calibrateMpuBool is initialised 'true')
    calibrateMpu();
  }

  // Accelerometer reading and manipulations
  updateAcReadings();
  updateAcAvg();
  calculateAcRP();

  // Gyroscope reading and manipulations
  updateGyReadings();

  // Calculation of true PRY
  timeNow = micros()/1000000.0;
  timeDelta = timeNow - timePrev;
  pry[0] = gyAcMix * (pry[0] + gyDeltaPRY[0] * timeDelta ) + (1-gyAcMix) * (acPR[0]);
  pry[1] = gyAcMix * (pry[1] + gyDeltaPRY[1] * timeDelta) + (1-gyAcMix) * (acPR[1]);
  pry[2] = pry[2] + gyDeltaPRY[2] * timeDelta;
  timePrev = timeNow;

  // Readable output
  Serial.print("pry[0] = ");
  Serial.print(pry[0]);
  Serial.print("\t");
  Serial.print("pry[1] = ");
  Serial.print(pry[1]);
  Serial.print("\t");
  Serial.print("pry[2] = ");
  Serial.print(pry[2]);
  Serial.println();
  delay(10);
}
