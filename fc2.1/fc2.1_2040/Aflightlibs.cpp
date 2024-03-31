#include "Aflightlibs.h"
#include "Arduino.h"
#include <SerialFlash.h>
#include "imuFilter.h"
#include "Orientation/Orientation.h"
#include "Orientation/Quaternion.h"
#include "Orientation/Orientation.cpp"
#include "Orientation/Quaternion.cpp"
#include "math.h"
//#include "vars.h"
//#define GET_FIELD(regname,value) ((value & regname##_MASK) >> regname##_POS)
//#define  SET_FIELD(regval,regname,value) ((regval & ~regname##_MASK) | ((value << regname##_POS) & regname##_MASK))

//LittleFSConfig cfg;
//cfg.setAutoFormat(false);
//LittleFS.setConfig(cfg);

float x, y, z;
float _pitch, _roll, _yaw;
float accelZ; //
float pyroTime = 250; //time for pyro channel to stay on
float pyroPin1 = 10;
float pyroPin2 = 11;
float pyroPin3 = 12;
float filteredaccely;
float accelalt;
float accelspeedz;
float _gravity;
boolean LEDEN, BUZEN;

float _Kp = 30.0;
float _Ki = 0.0;

uint64_t thisLoopMicros = 0; // Stores microsecond timestamp for current loop
uint64_t lastOriUpdate = 0; // Stores mircosecond timestamp for last time orientation updated

struct quaternion {
  float r;    /* real bit */
  float x, y, z;  /* imaginary bits */
};

quaternion masterQuaternion;
quaternion tempQuaternion;
float angX, angY, angZ;

float q[4] = {1.0, 0.0, 0.0, 0.0};

imuFilter quat;

Orientation ori; // Main orientation measurement
EulerAngles oriMeasure; // Quaternion converted to Euler Angles for maths etc.

//TVCClass::TVCClass(void){}

TVCClass::TVCClass() {
  LEDEN = true;
  BUZEN = true;
  pinMode(10, OUTPUT); //pyros
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  pinMode(18, OUTPUT); //leds
  pinMode(19, OUTPUT);
  pinMode(20, OUTPUT);
  thisLoopMicros = lastOriUpdate = micros(); // Set starting time after init/calibration
  quat.setup();
}

TVCClass::TVCClass(boolean _LEDEN, boolean _BUZEN) {
  LEDEN = _LEDEN;
  BUZEN = _BUZEN;
  pinMode(10, OUTPUT); //pyros
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  pinMode(18, OUTPUT); //leds
  pinMode(19, OUTPUT);
  pinMode(20, OUTPUT);
  thisLoopMicros = lastOriUpdate = micros(); // Set starting time after init/calibration
  quat.setup();
}

void TVCClass::pyro1() {
  static int olderMillis = millis();
  static boolean fired = false;
  digitalWrite(pyroPin1, HIGH);
  if (millis() - olderMillis >= pyroTime) {
    if (fired == false) {
      digitalWrite(pyroPin1, LOW);
      fired = true;
    }
  }
}

void TVCClass::pyro2() {
  static int olderMillis = millis();
  static boolean fired = false;
  digitalWrite(pyroPin2, HIGH);
  if (millis() - olderMillis >= pyroTime) {
    if (fired == false) {
      digitalWrite(pyroPin2, LOW);
      fired = true;
    }
  }
}

void TVCClass::pyro3() {
  static int olderMillis = millis();
  static boolean fired = false;
  digitalWrite(pyroPin3, HIGH);
  if (millis() - olderMillis >= pyroTime) {
    if (fired == false) {
      digitalWrite(pyroPin3, LOW);
      fired = true;
    }
  }
}

float TVCClass::anglex() {
  return _yaw;
}

float TVCClass::angley() {
  return _roll;
}

float TVCClass::anglez() {
  return _pitch;
}

float TVCClass::pitch() {
  return _pitch;
}

float TVCClass::roll() {
  return _roll;
}

float TVCClass::yaw() {
  return _yaw;
}

float TVCClass::filteredAccelY() {
  return filteredaccely;
}

float TVCClass::accelz() {
  return accelZ;
}

float TVCClass::accelAlt() {
  return accelalt;
}

float TVCClass::accelzSpeed() {
  return accelspeedz;
}

void TVCClass::estimateAngles(float opMode, float gain, float ax, float ay, float az, float gx, float gy, float gz) {
  float Kd = 0;
  //rp2040.idleOtherCore();
  float xerror;
  float zerror;
  static float xprev_error;
  static float zprev_error;
  static float xcumulative_error;
  static float zcumulative_error;
  static float pitchOffset;
  static float yawOffset;
  float accelAnglex;
  float accelAnglez;
  float xival;
  float xdval;
  float zival;
  float zdval;
  static int olderMillis;
  static int gravCount = 0;
  float dt = millis() - olderMillis;
  thisLoopMicros = micros(); // Get new microsecond timestamp for this loop

  float dtOri = (float)(thisLoopMicros - lastOriUpdate) / 1000000.; // Finds elapsed microseconds since last update, converts to float, and converts to seconds
  lastOriUpdate = thisLoopMicros; // We have updated, set the new timestamp

  accelAnglex = -(atan2(az, sqrt(pow(ax, 2) + pow(ay, 2))) * 180 / PI);
  accelAnglez = (atan2(ax, sqrt(pow(ay, 2) + pow(az, 2))) * 180 / PI);
  //accelAnglex = -(atan(az/ay) * (180/PI));
  //accelAnglez = (atan(ax/ay) * (180/PI));
  if (gravCount <= 500) {
    _gravity = sqrt(sq(sqrt(sq(ax) + sq(ay))) + sq(az));
    gravCount += 1;
  }
  accelZ = (sqrt(sq(sqrt(sq(ax) + sq(ay))) + sq(az)) - _gravity);
  accelspeedz += accelZ;
  accelalt += accelspeedz;

  //  quat.update(gx, gy, gz);
  //  Mahony_update(ax, ay, az, gx, gy, gz, dt);

  //_roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2])) * 0.73 * RAD_TO_DEG;
  //_pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3])) * 0.73 * RAD_TO_DEG;
  //_yaw   = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3])) * 0.73 * RAD_TO_DEG;

  _pitch += (gx * (M_PI / 180) * ((4078 / 71) * (dt / 10) * 0.73));  //0.6 mult
  _roll += (gy * (M_PI / 180) * ((4078 / 71) * (dt / 10) * 0.73));
  _yaw += (gz * (M_PI / 180) * ((4078 / 71) * (dt / 10) * 0.73));

  //  _pitch = (quat.pitch() * 0.73 * RAD_TO_DEG) + pitchOffset;
  //  _roll = quat.roll();
  //  _yaw = (quat.yaw() * 0.73 * RAD_TO_DEG) + yawOffset;

  if (opMode <= 0) {
    _pitch = accelAnglex;
    _roll = 0;
    ori.zeroRoll();
    _yaw = accelAnglez;
    pitchOffset = _pitch;
    yawOffset = _yaw;
  } else {
    ori.update(gz, gx, gy, dtOri); // '* DEG_TO_RAD' after all gyro functions if they return degrees/sec
    oriMeasure = ori.toEuler();
    //_pitch = oriMeasure.pitch * RAD_TO_DEG + pitchOffset;
    //_pitch = oriMeasure.pitch * RAD_TO_DEG;
    //_roll = oriMeasure.roll * RAD_TO_DEG;
    //_yaw = oriMeasure.yaw * RAD_TO_DEG;
    //_roll = sqrt(pow((oriMeasure.roll * RAD_TO_DEG), 2)) / 180; //1 - angle * roll as a decimal
    //_roll = sqrt(pow(sin(_roll/(PI * 2) * 45), 2)); //man idek
    //_yaw = oriMeasure.yaw * RAD_TO_DEG + yawOffset;
  }
  //  else {
  //    static boolean Setup = false;
  //    if (Setup == false) {
  //      pitchOffset = accelAnglex;
  //      yawOffset = accelAnglez;
  //      Setup = true;
  //    }
  //  }
  float ALPHA = 0.1; //0.2
  float currentaccely = ay;
  static float previousaccely = 0.0;

  filteredaccely = ALPHA * currentaccely + (1 - ALPHA) * previousaccely;
  previousaccely = filteredaccely;

  //baroVel();
  //smoothedAlt();

  //filteredaccely = 0;

  //  xerror = accelAnglex - x;
  //  xcumulative_error += xerror * dt;
  //  xival = (xcumulative_error * gain) / 10000;
  //  xdval = ((xerror - xprev_error) / dt) * Kd;
  //  x += xival;
  //
  //  zerror = accelAnglez - z;
  //  zcumulative_error += zerror * dt;
  //  zival = (zcumulative_error * gain) / 10000;
  //  xdval = ((zerror - zprev_error) / dt) * Kd;
  //  z += zival;
  //  if (opMode == 69) {
  //    if (_pitch <= accelAnglex) {
  //      _pitch += (gain * (dt / 10));
  //    }
  //    if (_pitch >= accelAnglex) {
  //      _pitch -= (gain * (dt / 10));
  //    }
  //    if (_yaw <= accelAnglez) {
  //      _yaw += (gain * (dt / 10));
  //    }
  //    if (_yaw >= accelAnglez) {
  //      _yaw -= (gain * (dt / 10));
  //    }
  //  }
  //Serial.println(dt);
  xprev_error = xerror;
  zprev_error = zerror;
  olderMillis = millis();
}

void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  //error terms
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
  float tmp;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)

  //  tmp = ax * ax + ay * ay + az * az;
  tmp = 0.0; //IGNORE ACCELEROMETER

  // ignore accelerometer if false (tested OK, SJR)
  if (tmp > 0.0)
  {

    // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
    recipNorm = 1.0 / sqrt(tmp);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity in the body frame (factor of two divided out)
    vx = q[1] * q[3] - q[0] * q[2];
    vy = q[0] * q[1] + q[2] * q[3];
    vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error is cross product between estimated and measured direction of gravity in body frame
    // (half the actual magnitude)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Compute and apply to gyro term the integral feedback, if enabled
    if (_Ki > 0.0f) {
      ix += _Ki * ex * deltat;  // integral error scaled by _Ki
      iy += _Ki * ey * deltat;
      iz += _Ki * ez * deltat;
      gx += ix;  // apply integral feedback
      gy += iy;
      gz += iz;
    }

    // Apply proportional feedback to gyro term
    gx += _Kp * ex;
    gy += _Kp * ey;
    gz += _Kp * ez;
  }

  // Integrate rate of change of quaternion, given by gyro term
  // rate of change = current orientation quaternion (qmult) gyro rate

  deltat = 0.5 * deltat;
  gx *= deltat;   // pre-multiply common factors
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];

  //add qmult*delta_t to current orientation
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] = q[0] * recipNorm;
  q[1] = q[1] * recipNorm;
  q[2] = q[2] * recipNorm;
  q[3] = q[3] * recipNorm;
}

void convertStringToNumbers(const String& inputString, float values[], int maxLength) {
  int count = 0;
  String currentNumber = "";

  for (int i = 0; i < inputString.length(); i++) {
    char c = inputString.charAt(i);

    if (c == '-' || i == inputString.length() - 1) {
      if (currentNumber.length() > 0) {
        values[count] = atof(currentNumber.c_str());
        count++;
        currentNumber = "";

        if (count >= maxLength) {
          break;
        }
      }
    } else {
      currentNumber += c;
    }
  }

  // Fill any remaining values with zero
  while (count < maxLength) {
    values[count] = 0.0f;
    count++;
  }
}

//void gyroPrelaunch() {
//
//  // set the baseline quaternion tilt values using accel data before launch
//  masterQuaternion = { .r = 1, .x = 0, .y = 0, .z = 0 };
//
//  LSM6.readAccelData(accelRead); //populate accelRead[3] array with raw sample data from accelerometer
//  Ax = accelRead[0] + AxOffset;
//  Ay = accelRead[1] + AyOffset;
//  Az = accelRead[2] + AzOffset;
//
//  double a1 = 0.00, a2 = 0.00;       //Roll & _pitch are the angles which rotate by the axis X and y
//  double x_Buff = float(Az);
//  double y_Buff = float(Ax);
//  double z_Buff = float(Ay);
//  //set angles for _pitch and yaw
//  a1 = atan2(y_Buff , z_Buff) * 57.3; //_pitch
//  a2 = atan2((- x_Buff) , sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3; //yaw
//
//  //now integrate the angles into our freshly minted master quaternion (ignoring roll)
//  gyroRotate(0, a1, a2);
//
//}
//
//void gyroRotate(float gRoll, float g_pitch, float gYaw) {
//
//  // R, P, Y inputs should be degrees factored for time interval
//
//  //first convert the gyro tiny rotation into a half euler and store it in a temporary quaternion
//  quaternionInitHalfEuler(gRoll, g_pitch, gYaw);
//
//  //now combine it with the masterQuaternion to get an integrated rotation
//  quaternionMultiply();
//
//  //now normalize the masterQuaternion in case it went out of bounds
//  quaternionNormalize();
//
//  //now we need to translate the masterQuaternion back to angles that humans can read
//  quaternionAxisAngles();
//}
//
//
//void quaternionAxisAngles() {
//
//  quaternion p = masterQuaternion;
//
//  // Compute the X (roll) angle in radians
//  if ((1 - 2 * (p.x * p.x + p.y * p.y)) != 0) {
//    angX = atan(2 * (p.r * p.x + p.y * p.z) / (1 - 2 * (p.x * p.x + p.y * p.y)));
//  } else {
//    if ((p.r * p.x + p.y * p.z) > 0) {
//      angX = M_PI / 2.0f;
//    } else {
//      if ((p.r * p.x + p.y * p.z) < 0) {
//        angX = -1.0f * M_PI / 2.0f;
//      } else {
//        angX = 9999999;
//        // SIGNAL ERROR DIVIDE BY ZERO!!!
//      }
//    }
//  }
//  // Convert x (roll) from radian to degrees
//  angX = angX * 180.0f / M_PI;
//
//
//  //Compute the Y (_pitch) angle in radians
//  if ((2 * (p.x * p.y - p.z * p.x)) <= -1) {
//    angY = -1.0f * M_PI / 2.0f;
//  } else {
//    if ((2 * (p.r * p.y - p.z * p.x)) >= 1) {
//      angY = M_PI / 2.0f;
//    } else {
//      angY = asin(2 * (p.r * p.y - p.z * p.x));
//    }
//  }
//  // Convert y (_pitch) from radian to degrees
//  angY = angY * 180.0f / M_PI;
//
//
//  // Compute the Z (Yaw) angle in radians
//  if ((1 - 2 * (p.x * p.x + p.y * p.y)) != 0) {
//    angZ = atan(2 * (p.r * p.z + p.x * p.y) / (1 - 2 * (p.y * p.y + p.z * p.z)));
//  } else {
//    if ((p.r * p.z + p.x * p.y) > 0) {
//      angZ = M_PI / 2.0f;
//    } else {
//      if ((p.r * p.z + p.x * p.y) < 0) {
//        angZ = -1.0f * M_PI / 2.0f;
//      } else {
//        angZ = 9999999;
//        // SIGNAL ERROR DIVIDE BY ZERO!!!
//      }
//    }
//  }
//  // Convert z (Yaw) from radian to degrees
//  angZ = angZ * 180.0f / M_PI;
//
//}
//
//void quaternionNormalize() {
//
//  quaternion p = masterQuaternion;
//  float testP = 0.0;
//
//  testP = (p.r * p.r) + (p.x * p.x) + (p.y * p.y) + (p.z * p.z);
//  if (testP > 1.0) {
//    p.r = p.r * (float)(1.0f / sqrtf(testP));
//    p.x = p.x * (float)(1.0f / sqrtf(testP));
//    p.y = p.y * (float)(1.0f / sqrtf(testP));
//    p.z = p.z * (float)(1.0f / sqrtf(testP));
//  }
//  masterQuaternion = p;
//}
//
//void quaternionMultiply() {
//
//  // combine t with the masterQuaternion to get an integrated rotation
//  quaternion p = masterQuaternion;
//  quaternion t = tempQuaternion;
//
//  masterQuaternion.r = (p.r * t.r) + (-p.x * t.x) + (-p.y * t.y) + (-p.z * t.z);
//  masterQuaternion.x = (p.r * t.x) + (p.x * t.r) + (p.y * t.z) + (-p.z * t.y);
//  masterQuaternion.y = (p.r * t.y) + (-p.x * t.z) + (p.y * t.r) + (p.z * t.x);
//  masterQuaternion.z = (p.r * t.z) + (p.x * t.y) + (-p.y * t.x) + (p.z * t.r);
//
//}
//
//
//void quaternionInitHalfEuler(float gRoll, float g_pitch, float gYaw) {
//
//  // store the tiny rotation from the gyro in a new temporary quaternion
//  // roll, _pitch, yaw input is in degrees
//
//  float s_x, c_x;
//  float s_y, c_y;
//  float s_z, c_z;
//
//  float x = (gRoll / 2.0f) * M_PI / 180.0f; // half the degree and convert it to radians
//  float y = (g_pitch / 2.0f) * M_PI / 180.0f;
//  float z = (gYaw / 2.0f) * M_PI / 180.0f;
//
//  s_x = sin(x); c_x = cos(x);
//  s_y = sin(y); c_y = cos(y);
//  s_z = sin(z); c_z = cos(z);
//
//  // This is our quaternion that represents the rotation difference
//  tempQuaternion.r = c_x * c_y * c_z + s_x * s_y * s_z;
//  tempQuaternion.x = s_x * c_y * c_z - c_x * s_y * s_z;
//  tempQuaternion.y = c_x * s_y * c_z + s_x * c_y * s_z;
//  tempQuaternion.z = c_x * c_y * s_z - s_x * s_y * c_z;
//
//}

void ledr(int input) {
  if (LEDEN == true) {
    if (input == 1) {
      digitalWrite(18, LOW);
    }
    if (input == 0) {
      digitalWrite(18, HIGH);
    }
    //  else if ((input >= 0) & (input <= 255)){
    //    analogWrite(18, 255 - input);
    //  }
  }
  else {
    digitalWrite(18, HIGH);
  }
}

void ledg(int input) {
  if (LEDEN == true) {
    if (input == 1) {
      digitalWrite(19, LOW);
    }
    if (input == 0) {
      digitalWrite(19, HIGH);
    }
    //  else if ((input >= 0) & (input <= 255)){
    //    analogWrite(19, 255 - input);
    //  }
  }
  else {
    digitalWrite(19, HIGH);
  }
}

void ledb(int input) {
  if (LEDEN == true) {
    if (input == 1) {
      digitalWrite(20, LOW);
    }
    if (input == 0) {
      digitalWrite(20, HIGH);
    }
    //  else if ((input >= 0) & (input <= 255)){
    //    analogWrite(20, 255 - input);
    //  }
  }
  else {
    digitalWrite(20, HIGH);
  }
}

void buz(int input) {
  if (BUZEN == true) {
    if (input == 1 ) {
      tone(9, 200, 5);
    }
    else {

    }
  }
  else {
    digitalWrite(9, LOW);
  }
}

//TVCClass TVC;
