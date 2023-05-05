#include "Aflightlibs.h"
#include "Arduino.h"
#include <SerialFlash.h>

//#define GET_FIELD(regname,value) ((value & regname##_MASK) >> regname##_POS)
//#define  SET_FIELD(regval,regname,value) ((regval & ~regname##_MASK) | ((value << regname##_POS) & regname##_MASK))

float x, y, z;
float _pitch, _roll, _yaw;
float pyroTime = 250; //time for pyro channel to stay on
float pyroPin1 = 10;
float pyroPin2 = 11;
float pyroPin3 = 12;
float filteredaccely;
boolean LEDEN, BUZEN;

#include "math.h"
struct quaternion {
  float r;    /* real bit */
  float x, y, z;  /* imaginary bits */
};
quaternion masterQuaternion;
quaternion tempQuaternion;
float angX, angY, angZ;

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

void TVCClass::estimateAngles(float opMode, float gain, float ax, float ay, float az, float gx, float gy, float gz) {
  float Kd = 0;
  //rp2040.idleOtherCore();
  float xerror;
  float zerror;
  static float xprev_error;
  static float zprev_error;
  static float xcumulative_error;
  static float zcumulative_error;
  float accelAnglex;
  float accelAnglez;
  float xival;
  float xdval;
  float zival;
  float zdval;
  static int olderMillis;
  int dt = millis() - olderMillis;
  accelAnglex = -(atan2(az, sqrt(pow(ax, 2) + pow(ay, 2))) * 180 / PI);
  accelAnglez = (atan2(ax, sqrt(pow(ay, 2) + pow(az, 2))) * 180 / PI);
  //accelAnglex = -(atan(az/ay) * (180/PI));
  //accelAnglez = (atan(ax/ay) * (180/PI));
  _pitch += (gx * (M_PI / 180) * ((4078 / 71) * (dt / 10) * 0.73));  //0.6
  _roll += (gy * (M_PI / 180) * ((4078 / 71) * (dt / 10) * 0.73));
  _yaw += (gz * (M_PI / 180) * ((4078 / 71) * (dt / 10) * 0.73));
  if (opMode <= 0) {
    _pitch = accelAnglex;
    _roll = 0;
    _yaw = accelAnglez;
  }

  float ALPHA = 0.2;
  float currentaccely = ay;
  static float previousaccely = 0.0;

  filteredaccely = ALPHA * currentaccely + (1 - ALPHA) * previousaccely;
  previousaccely = filteredaccely;

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
  if (opMode == 69) {
    if (_pitch <= accelAnglex) {
      _pitch += (gain * (dt / 10));
    }
    if (_pitch >= accelAnglex) {
      _pitch -= (gain * (dt / 10));
    }
    if (_yaw <= accelAnglez) {
      _yaw += (gain * (dt / 10));
    }
    if (_yaw >= accelAnglez) {
      _yaw -= (gain * (dt / 10));
    }
  }
  //Serial.println(dt);
  xprev_error = xerror;
  zprev_error = zerror;
  olderMillis = millis();
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
//  double a1 = 0.00, a2 = 0.00;       //Roll & Pitch are the angles which rotate by the axis X and y
//  double x_Buff = float(Az);
//  double y_Buff = float(Ax);
//  double z_Buff = float(Ay);
//  //set angles for pitch and yaw
//  a1 = atan2(y_Buff , z_Buff) * 57.3; //pitch
//  a2 = atan2((- x_Buff) , sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3; //yaw
//
//  //now integrate the angles into our freshly minted master quaternion (ignoring roll)
//  gyroRotate(0, a1, a2);
//
//}
//
//void gyroRotate(float gRoll, float gPitch, float gYaw) {
//
//  // R, P, Y inputs should be degrees factored for time interval
//
//  //first convert the gyro tiny rotation into a half euler and store it in a temporary quaternion
//  quaternionInitHalfEuler(gRoll, gPitch, gYaw);
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
//  //Compute the Y (pitch) angle in radians
//  if ((2 * (p.x * p.y - p.z * p.x)) <= -1) {
//    angY = -1.0f * M_PI / 2.0f;
//  } else {
//    if ((2 * (p.r * p.y - p.z * p.x)) >= 1) {
//      angY = M_PI / 2.0f;
//    } else {
//      angY = asin(2 * (p.r * p.y - p.z * p.x));
//    }
//  }
//  // Convert y (pitch) from radian to degrees
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
//void quaternionInitHalfEuler(float gRoll, float gPitch, float gYaw) {
//
//  // store the tiny rotation from the gyro in a new temporary quaternion
//  // roll, pitch, yaw input is in degrees
//
//  float s_x, c_x;
//  float s_y, c_y;
//  float s_z, c_z;
//
//  float x = (gRoll / 2.0f) * M_PI / 180.0f; // half the degree and convert it to radians
//  float y = (gPitch / 2.0f) * M_PI / 180.0f;
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
  else{
    digitalWrite(18,HIGH);
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
  else{
    digitalWrite(19,HIGH);
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
  else{
    digitalWrite(20,HIGH);
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
  else{
    digitalWrite(9,LOW);
  }
}

//TVCClass TVC;
