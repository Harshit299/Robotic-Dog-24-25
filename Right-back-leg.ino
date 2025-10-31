#include <Servo.h>
#include <math.h>

// Define servo objects
Servo servo1;  // Base servo
Servo servo2;  // Middle servo
Servo servo3;  // End servo

// Arm segment lengths
const float a1 = 4.5;
const float a2 = 6;
const float a3 = 5.4;

// Function to convert Cartesian coordinates to polar angle (0 to 2π)
float point_to_rad(float p1, float p2) {
  if (p1 > 0 && p2 >= 0) return atan(p2 / p1);
  else if (p1 == 0 && p2 >= 0) return PI / 2;
  else if (p1 < 0 && p2 >= 0) return -fabs(atan(p2 / p1)) + PI;
  else if (p1 < 0 && p2 < 0) return atan(p2 / p1) + PI;
  else if (p1 > 0 && p2 < 0) return -fabs(atan(p2 / p1)) + 2 * PI;
  else if (p1 == 0 && p2 < 0) return 3 * PI / 2;
  else return 3 * PI / 2;  // Edge case
}

// Inverse Kinematics with Rotation and Translation Matrices
void Inverse_kinematics(float x_centre, float y_centre, float z_centre) {

  float x = x_centre - 1;
  float y = y_centre;
  float z = z_centre + 8.5;
  float L = sqrt(x * x + y * y + z * z);

  if (L > (a1 + a2 + a3)) {
    Serial.println("Target is outside the workspace.");
    return;
  }

  float A = sqrt(x * x + y * y);
  float T1;

  Serial.println("Leg is RIGHT");
  T1 = point_to_rad(x, y) + acos(a1 / A);
  if (T1 > PI / 2) T1 -= 2 * PI;


  // Rotation and Translation Matrices
  float rot[4][4] = {
    { cos(-T1), -sin(-T1), 0, 0 },
    { sin(-T1), cos(-T1), 0, 0 },
    { 0, 0, 1, 0 },
    { 0, 0, 0, 1 }
  };

  float trans[4][4] = {
    { 1, 0, 0, -a1 * cos(T1) },
    { 0, 1, 0, -a1 * sin(T1) },
    { 0, 0, 1, 0 },
    { 0, 0, 0, 1 }
  };

  // Coordinate Matrix
  float actual_coord[4] = { x, y, z, 1 };
  float temp_coord[4] = { 0, 0, 0, 1 };
  float transformed_coord[4] = { 0, 0, 0, 1 };

  // Apply Transformation: temp = trans * actual
  for (int i = 0; i < 4; i++) {
    temp_coord[i] = 0;
    for (int j = 0; j < 4; j++) {
      temp_coord[i] += trans[i][j] * actual_coord[j];
    }
  }

  // Apply Rotation: transformed = rot * temp
  for (int i = 0; i < 4; i++) {
    transformed_coord[i] = 0;
    for (int j = 0; j < 4; j++) {
      transformed_coord[i] += rot[i][j] * temp_coord[j];
    }
  }

  float xdash = transformed_coord[0];
  float ydash = transformed_coord[1];
  float zdash = transformed_coord[2];

  // Calculate T2 and T3
  float B = sqrt(ydash * ydash + zdash * zdash);
  float alpha1 = point_to_rad(ydash, zdash);
  float beta1 = acos((B * B + a2 * a2 - a3 * a3) / (2 * B * a2));
  float T2 = alpha1 - beta1;
  float beta2 = acos((a2 * a2 + a3 * a3 - B * B) / (2 * a2 * a3));
  float T3 = PI - beta2;

  // Convert radians to degrees
  T1 = degrees(T1)-5;
  T2 = degrees(T2);
  T3 = degrees(T3)+6;

  Serial.print("T1 = ");
  Serial.println(T1);
  Serial.print("T2 = ");
  Serial.println(T2);
  Serial.print("T3 = ");
  Serial.println(T3);

  int angle1 = T1;
  int angle2 = T2;
  int angle3 = T3;

  // Move Servos
  servo1.write(angle1);
  servo2.write(angle2);
  servo3.write(angle3);
}

void setup() {
  Serial.begin(9600);

  // Attach servos to pins
  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);
  servo1.write(0);
  servo2.write(0);
  servo3.write(0);
}


void loop() {
  if(Serial.available()){
    Serial.println("Enter X_centre: ");
    float x_centre = Serial.parseFloat();
    Serial.println(x_centre);
    while (Serial.available()) {  // Clear any leftover characters
            Serial.read();
        }
    Serial.println("Enter Y_centre: ");
    float y_centre = Serial.parseFloat();
    Serial.println(y_centre);
    while (Serial.available()) {  // Clear any leftover characters
            Serial.read();
        }
    Serial.println("Enter Z_centre: ");
    float z_centre = Serial.parseFloat();
    Serial.println(z_centre);
    while (Serial.available()) {  // Clear any leftover characters
            Serial.read();
        }

    Inverse_kinematics(x_centre,y_centre,z_centre);
};