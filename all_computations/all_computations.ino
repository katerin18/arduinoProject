#include <Wire.h>

const int multiplexerAddress = 0x70;
const int MPU_ADDR = 0x68;

const int numSensors = 5;
const int STRIPE_LEN = 9;
const int DISTINCTION_LIMIT = 35;

double angles[numSensors][2];  // [sensor index][0 - angle X, 1 - angle Y]
double points[numSensors][2];
double base_control_points[numSensors][2] = { {0.00, 0.00}, {0.78,	8.12}, {-10.23,	6.41}, {1.23, 5.07}, {3.51, 9.43} };  // for 175cm height

double S1_x, S1_y;
double height_base = 175;

const int redPin = 9;
const int greenPin = 10;
const int bluePin = 11;

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  analogWrite(redPin, 255);
  analogWrite(greenPin, 255);
  analogWrite(bluePin, 255);

  Wire.begin();
  Serial.begin(9600);

  points[0][0] = 0.00;  // x coord of first point
  points[0][1] = 0.00;  // y coord of second point

  // MPU6050 initialization on each MUX channel
  for (int i = 0; i < numSensors; i++) {
    selectMultiplexerChannel(i);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);  // Register PWR_MGMT_1
    Wire.write(0);     // Set SLEEP to 0 (wake up MPU6050)
    Wire.endTransmission(true);
  }
}

void loop() {
  for (uint8_t i = 0; i < numSensors; i++) {
    selectMultiplexerChannel(i);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);  // Data register of accelerometer
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    // Read data from accelerometer and gyroscope
    int16_t accX = Wire.read() << 8 | Wire.read();
    int16_t accY = Wire.read() << 8 | Wire.read();
    int16_t accZ = Wire.read() << 8 | Wire.read();
    int16_t tempRaw = Wire.read() << 8 | Wire.read();
    int16_t gyroX = Wire.read() << 8 | Wire.read();
    int16_t gyroY = Wire.read() << 8 | Wire.read();
    int16_t gyroZ = Wire.read() << 8 | Wire.read();

    // Angles computing
    angles[i][0] = atan2(accY, accZ) * 180.0 / PI;  // Angle X
    angles[i][1] = atan2(accX, accZ) * 180.0 / PI;  // Angle Y
  }
  
  double cur_x = points[0][0];
  double cur_y = points[0][1];
  double next_x, next_y;

  for (uint8_t i = 0; i < numSensors - 1; i++) {
    double angleS0 = angles[i][0];
    double angleS1 = angles[i + 1][0];    

    getNextCoordinates(angleS0, angleS1, cur_x, cur_y, next_x, next_y);

    cur_x = next_x;
    cur_y = next_y;

    points[i + 1][0] = cur_x;
    points[i + 1][1] = cur_y;
  }
  Serial.println("Points coordinates: ");
  printMatrix(points);

  double health_curve[5][2];
  getIdealCoordinates(base_control_points, 175, health_curve);

  Serial.println("Ideal coordinates: ");
  printMatrix(health_curve);

  bool isPostureOk = isDistinctionOk(health_curve, points);
  controlRGB(isPostureOk);
}

void selectMultiplexerChannel(byte channel) {
  Wire.beginTransmission(multiplexerAddress);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void getNextCoordinates(double angle_S0, double angle_S1, double S0_x, double S0_y, double& S1_x, double& S1_y) {
  double start = 0;
  double end = 1000;
  double M_x, M_y;
  double lengthCurve = 0;

  // binary search
  while (end - start > 0.001) {
    double k = (end + start) / 2;

    testK(angle_S0, angle_S1, S0_x, S0_y, k, M_x, M_y, S1_x, S1_y);

    lengthCurve = lengthBezier(S0_x, S0_y, S1_x, S1_y, M_x, M_y);

    if (lengthCurve < STRIPE_LEN) {
      start = k;
    } else if (lengthCurve > STRIPE_LEN) {
      end = k;
    } else {
      return;
    }
  }
}

double lengthBezier(double S0_x, double S0_y, double S1_x, double S1_y, double M_x, double M_y) {
  double a_x = S0_x - 2 * M_x + S1_x;
  double a_y = S0_y - 2 * M_y + S1_y;
  double b_x = 2 * (M_x - S1_x);
  double b_y = 2 * (M_y - S1_y);
  double A = 4 * (pow(a_x, 2) + pow(a_y, 2));
  double B = 4 * (a_x * b_x + a_y * b_y);
  double C = pow(b_x, 2) + pow(b_y, 2);

  double length = 1 / (8 * pow(A, 1.5)) * (4 * pow(A, 1.5) * sqrt(A + B + C) + 2 * sqrt(A) * B * (sqrt(A + B + C) - sqrt(C)) + (4 * C * A - pow(B, 2)) * log(abs((2 * sqrt(A) + B / sqrt(A) + 2 * sqrt(A + B + C)) / (B / sqrt(A) + 2 * sqrt(C)))));

  return length;
}


void testK(double angle_S0, double angle_S1, double S0_x, double S0_y, float k, double& m_x, double& m_y, double& S1_x, double& S1_y) {
  double d0_x = cos(angle_S0);
  double d0_y = sin(angle_S0);
  double d1_x = cos(angle_S1);
  double d1_y = sin(angle_S1);

  m_x = S0_x + k * d0_x;
  m_y = S0_y + k * d0_y;

  S1_x = m_x - k * d1_x;
  S1_y = m_y - k * d1_y;
}

void getIdealCoordinates(double base_points[numSensors][2], double cur_height, double (&health_curve)[numSensors][2]) {
  double scale_factor = cur_height / height_base;

  for (int i = 0; i < numSensors; i++) {
    health_curve[i][0] = base_points[i][0];
    health_curve[i][1] = base_points[i][1] * scale_factor;
  }
}

bool isDistinctionOk(double health_curve[numSensors][2], double cur_curve[numSensors][2]) {
  double totalDeviation = 0.0;
  // euclidian distinction between points on two curves
  for (int i = 0; i < numSensors; i++) {
    double dx = health_curve[i][0] - cur_curve[i][0];
    double dy = health_curve[i][1] - cur_curve[i][1];
    double distance = sqrt(dx * dx + dy * dy);
    totalDeviation += distance;
  }
  Serial.print("LIMIT OF DISTINCTION: \t");
  Serial.println(DISTINCTION_LIMIT);
  Serial.print("COMPUTED DISTINCTION: \t");
  Serial.println(totalDeviation);

  return abs(totalDeviation) < DISTINCTION_LIMIT;
}

void controlRGB(bool isPostureOk) {
  analogWrite(bluePin, 0);

  if (isPostureOk) {
    Serial.println("POSTURE IS OK!");
    analogWrite(redPin, 0);
    analogWrite(greenPin, 255);
  } else {
    Serial.println("POSTURE IS NOT OK!");
    analogWrite(greenPin, 0);
    analogWrite(redPin, 255);
  }
}

void printMatrix(double matrix[numSensors][2]) {
  for (int i = 0; i < numSensors; i++) {
    for (int j = 0; j < 2; j++) {
      Serial.print(matrix[i][j]);
      Serial.print("\t");
    }
    Serial.println();
  }
}

void printAngles(double anglesList[numSensors][2]) {
  for (uint8_t i = 0; i < numSensors; i++) {
    Serial.print("Датчик ");
    Serial.print(i);
    Serial.print(": X = ");
    Serial.print(anglesList[i][0]);
    Serial.print(", Y = ");
    Serial.println(anglesList[i][1]);
  }
}