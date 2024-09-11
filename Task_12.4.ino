#include <PID_v1.h>

const int motorControlPin = 9;
const int encoderChannelA = 2;
const int encoderChannelB = 3;

volatile int encoderCount = 0;
unsigned long lastRPMCalculationTime = 0;

double targetRPM = 100;
double currentRPM = 0;
double pidOutput = 0;

double PG = 2.0, IG = 5.0, DG = 1.0;

PID myPID(&currentRPM, &pidOutput, &targetRPM, PG, IG, DG, DIRECT);

double smoothedOutput = 0;
double smoothingCoefficient = 0.1;

class PIDController {
public:
  PIDController(double* inputRPM, double* outputValue, double* desiredRPM, double kp, double ki, double kd)
    : rpmInput(inputRPM), controlOutput(outputValue), setpoint(desiredRPM), kp(kp), ki(ki), kd(kd), pid(inputRPM, outputValue, desiredRPM, kp, ki, kd, DIRECT) {
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(0, 255);
  }

  void Compute() {
    pid.Compute();
  }

private:
  double* rpmInput;
  double* controlOutput;
  double* setpoint;
  double kp, ki, kd;
  PID pid;
};

PIDController motorPID(&currentRPM, &pidOutput, &targetRPM, PG, IG, DG);

void setup() {
  pinMode(motorControlPin, OUTPUT);
  pinMode(encoderChannelA, INPUT_PULLUP);
  pinMode(encoderChannelB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderChannelA), encoderISR, RISING);

  Serial.begin(9600);
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastRPMCalculationTime >= 1000) {
    static unsigned long previousTime = 0;
    static int previousCount = 0;
    unsigned long currentTime = millis();
    int countDifference = encoderCount - previousCount;
    previousCount = encoderCount;
    double timeDifference = (currentTime - previousTime) / 60000.0;
    previousTime = currentTime;
    currentRPM = (countDifference / 20.0) / timeDifference;
    lastRPMCalculationTime = currentTime;
    Serial.print("RPM: ");
    Serial.println(currentRPM);
  }
  motorPID.Compute();
  smoothedOutput = (smoothingCoefficient * pidOutput) + ((1 - smoothingCoefficient) * smoothedOutput);
  analogWrite(motorControlPin, smoothedOutput);
}

void encoderISR() {
  if (digitalRead(encoderChannelA) == digitalRead(encoderChannelB)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}
