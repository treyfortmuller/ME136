#include "UserCode.hpp"
#include "UtilityFunctions.hpp"
#include "Vec3f.hpp"

#include <stdio.h> //for printf

//An example of a variable that persists beyond the function call.
float exampleVariable_float = 0.0f;  //Note the trailing 'f' in the number. This is to force single precision floating point.
Vec3f exampleVariable_Vec3f = Vec3f(0, 0, 0);
int exampleVariable_int = 0;

//We keep the last inputs and outputs around for debugging:
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

//Some constants that we may use:
const float mass = 30e-3f;  // mass of the quadcopter [kg]
const float gravity = 9.81f;  // acceleration of gravity [m/s^2]
const float inertia_xx = 16e-6f;  //MMOI about x axis [kg.m^2]
const float inertia_yy = inertia_xx;  //MMOI about y axis [kg.m^2]
const float inertia_zz = 29e-6f;  //MMOI about z axis [kg.m^2]

const float dt = 1.0f / 500.0f; //[s] period between successive calls to MainLoop
Vec3f estGyroBias = Vec3f(0,0,0);
Vec3f rateGyro_corr = Vec3f(0,0,0);
float estRoll = 0;
float estPitch = 0;
float estYaw = 0;
float p = 0.01f; //rho, the gyro/accel trade-off scalar default value: .01

// controller variable initialization
Vec3f cmdAngAcc = Vec3f(0,0,0);
Vec3f desAngVel = Vec3f(0,0,0);

Vec3f desAng = Vec3f(0,0,0);
Vec3f cmdAngVel = Vec3f(0,0,0);

// propeller distance:
float l = 33e-3f;

// propeller coefficient
float k = 0.01f;

//desired normalized total thrust:
float norm_tot_thrust = 8.0f;

// desired force
float des_total_force = mass * norm_tot_thrust;

// time constant for controllers on each axis
// D gains on each axis
float const timeConstant_rollRate = 0.04f; // [s]
float const timeConstant_pitchRate = timeConstant_rollRate;
float const timeConstant_yawRate = 0.1f; // [s] (CHANGED! 5.1.2, 0.5f->0.1f)

// P gains on each axis
float const timeConstant_rollAngle = 0.12f; // [s] (CHANGED! 5.1.2, 0.4f->0.12f)
float const timeConstant_pitchAngle = timeConstant_rollAngle;
float const timeConstant_yawAngle = 0.2f; // [s] (CHANGED! 5.1.2, 1.0f->0.2f)

float estHeight = 0;
float estVelocity_1 = 0;
float estVelocity_2 = 0;
float estVelocity_3 = 0;

// store last height measurement
float lastHeightMeas_meas = 0;
float lastHeightMeas_time = 0;




MainLoopOutput MainLoop(MainLoopInput const &in) {
  // gyro calibration
  if(in.currentTime < 1.0f){
    estGyroBias = estGyroBias + (in.imuMeasurement.rateGyro / 500.0f);
  }
  rateGyro_corr = in.imuMeasurement.rateGyro - estGyroBias;

  //  ***Gyro only attitude estimator***
  //estRoll = estRoll + dt*rateGyro_corr.x;
  //estPitch = estPitch + dt*rateGyro_corr.y;
  //estYaw = estYaw + dt*rateGyro_corr.z;

  // ***Gyro + accelerometer attitude estimator***
  // be aware of accelerometer and gyro measurements on different axis can reflect the same motion
  estRoll = (1.0f-p)*(estRoll + dt*rateGyro_corr.x) + p*(in.imuMeasurement.accelerometer.y / gravity);
  estPitch = (1.0f-p)*(estPitch + dt*rateGyro_corr.y) + p*(in.imuMeasurement.accelerometer.x / -gravity);
  estYaw = estYaw + dt*rateGyro_corr.z;


  // height estimator:
  // prediction step:
  estHeight = estHeight + estVelocity_3 * dt;
  estVelocity_3 = estVelocity_3 + 0 * dt; //assume constant(!)

  // correction step, directly after the prediction step:
  float const mixHeight = 0.3f;
  if (in.heightSensor.updated) {
    // check that the measurement is reasonable
    if (in.heightSensor.value < 5.0f) {
      float hMeas = in.heightSensor.value * cosf(estRoll) * cosf(estPitch);
      estHeight = (1 - mixHeight) * estHeight + mixHeight * hMeas;

      float v3Meas = (hMeas - lastHeightMeas_meas)
          / (in.currentTime - lastHeightMeas_time);

      estVelocity_3 = (1- mixHeight) * estVelocity_3 + mixHeight * v3Meas;
      // store this measurement for the next velocity update
      lastHeightMeas_meas = hMeas;
      lastHeightMeas_time = in.currentTime;
    }
  }

  // horizontal state estimator
  // prediction
  // (just assume velocity is constant):
  estVelocity_1 = estVelocity_1 + 0 * dt;
  estVelocity_2 = estVelocity_2 + 0 * dt;

  // correction step, directly after the prediction step:
  float const mixHorizVel = 0.1f;
  if (in.opticalFlowSensor.updated) {
    float sigma_1 = -in.opticalFlowSensor.value_x;
    float sigma_2 = -in.opticalFlowSensor.value_y;

    float div = (cosf(estRoll) * cosf(estPitch));

    if (div > 0.5f) {
      float deltaPredict = estHeight / div; //this is the delta in the eqution

      float v1Meas = (-sigma_1 + in.imuMeasurement.rateGyro.y) * deltaPredict;
      float v2Meas = (-sigma_2 - in.imuMeasurement.rateGyro.x) * deltaPredict;

      estVelocity_1 = (1 - mixHorizVel) * estVelocity_1 + mixHorizVel * v1Meas;
      estVelocity_2 = (1 - mixHorizVel) * estVelocity_2 + mixHorizVel * v2Meas;

    }
  }



  Vec3f estAngle = Vec3f(estRoll, estPitch, estYaw);

  // ***Angle Controller***
  cmdAngVel.x = (-1/timeConstant_rollRate)*(estAngle.x - desAng.x);
  cmdAngVel.y = (-1/timeConstant_pitchRate)*(estAngle.y - desAng.y);
  cmdAngVel.z = (-1/timeConstant_yawRate)*(estAngle.z - desAng.z);

  desAngVel.x = cmdAngVel.x;
  desAngVel.y = cmdAngVel.y;
  desAngVel.z = cmdAngVel.z;

  // ***Rate Controller***
  cmdAngAcc.x = (-1/timeConstant_rollAngle)*(rateGyro_corr.x - desAngVel.x);
  cmdAngAcc.y = (-1/timeConstant_pitchAngle)*(rateGyro_corr.y - desAngVel.y);
  cmdAngAcc.z = (-1/timeConstant_yawAngle)*(rateGyro_corr.z - desAngVel.z);

  // desired torques:
  float n1 = cmdAngAcc.x*inertia_xx;
  float n2 = cmdAngAcc.y*inertia_yy;
  float n3 = cmdAngAcc.z*inertia_zz;

  // MIXER
  // convert desired torque + total force to four motor forces
  float cp1 = (0.25f)*( (1.0f*des_total_force) + ((1.0f/l)*n1) + ((-1.0f/l)*n2) + ((1.0f/k)*n3) );
  float cp2 = (0.25f)*( (1.0f*des_total_force) + ((-1.0f/l)*n1) + ((-1.0f/l)*n2) + ((-1.0f/k)*n3) );
  float cp3 = (0.25f)*( (1.0f*des_total_force) + ((-1.0f/l)*n1) + ((1.0f/l)*n2) + ((1.0f/k)*n3) );
  float cp4 = (0.25f)*( (1.0f*des_total_force) + ((1.0f/l)*n1) + ((1.0f/l)*n2) + ((-1.0f/k)*n3) );

  // The function input (named "in") is a struct of type
  // "MainLoopInput". You can understand what values it
  // contains by going to its definition (click on "MainLoopInput",
  // and then hit <F3> -- this should take you to the definition).
  // For example, "in.joystickInput.buttonBlue" is true if the
  // joystick's blue button is pushed, false otherwise.

  //Define the output numbers (in the struct outVals):
  MainLoopOutput outVals;
  // motorCommand1 -> located at body +x +y
  // motorCommand2 -> located at body +x -y
  // motorCommand3 -> located at body -x -y
  // motorCommand4 -> located at body -x +y

  // default behavior
  //  outVals.motorCommand1 = 0;
  //  outVals.motorCommand2 = 0;
  //  outVals.motorCommand3 = 0;
  //  outVals.motorCommand4 = 0;

  // run the controller
  if(in.joystickInput.buttonRed) {
    outVals.motorCommand1 = pwmCommandFromSpeed(speedFromForce(cp1));
    outVals.motorCommand2 = pwmCommandFromSpeed(speedFromForce(cp2));
    outVals.motorCommand3 = pwmCommandFromSpeed(speedFromForce(cp3));
    outVals.motorCommand4 = pwmCommandFromSpeed(speedFromForce(cp4));
  } 
  else {
    outVals.motorCommand1 = 0;
    outVals.motorCommand2 = 0;
    outVals.motorCommand3 = 0;
    outVals.motorCommand4 = 0;
  }
  
//  // 4.4.1:
//  if (in.joystickInput.buttonBlue) {
//    desAng.y = 0.5236f;
//  }
//  else {
//    desAng.y = 0;
//  }
  //copy the inputs and outputs:
  lastMainLoopInputs = in;
  lastMainLoopOutputs = outVals;
  outVals.telemetryOutputs_plusMinus100[0] = estRoll;
  outVals.telemetryOutputs_plusMinus100[1] = estPitch;
  outVals.telemetryOutputs_plusMinus100[2] = estYaw;
  outVals.telemetryOutputs_plusMinus100[3] = estVelocity_1;
  outVals.telemetryOutputs_plusMinus100[4] = estVelocity_2;
  outVals.telemetryOutputs_plusMinus100[5] = estVelocity_3;
  outVals.telemetryOutputs_plusMinus100[6] = estHeight;
  return outVals;

}

void PrintStatus() {
  //For a quick reference on the printf function, see: http://www.cplusplus.com/reference/cstdio/printf/
  // Note that \n is a "new line" character.
  // Also, note that to print a `float` variable, you have to explicitly cast it to
  //  `double` in the printf function, and explicitly specify precision using something
  //  like %6.3f (six significant digits, three after the period). Example:
  //   printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //Accelerometer measurement
  printf("Acc: ");
  printf("x=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.x));
  printf("y=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.y));
  printf("z=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.z));
  printf("\n");  //new line
  printf("Gyro bias:");
  printf("x_bias=%6.3f, ",
         double(estGyroBias.x));
  printf("y_bias=%6.3f, ",
         double(estGyroBias.y));
  printf("z_bias=%6.3f, ",
           double(estGyroBias.z));
  printf("\n");  //new line
  printf("Gyro Corrected:");
  printf("x_corrected=%6.3f, ",
         double(rateGyro_corr.x));
  printf("y_corrected=%6.3f, ",
         double(rateGyro_corr.y));
  printf("z_corrected=%6.3f, ",
         double(rateGyro_corr.z));
  printf("\n");  //new line
  printf("Raw gyro: ");
  printf("x=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.x));
  printf("y=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.y));
  printf("z=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.z));
  printf("\n");  //new line
  printf("Attitude: ");
  printf("estRoll=%6.3f, ",
         double(estRoll));
  printf("estPitch=%6.3f, ",
         double(estPitch));
  printf("estYaw=%6.3f, ",
         double(estYaw));

  //  Start Code Block 5.1.1:
  printf("\n");
  printf("Last range = %6.3f, ", \
         double(lastMainLoopInputs.heightSensor.value));
  printf("Last flow: x=%6.3f,  y=%6.3f\n", \
         double(lastMainLoopInputs.opticalFlowSensor.value_x), \
         double(lastMainLoopInputs.opticalFlowSensor.value_y));
  printf("\n");
  //  End Code Block 5.1.1:

  //  printf("Example variable values:\n");
  //  printf("  exampleVariable_int = %d\n", exampleVariable_int);
  //  //Note that it is somewhat annoying to print float variables.
  //  //  We need to cast the variable as double, and we need to specify
  //  //  the number of digits we want (if you used simply "%f", it would
  //  //  truncate to an integer.
  //  //  Here, we print 6 digits, with three digits after the period.
  //  printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));
  //
  //  //We print the Vec3f by printing it's three components independently:
  //  printf("  exampleVariable_Vec3f = (%6.3f, %6.3f, %6.3f)\n",
  //         double(exampleVariable_Vec3f.x), double(exampleVariable_Vec3f.y),
  //         double(exampleVariable_Vec3f.z));cc

  //just an example of how we would inspect the last main loop inputs and outputs:
  printf("Last main loop inputs:\n");
  printf("  batt voltage = %6.3f\n",
         double(lastMainLoopInputs.batteryVoltage.value));
  printf("  JS buttons: ");
  if (lastMainLoopInputs.joystickInput.buttonRed)
    printf("buttonRed ");
  if (lastMainLoopInputs.joystickInput.buttonGreen)
    printf("buttonGreen ");
  if (lastMainLoopInputs.joystickInput.buttonBlue)
    printf("buttonBlue ");
  if (lastMainLoopInputs.joystickInput.buttonYellow)
    printf("buttonYellow ");
  if (lastMainLoopInputs.joystickInput.buttonStart)
    printf("buttonStart ");
  if (lastMainLoopInputs.joystickInput.buttonSelect)
    printf("buttonSelect ");
  printf("\n");
  printf("Last main loop outputs:\n");
  printf("  motor command 1 = %6.3f\n",
         double(lastMainLoopOutputs.motorCommand1));
}