#include "UserCode.hpp"
#include "UtilityFunctions.hpp"
#include "Vec3f.hpp"

#include <stdio.h> //for printf
#include <math.h> //for trig functions
#include <stdlib.h>

//An example of a variable that persists beyond the function call.
float exampleVariable_float = 0.0f;  //Note the trailing 'f' in the number. This is to force single precision floating point.
Vec3f exampleVariable_Vec3f = Vec3f(0, 0, 0);
int exampleVariable_int = 0;

//We keep the last inputs and outputs around for debugging:
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

//Some constants that we may use:
const float mass = 30.4e-3f;  // mass of the quadcopter [kg]
const float gravity = 9.81f;  // acceleration of gravity [m/s^2]
const float inertia_xx = 16e-6f;  //MMOI about x axis [kg.m^2]
const float inertia_yy = inertia_xx;  //MMOI about y axis [kg.m^2]
const float inertia_zz = 29e-6f;  //MMOI about z axis [kg.m^2]
const float deg2rad = M_PI/180.0;

float tot_mot_force = 0;

const float dt = 1.0f / 500.0f; //[s] period between successive calls to MainLoop
Vec3f estGyroBias = Vec3f(0,0,0);
Vec3f rateGyro_corr = Vec3f(0,0,0);
float estRoll = 0;
float estPitch = 0;
float estYaw = 0;
float p = 0.01f; //rho, the gyro/accel trade-off scalar default value: .01

// controller variable initialization
float loop_count = 0; // used for scaling up the desired height during take off
float loop_count_roll = 0;
Vec3f cmdAngAcc = Vec3f(0,0,0);

Vec3f desAngVel = Vec3f(0,0,0);

Vec3f cmdAngVel = Vec3f(0,0,0);

float desHeight = 0;

float desAcc3 = 0;

// propeller distance:
float l = 33e-3f;

// propeller coefficient
float k = 0.01f;

// time constant for controllers on each axis
// D gains on each axis
const float d_roll = 0.04f;
const float d_pitch = 0.04f;
const float d_yaw = 0.1f;
float timeConstant_rollRate = d_roll; // [s]
float timeConstant_pitchRate = d_pitch;
float timeConstant_yawRate = d_yaw; // [s] (CHANGED! 5.1.2, 0.5f->0.1f)


// P gains on each axis
const float p_roll = 0.16f; // from 0.12
const float p_pitch = 0.16f; // from 0.12
const float p_yaw = 0.2f; // from 0.2
float timeConstant_rollAngle = p_roll; // [s] (CHANGED! 5.1.2, 0.4f->0.12f)
float timeConstant_pitchAngle = p_pitch;
float timeConstant_yawAngle = p_yaw; // [s] (CHANGED! 5.1.2, 1.0f->0.2f)

    
// time constant for horizontal controller:
const float h_vel = 1.0f; //1.0f
const float h_pos1 = 5.0f; //5.0  //2.0f
const float h_pos2 = 5.0f; //5.0 // 2.0f
float timeConst_horizVel = h_vel;
float timeConst_horizPos_1 = h_pos1;
float timeConst_horizPos_2 = h_pos2;

float desYawAng = 0;

// time constants for the attitude control
float natFreq_height = 2.0f;
float dampingRatio_height = 0.7f; //0.7 before

float estHeight = 0;
float estVelocity_1 = 0;
float estVelocity_2 = 0;
float estVelocity_3 = 0;

float oldEstVelocity_1 = 0;
float oldEstVelocity_2 = 0;

const float p_v = 0; // update estimated translational velocity with acceleration mixing constant

// integrating optical flow to control around 0 horizontal position
float estPos_1 = 0;
float estPos_2 = 0;

float g1 = 0; // accumated position error for integral control on position
float g2 = 0; // ""

const float g_lim = 5; // windup prevention for position integral control

const float k_g = 1.0f; // integral control gain

// store last height measurement
float lastHeightMeas_meas = 0;
float lastHeightMeas_time = 0;

//initialize angular velocity variable
Vec3f AngVel = Vec3f(0,0,0);


////lab 5 code
MainLoopOutput MainLoop(MainLoopInput const &in) {

  MainLoopOutput outVals;
  //  motorCommand1 -> located at body +x +y
  //  motorCommand2 -> located at body +x -y
  //  motorCommand3 -> located at body -x -y
  //  motorCommand4 -> located at body -x +y

  // gyro calibration
  if(in.currentTime < 1.0f){
    estGyroBias = estGyroBias + (in.imuMeasurement.rateGyro / 500.0f);
  }
  rateGyro_corr = in.imuMeasurement.rateGyro - estGyroBias;
  
  //gain scheduling
  if (loop_count < 3){
    // time constant for horizontal controller:
    timeConst_horizVel = h_vel*1.0f/(1.5f - loop_count/6.0f); //2.0
    timeConst_horizPos_1 = h_pos1*1.0f/(1.5f - loop_count/6.0f);
    timeConst_horizPos_2 = h_pos2*1.0f/(1.5f - loop_count/6.0f);
  }
  else{
    // time constant for horizontal controller:
    timeConst_horizVel = h_vel; //2.0
    timeConst_horizPos_1 = h_pos1;
    timeConst_horizPos_2 = h_pos2;
  }

  //  ***Gyro only attitude estimator***
  //estRoll = estRoll + dt*rateGyro_corr.x;
  //estPitch = estPitch + dt*rateGyro_corr.y;
  //estYaw = estYaw + dt*rateGyro_corr.z;

  // ***Gyro + accelerometer attitude estimator***
  // be aware of accelerometer and gyro measurements on different axis can reflect the same motion
  //estRoll = (1.0f-p)*(estRoll + dt*rateGyro_corr.x) + p*(in.imuMeasurement.accelerometer.y / gravity);
  //estPitch = (1.0f-p)*(estPitch + dt*rateGyro_corr.y) + p*(in.imuMeasurement.accelerometer.x / -gravity);
  //estYaw = estYaw + dt*rateGyro_corr.z;

  // ***Gyro + accelerometer attitude estimator + no small angle aprroximations***
  AngVel.x = rateGyro_corr.x + rateGyro_corr.y*(sinf(estRoll)*tanf(estPitch)) + rateGyro_corr.z*(cosf(estRoll)*tanf(estPitch));
  AngVel.y = rateGyro_corr.y*cosf(estRoll) - rateGyro_corr.z*sinf(estRoll);
  AngVel.z = rateGyro_corr.y*((sinf(estRoll))/(cosf(estPitch))) + rateGyro_corr.z*((cosf(estRoll))/(cosf(estPitch)));

  // be aware of accelerometer and gyro measurements on different axis can reflect the same motion
  estRoll = (1.0f-p)*(estRoll + dt*AngVel.x) + p*(asinf( in.imuMeasurement.accelerometer.y / (gravity*cosf(estPitch))));
  estPitch = (1.0f-p)*(estPitch + dt*AngVel.y) + p*(asinf( in.imuMeasurement.accelerometer.x / -gravity));
  estYaw = estYaw + dt*AngVel.z;

  // height estimator:
  // prediction step:
  estHeight = estHeight + estVelocity_3 * dt;
  estVelocity_3 = estVelocity_3 + 0 * dt; //assume constant(!)

  // correction step, directly after the prediction step:
  float const mixHeight = 0.3f;
  if (in.heightSensor.updated) {
    // check that the measurement is reasonable
    if (in.heightSensor.value < 2.0f) {
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

  // horizontal state estimate:

  // prediction
  // (just assume velocity is constant):
  //estVelocity_1 = estVelocity_1 + 0 * dt;
  //estVelocity_2 = estVelocity_2 + 0 * dt;

  // trying to use accelerometer to update translational velocities
  oldEstVelocity_1 = estVelocity_1;
  oldEstVelocity_2 = estVelocity_2;

  // correction step, directly after the prediction step:
  float const mixHorizVel = 0.5f; //.1
  if (in.opticalFlowSensor.updated) {
    float sigma_1 = -in.opticalFlowSensor.value_x;
    float sigma_2 = -in.opticalFlowSensor.value_y;

    float div = (cosf(estRoll) * cosf(estPitch));

    if (div > 0.5f) {
      float deltaPredict = estHeight / div; //this is the delta in the eqution

      float v1Meas = (-sigma_1 + in.imuMeasurement.rateGyro.y) * deltaPredict;
      float v2Meas = (-sigma_2 - in.imuMeasurement.rateGyro.x) * deltaPredict;

      estVelocity_1 = (1.0f - mixHorizVel) * estVelocity_1 + mixHorizVel * v1Meas;
      estVelocity_2 = (1.0f - mixHorizVel) * estVelocity_2 + mixHorizVel * v2Meas;
    }
  }

  // Integrate optical flow for position estimation
  float desPos1 = 0;
  float desPos2 = 0;

  float oldEstPos_1 = estPos_1;
  float oldEstPos_2 = estPos_2;

  estPos_1 = oldEstPos_1 + (dt * estVelocity_1);
  estPos_2 = oldEstPos_2 + (dt * estVelocity_2);
  
  // add integral action to horizontal position controller
  g1 += k_g * (estPos_1 - desPos1) * dt;
  g2 += k_g * (estPos_2 - desPos2) * dt;

  // prevent integral windup
  if (g1 > g_lim){
    g1 = g_lim;
  }
  if (g2 > g_lim){
    g2 = g_lim;
  }
  if (g1 < -g_lim){
    g1 = -g_lim;
  }
  if (g2 < -g_lim){
    g2 = -g_lim;
  }

  // when we're close to 0, reset integral action
  if (-0.1f < estPos_1 and  estPos_1 < 0.1f) {
    g1 = 0;
  }
  if (-0.1f < estPos_2 and estPos_2 < 0.1f){
    g2 = 0;
  }

  desPos1 = -g1;
  desPos2 = -g2;

  // Horizontal Controller
  float desVel1 = -(1.0f / timeConst_horizPos_1) * (estPos_1 - desPos1);
  float desVel2 = -(1.0f / timeConst_horizPos_2) * (estPos_2 - desPos2);

  float desAcc1 = -(1.0f / timeConst_horizVel) * (estVelocity_1 - desVel1);
  float desAcc2 = -(1.0f / timeConst_horizVel) * (estVelocity_2 - desVel2);

  //control around velocity
  float desRollAng = -desAcc2/ gravity;
  float desPitchAng = desAcc1/ gravity;

  // Vertical Controller
  // scale up the desired height value for a smooth takeoff
  if (loop_count < 5.0f) {
    dampingRatio_height = 1.4f;
    desHeight = loop_count / 10.0f;
  }
  else {
    dampingRatio_height = 0.7f;
    desHeight = 0.5f;
  }
    desAcc3 = -2 * dampingRatio_height * natFreq_height * estVelocity_3
      - natFreq_height * natFreq_height * (estHeight - desHeight);

  //desired normalized total thrust:
  float desNormalizedAcceleration = (gravity + desAcc3) / (cosf(estRoll) * cosf(estPitch));

  // desired force
  float des_total_force = mass * desNormalizedAcceleration;

  // ***Angle Controller***
  Vec3f estAngle = Vec3f(estRoll, estPitch, estYaw);

  cmdAngVel.x = (-1.0f/timeConstant_rollAngle)*(estAngle.x - desRollAng);
  cmdAngVel.y = (-1.0f/timeConstant_pitchAngle)*(estAngle.y - desPitchAng);
  cmdAngVel.z = (-1.0f/timeConstant_yawAngle)*(estAngle.z - desYawAng);

  desAngVel.x = cmdAngVel.x;
  desAngVel.y = cmdAngVel.y;
  desAngVel.z = cmdAngVel.z;

  // ***Rate Controller***
  cmdAngAcc.x = (-1.0f/timeConstant_rollRate)*(rateGyro_corr.x - desAngVel.x);
  cmdAngAcc.y = (-1.0f/timeConstant_pitchRate)*(rateGyro_corr.y - desAngVel.y);
  cmdAngAcc.z = (-1.0f/timeConstant_yawRate)*(rateGyro_corr.z - desAngVel.z);

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

  tot_mot_force = cp1 + cp2 + cp3 + cp4;

  // run the controller

  if(in.joystickInput.buttonRed) {
    if (in.joystickInput.buttonBlue) {
      if (loop_count_roll < 1) {
        float fact = 1.0f;
        outVals.motorCommand1 = pwmCommandFromSpeed(speedFromForce(cp1+(fact*cp1)));
        outVals.motorCommand2 = pwmCommandFromSpeed(speedFromForce(cp2+(fact*cp2)));
        outVals.motorCommand3 = pwmCommandFromSpeed(speedFromForce(cp3+(fact*cp3)));
        outVals.motorCommand4 = pwmCommandFromSpeed(speedFromForce(cp4+(fact*cp4)));
      }
      else if (loop_count_roll > 1 and estRoll < (360*deg2rad)) {
          int quat_thr = 255.0;
          outVals.motorCommand1 = quat_thr;
          outVals.motorCommand2 = 0;
          outVals.motorCommand3 = 0;
          outVals.motorCommand4 = quat_thr;
      } else {
        outVals.motorCommand1 = pwmCommandFromSpeed(speedFromForce(cp1));
        outVals.motorCommand2 = pwmCommandFromSpeed(speedFromForce(cp2));
        outVals.motorCommand3 = pwmCommandFromSpeed(speedFromForce(cp3));
        outVals.motorCommand4 = pwmCommandFromSpeed(speedFromForce(cp4));
      }
      if (estRoll >= (360*deg2rad)) {
        estRoll = 0;
      }
      loop_count_roll += dt;
    }
    else {
      outVals.motorCommand1 = pwmCommandFromSpeed(speedFromForce(cp1));
      outVals.motorCommand2 = pwmCommandFromSpeed(speedFromForce(cp2));
      outVals.motorCommand3 = pwmCommandFromSpeed(speedFromForce(cp3));
      outVals.motorCommand4 = pwmCommandFromSpeed(speedFromForce(cp4));
    }
    loop_count += dt;
  }
  else {
    outVals.motorCommand1 = 0;
    outVals.motorCommand2 = 0;
    outVals.motorCommand3 = 0;
    outVals.motorCommand4 = 0;
  }

  if (in.joystickInput.buttonYellow) {
    if (desYawAng >= (360*deg2rad)) {
      desYawAng = 0;
      estYaw = 0;
    } else {
      desYawAng += (180*deg2rad)*dt;
    }
  }


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
  outVals.telemetryOutputs_plusMinus100[7] = desRollAng;
  outVals.telemetryOutputs_plusMinus100[8] = desPitchAng;
  outVals.telemetryOutputs_plusMinus100[9] = desNormalizedAcceleration;
  outVals.telemetryOutputs_plusMinus100[10] = desHeight;
  outVals.telemetryOutputs_plusMinus100[11] = tot_mot_force;
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

  printf("des_yaw, est_yaw: %6.3f,  %6.3f\n", \
         double(desYawAng), \
         double(estYaw));


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
