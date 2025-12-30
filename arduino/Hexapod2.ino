//***********************************************************************
// Hexapod Program
// Code for Arduino Mega
//***********************************************************************

//***********************************************************************
// IK and Hexapod gait references:
//  https://www.projectsofdan.com/?cat=4
//  http://www.gperco.com/2015/06/hex-inverse-kinematics.html
//  http://virtual-shed.blogspot.com/2012/12/hexapod-inverse-kinematics-part-1.html
//  http://virtual-shed.blogspot.com/2013/01/hexapod-inverse-kinematics-part-2.html
//  https://www.robotshop.com/community/forum/t/inverse-kinematic-equations-for-lynxmotion-3dof-legs/21336
//  http://arduin0.blogspot.com/2012/01/inverse-kinematics-ik-implementation.html?utm_source=rb-community&utm_medium=forum&utm_campaign=inverse-kinematic-equations-for-lynxmotion-3dof-legs
//***********************************************************************

//***********************************************************************
// Includes
//***********************************************************************
#include <Servo.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

//***********************************************************************
// Constant Declarations
//***********************************************************************
const int BATT_VOLTAGE = 0;
float voltage = 0;          // 12V Battery analog voltage input port
const int COXA1_SERVO = 37; // servo port definitions
const int FEMUR1_SERVO = 39;
const int TIBIA1_SERVO = 41;
const int COXA2_SERVO = 43;
const int FEMUR2_SERVO = 45;
const int TIBIA2_SERVO = 47;
const int COXA3_SERVO = 49;
const int FEMUR3_SERVO = 51;
const int TIBIA3_SERVO = 53;
const int COXA4_SERVO = 19;
const int FEMUR4_SERVO = 21;
const int TIBIA4_SERVO = 23;
const int COXA5_SERVO = 25;
const int FEMUR5_SERVO = 27;
const int TIBIA5_SERVO = 29;
const int COXA6_SERVO = 31;
const int FEMUR6_SERVO = 33;
const int TIBIA6_SERVO = 35;

const int COXA_LENGTH = 51; // leg part lengths
const int FEMUR_LENGTH = 65;
const int TIBIA_LENGTH = 121;

const int TRAVEL = 30; // translate and rotate travel limit constant

const long A12DEG = 209440; // 12 degrees in radians x 1,000,000
const long A30DEG = 523599; // 30 degrees in radians x 1,000,000

const int FRAME_TIME_MS = 20; // frame time (20msec = 50Hz)

const float HOME_X[6] = {82.0, 0.0, -82.0, -82.0, 0.0, 82.0}; // coxa-to-toe home positions
const float HOME_Y[6] = {82.0, 116.0, 82.0, -82.0, -116.0, -82.0};
const float HOME_Z[6] = {-80.0, -80.0, -80.0, -80.0, -80.0, -80.0};

const float BODY_X[6] = {110.4, 0.0, -110.4, -110.4, 0.0, 110.4}; // body center-to-coxa servo distances
const float BODY_Y[6] = {58.4, 90.8, 58.4, -58.4, -90.8, -58.4};
const float BODY_Z[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

const int COXA_CAL[6] = {2, -1, 0, 0, -2, -4}; // servo calibration constants
const int FEMUR_CAL[6] = {-5, 3, -3, 2, -4, 0};
const int TIBIA_CAL[6] = {-3, -4, -5, 5, 0, -4};

//***********************************************************************
// Variable Declarations
//***********************************************************************
unsigned long currentTime; // frame timer variables
unsigned long previousTime;

int temp; // mode and control variables
int mode;
int gait;
int gait_speed;
int reset_position;

int batt_voltage;

float L0, L3; // inverse kinematics variables
float gamma_femur;
float phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;

int leg1_IK_control, leg6_IK_control; // leg lift mode variables
float leg1_coxa, leg1_femur, leg1_tibia;
float leg6_coxa, leg6_femur, leg6_tibia;

int leg_num; // positioning and walking variables
int totalX, totalY, totalZ;
int tick, duration, numTicks;
int commandedX, commandedY, commandedR;
float step_height_multiplier;
float strideX, strideY, strideR;
float sinRotX, sinRotY, sinRotZ;
float cosRotX, cosRotY, cosRotZ;
float rotOffsetX, rotOffsetY, rotOffsetZ;
float amplitudeX, amplitudeY, amplitudeZ;
float offset_X[6], offset_Y[6], offset_Z[6];
float current_X[6], current_Y[6], current_Z[6];

// serial control variables
int walk_command_X = 0;
int walk_command_Y = 0;
int walk_command_R = 0;
int translate_command_X = 0;
int translate_command_Y = 0;
int translate_command_Z = 0;
int rotate_command_RX = 0;
int rotate_command_RY = 0;
int rotate_command_RZ = 0;
int rotate_command_Z = 0;
int z_height_command = 1;

const int SERIAL_BUFFER = 96;
char serial_buffer[SERIAL_BUFFER];
int serial_index = 0;

int tripod_case[6] = {1, 2, 1, 2, 1, 2};   // for tripod gait walking
int ripple_case[6] = {2, 6, 4, 1, 3, 5};   // for ripple gait
int wave_case[6] = {1, 2, 3, 4, 5, 6};     // for wave gait
int tetrapod_case[6] = {1, 3, 2, 1, 2, 3}; // for tetrapod gait

//***********************************************************************
// Object Declarations
//***********************************************************************
Servo coxa1_servo; // 18 servos
Servo femur1_servo;
Servo tibia1_servo;
Servo coxa2_servo;
Servo femur2_servo;
Servo tibia2_servo;
Servo coxa3_servo;
Servo femur3_servo;
Servo tibia3_servo;
Servo coxa4_servo;
Servo femur4_servo;
Servo tibia4_servo;
Servo coxa5_servo;
Servo femur5_servo;
Servo tibia5_servo;
Servo coxa6_servo;
Servo femur6_servo;
Servo tibia6_servo;

//***********************************************************************
// Function Prototypes
//***********************************************************************
void process_serial_commands();
void handle_serial_command(char *line);
void update_body_pose_offsets();

//***********************************************************************
// Initialization Routine
//***********************************************************************
void setup()
{
  // start serial
  Serial.begin(115200);
  Serial2.begin(9600);

  // attach servos
  coxa1_servo.attach(COXA1_SERVO, 610, 2400);
  femur1_servo.attach(FEMUR1_SERVO, 610, 2400);
  tibia1_servo.attach(TIBIA1_SERVO, 610, 2400);
  coxa2_servo.attach(COXA2_SERVO, 610, 2400);
  femur2_servo.attach(FEMUR2_SERVO, 610, 2400);
  tibia2_servo.attach(TIBIA2_SERVO, 610, 2400);
  coxa3_servo.attach(COXA3_SERVO, 610, 2400);
  femur3_servo.attach(FEMUR3_SERVO, 610, 2400);
  tibia3_servo.attach(TIBIA3_SERVO, 610, 2400);
  coxa4_servo.attach(COXA4_SERVO, 610, 2400);
  femur4_servo.attach(FEMUR4_SERVO, 610, 2400);
  tibia4_servo.attach(TIBIA4_SERVO, 610, 2400);
  coxa5_servo.attach(COXA5_SERVO, 610, 2400);
  femur5_servo.attach(FEMUR5_SERVO, 610, 2400);
  tibia5_servo.attach(TIBIA5_SERVO, 610, 2400);
  coxa6_servo.attach(COXA6_SERVO, 610, 2400);
  femur6_servo.attach(FEMUR6_SERVO, 610, 2400);
  tibia6_servo.attach(TIBIA6_SERVO, 610, 2400);

  // initialize manual leg defaults
  leg1_coxa = 90.0;
  leg1_femur = 90.0;
  leg1_tibia = 90.0;
  leg6_coxa = 90.0;
  leg6_femur = 90.0;
  leg6_tibia = 90.0;

  Serial.println("Hexapod listo. Escriba HELP para ver comandos seriales.");

  // clear offsets
  for (leg_num = 0; leg_num < 6; leg_num++)
  {
    offset_X[leg_num] = 0.0;
    offset_Y[leg_num] = 0.0;
    offset_Z[leg_num] = 0.0;
  }
  step_height_multiplier = 1.0;

  // initialize mode and gait variables
  mode = 0;
  gait = 2;
  gait_speed = 0;
  reset_position = true;
  leg1_IK_control = true;
  leg6_IK_control = true;
}

//***********************************************************************
// Main Program
//***********************************************************************
void loop()
{
  process_serial_commands();

  // set up frame time
  currentTime = millis();
  if ((currentTime - previousTime) > FRAME_TIME_MS)
  {
    previousTime = currentTime;

    // reset legs to home position when commanded
    if (reset_position == true)
    {
      for (leg_num = 0; leg_num < 6; leg_num++)
      {
        current_X[leg_num] = HOME_X[leg_num];
        current_Y[leg_num] = HOME_Y[leg_num];
        current_Z[leg_num] = HOME_Z[leg_num];
      }
      reset_position = false;
    }

    update_body_pose_offsets();

    // position legs using IK calculations - unless set all to 90 degrees mode
    if (mode < 99)
    {
      for (leg_num = 0; leg_num < 6; leg_num++)
        leg_IK(leg_num, current_X[leg_num] + offset_X[leg_num], current_Y[leg_num] + offset_Y[leg_num], current_Z[leg_num] + offset_Z[leg_num]);
    }

    // reset leg lift first pass flags if needed
    if (mode != 4)
    {
      leg1_IK_control = true;
      leg6_IK_control = true;
    }

    battery_monitor(); // battery monitor update
    // print_debug();     // print debug data

    // process modes (mode 0 is default 'home idle' do-nothing mode)
    if (mode == 1) // walking mode
    {
      if (gait == 0)
        tripod_gait(); // walk using gait 0
      if (gait == 1)
        wave_gait(); // walk using gait 1
      if (gait == 2)
        ripple_gait(); // walk using gait 2
      if (gait == 3)
        tetrapod_gait(); // walk using gait 3
    }
    if (mode == 4)
      one_leg_lift(); // one leg lift mode
    if (mode == 99)
      set_all_90(); // set all servos to 90 degrees mode
  }
}

//***********************************************************************
// Serial command handling
//***********************************************************************
void process_serial_commands()
{
  while (Serial2.available()>0) //>0
  {
    char incoming = Serial2.read();

    if (incoming == '\n' || incoming == '\r')
    {
      if (serial_index > 0)
      {
        serial_buffer[serial_index] = '\0';
        handle_serial_command(serial_buffer);
        serial_index = 0;
      }
    }
    else if (serial_index < (SERIAL_BUFFER - 1))
    {
      serial_buffer[serial_index++] = incoming;
    }
  }
}

void handle_serial_command(char *line)
{
  if (line[0] == '\0')
    return;

  for (int i = 0; line[i] != '\0'; i++)
    line[i] = toupper(line[i]);

  char *cmd = strtok(line, " ");
  if (cmd == NULL)
    return;

  if (strcmp(cmd, "HELP") == 0)
  {
    Serial.println("Comandos: MODE, GAIT, SPEED, WALK x y r, TRANS x y z, ROT rx ry rz z, STEPH n, LEG1 c f t, LEG6 c f t, RESET, SET90, BATT");
    return;
  }

  if (strcmp(cmd, "MODE") == 0)
  {
    char *arg = strtok(NULL, " ");
    if (arg != NULL)
    {
      int requested_mode = constrain(atoi(arg), 0, 99);
      if (requested_mode == 2 || requested_mode == 3)
        requested_mode = 1;
      mode = requested_mode;
      reset_position = true;
      if (mode != 4)
      {
        leg1_IK_control = true;
        leg6_IK_control = true;
      }
      else
      {
        leg1_IK_control = false;
        leg6_IK_control = false;
      }
    }
    return;
  }

  if (strcmp(cmd, "GAIT") == 0)
  {
    char *arg = strtok(NULL, " ");
    if (arg != NULL)
      gait = constrain(atoi(arg), 0, 3);
    return;
  }

  if (strcmp(cmd, "SPEED") == 0)
  {
    char *arg = strtok(NULL, " ");
    if (arg != NULL)
      gait_speed = (atoi(arg) != 0) ? 1 : 0;
    return;
  }

  if (strcmp(cmd, "WALK") == 0)
  {
    char *argX = strtok(NULL, " ");
    char *argY = strtok(NULL, " ");
    char *argR = strtok(NULL, " ");
    if (argX && argY && argR)
    {
      walk_command_X = constrain(atoi(argX), -127, 127);
      walk_command_Y = constrain(atoi(argY), -127, 127);
      walk_command_R = constrain(atoi(argR), -127, 127);
    }
    return;
  }

  if (strcmp(cmd, "TRANS") == 0)
  {
    char *argX = strtok(NULL, " ");
    char *argY = strtok(NULL, " ");
    char *argZ = strtok(NULL, " ");
    if (argX && argY && argZ)
    {
      translate_command_X = constrain(atoi(argX), -2 * TRAVEL, 2 * TRAVEL);
      translate_command_Y = constrain(atoi(argY), -2 * TRAVEL, 2 * TRAVEL);
      translate_command_Z = constrain(atoi(argZ), -3 * TRAVEL, TRAVEL);
    }
    return;
  }

  if (strcmp(cmd, "ROT") == 0)
  {
    char *argRX = strtok(NULL, " ");
    char *argRY = strtok(NULL, " ");
    char *argRZ = strtok(NULL, " ");
    char *argZ = strtok(NULL, " ");
    if (argRX && argRY && argRZ && argZ)
    {
      rotate_command_RX = constrain(atoi(argRX), -12, 12);
      rotate_command_RY = constrain(atoi(argRY), -12, 12);
      rotate_command_RZ = constrain(atoi(argRZ), -30, 30);
      rotate_command_Z = constrain(atoi(argZ), -3 * TRAVEL, TRAVEL);
    }
    return;
  }

  if (strcmp(cmd, "STEPH") == 0)
  {
    char *arg = strtok(NULL, " ");
    if (arg != NULL)
    {
      z_height_command = constrain(atoi(arg), 1, 8);
      step_height_multiplier = 1.0 + ((z_height_command - 1.0) / 3.0);
    }
    return;
  }

  if (strcmp(cmd, "BATT") == 0)
  {
    Serial.print("BATT ");
    Serial.print(voltage * 3, 2);
    Serial.println(" V");
    return;
  }

  if (strcmp(cmd, "LEG1") == 0)
  {
    char *argC = strtok(NULL, " ");
    char *argF = strtok(NULL, " ");
    char *argT = strtok(NULL, " ");
    if (argC && argF && argT)
    {
      leg1_coxa = constrain(atoi(argC), 45, 135);
      leg1_femur = constrain(atoi(argF), 0, 170);
      leg1_tibia = constrain(atoi(argT), 0, 170);
      leg1_IK_control = false;
    }
    return;
  }

  if (strcmp(cmd, "LEG6") == 0)
  {
    char *argC = strtok(NULL, " ");
    char *argF = strtok(NULL, " ");
    char *argT = strtok(NULL, " ");
    if (argC && argF && argT)
    {
      leg6_coxa = constrain(atoi(argC), 45, 135);
      leg6_femur = constrain(atoi(argF), 0, 170);
      leg6_tibia = constrain(atoi(argT), 0, 170);
      leg6_IK_control = false;
    }
    return;
  }

  if (strcmp(cmd, "RESET") == 0)
  {
    reset_position = true;
    return;
  }

  if (strcmp(cmd, "SET90") == 0)
  {
    mode = 99;
    return;
  }

  Serial.println("Comando desconocido");
}

// offsets are recalculated dynamically in update_body_pose_offsets()

//***********************************************************************
// Leg IK Routine
//***********************************************************************
void leg_IK(int leg_number, float X, float Y, float Z)
{
  // compute target femur-to-toe (L3) length
  L0 = sqrt(sq(X) + sq(Y)) - COXA_LENGTH;
  L3 = sqrt(sq(L0) + sq(Z));

  // process only if reach is within possible range (not too long or too short!)
  if ((L3 < (TIBIA_LENGTH + FEMUR_LENGTH)) && (L3 > (TIBIA_LENGTH - FEMUR_LENGTH)))
  {
    // compute tibia angle
    phi_tibia = acos((sq(FEMUR_LENGTH) + sq(TIBIA_LENGTH) - sq(L3)) / (2 * FEMUR_LENGTH * TIBIA_LENGTH));
    theta_tibia = phi_tibia * RAD_TO_DEG - 23.0 + TIBIA_CAL[leg_number];
    theta_tibia = constrain(theta_tibia, 0.0, 180.0);

    // compute femur angle
    gamma_femur = atan2(Z, L0);
    phi_femur = acos((sq(FEMUR_LENGTH) + sq(L3) - sq(TIBIA_LENGTH)) / (2 * FEMUR_LENGTH * L3));
    theta_femur = (phi_femur + gamma_femur) * RAD_TO_DEG + 14.0 + 90.0 + FEMUR_CAL[leg_number];
    theta_femur = constrain(theta_femur, 0.0, 180.0);

    // compute coxa angle
    theta_coxa = atan2(X, Y) * RAD_TO_DEG + COXA_CAL[leg_number];

    // output to the appropriate leg
    switch (leg_number)
    {
    case 0:
      if (leg1_IK_control == true) // flag for IK or manual control of leg
      {
        theta_coxa = theta_coxa + 45.0; // compensate for leg mounting
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa1_servo.write(int(theta_coxa));
        femur1_servo.write(int(theta_femur));
        tibia1_servo.write(int(theta_tibia));
      }
      break;
    case 1:
      theta_coxa = theta_coxa + 90.0; // compensate for leg mounting
      theta_coxa = constrain(theta_coxa, 0.0, 180.0);
      coxa2_servo.write(int(theta_coxa));
      femur2_servo.write(int(theta_femur));
      tibia2_servo.write(int(theta_tibia));
      break;
    case 2:
      theta_coxa = theta_coxa + 135.0; // compensate for leg mounting
      theta_coxa = constrain(theta_coxa, 0.0, 180.0);
      coxa3_servo.write(int(theta_coxa));
      femur3_servo.write(int(theta_femur));
      tibia3_servo.write(int(theta_tibia));
      break;
    case 3:
      if (theta_coxa < 0)                // compensate for leg mounting
        theta_coxa = theta_coxa + 225.0; // (need to use different
      else                               //  positive and negative offsets
        theta_coxa = theta_coxa - 135.0; //  due to atan2 results above!)
      theta_coxa = constrain(theta_coxa, 0.0, 180.0);
      coxa4_servo.write(int(theta_coxa));
      femur4_servo.write(int(theta_femur));
      tibia4_servo.write(int(theta_tibia));
      break;
    case 4:
      if (theta_coxa < 0)                // compensate for leg mounting
        theta_coxa = theta_coxa + 270.0; // (need to use different
      else                               //  positive and negative offsets
        theta_coxa = theta_coxa - 90.0;  //  due to atan2 results above!)
      theta_coxa = constrain(theta_coxa, 0.0, 180.0);
      coxa5_servo.write(int(theta_coxa));
      femur5_servo.write(int(theta_femur));
      tibia5_servo.write(int(theta_tibia));
      break;
    case 5:
      if (leg6_IK_control == true) // flag for IK or manual control of leg
      {
        if (theta_coxa < 0)                // compensate for leg mounting
          theta_coxa = theta_coxa + 315.0; // (need to use different
        else                               //  positive and negative offsets
          theta_coxa = theta_coxa - 45.0;  //  due to atan2 results above!)
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa6_servo.write(int(theta_coxa));
        femur6_servo.write(int(theta_femur));
        tibia6_servo.write(int(theta_tibia));
      }
      break;
    }
  }
}

//***********************************************************************
// Tripod Gait
// Group of 3 legs move forward while the other 3 legs provide support
//***********************************************************************
void tripod_gait()
{
  // read commanded values from controller
  commandedX = constrain(walk_command_X, -127, 127);
  commandedY = constrain(walk_command_Y, -127, 127);
  commandedR = constrain(walk_command_R, -127, 127);

  // if commands more than deadband then process
  if ((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 2.0); // total ticks divided into the two cases
    for (leg_num = 0; leg_num < 6; leg_num++)
    {
      compute_amplitudes();
      switch (tripod_case[leg_num])
      {
      case 1: // move foot forward (raise and lower)
        current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / numTicks);
        current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / numTicks);
        current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / numTicks);
        if (tick >= numTicks - 1)
          tripod_case[leg_num] = 2;
        break;
      case 2: // move foot back (on the ground)
        current_X[leg_num] = HOME_X[leg_num] + amplitudeX * cos(M_PI * tick / numTicks);
        current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY * cos(M_PI * tick / numTicks);
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          tripod_case[leg_num] = 1;
        break;
      }
    }
    // increment tick
    if (tick < numTicks - 1)
      tick++;
    else
      tick = 0;
  }
}

//***********************************************************************
// Wave Gait
// Legs move forward one at a time while the other 5 legs provide support
//***********************************************************************
void wave_gait()
{
  // read commanded values from controller
  commandedX = constrain(walk_command_X, -127, 127);
  commandedY = constrain(walk_command_Y, -127, 127);
  commandedR = constrain(walk_command_R, -127, 127);

  // if commands more than deadband then process
  if ((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0); // total ticks divided into the six cases
    for (leg_num = 0; leg_num < 6; leg_num++)
    {
      compute_amplitudes();
      switch (wave_case[leg_num])
      {
      case 1: // move foot forward (raise and lower)
        current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / numTicks);
        current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / numTicks);
        current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / numTicks);
        if (tick >= numTicks - 1)
          wave_case[leg_num] = 6;
        break;
      case 2: // move foot back one-fifth (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          wave_case[leg_num] = 1;
        break;
      case 3: // move foot back one-fifth (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          wave_case[leg_num] = 2;
        break;
      case 4: // move foot back one-fifth (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          wave_case[leg_num] = 3;
        break;
      case 5: // move foot back one-fifth (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          wave_case[leg_num] = 4;
        break;
      case 6: // move foot back one-fifth (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          wave_case[leg_num] = 5;
        break;
      }
    }
    // increment tick
    if (tick < numTicks - 1)
      tick++;
    else
      tick = 0;
  }
}

//***********************************************************************
// Ripple Gait
// Left legs move forward rear-to-front while right also do the same,
// but right side is offset so RR starts midway through the LM stroke
//***********************************************************************
void ripple_gait()
{
  // read commanded values from controller
  commandedX = constrain(walk_command_X, -127, 127);
  commandedY = constrain(walk_command_Y, -127, 127);
  commandedR = constrain(walk_command_R, -127, 127);

  // if commands more than deadband then process
  if ((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0); // total ticks divided into the six cases
    for (leg_num = 0; leg_num < 6; leg_num++)
    {
      compute_amplitudes();
      switch (ripple_case[leg_num])
      {
      case 1: // move foot forward (raise)
        current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / (numTicks * 2));
        current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / (numTicks * 2));
        current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / (numTicks * 2));
        if (tick >= numTicks - 1)
          ripple_case[leg_num] = 2;
        break;
      case 2: // move foot forward (lower)
        current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * (numTicks + tick) / (numTicks * 2));
        current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * (numTicks + tick) / (numTicks * 2));
        current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * (numTicks + tick) / (numTicks * 2));
        if (tick >= numTicks - 1)
          ripple_case[leg_num] = 3;
        break;
      case 3: // move foot back one-quarter (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          ripple_case[leg_num] = 4;
        break;
      case 4: // move foot back one-quarter (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          ripple_case[leg_num] = 5;
        break;
      case 5: // move foot back one-quarter (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          ripple_case[leg_num] = 6;
        break;
      case 6: // move foot back one-quarter (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          ripple_case[leg_num] = 1;
        break;
      }
    }
    // increment tick
    if (tick < numTicks - 1)
      tick++;
    else
      tick = 0;
  }
}

//***********************************************************************
// Tetrapod Gait
// Right front and left rear legs move forward together, then right
// rear and left middle, and finally right middle and left front.
//***********************************************************************
void tetrapod_gait()
{
  // read commanded values from controller
  commandedX = constrain(walk_command_X, -127, 127);
  commandedY = constrain(walk_command_Y, -127, 127);
  commandedR = constrain(walk_command_R, -127, 127);

  // if commands more than deadband then process
  if ((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 3.0); // total ticks divided into the three cases
    for (leg_num = 0; leg_num < 6; leg_num++)
    {
      compute_amplitudes();
      switch (tetrapod_case[leg_num])
      {
      case 1: // move foot forward (raise and lower)
        current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / numTicks);
        current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / numTicks);
        current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / numTicks);
        if (tick >= numTicks - 1)
          tetrapod_case[leg_num] = 2;
        break;
      case 2: // move foot back one-half (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          tetrapod_case[leg_num] = 3;
        break;
      case 3: // move foot back one-half (on the ground)
        current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks;
        current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks;
        current_Z[leg_num] = HOME_Z[leg_num];
        if (tick >= numTicks - 1)
          tetrapod_case[leg_num] = 1;
        break;
      }
    }
    // increment tick
    if (tick < numTicks - 1)
      tick++;
    else
      tick = 0;
  }
}

//***********************************************************************
// Compute walking stride lengths
//***********************************************************************
void compute_strides()
{
  // compute stride lengths
  strideX = 90 * commandedX / 127;
  strideY = 90 * commandedY / 127;
  strideR = 35 * commandedR / 127;

  // compute rotation trig
  sinRotZ = sin(radians(strideR));
  cosRotZ = cos(radians(strideR));

  // set duration for normal and slow speed modes
  if (gait_speed == 0)
    duration = 1080;
  else
    duration = 3240;
}

//***********************************************************************
// Compute walking amplitudes
//***********************************************************************
void compute_amplitudes()
{
  // compute total distance from center of body to toe
  totalX = HOME_X[leg_num] + BODY_X[leg_num];
  totalY = HOME_Y[leg_num] + BODY_Y[leg_num];

  // compute rotational offset
  rotOffsetX = totalY * sinRotZ + totalX * cosRotZ - totalX;
  rotOffsetY = totalY * cosRotZ - totalX * sinRotZ - totalY;

  // compute X and Y amplitude and constrain to prevent legs from crashing into each other
  amplitudeX = ((strideX + rotOffsetX) / 2.0);
  amplitudeY = ((strideY + rotOffsetY) / 2.0);
  amplitudeX = constrain(amplitudeX, -50, 50);
  amplitudeY = constrain(amplitudeY, -50, 50);

  // compute Z amplitude
  if (abs(strideX + rotOffsetX) > abs(strideY + rotOffsetY))
    amplitudeZ = step_height_multiplier * (strideX + rotOffsetX) / 4.0;
  else
    amplitudeZ = step_height_multiplier * (strideY + rotOffsetY) / 4.0;
}

//***********************************************************************
// Body pose offsets (translation + rotation) applied in all modes
//***********************************************************************
void update_body_pose_offsets()
{
  float translate_x = constrain(translate_command_X, -2 * TRAVEL, 2 * TRAVEL);
  float translate_y = constrain(translate_command_Y, -2 * TRAVEL, 2 * TRAVEL);
  float translate_z = constrain(translate_command_Z, -3 * TRAVEL, TRAVEL);
  float rotate_z_shift = constrain(rotate_command_Z, -3 * TRAVEL, TRAVEL);
  float body_z_shift = constrain(translate_z + rotate_z_shift, -3 * TRAVEL, TRAVEL);

  float rx = radians(constrain(rotate_command_RX, -12, 12));
  float ry = radians(constrain(rotate_command_RY, -12, 12));
  float rz = radians(constrain(rotate_command_RZ, -30, 30));

  float sinRx = sin(rx);
  float cosRx = cos(rx);
  float sinRy = sin(ry);
  float cosRy = cos(ry);
  float sinRz = sin(rz);
  float cosRz = cos(rz);

  for (leg_num = 0; leg_num < 6; leg_num++)
  {
    float legX = current_X[leg_num];
    float legY = current_Y[leg_num];
    float legZ = current_Z[leg_num];

    float totalX = legX + BODY_X[leg_num];
    float totalY = legY + BODY_Y[leg_num];
    float totalZ = legZ + BODY_Z[leg_num];

    float rotatedX = totalX * cosRy * cosRz + totalY * sinRx * sinRy * cosRz + totalY * cosRx * sinRz - totalZ * cosRx * sinRy * cosRz + totalZ * sinRx * sinRz;
    float rotatedY = -totalX * cosRy * sinRz - totalY * sinRx * sinRy * sinRz + totalY * cosRx * cosRz + totalZ * cosRx * sinRy * sinRz + totalZ * sinRx * cosRz;
    float rotatedZ = totalX * sinRy - totalY * sinRx * cosRy + totalZ * cosRx * cosRy;

    offset_X[leg_num] = translate_x + (rotatedX - totalX);
    offset_Y[leg_num] = translate_y + (rotatedY - totalY);
    offset_Z[leg_num] = body_z_shift + (rotatedZ - totalZ);
  }
}

//***********************************************************************
// One leg lift mode
// also can set z step height using capture offsets
//***********************************************************************
void one_leg_lift()
{
  leg1_IK_control = false;
  leg6_IK_control = false;

  coxa1_servo.write(constrain(int(leg1_coxa), 45, 135));
  femur1_servo.write(constrain(int(leg1_femur), 0, 170));
  tibia1_servo.write(constrain(int(leg1_tibia), 0, 170));

  coxa6_servo.write(constrain(int(leg6_coxa), 45, 135));
  femur6_servo.write(constrain(int(leg6_femur), 0, 170));
  tibia6_servo.write(constrain(int(leg6_tibia), 0, 170));

  step_height_multiplier = 1.0 + ((z_height_command - 1.0) / 3.0);
}

//***********************************************************************
// Set all servos to 90 degrees
// Note: this is useful for calibration/alignment of the servos
// i.e: set COXA_CAL[6], FEMUR_CAL[6], and TIBIA_CAL[6] values in
//      constants section above so all angles appear as 90 degrees
//***********************************************************************
void set_all_90()
{
  coxa1_servo.write(90 + COXA_CAL[0]);
  femur1_servo.write(90 + FEMUR_CAL[0]);
  tibia1_servo.write(90 + TIBIA_CAL[0]);

  coxa2_servo.write(90 + COXA_CAL[1]);
  femur2_servo.write(90 + FEMUR_CAL[1]);
  tibia2_servo.write(90 + TIBIA_CAL[1]);

  coxa3_servo.write(90 + COXA_CAL[2]);
  femur3_servo.write(90 + FEMUR_CAL[2]);
  tibia3_servo.write(90 + TIBIA_CAL[2]);

  coxa4_servo.write(90 + COXA_CAL[3]);
  femur4_servo.write(90 + FEMUR_CAL[3]);
  tibia4_servo.write(90 + TIBIA_CAL[3]);

  coxa5_servo.write(90 + COXA_CAL[4]);
  femur5_servo.write(90 + FEMUR_CAL[4]);
  tibia5_servo.write(90 + TIBIA_CAL[4]);

  coxa6_servo.write(90 + COXA_CAL[5]);
  femur6_servo.write(90 + FEMUR_CAL[5]);
  tibia6_servo.write(90 + TIBIA_CAL[5]);
}

//***********************************************************************
// Battery monitor routine
// Note: my hexapod uses a 3S LiPo battery
// (fully charged = 12.6V, nominal = 11.4V, discharged = 10.2V)
//***********************************************************************
void battery_monitor()
{
  int sensorValue = analogRead(A1);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  voltage = sensorValue * (5.0 / 1023.0);
}

//***********************************************************************
// Print Debug Data
//***********************************************************************
void print_debug()
{
  // output IK data
  //  Serial.print(int(theta_coxa));
  //  Serial.print(",");
  //  Serial.print(int(theta_femur));
  //  Serial.print(",");
  //  Serial.print(int(theta_tibia));
  //  Serial.print(",");

  // output XYZ coordinates for all legs
  //  for(leg_num=0; leg_num<6; leg_num++)
  //  {
  //    Serial.print(int(current_X[leg_num]));
  //    if(leg_num<5) Serial.print(",");
  //  }
  //  Serial.print("  ");
  //  for(leg_num=0; leg_num<6; leg_num++)
  //  {
  //    Serial.print(int(current_Y[leg_num]));
  //    if(leg_num<5) Serial.print(",");
  //  }
  //  Serial.print("  ");
  //  for(leg_num=0; leg_num<6; leg_num++)
  //  {
  //    Serial.print(int(current_Z[leg_num]));
  //    if(leg_num<5) Serial.print(",");
  //  }
  //  Serial.print("  ");

  // display elapsed frame time (ms) and battery voltage (V)
  currentTime = millis();
  Serial.print(currentTime - previousTime);
  Serial.print(",");
  Serial.print(float(batt_voltage) / 100.0);
  Serial.print("\n");
}
