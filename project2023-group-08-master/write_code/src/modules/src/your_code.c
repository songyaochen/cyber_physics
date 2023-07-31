#include "your_code.h"

/***
 *
 * This file is where you should add you tasks. You already know the structure
 * Required to do so from the work with the simulator.
 *
 * The function yourCodeInit() is set to automatically execute when the
 * quadrotor is started. This is where you need to create your tasks. The
 * scheduler that runs the tasks is already up and running so you should
 * NOT make a call to vTaskStartScheduler();.
 *
 * Below that you can find a few examples of useful function calls and code snippets.
 *
 * For further reference on how this is done: Look into the file stabilizer.c
 * which usually handles the control of the crazyflie.
 *
 ***/

// NO NEED TO CHANGE THIS
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;
static StateEstimatorType estimatorType;
// END NO NEED TO CHANGE THIS
//STATIC_MEM_TASK_ALLOC(my_reference, configMINIMAL_STACK_SIZE);
STATIC_MEM_TASK_ALLOC(my_filter, configMINIMAL_STACK_SIZE);
STATIC_MEM_TASK_ALLOC(my_controller, configMINIMAL_STACK_SIZE);
static sensorData_t sensorData;
static uint16_t motor_PWM[4];
static state_t state;
static setpoint_t setpoint;

SemaphoreHandle_t motors_sem; //= params->motors_sem;
SemaphoreHandle_t references_sem; //= params->references_sem;
SemaphoreHandle_t sensors_sem; //= params->sensors_sem;
SemaphoreHandle_t estimate_sem; //= params->estimate_sem;

//static void my_reference(void* param);
static void my_filter(void* param);
static void my_controller(void* param);

void yourCodeInit(void)
{
    estimatorType = getStateEstimator();
    motors_sem     = xSemaphoreCreateBinary(); xSemaphoreGive(motors_sem);
    references_sem = xSemaphoreCreateBinary(); xSemaphoreGive(references_sem);
    sensors_sem    = xSemaphoreCreateBinary(); xSemaphoreGive(sensors_sem);
    estimate_sem   = xSemaphoreCreateBinary(); xSemaphoreGive(estimate_sem);

	/*
   * CREATE AND EXECUTE YOUR TASKS FROM HERE
   */
    //STATIC_MEM_TASK_CREATE(my_reference, my_reference,"MY_REFERENCE_NAME", NULL, 1);
    STATIC_MEM_TASK_CREATE(my_filter, my_filter,"MY_FILTER_NAME", NULL, 3);
    STATIC_MEM_TASK_CREATE(my_controller, my_controller,"MY_CONTROLLER_NAME", NULL, 1);
    
	
}

/* 
* ADD TASKS HERE

static void my_reference(void *pvParameters){
  uint32_t tick;
  uint32_t lastWakeTime;
  lastWakeTime = xTaskGetTickCount();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  tick = 1;

  while(1){
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ));
    //stateEstimator(&state, tick);
    //commanderGetSetpoint(&setpoint, &state);
    xSemaphoreTake(references_sem,portMAX_DELAY);
    setpoint.attitude.roll=10.0;
    setpoint.attitude.pitch=0.0;
    xSemaphoreGive(references_sem);
  }

}*/
static void my_filter(void *pvParameters){
  uint32_t tick;
  uint32_t lastWakeTime;
  lastWakeTime = xTaskGetTickCount();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  tick = 1;
  /*struct FilterParams *params =
        (struct FilterParams*)pvParameters;
  */
    
  double estimate[3] = {0.0};
   // define variables lambda and h for filter
  double lambda = 0.98;
  double h = 1.0/1000.0;
  double pi= 3.1415;
  while(1){
      vTaskDelayUntil(&lastWakeTime, F2T(RATE_250_HZ));
      
      sensorsAcquire(&sensorData, tick);

      xSemaphoreTake(sensors_sem,portMAX_DELAY);
      double gyro_data[3]={sensorData.gyro.x,sensorData.gyro.y,sensorData.gyro.z};
      double acc_data[3] ={sensorData.acc.x,sensorData.acc.y,sensorData.acc.z};
      xSemaphoreGive(sensors_sem);

      double phi_a = atan2(acc_data[1],acc_data[2]);
      double theta_a =atan2(-acc_data[0],sqrt(pow(acc_data[1],2)+pow(acc_data[2],2)));

        
     // Roll
      estimate[0] =  (1-lambda)*(180/pi)*phi_a+lambda*(estimate[0]+h*gyro_data[0]);

      // pitch
      estimate[1] =  (1-lambda)*(180/pi)*theta_a+lambda*(estimate[1]+h*gyro_data[1]);
      LOG_GROUP_START(est_my)
      LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
      LOG_ADD(LOG_FLOAT, pitch,&state.attitude.pitch)
      LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
      LOG_GROUP_STOP(est_my)
        
      xSemaphoreTake(estimate_sem,portMAX_DELAY);
        //memcpy(params->estimate, estimate, sizeof(estimate));
        state.attitude.roll = estimate[0];
        state.attitude.pitch = estimate[1];
        state.attitude.yaw = estimate[2];
      xSemaphoreGive(estimate_sem);
  }
}

static void my_controller(void *pvParameters){
  uint32_t tick;
  uint32_t lastWakeTime;
  lastWakeTime = xTaskGetTickCount();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  tick = 1;

  /*struct ControlSystemParams *params =
        (struct ControlSystemParams*)pvParameters;*/
  //double gyro_data[3];
  //double acc_data[3];
  //double r_rpdy[3];
  //double estimate[3] = {0.0};

  // constants
  //double pi=3.1415;
  double error_states[5] = {0.0}; // error array
  //uint16_t base_thrust = 10;       // base thrust



  // slow repons 
  //double K[4][5]={{-156.3298,-156.6719,-15.4454,-18.5110,-49.7253},{-156.3298,156.6719,-15.4454,18.5110,49.7253},{156.3298,156.6719,15.4454,18.5110,-49.7253},{156.3298,-156.6719,15.4454,-18.5110,49.7253}};

  // Qunz
  //double K[4][5]={{-479.8212,-485.6122,-54.4011,-57.8290,-49.7253},{-479.8212,485.6122,-54.4011,57.8290,49.7253},{479.8212,485.6122,54.4011,57.8290,-49.7253},{479.8212,-485.6122,54.4011,-57.8290,49.7253}};

  // medium respons
  //double K[4][5]={{-1067.4,-1080.6,-61.2,-67.3,-155.4},{-1067.4,1080.6,-61.2,67.3,155.4},{1067.4,1080.6,61.2,67.3,-155.4},{1067.4,-1080.6,61.2,-67.3,155.4}};

  // Faster respons
  //double K[4][5]={{-1504.2,-1523.4,-65.7,-73.6,-49.7},{-1504.2,1523.4,-65.7,73.6,49.7},{1504.2,1523.4,65.7,73.6,-49.7},{1504.2,-1523.4,65.7,-73.6,49.7}};
  
    // Fastest respons
  //double K[4][5]={{-1525.4,-1536.0,-48.2,-57.9,-155.4},{-1525.4,1536.0,-48.2,57.9,155.4},{1525.4,1536.0,48.2,57.9,-155.4},{1525.4,-1536.0,48.2,-57.9,155.4}};
  
    // more Fastest respons
  //double K[4][5]={{-4701.8,-4755.1,-81.6,-99.4,-155.4},{-4701.8,4755.1,-81.6,99.4,155.4},{4701.8,4755.1,81.6,99.4,-155.4},{4701.8,-4755.1,81.6,-99.4,155.4}};
  
   // Best result with R = 1
   double K[4][5]={{-5866.7,-7688.9,-593.5,-779.0,-1329.5},{-5866.7,7688.9,-593.5,779.0,1329.5},{5866.7,7688.9,593.5,779.0,-1329.5},{5866.7,-7688.9,593.5,-779.0,1329.5}};

  //uint16_t count = 0;
  while(1){
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));

    sensorsAcquire(&sensorData, tick);
    stateEstimator(&state, tick);
    commanderGetSetpoint(&setpoint, &state);

    xSemaphoreTake(sensors_sem,portMAX_DELAY);
         double gyro_data[3] ={sensorData.gyro.x,sensorData.gyro.y,sensorData.gyro.z};
    xSemaphoreGive(sensors_sem);

    xSemaphoreTake(estimate_sem,portMAX_DELAY);
        double estimate[3] = {state.attitude.roll,state.attitude.pitch,state.attitude.yaw};
    xSemaphoreGive(estimate_sem);

    xSemaphoreTake(references_sem,portMAX_DELAY);
        //count =count + 1;
        //if(count > 1000){
        //setpoint.attitude.roll=10.0;}
        double r_rpdy[3] = {setpoint.attitude.roll,setpoint.attitude.pitch,setpoint.attitudeRate.yaw}; 
    xSemaphoreGive(references_sem);

     error_states[0]= r_rpdy[0]- estimate[0];              // (deg)
     error_states[1]= r_rpdy[1] - estimate[1];             // (deg)
     error_states[2] = -gyro_data[0]; // derivative roll   (deg/s)
     error_states[3] = -gyro_data[1]; // derivative pitch  (deg/s)
     error_states[4] =  r_rpdy[2]-gyro_data[2];    // derivative yaw    (deg/s)

    /*
    LOG_GROUP_START(PWM_my)
    LOG_ADD(LOG_FLOAT, motor1, MOTOR_M1)
    LOG_ADD(LOG_FLOAT, motor2, MOTOR_M2)
    LOG_ADD(LOG_FLOAT, motor3, MOTOR_M3)
    LOG_ADD(LOG_FLOAT, motor4, MOTOR_M4)
    LOG_GROUP_STOP(PWM_my)
    */
    LOG_GROUP_START(ref_my)
    LOG_ADD(LOG_FLOAT, roll_ref, &setpoint.attitude.roll)
    LOG_ADD(LOG_FLOAT, pitch_ref, &setpoint.attitude.pitch)
    LOG_ADD(LOG_FLOAT, yaw_ref, &setpoint.attitude.yaw)
    LOG_GROUP_STOP(ref_my)



      
      motor_PWM[0]=K[0][0]*error_states[0]+K[0][1]*error_states[1]+K[0][2]*error_states[2]+K[0][3]*error_states[3]+K[0][4]*error_states[4]+30000;
      motor_PWM[1]=K[1][0]*error_states[0]+K[1][1]*error_states[1]+K[1][2]*error_states[2]+K[1][3]*error_states[3]+K[1][4]*error_states[4]+30000;
      motor_PWM[2]=K[2][0]*error_states[0]+K[2][1]*error_states[1]+K[2][2]*error_states[2]+K[2][3]*error_states[3]+K[2][4]*error_states[4]+30000;
      motor_PWM[3]=K[3][0]*error_states[0]+K[3][1]*error_states[1]+K[3][2]*error_states[2]+K[3][3]*error_states[3]+K[3][4]*error_states[4]+30000;
      
      
      //motor_PWM[0]=1540;
      //motor_PWM[1]=1540;
      //motor_PWM[2]=1500;
      //motor_PWM[3]=1540;
      

    xSemaphoreTake(motors_sem,portMAX_DELAY);
    motorsSetRatio(MOTOR_M1, motor_PWM[0]);//+1400);
    motorsSetRatio(MOTOR_M2, motor_PWM[1]);//+900);
    motorsSetRatio(MOTOR_M3, motor_PWM[2]);//+800);
    motorsSetRatio(MOTOR_M4, motor_PWM[3]);//+900);
    xSemaphoreGive(motors_sem);
  }
}


void stabilizerSetEmergencyStop()
{
  emergencyStop = true;
}

void stabilizerResetEmergencyStop()
{
  emergencyStop = false;
}

void stabilizerSetEmergencyStopTimeout(int timeout)
{
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}

/*************************************************
 * CREATE A TASK (Example, a task with priority 5)
 ************************************************/
 //STATIC_MEM_TASK_ALLOC(yourTaskName, configMINIMAL_STACK_SIZE);
 //STATIC_MEM_TASK_CREATE(yourTaskName, yourTaskName,
 //   "TASK_IDENTIFIER_NAME", NULL, 5);

/*************************************************
 * WAIT FOR SENSORS TO BE CALIBRATED
 ************************************************/
// lastWakeTime = xTaskGetTickCount ();
// while(!sensorsAreCalibrated()) {
//     vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
// }



/*************************************************
 * RETRIEVE THE MOST RECENT SENSOR DATA
 *
 * The code creates a variable called sensorData and then calls a function
 * that fills this variable with the latest data from the sensors.
 *
 * sensorData_t sensorData = struct {
 *     Axis3f acc;
 *     Axis3f gyro;
 *     Axis3f mag;
 *     baro_t baro;
 *     zDistance_t zrange;
 *     point_t position;
 * }
 *
 * Before starting the loop, initialize tick:
 * uint32_t tick;
 * tick = 1;
 ************************************************/
// sensorData_t sensorData;
// sensorsAcquire(&sensorData, tick);



/*************************************************
 * RETRIEVE THE SET POINT FROM ANY EXTERNAL COMMAND INTERFACE
 *
 * The code creates a variable called setpoint and then calls a function
 * that fills this variable with the latest command input.
 *
 * setpoint_t setpoint = struct {
 *     uint32_t timestamp;
 *
 *     attitude_t attitude;      // deg
 *     attitude_t attitudeRate;  // deg/s
 *     quaternion_t attitudeQuaternion;
 *     float thrust;
 *     point_t position;         // m
 *     velocity_t velocity;      // m/s
 *     acc_t acceleration;       // m/s^2
 *     bool velocity_body;       // true if velocity is given in body frame; false if velocity is given in world frame
 *
 *     struct {
 *         stab_mode_t x;
 *         stab_mode_t y;
 *         stab_mode_t z;
 *         stab_mode_t roll;
 *         stab_mode_t pitch;
 *         stab_mode_t yaw;
 *         stab_mode_t quat;
 *     } mode;
 * }
 *
 ************************************************/
// state_t state;
// setpoint_t setpoint;
// stateEstimator(&state, tick);
// commanderGetSetpoint(&setpoint, &state);



/*************************************************
 * SENDING OUTPUT TO THE MOTORS
 *
 * The code sends an output to each motor. The output should have the be
 * of the typ unsigned 16-bit integer, i.e. use variables such as:
 * uint16_t value_i
 *
 ************************************************/
// motorsSetRatio(MOTOR_M1, value_1);
// motorsSetRatio(MOTOR_M2, value_2);
// motorsSetRatio(MOTOR_M3, value_3);
// motorsSetRatio(MOTOR_M4, value_4);


/*************************************************
 * LOGGING VALUES THAT CAN BE PLOTTEN IN PYTHON CLIENT
 *
 * We have already set up three log blocks to for the accelerometer data, the
 * gyro data and the setpoints, just uncomment the block to start logging. Use
 * them as reference if you want to add custom blocks.
 *
 ************************************************/

/*
LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)
*/

/*
LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)
*/

/*
LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)
*/
