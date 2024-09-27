// define the number of inputs, states and outputs of the system in order to configure the matrices of the state space
#define N_INPUTS 1
#define N_STATES 1
#define N_OUTPUTS 2

// the 4 matrices that describe the associated system are defined as global variables
// Do not change the definition of the matrices as those are defined by the size of the inputs, states and outputs
double A[N_STATES][N_STATES],
    B[N_STATES][N_INPUTS],
    C[N_OUTPUTS][N_STATES],
    D[N_OUTPUTS][N_INPUTS],
    inputs[N_INPUTS],
    currentStatuses[N_STATES],
    outputs[N_OUTPUTS];

bool dxCalibrated = false;

// this function is used to configure the output pins and to define the matrices of the state space
void SS_Setup()
{
  Serial.begin(115200);
  // set some pins as output through the ledc library, for example here pins 15 and 16 are set as output
  // with a frequency of 5000 Hz and a resolution of 8 bits, you can choose based on your needs
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(15, 0);
  ledcAttachPin(16, 1);
  // configure the matrices of the state space and initialize the inputs and the initial conditions if not null
  // A = ...;
  // B = ...;
  // C = ...;
  // the initial conditions are set in the currentStatuses array:
  // currentStatuses = ...;
}

// this function handles the reading of the inputs from the sensors into the inputs array
void readInput()
{
  // read the inputs from the sensors, for example here the input is read from pin 13 and scaled to the range [0, 1]
  // change the code based on your needs, you can read multiple inputs if needed
  inputs[0] = map(analogRead(13), 0, 4095, 0, 1);
}

// this function handles the writing of the outputs to the actuators from the outputs array
void writeOutput()
{
  // write the outputs to the actuators, for example here the output is written to pins 15 and 16 set before
  ledcWrite(0, map(outputs[0], 0, 1, 0, 256));
  ledcWrite(1, map(outputs[1], 0, 1, 0, 256));
  // you could also print the outputs to the serial monitor like this:
  Serial.println(outputs[0]);
}

// STOP CHANGING THE CODE BELOW THIS LINE

#include "soc/rtc_wdt.h"
#include <esp_task_wdt.h>
TaskHandle_t core0;
TaskHandle_t core1;

void setup()
{
  // Disable the watchdogs as the functions that compute the matrices and the ones that handle the IO are blocking
  disableCore0WDT();
  disableCore1WDT();
  // initialize the matrices and the vectors to zero, we assume that the values not defined in the SS_Setup function
  // are zero so we have to manually define only the non-zero values
  for (int i = 0; i < N_STATES; i++)
    currentStatuses[i] = 0;
  for (int i = 0; i < N_INPUTS; i++)
    inputs[i] = 0;
  for (int st = 0; st < N_STATES; st++)
  {
    for (int st2 = 0; st2 < N_STATES; st2++)
      A[st][st2] = 0;
    for (int ing = 0; ing < N_INPUTS; ing++)
      B[st][ing] = 0;
  }
  for (int usc = 0; usc < N_OUTPUTS; usc++)
  {
    for (int st = 0; st < N_STATES; st++)
      C[usc][st] = 0;
    for (int ing = 0; ing < N_INPUTS; ing++)
      D[usc][ing] = 0;
  }
  SS_Setup();

  // create the tasks that handle the IO and the state space evolution and assign them to two different cores to run in parallel
  xTaskCreatePinnedToCore(IO_manager, "IO_manager", 10000, NULL, 1, &core0, 0);
  xTaskCreatePinnedToCore(SS_process, "SS_evolver", 10000, NULL, 1, &core1, 1);
}

void loop() {}

void IO_manager(void *pvParameters)
{
  // the IO manager is responsible for reading the inputs and writing the outputs, it must start after the dx calibration
  while (!dxCalibrated)
    vTaskDelay(1);

  while (true)
  {
    readInput();
    writeOutput();
  }
}

// the following function implements the Runge-Kutta 4th order method to evolve the state space of the system
// this function first makes an estimation of the step size based on the time it took to compute 10000 iterations
// then it uses this dx to compute the next state of the system
void SS_process(void *pvParameters)
{
  double intermediateStatuses[N_STATES],
      BU[N_STATES],
      CX[N_OUTPUTS],
      DU[N_OUTPUTS],
      K1[N_STATES],
      K2[N_STATES],
      K3[N_STATES],
      K4[N_STATES];

  // make a backup of the current statuses as we need to estimate the step dx and it will change the current statuses
  double currentStatusesBackup[N_STATES];
  for (int i = 0; i < N_STATES; i++)
    currentStatusesBackup[i] = currentStatuses[i];

  double STEP = 0.00001; // temporary value for the step size (in seconds)

  int i;
  int forLoopIncrement = 1;
  while (true)
  {
    unsigned long t = micros();
    for (int j = 0; j < 10000; j += forLoopIncrement)
    {
      matrixVectorProduct((double *)B, inputs, N_STATES, N_INPUTS, BU);
      matrixVectorProduct((double *)C, currentStatuses, N_OUTPUTS, N_STATES, CX);
      matrixVectorProduct((double *)D, inputs, N_OUTPUTS, N_INPUTS, DU);

      // K1 = h * f(x0)
      matrixVectorProduct((double *)A, currentStatuses, N_STATES, N_STATES, K1);
      for (i = 0; i < N_STATES; i++)
      {
        K1[i] = STEP * (K1[i] + BU[i]);
        intermediateStatuses[i] = currentStatuses[i] + K1[i] * 0.5;
      }

      // K2 = h * f(x0 + K1/2)
      matrixVectorProduct((double *)A, intermediateStatuses, N_STATES, N_STATES, K2);
      for (i = 0; i < N_STATES; i++)
      {
        K2[i] = STEP * (K2[i] + BU[i]);
        intermediateStatuses[i] = currentStatuses[i] + K2[i] * 0.5;
      }

      // K3 = h * f(x0 + K2/2)
      matrixVectorProduct((double *)A, intermediateStatuses, N_STATES, N_STATES, K3);
      for (i = 0; i < N_STATES; i++)
      {
        K3[i] = STEP * (K3[i] + BU[i]);
        intermediateStatuses[i] = currentStatuses[i] + K3[i];
      }

      // K4 = h * f(x0 + K3)
      matrixVectorProduct((double *)A, intermediateStatuses, N_STATES, N_STATES, K4);
      for (i = 0; i < N_STATES; i++)
      {
        K4[i] = STEP * (K4[i] + BU[i]);
        currentStatuses[i] += (K1[i] + 2 * K2[i] + 2 * K3[i] + K4[i]) / 6;
      }

      for (i = 0; i < N_STATES; i++)
        outputs[i] = CX[i] + DU[i];
    }

    // estimate the step dx based on the number of iterations it took to compute the step
    STEP = (double)(micros() - t) / 10000000000;

    // restore the current statuses from the backup
    for (int i = 0; i < N_STATES; i++)
      currentStatuses[i] = currentStatusesBackup[i];

    // set the increment of the for loop to 0, basically transforming the for loop into a while with condition (true)
    forLoopIncrement = 0;

    // set the flag to true to indicate that the step dx has been calibrated
    dxCalibrated = true;
  }
}

void matrixVectorProduct(double *A, double *B, int rows, int cols, double *result)
{
  int i, j;
  for (i = 0; i < cols; i++)
    result[i] = 0;
  for (i = 0; i < rows; i++)
  {
    for (j = 0; j < cols; j++)
    {
      result[i] += A[i * rows + j] * B[j];
    }
  }
}
