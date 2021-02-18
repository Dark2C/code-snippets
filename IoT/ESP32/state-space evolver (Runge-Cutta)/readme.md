# State Space System Evolver (Runge-Kutta)

This project configures a state space system using an ESP32 and implements a Runge-Kutta method to evolve the system. The system is defined by the matrices A, B, C, and D, and the number of inputs, states, and outputs.
The system is evolved using a Runge-Kutta method of order 4 with a fixed time step computed based on the hardware performance.

## Configuration

The script defines the following constants to configure the system:

- `N_INPUTS`: Number of inputs to the system.
- `N_STATES`: Number of states in the system.
- `N_OUTPUTS`: Number of outputs from the system.

Additionally, the following functions need to be updated based on the needs of the system:
- `void SS_Setup()`: Function to configure the matrices of the state space and initialize the initial conditions.
- `void readInput()`: Function to read the inputs (e.g., from sensors or mathematically).
- `void writeOutput()`: Function to write the outputs (e.g., to actuators or serial monitor).