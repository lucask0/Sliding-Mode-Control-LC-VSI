
// Global constants
#define L 10.0e-3f      // inductance
#define R 1.0f          // resistance
#define k 1.0f/25.0e-6f // equivalent "elasticity"
#define lambda 1.0e-4f  // decay rate
#define f_s 20000.0f    // sampling frequency
#define T_s 1.0f/f_s    // sampling time
#define eta 0.1f*300.0f // define eta as a fraction of signal frequency

#define FLOOR_PREC 0.1f
#define CEIL_PREC 0.1f

// Define uncertainties
#define L_PLUS (1 + CEIL_PREC) * L
#define L_MINUS (1 - FLOOR_PREC) * L
#define R_PLUS (1 + CEIL_PREC) * R
#define R_MINUS (1 - FLOOR_PREC) * R
#define k_PLUS (1 + CEIL_PREC) * k
#define k_MINUS (1 - FLOOR_PREC) * k

#define INTEGRATOR_SATVAL 10.0f

#define BOUNDARY_LAYER 1

#define AMPLITUDE 0.1f
#define FREQUENCY 4146.9

#define PI 3.1415926535f

// d according to (14)
#define DELTA (L_PLUS - L_MINUS) / L_MINUS