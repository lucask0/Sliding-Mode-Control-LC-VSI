#include <math.h>

// GLobal constants
#define L 0.0f // inductance
#define R 0.0f // resistance
#define k 0.0f // equivalent "elasticity"
#define lambda 0.0f // decay rate
#define f_s 20000.0f // sampling frequency
#define T_s 1/f_s // sampling time

// Global variables
static float u; // Control effort
static float K_big; // Gain
static float epsilon; // Sliding function
static float i; // Measured current
static float q; // Measured charge
static float d_i_star; // First-order derivative of reference current
static float d_qtilde; // First-order derivatide of tilde measured charge

// Function declarations
void update_i();
void update_q();
void update_d_i_star();
void update_d_qtilde();
void update_epsilon();
void update_K_big();


// Function implementations

void calculate_u()
{
    // This function calculates the new value of the control effort
    // and should be called periodically by the microcontroller
    //
    // OUTPUTS
    // u: control effort
    //
    // INPUTS
    // L, R, k, lambda : constants
    // K_big, epsilon, i, q, d_i, d_qtilde : to be calculated

    // Update necessary variables
    // Execution order matters!
    update_i();
    update_q();
    update_d_i_star();
    update_d_qtilde();
    update_epsilon();
    update_K_big();

    // Calculate control effort acc. to (4)

    u =  -L*(K*epsilon-(R/L*i+k/L*q)-d_i_star+lambda*d_qtilde);

}

void update_i()
{
    // This function updates the value of i
    //
    // OUTPUTS
    // i: current
    //
    // INPUTS
    //
    // Microcontroller internal variables

    i = adc; // TODO: update with actual microcontroller variables

}

void update_q()
{
    // This function updates the value of q using simple integration of i
    //
    // OUTPUTS
    // q: charge
    //
    // INPUTS
    //
    // i: current
    //
    // INTERNAL VARIABLES
    // 
    // integrator: accumulator for integral

    static float integrator = 0.0f;

    // do integral of i
    integrator += i * T_s; 

    // check for overflow
    if (integrator > 1000.0f){
        integrator = 0.0f;
    }
    if (integrator < -1000.0f){
        integrator = 0.0f;
    }

    // output results
    q = integrator;

}

void d_i_star()
{
    // This function updates the value of d_i_star by calculating the first derivative
    // reference current
    //
    // OUTPUTS
    // d_i_star: derivative of the reference current
    //
    // INPUTS
    //
    // none
    //
    // INTERNAL VARIABLES
    // 
    // angle: reference angle (updated internally)
    // i_star: reference current
    // i_star_old: previous value for the reference current
    
    static float angle = 0.0f;
    static float i_star = 0.0f;
    static float i_star_old = 0.0f;

    // update angle step

    angle += T_s*1884.95559f; // calculated for 2*pi*f*t with 300Hz

    // handle overflow (angle>2*pi)
    if (angle>6.28318531f)
    {
        angle = 0.0f;
    }

    // store old reference
    i_star_old = i_star;

    // calculate reference current
    i_star = sinf(angle);

    // calculate derivative output
    d_i_star = (i_star - i_star_old) / T_s;

}