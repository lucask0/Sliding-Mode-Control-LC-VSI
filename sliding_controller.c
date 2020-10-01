// @Authors: 
//     - Bruno Peixoto
//     - Lucas Koleff
// https://www.overleaf.com/read/gcxtkqsndkkq
// @Date: 30/09/2020
#include "sliding_controller.h"
#include "includes.h"

#include <math.h>
#include "includes.h"
#include "sliding_controller.h"

void update_control_vars(){
	// Update necessary variables
    // Execution order matters!
    update_di();
    update_i();
    update_q();

    update_d_i_star();

    update_d_qtilde();
    update_d_itilde();

    update_epsilon();
    update_K();
}

void update_di(){
// This function updates the value of i
//
// OUTPUTS
// i: current
//
// INPUTS
//
// Microcontroller internal variables

    // TODO: Update with precise values
    d_i = (i - i_old)/T_s;
}

void update_i(){
// This function updates the value of i
//
// OUTPUTS
// i: current
//
// INPUTS
//
// Microcontroller internal variables
    // TODO: update with actual microcontroller variables
    i = adc;
    i_old = i;
}

void update_q(){
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
    if (integrator > INTEGRATOR_SATVAL){
        integrator = 0.0f;
    }
    if (integrator < -INTEGRATOR_SATVAL){
        integrator = 0.0f;
    }

    // output results
    q = integrator;
}

void update_states(){
	// Order matters
	update_q();
	update_i();
	update_di();
}

void update_dq_star()
{
// This function updates the value of d_q_star by calculating the reference current 
// first derivative
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
    static float i_star_old = 0.0f;

    // update angle step
    angle = angle + T_s*FREQUENCY; // calculated for 2*pi*f*t with 660Hz

    // handle overflow (angle>2*pi)
    if (angle>2*PI){
        angle = 0.0f;
    }

    // store old reference
    q_star_old = q_star;

    // calculate reference current
    q_star = (AMPLITUDE/FREQUENCY)*sin(angle);

    // calculate reference current
    d_q_star = AMPLITUDE*cos(angle);
}

void update_di_star()
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

    // update angle step - calculated for 2*pi*f*t with 660Hz
    angle = angle + T_s*FREQUENCY;

    // handle overflow (angle>2*pi)
    if (angle>2*PI){
        angle = 0.0f;
    }

    // calculate reference current
    i_star = AMPLITUDE*cos(angle);

    // calculate derivative output
    d_i_star = -(FREQUENCY*AMPLITUDE)*sin(angle);
}

void update_reference(){
	update_d_q_star();
	update_d_i_star();
}

void update_dq_tilde(){
    // This function updates the value of d_qtilde by calculating the first derivative
    // of the estimated charge
    //
    // OUTPUTS
    // d_qtilde: derivative of the estimated charge
    //
    // INPUTS
    //
    // q: charge
    //

    // calculate derivative output
    d_qtilde = i - i_star;

}

void update_di_tilde(){
// This function updates the value of d_itilde by calculating the first derivative
// of the measured/estimated current
//
// OUTPUTS
// d_itilde: derivative of the measured/estimated current
//
// INPUTS
//
// i: current
//

    // calculate derivative output
    d_itilde = d_i - d_i_star;
}

void update_errors(){
	update_d_q_tilde();
	update_d_i_tilde();
}

void update_epsilon(){
// This function updates the value of epsilon(s) 
//
// OUTPUTS
// epsilon: derivative of the estimated charge
//
// INPUTS
//
// q: charge
// i: current
//
// INTERNAL VARIABLES
// 
// s: sliding surface
    
    float s = 0.0f;

    // first, calculate s according to (2)
    s = i_tilde + lambda * q_tilde;

    // apply saturation function on s to calculate epsilon
    if (s > BOUNDARY_LAYER){
        epsilon = 1;
    } else if (s<-BOUNDARY_LAYER){
        epsilon = -1;
    } else {
        epsilon = s;
    }
}

void update_K(){
// This function updates the value of K
//
// OUTPUTS
// K
//
// INPUTS
//
// q, i, eta, Fs, d, R, L, k, d_i_star, lambda, d_i_tilde
// R_PLUS, R_MINUS, L_PLUS, L_MINUS, k_PLUS, k_MINUS
//
// INTERNAL VARIABLES
// 
// Fs
    float Fs = 0.0f;

    Fs = fabs((R_PLUS*L_PLUS-R_MINUS*L_MINUS))/L_MINUS/L_MINUS*fabs(i) + fabs((k_PLUS*L_PLUS-k_MINUS*L_MINUS))/L_MINUS/L_MINUS*fabs(q);

    C = 1/(1-d);
    K = C*(eta + Fs + DELTA * fabs(-R/L * i - k/L * q - d_i_star + lambda * itilde));
}

// Function implementations
void calculate_u(){
// This function calculates the new value of the control effort
// and should be called periodically by the microcontroller
//
// OUTPUTS
// u: control effort
//
// INPUTS
// L, R, k, lambda : constants
// K, epsilon, i, q, d_i, d_qtilde : to be calculated

    // Calculate control effort acc. to (4)
    s_r = -d_i_star + lambda*itilde;
    fhat_s = (R/L)*i + (k/L)*q;
    u =  -L*(-fhat_s + s_r + K*epsilon);
}