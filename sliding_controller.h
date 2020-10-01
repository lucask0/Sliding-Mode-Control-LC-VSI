// @Authors: 
//     - Bruno Peixoto
//     - Lucas Koleff
// https://www.overleaf.com/read/gcxtkqsndkkq
// @Date: 30/09/2020
#ifndef SLIDING_CONTROLLER_H
#define SLIDING_CONTROLLER_H

	// Global variables
	static extern float u; 			// Control effort
	static extern float K; 			// Gain
	static extern float epsilon; 	// Sliding function
	static extern float i_old;		// Old current
	static extern float d_i; 		// Measured current
	static extern float i; 			// Measured current
	static extern float q; 			// Measured charge
	static extern float d_i_star; 	// First-order derivative of reference current
	static extern float d_qtilde; 	// First-order derivatide of tilde measured charge
	static extern float d_itilde; 	// First-order derivative of
	static extern float i_star;

	// Function declarations
	void update_control_vars();

	void update_states();
	void update_di();
	void update_i();
	void update_q();

	void update_reference();
	void update_di_star();
	void update_dq_star();

	void update_errors();
	void update_di_tilde();
	void update_dq_tilde();

	void update_epsilon();

	void update_K();
	void calculate_u()
#endif