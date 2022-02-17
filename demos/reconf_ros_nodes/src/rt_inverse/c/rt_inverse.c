/********************************************************************          
* main.c           -software implementation inverse thread function	*
*                        				                            *
*                                                                   *  
* Author(s): Christoph Rueting, Christian Lienen                    *   
*                                                                   *   
********************************************************************/
#include "reconos_thread.h"
#include "reconos_calls.h"
#include "utils.h"

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <float.h>


#define DIRECTAXIACCESS 1
#define DEBUG 0
#define DEBUG_LIGHT 1

// definitions of stewart platform

// coordinates of platform joints in base coordinates
int p_p_j_x[6] = {51,-51,-64,-13,13,64};
int p_p_j_y[6] = {44,44,22,-66,-66,22};

// transformation to base coordinates
float t_p2b_t_z = 80 + 25;

// transformation to servo coordinates
float t_b2s_rz_sin[6] = {0.00000000,0.00000000,-0.86602540,-0.86602540,0.86602540,0.86602540};
float t_b2s_rz_cos[6] = {1.00000000,1.00000000,-0.50000000,-0.50000000,-0.50000000,-0.50000000};
int t_b2s_s_x[6] = {-1,1,-1,1,-1,1};
int t_b2s_s_y[6] = {1,1,1,1,1,1};
int t_b2s_s_z[6] = {1,-1,1,-1,1,-1};
int t_b2s_t_x = 33;
int t_b2s_t_y = -69;
int t_b2s_t_z = 0;

// leg and arm lengths
int c_leg = 115;
int c_legs = 115 * 115;
int c_arm = 20;

// sine and cosine lookup tables
#define TRIG_COUNT 2048
#define TRIG_ADDR 11
#define TRIG_MIN_ANGLE_DEG 0
#define TRIG_MAX_ANGLE_DEG 204.7
#define TRIG_STEP 0.1



static inline float radians(float deg) {
	return deg * (M_PI / 180.0);
}
static inline float sin_lut(float a) {
	return sin(radians(a * TRIG_STEP));
}
static inline float cos_lut(float a) {
	return cos(radians(a * TRIG_STEP));
}

	static inline uint32_t fltofi(float f, int n, int dn) {
		int i;
		int wn = n - dn;

		int d;
		float w;

		uint32_t df, wf;
		uint32_t m = 0xffffffff;

		if (f > 0) {
			d = floor(f);
			w = f - d;
		} else {
			d = floor(f);
			w = f - d;
		}

		df = 0 | (d << wn);

		wf = 0;
		for (i = -1; i >= -wn; i--) {
			if (w >= pow(2, i)) {
				wf |= 1 << (wn + i);
				w -= pow(2, i);
			}
		}

		return (df | wf) & (m >> (32 - n));
	}

	static inline float fitofl(uint32_t f, int n, int dn) {
		int i;
		int wn = n - dn;

		int d;
		float w;

		uint32_t m = 0xffffffff;

		if (((f >> (n - 1)) & 0x1) == 1) {
			d = (m << dn) | (f >> wn);
		} else {
			d = 0 | (f >> wn);
		}

		w = 0;
		for (i = -1; i >= -wn; i--) {
			if (((f >> (wn + i)) & 0x1) == 1) {
				w += pow(2, i);
			}
		}

		return d + w;
	}


THREAD_ENTRY() {
	int i;
	clock_t start, end;
	
	THREAD_INIT();

	while (1) {
		

		ROS_SUBSCRIBE_TAKE(inverse_subdata, inverse_input_msg);
		start = clock();
		uint32_t data = inverse_input_msg->data;

		float t_p2b_alpha = fitofl((data >> 17) & 0x3fff, 14, 6);
		float t_p2b_beta  = fitofl((data >> 3) & 0x3fff, 14, 6);
#if (DEBUG == 1) || (DEBUG_LIGHT == 1)
		//printf("[Inverse] alpha %f, beta %f \n", t_p2b_alpha, t_p2b_beta);
#endif
		float t_p2b_alpha_sin = sin_lut(t_p2b_alpha * 10.0f);
		float t_p2b_alpha_cos = cos_lut(t_p2b_alpha * 10.0f);
		float t_p2b_beta_sin  = sin_lut(t_p2b_beta  * 10.0f);
		float t_p2b_beta_cos  = cos_lut(t_p2b_beta  * 10.0f);
#if DEBUG == 1
		printf("[Inverse] t_p2b_alpha_sin %f, t_p2b_alpha_cos %f, t_p2b_beta_sin %f, t_p2b_beta_cos %f \n", t_p2b_alpha_sin, t_p2b_alpha_cos, t_p2b_beta_sin, t_p2b_beta_cos);
#endif

		int leg = (data >> 0) & 0x7;

		float p_b_j_x, p_b_j_y, p_b_j_z;
		float p_s_j_x, p_s_j_y, p_s_j_z;

		float v_s_aj_x, v_s_aj_y, v_s_aj_z;
		float v_s_aj_l_min, v_s_aj_l;
		unsigned int v_s_aj_l_mina = 0;


		// transform into base coordinatesystem
		// rotate around ra_x, ra_y, ra_z by ra_sin/ra_cos
		p_b_j_x =  	 t_p2b_beta_cos * p_p_j_x[leg];

		p_b_j_y =    t_p2b_alpha_sin * t_p2b_beta_sin * p_p_j_x[leg] 
				  +  t_p2b_alpha_cos * p_p_j_y[leg]; 

		p_b_j_z =   -t_p2b_alpha_cos * t_p2b_beta_sin * p_p_j_x[leg] 
				  +  t_p2b_alpha_sin * p_p_j_y[leg];
#if DEBUG == 1
		printf("[Inverse] p_b_j_x %f, p_b_j_y %f, p_b_j_z %f \n", p_b_j_x, p_b_j_y,p_b_j_z);
#endif
		// translate by t_Z
		p_b_j_z = p_b_j_z + t_p2b_t_z;

		// transform into servo coordinatesystem based on leg id
		// rotate around z axis by rz_sin/rz_cos
		p_s_j_x = t_b2s_rz_cos[leg] * p_b_j_x - t_b2s_rz_sin[leg] * p_b_j_y + 0 * p_b_j_z;
		p_s_j_y = t_b2s_rz_sin[leg] * p_b_j_x + t_b2s_rz_cos[leg] * p_b_j_y + 0 * p_b_j_z;
		p_s_j_z = 0                 * p_b_j_x + 0                 * p_b_j_y + 1 * p_b_j_z;

		// scale axis by s_x, s_y and s_z
		p_s_j_x = t_b2s_s_x[leg] * p_s_j_x;
		p_s_j_y = t_b2s_s_y[leg] * p_s_j_y;
		p_s_j_z = t_b2s_s_z[leg] * p_s_j_z;

		// translate by t_x, t_y and t_z
		p_s_j_x = p_s_j_x + t_b2s_t_x;
		p_s_j_y = p_s_j_y + t_b2s_t_y;
		p_s_j_z = p_s_j_z + t_b2s_t_z;


		// find angle for leg length
		v_s_aj_l_min = FLT_MAX;
		for (i = 0; i < TRIG_COUNT; i++) {
			// calculate vector from arm to joint
			v_s_aj_x = p_s_j_x - sin_lut(i) * c_arm;
			v_s_aj_y = p_s_j_y - 0;
			v_s_aj_z = p_s_j_z - cos_lut(i) * c_arm;
			v_s_aj_l = v_s_aj_x * v_s_aj_x + v_s_aj_y * v_s_aj_y + v_s_aj_z * v_s_aj_z;

			if (c_legs < v_s_aj_l) {
				if (v_s_aj_l - c_legs < v_s_aj_l_min) {
					v_s_aj_l_min = v_s_aj_l - c_legs;
					v_s_aj_l_mina = i;
				}
			} else {
				if (c_legs - v_s_aj_l < v_s_aj_l_min) {
					v_s_aj_l_min = c_legs - v_s_aj_l;
					v_s_aj_l_mina = i;
				}
			}
		}

		inverse_output_msg->data = ((v_s_aj_l_mina << 21) | (leg << 18) | 0);
		end = clock();
		ROS_PUBLISH(inverse_pubdata, inverse_output_msg);
		printf("%3.6f; \n", (double)(end-start)/CLOCKS_PER_SEC);
	}
}