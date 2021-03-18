/* _________________________________________________________________________________________________________________*/
/* Include-Files */

#include "FOC.h"

/* _________________________________________________________________________________________________________________*/
/* Definition von Makros */

/* _________________________________________________________________________________________________________________*/
/* Definition von Konstanten */

#define ALPHA_CUPPER_20 3.9e-3f		/* Temperaturkoeffizent von Kupfer 																							 */
#define TWO_PI 6.28318530717958f	/* 2*PI 																																				 */
#define V_REF_CURRENT 3.338f 			/* V_ref der ADC-Messung, V_REF_CURRENT = (4095/eingelesenerWert)/gemessenerWert */

/* _________________________________________________________________________________________________________________*/
/* Datentypdefinitionen */

/* _________________________________________________________________________________________________________________*/
/* Funktionsdeklarationen */

static float32_t ADC_scale_Current (uint16_t ADC, float32_t Offset);
#ifdef PLECS_MODE
static float32_t ADCscaleAngle_grad (uint16_t ADC);
#endif
static float32_t Convert_Angle_grad (float32_t Angle_mech, uint16_t ppz, float32_t Offset);
//static float32_t radtograd (float32_t Angle);
static uint16_t limit_dq (float32_t max, DQ *ist);
void emulat_theta(float32_t *theta);

/* _________________________________________________________________________________________________________________*/
/* Definition globaler Variablen */

/* injected ADC-Kanaele */ 
static uint32_t Phase_U = TIM_CHANNEL_1;
static uint32_t Phase_V = TIM_CHANNEL_2;
static uint32_t Phase_W = TIM_CHANNEL_3;
static uint32_t Trigger = TIM_CHANNEL_4;

/* Inverter- & Motorvariablen */
static Inverter inverter;
static Motor motor;
								
/* Matlab-Reglervariablen */
static RT_MODEL_T M;
static float32_t K_pid = 0;
static float32_t K_piq = 0;
static float32_t I_pid = 0;
static float32_t I_piq = 0;
const static float32_t attenuation_factor_p = 0.25f;
const static float32_t attenuation_factor_i = 0.25f;

//alles bis auf i_d und i_q darf nicht global sein
volatile DQ i_dq = { 0, 0};
volatile AB i_ab = { 0, 0};
volatile float32_t theta_emulat 	 = 0;
static int16_t pos_init_flag 			 = 0;
volatile float32_t theta_grad_mech = 0;

/* Offset der ADC-Eingaenge zur Strommessung [V] */
static float32_t	offset_i_a = 0;		
static float32_t	offset_i_b = 0;
uint8_t data_angle[4];

/* Logging */
static volatile float32_t logcounter = 0;	/* Counter, wie oft Logging bereits aufgerufen wurde */


/* hier für den Test von Ausführungszeiten ... */
/*
volatile float32_t diff_without_sd;
volatile float32_t diff_with_sd;
volatile uint32_t	tick_start;
volatile uint32_t tick_end;
volatile uint32_t tick_end_sd;
*/

/* _________________________________________________________________________________________________________________*/
/* Funktionsdefinitionen */

/* Initialsierung der Schleife zur Berechnung der feldorientrierten Regelung */
//void INIT_FOC(void)
//{
//	/* Motorparamter initialisieren */
//	get_motor_param(&motor);
//	get_inverter_param(&inverter);
//	
//	/* erstmaliges Berechnen der Reglerparameter */
//	//calc_param_controller(Temp_Motor);
//	calc_param_controller(25.0f);
//		
//	/* BSP PWM Initialisation */
//	PWM_modulation_start();
//}

/* Hauptschleife zur Berechnung der feldorientrierten Regelung
 * This function is called as callback from the Timer-Update Event: HAL_ADCEx_InjectedConvCpltCallback()

typedef struct
{
  __IOM uint32_t CTRL;                   //!< Offset: 0x000 (R/W)  SysTick Control and Status Register 
  __IOM uint32_t LOAD;                   //!< Offset: 0x004 (R/W)  SysTick Reload Value Register 
  __IOM uint32_t VAL;                    //!< Offset: 0x008 (R/W)  SysTick Current Value Register 
  __IM  uint32_t CALIB;                  //!< Offset: 0x00C (R/ )  SysTick Calibration Register 
} SysTick_Type;

 */
void FOC_loop(uint16_t *ADC_InjectedValues)
{
	/* Variablendeklarationen */
	float32_t sinVal, cosVal;
	uint16_t testval;
	AlphaBeta i_alphabeta;
	AlphaBeta u_alphabeta;
	DQ u_ref;
	UVW T_on;
	uint16_t angle_mangle;
				
	/* ADC-Werte Einlesen und Skalieren */
	i_ab.a = ADC_scale_Current(ADC_InjectedValues[CURRENT_PHASE_U], offset_i_a);														
	i_ab.b = (-1) * ADC_scale_Current(ADC_InjectedValues[CURRENT_PHASE_V], offset_i_b); 	/* Korrektur des gespiegelten Hardwareaufbaus */
//  testval = ADC_InjectedValues[CURRENT_PHASE_U];

	
	//hier reinpfuschen um Zeiger weiterzudrehen :D
	
//	#ifndef RESOLVERLESS_MODE
//	/* Positionswert Einlesen und Skalieren */	
//	#ifdef PLECS_MODE
//		theta_grad_mech = ADCscaleAngle_grad(ADC_InjectedValues[Theata_RT_Box]);
//	#else
//		theta_grad_mech = get_position_estimate(omega_mech);
//	
////		angle_mangle = theta_grad_mech;
////		data_angle[0] = 'a';
////		data_angle[1] = angle_mangle/100 + 48;
////		data_angle[2] = ((angle_mangle/10) % 10) + 48;
////		data_angle[3] = angle_mangle % 10 + 48;
////	  HAL_UART_Transmit(&huart3,data_angle,4,10);
//	#endif
//	#else
	//elektrisches Feld soll mit 500 Hz drehen ... -> Mechanisches Feld = elektrisches Feld / PPZ = 100 Hz;
		while(theta_grad_mech > 360.0f)	/* Modulo 360? */
			{
				theta_grad_mech = theta_grad_mech - 360.0f;
			}
			theta_grad_mech = theta_grad_mech + 3.6f;
	#endif
	theta_grad_el = Convert_Angle_grad(theta_grad_mech, motor.PPZ, motor.offset_pos);
	
	
	/* Clarke/Park Vorwaertstransformation */
	arm_sin_cos_f32 (theta_grad_el, &sinVal, &cosVal);	
	arm_clarke_f32 (i_ab.a, i_ab.b, &i_alphabeta.alpha , &i_alphabeta.beta);	
	arm_park_f32 (i_alphabeta.alpha, i_alphabeta.beta, &i_dq.d, &i_dq.q, sinVal, cosVal);			

	/* Stromregelung */
	pi_controller_step(&M, i_d_set, i_q_set, i_dq.d, i_dq.q, K_pid, K_piq,
  I_pid, I_piq, omega_el, motor.L_d, motor.L_q, motor.flux, 
	U_zk, &u_ref.d, &u_ref.q);
	
	/* Kompensation der unterschiedliche Definitionen der Transformationsmatritzen 
	 * zwischen ARM-Funktionen und dem Matlab-Regler */
		//keine Ahnung warum das hier passiert/nötig ist ...
	u_ref.d = (-1) * u_ref.d;
	u_ref.q = (-1) * u_ref.q;
		
	/* Limitierung der Reglerausgangswerte UQ & UD auf die maximale Systemspannung */
	limit_dq(600.0f, &u_ref);		
			
	/* Ruecktransforamtion in Alpha- & Beta-Werte */
	arm_inv_park_f32 (u_ref.d, u_ref.q, &u_alphabeta.alpha, &u_alphabeta.beta, sinVal, cosVal); 		

	/* Berechnung der duty cycle */
	SVPWM(&u_alphabeta, U_zk, &T_on);	
	
	/* Uebernahme der duty cycle Werte in die PWM-Register */	
	PWM_set_dutyclclye(htim1, T_on.u, Phase_U);													
	PWM_set_dutyclclye(htim1, T_on.v, Phase_V);													
	PWM_set_dutyclclye(htim1, T_on.w, Phase_W);	
	
	/* Logging */
	//log_test_3(i_q_set, i_dq.q, u_ref.q);
	
//	#ifdef LOG_SD_CARD
//	/* Logging auf SD-Karte */
//	logcounter++;
//	log_value(logcounter, LOG_CH_1, PWM_FREQUENZ, SAMPLING_RATE_LOGGING_CH1);
//	log_value(i_d_set, LOG_CH_2, PWM_FREQUENZ, SAMPLING_RATE_LOGGING_CH2);
//	log_value(i_q_set, LOG_CH_3, PWM_FREQUENZ, SAMPLING_RATE_LOGGING_CH3);
//	//log_value(i_dq.d, LOG_CH_4, PWM_FREQUENZ, SAMPLING_RATE_LOGGING_CH4);
//	log_value(i_dq.q, LOG_CH_5, PWM_FREQUENZ, SAMPLING_RATE_LOGGING_CH5);
//	#endif

}			

static float32_t ADC_scale_Current (uint16_t ADC, float32_t Offset)
{
	float32_t Current;
	/* Offsetkorrektur und Skalierung (Sensoraufloesung = 13,33mV/A, Spannungsteiler = 1200/1760) */
	Current = ((((((float32_t)ADC) / 4095.0f) * V_REF_CURRENT) - Offset)/0.547f) * 150.0f;
	return Current;
}

//void ADC_Injected_Offset_Compensation(uint16_t *ADC_InjectedValues)
//{
//	/* Offset des Stromsensors wird eingelesen und abgespeichert, laut Datenblatt betraegt er nach dem Spannungsteiler 1,65V */
//	offset_i_a = ((float32_t)ADC_InjectedValues[CURRENT_PHASE_U] / 4095.0f) * V_REF_CURRENT;
//	offset_i_b = ((float32_t)ADC_InjectedValues[CURRENT_PHASE_V] / 4095.0f) * V_REF_CURRENT;
//	FLAG_ADC_InjectedOffsetCompensationCplt = 1;	
//}

//static float32_t Convert_Angle_grad (float32_t Angle_mech, uint16_t PPZ, float32_t Offset)
//{
//	float32_t Angle_el;
//	
//	Angle_el = PPZ * Angle_mech;
//	Angle_el = Angle_el + Offset;
//	
//	/* Modulo 360 */
//	while (Angle_el > 360.0f)
//		Angle_el = Angle_el - 360.0f;
//	
//	return Angle_el;
//}

//static float32_t radtograd (float32_t Angle)
//{
//	/* Angle * 180 / PI */
//	Angle = (Angle * 57.29577951308238f);
//	return Angle;
//}


/*hier nochmal reinpfuschen (wenn d > max und q > max -> d = max und q > max) */
static uint16_t limit_dq (float32_t max, DQ *ist)
{
	
uint16_t state = 0;
		
	if((*ist).d < max * (-1)) {
		(*ist).d = max * (-1);
		state = 1;
		}
	else if ((*ist).d > max) {
		(*ist).d = max;
		state = 1;
		}
	if((*ist).q < max * (-1)) {
		(*ist).q = max * (-1);
		state = 1;
		}
	else if ((*ist).q > max) {
		(*ist).q = max;
		state = 1;
		}
	return state;
}

//void calc_param_controller(float32_t Temp_Motor)
//{
//	float32_t R_a_calc;
//	
//	R_a_calc = motor.R_a * (1.0f + (ALPHA_CUPPER_20 * (Temp_Motor - 20.0f)));
//	
//	param_controller_step(&M, R_a_calc, motor.U_an, motor.I_an, motor.T_sigid, motor.T_sigiq, motor.L_d,
//  motor.L_q, attenuation_factor_p, attenuation_factor_i, &K_pid, &I_pid, &K_piq, &I_piq);
//}

//float32_t calc_Motor_Torque(void)
//{
//	float32_t Motor_Torque;

//	torque_calculation_step(&M, i_dq.d, i_dq.q, motor.L_d, motor.L_q, motor.PPZ, motor.flux, &Motor_Torque);
//	
//	return Motor_Torque;
//}

//void emulat_theta(float32_t *theta)
//{
//	*theta += 10;
//	if (*theta > 360.0f)
//	{
//		*theta = 0;
//	}
//}
