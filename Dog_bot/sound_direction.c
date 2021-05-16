#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>

#include <audio/microphone.h>
#include <sound_direction.h>
#include <arm_math.h>
#include <arm_const_structs.h>
#include <math.h>

#define PHASE_SAMPLES 				5		// mean of PHASE_SAMPLES values (amplitude) for more stability
#define MIN_VALUE_THRESHOLD			4000 	// amplitude filtering below this value
#define FFT_SIZE 					1024
#define MAX_TIME_BETWEEN_READING 	200 	// [ms]
#define MAX_PHASE_SHIFT				5		// maximum value to filter incoherent phase shift

#define SOUND_SPEED					343		// [m/s]
#define DISTANCE_M1_M2				0.0606	// [m]
#define DISTANCE_M3_M4				0.055	// [m]
#define BACK_FRONT_OFFSET			4		// [deg] axis offset from front direction

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];
//Array of amplitudes for 4 mics to filter parasitic sound
static float amps_in[4];
static uint8_t order_in[4] = {0, 1, 2, 3}; //order of mics: Right(0), Left(1), Back(2), Front(3)
//Array of amplitude values out of compare_amp for mic selection: Two medium amplitudes, and their corresponding mic number
static float mid_amps[2];
static uint8_t order_out[2];
// Array of secondary amplitudes from unused mics (Highest and lowest amplitudes). Used in angle_calcul
static float sec_amps[2];
//Phase calculation
static uint16_t counter = 0;
static float mean_phase=0;
static float mean_angle=0;
static float phase_old=0;

static float FFT_tab[8*PHASE_SAMPLES];	//raw FFT data used in phase_calcul
static float amp_tab[4*PHASE_SAMPLES];	//raw amplitude data that will later be averaged
static float right_amp_average;			//amplitude average for each mic for more stable values
static float left_amp_average;
static float back_amp_average;
static float front_amp_average;
static float amps_average[4];

static systime_t value_input_time=0;
static systime_t last_value_input_time=0;

static bool got_new_direction_flag = FALSE;

static bool new_angle_flag = FALSE;

////// PRIVATE FUNCTIONS ///////
static void processAudioData(int16_t *data, uint16_t num_samples);
static bool amp_filter(float* mic_amp_output);
static void compare_amp(float* amplitude_input);
static float phase_calcul(uint8_t mic_selection, uint8_t position);
static float angle_calcul(float phase_shift, uint8_t mic_selection);
////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// PUBLIC FUNCTIONS ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
void sound_direction_setUp(void)
{
	mic_start(&processAudioData);
}

bool get_sound_angle(float* sound_direction){

	*sound_direction = mean_angle;
	new_angle_flag?got_new_direction_flag=TRUE:0;

	return new_angle_flag;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// PRIVATE FUNCTIONS //////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

/*
*	Callback called when the demodulation of the four microphones is done.
*	Stores the right samples and computes the sound direction.
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
static void processAudioData(int16_t *data, uint16_t num_samples){

	if(got_new_direction_flag)	//reset new angle flag if the new direction was copied by dog_mode.c
	{
		new_angle_flag = 0;
	}

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4)
	{
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE))
	{
		//	FFT processing
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, micRight_cmplx_input, 0, 1);
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, micLeft_cmplx_input, 0, 1);
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, micBack_cmplx_input, 0, 1);
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, micFront_cmplx_input, 0, 1);

		//	Magnitude processing
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);

		//Only keeps the amplitudes at SCAN_FREQUENCY
		amps_in[0]= micRight_output[SCAN_FREQUENCY_NUM];
		amps_in[1]= micLeft_output[SCAN_FREQUENCY_NUM];
		amps_in[2]= micBack_output[SCAN_FREQUENCY_NUM];
		amps_in[3]= micFront_output[SCAN_FREQUENCY_NUM];

		if(amp_filter(amps_in)) // amplitude filter to eliminate low parasitic sound
		{
			systime_t timenow = ST2MS(chVTGetSystemTime());	//regulation of time between inputs: if the time is too long, it is considered parasitic and won't be taken into account.
			value_input_time = timenow-last_value_input_time;

			if(value_input_time <= MAX_TIME_BETWEEN_READING)
			{
				//Copy of values for later use (mean value for amplitude, phase calculation for FFT)
				FFT_tab[8*counter] = micRight_cmplx_input[2*SCAN_FREQUENCY_NUM];
				FFT_tab[8*counter+1] = micRight_cmplx_input[2*SCAN_FREQUENCY_NUM+1];
				FFT_tab[8*counter+2] = micLeft_cmplx_input[2*SCAN_FREQUENCY_NUM];
				FFT_tab[8*counter+3] = micLeft_cmplx_input[2*SCAN_FREQUENCY_NUM+1];
				FFT_tab[8*counter+4] = micBack_cmplx_input[2*SCAN_FREQUENCY_NUM];
				FFT_tab[8*counter+5] = micBack_cmplx_input[2*SCAN_FREQUENCY_NUM+1];
				FFT_tab[8*counter+6] = micFront_cmplx_input[2*SCAN_FREQUENCY_NUM];
				FFT_tab[8*counter+7] = micFront_cmplx_input[2*SCAN_FREQUENCY_NUM+1];

				amp_tab[4*counter] = micRight_output[SCAN_FREQUENCY_NUM];
				amp_tab[4*counter+1] = micLeft_output[SCAN_FREQUENCY_NUM];
				amp_tab[4*counter+2] = micBack_output[SCAN_FREQUENCY_NUM];
				amp_tab[4*counter+3] = micFront_output[SCAN_FREQUENCY_NUM];

				counter++;
			}
			else
			{
				counter=0;
			}
			last_value_input_time = timenow;
		}

		if(counter == PHASE_SAMPLES)
		{
			for(uint8_t a=0; a<PHASE_SAMPLES; a++)
			{
				right_amp_average += amp_tab[4*a]; //amplitude accumulation for each mic
				left_amp_average += amp_tab[4*a+1];
				back_amp_average += amp_tab[4*a+2];
				front_amp_average += amp_tab[4*a+3];
			}

			right_amp_average /= PHASE_SAMPLES;
			left_amp_average /= PHASE_SAMPLES;
			back_amp_average /= PHASE_SAMPLES;
			front_amp_average /= PHASE_SAMPLES;

			amps_average[0] = right_amp_average;
			amps_average[1] = left_amp_average;
			amps_average[2] = back_amp_average;
			amps_average[3] = front_amp_average;

			//FUNCTION: finds the two mics to analyze (with 2nd and 3rd highest amplitudes) so that we don't compute values that are close to 90° from the front
			//There is a loss of accuracy in angle calculation when the sound is coming from + or - 90° from the front of the robot
			compare_amp(amps_average);

			for(uint8_t b=0; b<PHASE_SAMPLES; b++)
			{
				mean_phase += phase_calcul(order_out[0], b);
			}

			mean_phase /= PHASE_SAMPLES;

			//Angle computation and adjustment in function of mic number
			//We want the angle from the front of the robot to know how much to turn
			mean_angle = angle_calcul(mean_phase, order_out[0]);

			//chprintf((BaseSequentialStream *)&SD3, "angle=%f\n\n", mean_angle);

			//Reset of incremental values
			counter=0;

			mean_phase=0;

			//New angle value has been computed
			new_angle_flag = TRUE;
			got_new_direction_flag = FALSE;
		}

		nb_samples = 0;
	}
}

static void compare_amp(float* amplitude_input)
{
	//for amplitude and mic order manipulation only
	float sub_var_amp =0;			//substitution variable for storing amplitude values
	uint8_t sub_var_order =0;		//substitution variable for storing order values
	float list_amps[4];				//array containing final sorted amplitudes

	//copy so that there is no manipulation directly on the input values
	for(uint8_t k=0; k<=3; ++k)
	{
		list_amps[k]=amplitude_input[k];
	}

	//Classification from highest to lowest amplitude (with corresponding mic number to identify)
	for(uint8_t j=0; j<=3; ++j) {
		for(uint8_t i=0; i<=3; ++i) {
			if(i!=j){
				if(list_amps[i]<list_amps[j]){
					sub_var_amp=list_amps[i];
					list_amps[i]=list_amps[j];
					list_amps[j]=sub_var_amp;
					sub_var_order=order_in[i];
					order_in[i]=order_in[j];
					order_in[j]=sub_var_order;
				}
			}
		}
	}

	if(order_in[0]==0 || order_in[0]==1)
	{
		// if RIGHT or LEFT mics have the highest and lowest amplitudes, FRONT and BACK pair is selected.
		// a double if condition is necessary because FRONT and BACK do not have the same area of contact with sound as LEFT and RIGHT
		//LEFT and RIGHT both have the same area of contact so they are considered more reliable, therefore the default choice if the amplitudes from FRONT and BACK are inconsistent.
		if(order_in[3]==0 || order_in[3]==1)
		{
			order_out[0] = 2;
			mid_amps[0] = amplitude_input[2]; // Medium amplitudes corresponding to the selected pair
			order_out[1] = 3;
			mid_amps[1] = amplitude_input[3];
			sec_amps[0] = amplitude_input[0]; //Highest and lowest amplitudes (unused pair)
			sec_amps[1] = amplitude_input[1];
		} else // if the double condition is not fulfilled, LEFT and RIGHT mics are selected
		{
			order_out[0] = 0;
			mid_amps[0] = amplitude_input[0];
			order_out[1] = 1;
			mid_amps[1] = amplitude_input[1];
			sec_amps[0] = amplitude_input[2];
			sec_amps[1] = amplitude_input[3];
		}
	}

	// LEFT and RIGHT mics are selected
	else if((order_in[0]==2 || order_in[0]==3)&&(order_in[3]==2 || order_in[3]==3)) //LEFT - RIGHT
	{
		order_out[0] = 0; 		//The medium amplitudes correspond to the selected mic pair
		mid_amps[0] = amplitude_input[0];
		order_out[1] = 1;
		mid_amps[1] = amplitude_input[1];
		sec_amps[0] = amplitude_input[2];	//The mic pair corresponding to the highest and lowest amplitudes will be used for the choice of formula for angle_calcul
		sec_amps[1] = amplitude_input[3];
	}

	order_in[0] = 0;		//reset of order_in values for next call to compare_amps
	order_in[1] = 1;
	order_in[2] = 2;
	order_in[3] = 3;
}

static float phase_calcul(uint8_t mic_selection, uint8_t position)
{
	float phase1;
	float phase2;
	float FFT_re1;
	float FFT_im1;
	float FFT_re2;
	float FFT_im2;

	if(mic_selection==0){	// RIGHT - LEFT
		FFT_re1 = FFT_tab[8*position];		//Right
		FFT_im1 = FFT_tab[8*position+1];
		FFT_re2 = FFT_tab[8*position+2];	//Left
		FFT_im2 = FFT_tab[8*position+3];
	} else {	  // BACK - FRONT
		FFT_re1 = FFT_tab[8*position+4];	//Back
		FFT_im1 = FFT_tab[8*position+5];
		FFT_re2 = FFT_tab[8*position+6];	//Front
		FFT_im2 = FFT_tab[8*position+7];
	}

	phase1 = atan2f(FFT_im1,FFT_re1);
	phase2 = atan2f(FFT_im2,FFT_re2);

	float phase_shift = phase1 - phase2; // [radians]

	if((phase_shift > MAX_PHASE_SHIFT) || (phase_shift < -MAX_PHASE_SHIFT)){ // replace improbable phase values by previous phase value
		phase_shift = phase_old;
	} else {
		phase_old = phase_shift;
	}

	return phase_shift;
}

static bool amp_filter(float* mic_amp_output)
{
	if(mic_amp_output[0] < MIN_VALUE_THRESHOLD || mic_amp_output[1] < MIN_VALUE_THRESHOLD || mic_amp_output[2] < MIN_VALUE_THRESHOLD || mic_amp_output[3] < MIN_VALUE_THRESHOLD) {
		//values are considered parasitic and not taken into account
		return FALSE;
	}
	return TRUE;
}

static float angle_calcul(float phase_shift, uint8_t mic_selection)
{
	float angle=0;

	//RIGHT - LEFT
	if(mic_selection==0)
	{
		//sinus of the angle in radians
		float conditionLR = phase_shift*(SOUND_SPEED/SCAN_FREQUENCY)/(2*M_PI*DISTANCE_M1_M2);

		if(conditionLR < -1) //sets all improbable sinus values to 1 or -1
		{
			conditionLR = -1;
		} 
		else if(conditionLR > 1)
		{
			conditionLR = 1;
		}

		angle = 180*asinf(conditionLR)/M_PI;

		if(sec_amps[0]>=sec_amps[1])		// BACK > FRONT
		{
			if(angle > 0)		// RIGHT > LEFT
			{
				angle = 180 - angle;
			}
			else // LEFT > RIGHT
			{
				angle = -180 - angle;
			}
		}
	}
	// BACK - FRONT
	else if(mic_selection==2)
	{
		//sinus of the angle in radians
		float conditionFB = phase_shift*(SOUND_SPEED/SCAN_FREQUENCY)/(2*M_PI*DISTANCE_M3_M4);

		if(conditionFB < -1)	//sets all improbable sinus values to 1 or -1
		{
			conditionFB = -1;
		} 
		else if(conditionFB>1)
		{
			conditionFB = 1;
		}

		angle = 180*asinf(conditionFB)/M_PI;

		if(sec_amps[0]>=sec_amps[1]) // RIGHT > LEFT
		{
			angle = 90 + BACK_FRONT_OFFSET + angle;
		} 
		else		// LEFT > RIGHT
		{
			angle = -90 + BACK_FRONT_OFFSET - angle;
		}
	}

	return angle;
}
