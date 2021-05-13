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
#define MIN_VALUE_THRESHOLD			8000 	// amplitude filtering below this value
#define SCAN_FREQUENCY_NUM			32		// 500Hz
#define SCAN_FREQUENCY				(SCAN_FREQUENCY_NUM * 15.625)
#define FFT_SIZE 					1024
#define MAX_TIME_BETWEEN_READING 	200 	// [ms]

#define SOUND_SPEED					343		// [m/s]
#define DISTANCE_M1_M2				0.0606	// [m]
#define DISTANCE_M3_M4				0.055	// [m]

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
static int order_in[4] = {0, 1, 2, 3}; //order of mics: Right(0), Left(1), Back(2), Front(3)
//Array of amplitude values out of compare_amp for mic selection: Two medium amplitudes, and their corresponding mic number
static float mid_amps[2];
static int order_out[2];
// Array of secondary amplitudes from unused mics (Highest and lowest amplitudes). Used in angle_calcul
static float sec_amps[2];
//Phase calculation
static uint16_t compteur = 0;
static float phase_moyenne=0;
static float angle_moyenne=0;
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

static int new_angle_flag = 0;

////// PRIVATE FUNCTIONS ///////
static void processAudioData(int16_t *data, uint16_t num_samples);
static void filtre_amp(float* mic_amp_output);
static void compare_amp(float* amplitude_input);
static float phase_calcul(uint8_t micro_selection, uint8_t position);
static float angle_calcul(float dephasage, uint8_t micro_selection);
////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// PUBLIC FUNCTIONS ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
void sound_direction_setUp(void)
{
	mic_start(&processAudioData);
}

int get_sound_angle(float* sound_direction){

	*sound_direction = angle_moyenne;
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
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
static void processAudioData(int16_t *data, uint16_t num_samples){

	if(got_new_direction_flag)	//starts a new angle measure when an old one has been acquired
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
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
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

		// amplitude filter to eliminate low parasitic sound
		filtre_amp(amps_in);

		if(amps_in[0] !=0)
		{
			systime_t timenow = ST2MS(chVTGetSystemTime());	//regulation of time between inputs: if the time is too long, it is considered parasitic and won't be taken into account.
			value_input_time = timenow-last_value_input_time;

			if(value_input_time <= MAX_TIME_BETWEEN_READING)
			{
				//Copy of values for later use (mean value for amplitude, phase calculation for FFT)
				FFT_tab[8*compteur]=micRight_cmplx_input[2*SCAN_FREQUENCY_NUM];
				FFT_tab[8*compteur+1]=micRight_cmplx_input[2*SCAN_FREQUENCY_NUM+1];
				FFT_tab[8*compteur+2]=micLeft_cmplx_input[2*SCAN_FREQUENCY_NUM];
				FFT_tab[8*compteur+3]=micLeft_cmplx_input[2*SCAN_FREQUENCY_NUM+1];
				FFT_tab[8*compteur+4]=micBack_cmplx_input[2*SCAN_FREQUENCY_NUM];
				FFT_tab[8*compteur+5]=micBack_cmplx_input[2*SCAN_FREQUENCY_NUM+1];
				FFT_tab[8*compteur+6]=micFront_cmplx_input[2*SCAN_FREQUENCY_NUM];
				FFT_tab[8*compteur+7]=micFront_cmplx_input[2*SCAN_FREQUENCY_NUM+1];

				amp_tab[4*compteur] = micRight_output[SCAN_FREQUENCY_NUM];
				amp_tab[4*compteur+1] = micLeft_output[SCAN_FREQUENCY_NUM];
				amp_tab[4*compteur+2] = micBack_output[SCAN_FREQUENCY_NUM];
				amp_tab[4*compteur+3] = micFront_output[SCAN_FREQUENCY_NUM];

				compteur++;
			}
			else
			{
				compteur=0;
			}
			last_value_input_time = timenow;
		}

		if(compteur == PHASE_SAMPLES)
		{
			for(int a=0; a<PHASE_SAMPLES; a++){

				right_amp_average+=amp_tab[4*a]; //amplitude accumulation for each mic
				left_amp_average+=amp_tab[4*a+1];
				back_amp_average+=amp_tab[4*a+2];
				front_amp_average+=amp_tab[4*a+3];

			}

			right_amp_average/=PHASE_SAMPLES;
			left_amp_average/=PHASE_SAMPLES;
			back_amp_average/=PHASE_SAMPLES;
			front_amp_average/=PHASE_SAMPLES;

			amps_average[0]= right_amp_average;
			amps_average[1]= left_amp_average;
			amps_average[2]= back_amp_average;
			amps_average[3]= front_amp_average;

			//FUNCTION: finds the two mics to analyze (with 2nd and 3rd highest amplitudes) so that we don't compute values that are close to 90° from the front
			//There is a loss of accuracy in angle calculation when the sound is coming from + or - 90° from the front of the robot
			compare_amp(amps_average);

			for(uint8_t b=0; b<PHASE_SAMPLES; b++){

				phase_moyenne += phase_calcul(order_out[0], b);

			}

			phase_moyenne /= PHASE_SAMPLES;

			//Angle computation and adjustment in function of mic number
			//We want the angle from the front of the robot to know how much to turn
			angle_moyenne = angle_calcul(phase_moyenne, order_out[0]);

			chprintf((BaseSequentialStream *)&SD3, "angle=%f\n\n", angle_moyenne);

			//Reset of incremental values
			compteur=0;

			phase_moyenne=0;

			//Preparation to send new angle value
			new_angle_flag = 1;

			got_new_direction_flag = FALSE;

		}

		nb_samples = 0;

	}
}

static void compare_amp(float* amplitude_input){

	//for amplitude and mic order manipulation only
	static float a=0;
	static int b=0;
	static float list_amps[4];
	static int list_order[4];

	//copy so that there is no manipulation directly on the input values
	for(uint8_t k=0; k<=3; ++k){

		list_amps[k]=amplitude_input[k];
		list_order[k]=order_in[k];

	}

	//Classification from highest to lowest amplitude (with corresponding mic number to identify)
	for(uint8_t j=0; j<=3; ++j) {
		for(uint8_t i=0; i<=3; ++i) {
			if(i!=j){
				if(list_amps[i]<list_amps[j]) {
					a=list_amps[i];
					list_amps[i]=list_amps[j];
					list_amps[j]=a;
					b=list_order[i];
					list_order[i]=list_order[j];
					list_order[j]=b;
				}
			}
		}
	}

	if(list_order[0]==0 || list_order[0]==1){
		// if RIGHT or LEFT mics have the highest and lowest amplitudes, FRONT and BACK pair is selected.
		// a double if condition is necessary because FRONT and BACK do not have the same area of contact with sound as LEFT and RIGHT
		//LEFT and RIGHT both have the same area of contact so they are considered more reliable, therefore the default choice if the amplitudes from FRONT and BACK are inconsistent.
		if(list_order[3]==0 || list_order[3]==1){

			order_out[0]=order_in[2];
			mid_amps[0]=amplitude_input[2]; // Medium amplitudes corresponding to the selected pair
			order_out[1]=order_in[3];
			mid_amps[1]=amplitude_input[3];
			sec_amps[0] = amplitude_input[0]; //Highest and lowest amplitudes (unused pair)
			sec_amps[1] = amplitude_input[1];

		} else {		// if the double condition is not fulfilled, LEFT and RIGHT mics are selected

			order_out[0]=order_in[0];
			mid_amps[0]=amplitude_input[0];
			order_out[1]=order_in[1];
			mid_amps[1]=amplitude_input[1];
			sec_amps[0] = amplitude_input[2];
			sec_amps[1] = amplitude_input[3];

		}
	}

	// LEFT and RIGHT mics are selected
	else if((list_order[0]==2 || list_order[0]==3)&&(list_order[3]==2 || list_order[3]==3)){	//LEFT - RIGHT
		order_out[0]=order_in[0]; 		//on retient les amplitudes intermédiaires
		mid_amps[0]=amplitude_input[0];
		order_out[1]=order_in[1];
		mid_amps[1]=amplitude_input[1];
		sec_amps[0] = amplitude_input[2];
		sec_amps[1] = amplitude_input[3];
	}
}


static float phase_calcul(uint8_t micro_selection, uint8_t position) {
	float phase1;
	float phase2;
	float FFT_re1;
	float FFT_im1;
	float FFT_re2;
	float FFT_im2;

	if(micro_selection==0){	// RIGHT - LEFT
		FFT_re1= FFT_tab[8*position];		//Right
		FFT_im1= FFT_tab[8*position+1];
		FFT_re2= FFT_tab[8*position+2];	//Left
		FFT_im2= FFT_tab[8*position+3];
	} else {	  // BACK - FRONT
		FFT_re1= FFT_tab[8*position+4];	//Back
		FFT_im1= FFT_tab[8*position+5];
		FFT_re2=	FFT_tab[8*position+6];	//Front
		FFT_im2= FFT_tab[8*position+7];
	}

	phase1=atan2f(FFT_im1,FFT_re1);
	phase2=atan2f(FFT_im2,FFT_re2);

	float dephasage=phase1-phase2; // [radians]

	if((dephasage > 5) || (dephasage < -5)){ // replace improbable phase values by previous phase value
		dephasage = phase_old;
	} else {
		phase_old = dephasage;
	}

	return dephasage;
}

static void filtre_amp(float* mic_amp_output)
{
	if(mic_amp_output[0] < MIN_VALUE_THRESHOLD || mic_amp_output[1] < MIN_VALUE_THRESHOLD) {
		mic_amp_output[0] = 0;	//values are considered parasitic and not taken into account
		mic_amp_output[1] = 0;
	}
}

static float angle_calcul(float dephasage, uint8_t micro_selection) {

	float angle=0;

	//sinus of the angle in radians
	float conditionLR=dephasage*(SOUND_SPEED/SCAN_FREQUENCY)/(2*M_PI*DISTANCE_M1_M2);	//The distance between each pair is not equal (the microphones are not symmetrical)
	float conditionFB=dephasage*(SOUND_SPEED/SCAN_FREQUENCY)/(2*M_PI*DISTANCE_M3_M4);

	//sets all improbable sinus values to 1 or -1
	if(conditionLR < -1){

		conditionLR = -1;

	} else if(conditionLR>1){

		conditionLR = 1;

	}

	if(conditionFB < -1){

		conditionFB = -1;

	} else if(conditionFB>1){

		conditionFB = 1;

	}

	//RIGHT - BACK
	if(micro_selection==0){

		angle = 180*asinf(conditionLR)/M_PI;

		if(sec_amps[0]>=sec_amps[1]){		// BACK > FRONT

			if(mid_amps[0]>=mid_amps[1]){		// RIGHT > LEFT

				angle = 180 - angle;

			} else { // LEFT > RIGHT

				angle = -180 - angle;

			}

		}

	// BACK - FRONT
	} else if(micro_selection==2) {

		angle = 180*asinf(conditionFB)/M_PI;

		if(sec_amps[0]>=sec_amps[1]){ // RIGHT > LEFT

			angle = 90 + 4 + angle;

		} else {		// LEFT > RIGHT

			angle = -90 + 4 - angle;

		}

	}


	if(angle>180 || angle<-180){

		angle=180;

	}

	return angle;
}
