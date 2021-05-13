#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>

#include <audio/microphone.h>
#include <sound_direction.h>
#include <fft.h>
#include <arm_math.h>
#include <math.h>

#define PHASE_SAMPLES 	5

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
//Array of amplitude values out of compare_amp: Two highest amplitude
static float mid_amps[2];
static int order_out[2]; // which are the mics with the highest amplitude?
// Array of secondary amplitudes from unused mics
static float sec_amps[2];
//Array of amplitude values into compare_amp
static float amps_in[4];
static int order_in[4] = {0, 1, 2, 3}; //order of mics: Right(0), Left(1), Back(2), Front(3)
//Calcul d'phase
float fsig1=0; //fréquence du signal 1
float fsig2=1; // fréquence du signal 2
static uint16_t compteur = 0;
float phase_final;
static float angle_inter=0;
static float phase_moyenne=0;
static float angle_moyenne=0;
static float phase_old=0;

static float phase_tab[PHASE_SAMPLES];
static int phase_side_tab[PHASE_SAMPLES];
static int decided_side;
static int decided_side_flag = FALSE;
static float FFT_tab[8*PHASE_SAMPLES];
static float amp_tab[4*PHASE_SAMPLES];
static float right_amp_average;
static float left_amp_average;
static float back_amp_average;
static float front_amp_average;
static float amps_average[4];

static systime_t value_input_time=0;
static systime_t last_value_input_time=0;

static bool got_new_direction_flag = FALSE;

static int new_angle_flag = 0;

#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define MAX_FREQ		32	//J'ai mis 32 pour avoir 500Hz tout rond pour les tests //453Hz: we don't analyze after this index to not use resources for nothing

void sound_direction_setUp(void)
{
	mic_start(&processAudioData);
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	if(got_new_direction_flag)
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
	//static uint8_t mustSend = 0;
	static uint8_t o;

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
		/*	FFT processing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function.
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);

		amps_in[0]= micRight_output[MAX_FREQ];
		amps_in[1]= micLeft_output[MAX_FREQ];
		amps_in[2]= micBack_output[MAX_FREQ];
		amps_in[3]= micFront_output[MAX_FREQ];

		filtre_amp(amps_in); // filtrage en amplitude pour eliminer le son parasite

		if(amps_in[0] !=0)
		{
			systime_t timenow = ST2MS(chVTGetSystemTime());
			value_input_time = timenow-last_value_input_time;

			//chprintf((BaseSequentialStream *)&SD3, "value_input_time=%d\n", value_input_time);

			if(value_input_time <= 500)
			{
				FFT_tab[8*compteur]=micRight_cmplx_input[2*MAX_FREQ];	//enregistrement dans un array les valeurs des FFT d'un son
				FFT_tab[8*compteur+1]=micRight_cmplx_input[2*MAX_FREQ+1];
				FFT_tab[8*compteur+2]=micLeft_cmplx_input[2*MAX_FREQ];
				FFT_tab[8*compteur+3]=micLeft_cmplx_input[2*MAX_FREQ+1];
				FFT_tab[8*compteur+4]=micBack_cmplx_input[2*MAX_FREQ];
				FFT_tab[8*compteur+5]=micBack_cmplx_input[2*MAX_FREQ+1];
				FFT_tab[8*compteur+6]=micFront_cmplx_input[2*MAX_FREQ];
				FFT_tab[8*compteur+7]=micFront_cmplx_input[2*MAX_FREQ+1];

				amp_tab[4*compteur] = micRight_output[MAX_FREQ];
				amp_tab[4*compteur+1] = micLeft_output[MAX_FREQ];
				amp_tab[4*compteur+2] = micBack_output[MAX_FREQ];
				amp_tab[4*compteur+3] = micFront_output[MAX_FREQ];

				compteur++;
			}
			else
			{
				compteur=0;
			}
			last_value_input_time = timenow;

			//chprintf((BaseSequentialStream *)&SD3, "accumulation en cours=%f\n", angle_moyenne);
			//chprintf((BaseSequentialStream *)&SD3, "Front Mic amplitude=%f\n", sec_amps[1]);
			//chprintf((BaseSequentialStream *)&SD3, "Back Mic amplitude=%f\n\n", sec_amps[0]);
			//chprintf((BaseSequentialStream *)&SD3, "compteur=%d\n", compteur);
		}

		if(compteur == PHASE_SAMPLES)
		{
			for(int a=0; a<PHASE_SAMPLES; a++){
				right_amp_average+=amp_tab[4*a]; //accumulation de l'amplitude sur chaque micro
				left_amp_average+=amp_tab[4*a+1];
				back_amp_average+=amp_tab[4*a+2];
				front_amp_average+=amp_tab[4*a+3];
			}
			right_amp_average/=PHASE_SAMPLES;	//moyenne des amplitudes de chaque micro
			//chprintf((BaseSequentialStream *)&SD3, "right_amp_init=%f\n", right_amp_average);
			left_amp_average/=PHASE_SAMPLES;
			//chprintf((BaseSequentialStream *)&SD3, "left_amp_init=%f\n", left_amp_average);
			back_amp_average/=PHASE_SAMPLES;
			//chprintf((BaseSequentialStream *)&SD3, "back_amp_init=%f\n", back_amp_average);
			front_amp_average/=PHASE_SAMPLES;
			//chprintf((BaseSequentialStream *)&SD3, "front_amp_init=%f\n\n", front_amp_average);

			amps_average[0]= right_amp_average;
			amps_average[1]= left_amp_average;
			amps_average[2]= back_amp_average;
			amps_average[3]= front_amp_average;

			/*
			chprintf((BaseSequentialStream *)&SD3, "----------INPUT 1---------\n");

			chprintf((BaseSequentialStream *)&SD3, "order in right =%d\n", order_in[0]);
			chprintf((BaseSequentialStream *)&SD3, "input right =%f\n", amps_average[0]);
			chprintf((BaseSequentialStream *)&SD3, "order in left =%d\n", order_in[1]);
			chprintf((BaseSequentialStream *)&SD3, "input left =%f\n", amps_average[1]);
			chprintf((BaseSequentialStream *)&SD3, "order in back =%d\n", order_in[2]);
			chprintf((BaseSequentialStream *)&SD3, "input back =%f\n", amps_average[2]);
			chprintf((BaseSequentialStream *)&SD3, "order in front =%d\n", order_in[3]);
			chprintf((BaseSequentialStream *)&SD3, "input front =%f\n\n", amps_average[3]);
			*/

			compare_amp(amps_average); // donne la bonne paire de micro

			for(uint8_t b=0; b<PHASE_SAMPLES; b++){
				phase_moyenne += phase_calcul(order_out[0], b); //donne le numéro du micro sélectionné
			}
			phase_moyenne /= PHASE_SAMPLES;
			chprintf((BaseSequentialStream *)&SD3, "phase moyenne=%f\n", phase_moyenne);
			angle_moyenne = angle_calcul(phase_moyenne, order_out[0]);
			chprintf((BaseSequentialStream *)&SD3, "angle=%f\n\n", angle_moyenne);
			compteur=0;
			phase_moyenne=0; //enlever quand on utilise le return
			new_angle_flag = 1;
			got_new_direction_flag = FALSE;
		}
		/*
		if(high_amps[0] !=0){
			phase_final = phase_calcul(MAX_FREQ); // calcul de phase
			chprintf((BaseSequentialStream *)&SD3, "phase_final=%f\n", phase_final);
			phase_moyenne = phase_final+phase_moyenne;	//!!!!!!!!!!!!!!!!!!!!!!!ATTENTION!!!!!!!!!!!!!!!!!!!!!! Pour les tests, angle_moyenne = phase moyenne
			//chprintf((BaseSequentialStream *)&SD3, "accumulation en cours=%f\n", angle_moyenne);
			compteur++;
			//chprintf((BaseSequentialStream *)&SD3, "Front Mic amplitude=%f\n", sec_amps[1]);
			//chprintf((BaseSequentialStream *)&SD3, "Back Mic amplitude=%f\n\n", sec_amps[0]);
			//chprintf((BaseSequentialStream *)&SD3, "compteur=%d\n", compteur);
		}
		if((compteur == 5)){	//envoyer angle
			phase_moyenne = phase_moyenne/5;
			chprintf((BaseSequentialStream *)&SD3, "phase moyenne=%f\n", phase_moyenne); //!!!!!!!!!!!!!!!!!!!!!!!ATTENTION!!!!!!!!!!!!!!!!!!!!!! Pour les tests, angle_moyenne = phase moyenne
			angle_moyenne = angle_calcul(phase_moyenne);
			chprintf((BaseSequentialStream *)&SD3, "angle=%f\n", angle_moyenne);
			compteur=0;
			phase_moyenne=0; //enlever quand on utilise le return
			new_angle_flag = 1;
			got_new_direction_flag = FALSE;
		}
		*/
		nb_samples = 0;
	}
}

void compare_amp(float* amplitude_input){
	static float a=0;
	static int b=0;
	static int c;

	static float list_amps[4]; //for amps manipulation only
	static int list_order[4];

	for(uint8_t k=0; k<=3; ++k){
		list_amps[k]=amplitude_input[k];
		list_order[k]=order_in[k];
		//chprintf((BaseSequentialStream *)&SD3, "order_in =%d\n", order_in[k]);
		//chprintf((BaseSequentialStream *)&SD3, "list_order =%d\n", list_order[k]);
	}

	chprintf((BaseSequentialStream *)&SD3, "\n");
	chprintf((BaseSequentialStream *)&SD3, "----------INPUT 2---------\n");

	chprintf((BaseSequentialStream *)&SD3, "order in right =%d\n", list_order[0]);
	chprintf((BaseSequentialStream *)&SD3, "input right =%f\n", list_amps[0]);
	chprintf((BaseSequentialStream *)&SD3, "order in left =%d\n",list_order[1]);
	chprintf((BaseSequentialStream *)&SD3, "input left =%f\n", list_amps[1]);
	chprintf((BaseSequentialStream *)&SD3, "order in back =%d\n", list_order[2]);
	chprintf((BaseSequentialStream *)&SD3, "input back =%f\n", list_amps[2]);
	chprintf((BaseSequentialStream *)&SD3, "order in front =%d\n", list_order[3]);
	chprintf((BaseSequentialStream *)&SD3, "input front =%f\n\n", list_amps[3]);

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

	chprintf((BaseSequentialStream *)&SD3, "----------OUTPUT---------\n");
	if(list_order[0]==0 || list_order[0]==1){ 	//FRONT - BACK			On elimine les amplitudes les plus fortes
		if(list_order[3]==0 || list_order[3]==1){
		order_out[0]=order_in[2]; 		//on retient les amplitudes intermédiaires
		mid_amps[0]=amplitude_input[2];
		order_out[1]=order_in[3];
		mid_amps[1]=amplitude_input[3];
		sec_amps[0] = amplitude_input[0]; //pour le calcul d'angle
		sec_amps[1] = amplitude_input[1];
		chprintf((BaseSequentialStream *)&SD3, "FRONT-BACK\n");
		} else {
			order_out[0]=order_in[0]; 		//on retient les amplitudes intermédiaires
			mid_amps[0]=amplitude_input[0];
			order_out[1]=order_in[1];
			mid_amps[1]=amplitude_input[1];
			sec_amps[0] = amplitude_input[2];
			sec_amps[1] = amplitude_input[3];
			chprintf((BaseSequentialStream *)&SD3, "RIGHT-LEFT\n");
		}
	}

	else if((list_order[0]==2 || list_order[0]==3)&&(list_order[3]==2 || list_order[3]==3)){	//LEFT - RIGHT
		order_out[0]=order_in[0]; 		//on retient les amplitudes intermédiaires
		mid_amps[0]=amplitude_input[0];
		order_out[1]=order_in[1];
		mid_amps[1]=amplitude_input[1];
		sec_amps[0] = amplitude_input[2];
		sec_amps[1] = amplitude_input[3];
		chprintf((BaseSequentialStream *)&SD3, "RIGHT-LEFT\n");

	chprintf((BaseSequentialStream *)&SD3, "order 0=%d\n", order_out[0]);
	chprintf((BaseSequentialStream *)&SD3, "mid amps 0=%f\n", mid_amps[0]);
	chprintf((BaseSequentialStream *)&SD3, "order 1=%d\n", order_out[1]);
	chprintf((BaseSequentialStream *)&SD3, "mid amps 1=%f\n", mid_amps[1]);
	chprintf((BaseSequentialStream *)&SD3, "secondary amps 0=%f\n", sec_amps[0]);
	chprintf((BaseSequentialStream *)&SD3, "secondary amps 1=%f\n\n", sec_amps[1]);
	}
	/*
	chprintf((BaseSequentialStream *)&SD3, "order 0=%d\n", order_out[0]); //unsigned integer
	chprintf((BaseSequentialStream *)&SD3, "amplitude 0=%f\n", mid_amps[0]);
	chprintf((BaseSequentialStream *)&SD3, "order 1=%d\n", order_out[1]); //unsigned integer
	chprintf((BaseSequentialStream *)&SD3, "amplitude 1=%f\n", mid_amps[1]);
	*/

}

float phase_calcul(uint8_t micro_selection, uint8_t position) {
	float phase1;
	float phase2;
	float FFT_re1;
	float FFT_im1;
	float FFT_re2;
	float FFT_im2;

	if(micro_selection==0){//Right - Left
		FFT_re1= FFT_tab[8*position];		//Right
		FFT_im1= FFT_tab[8*position+1];
		FFT_re2= FFT_tab[8*position+2];	//Left
		FFT_im2= FFT_tab[8*position+3];
	} else {	//Back - Front
		FFT_re1= FFT_tab[8*position+4];	//Back
		FFT_im1= FFT_tab[8*position+5];
		FFT_re2=	FFT_tab[8*position+6];	//Front
		FFT_im2= FFT_tab[8*position+7];
	}

	phase1=atan2f(FFT_im1,FFT_re1);
	phase2=atan2f(FFT_im2,FFT_re2);

	float dephasage=phase1-phase2; // en radians

	if((dephasage > 5) || (dephasage < -5)){ // eliminer les phases improbables //J'ai remplacé 0,4 par 5 car il y a seulement des pics au dessus de 5 qui apparaissent, et la pahse peut aller jusqu'a plus de 1 (mais toujours moins de 5) selon les fréquences
		dephasage = phase_old;
	} else {
		phase_old = dephasage;
	}

	return dephasage;
}

float passe_bande(uint8_t position, float mic_amp_output) {
	float mic_amp;
	//chprintf((BaseSequentialStream *)&SD3, "position=%d\n", position);
	//chprintf((BaseSequentialStream *)&SD3, "amplitude output=%f\n", mic_amp_output);

	if((position < MIN_FREQ) || (position > MAX_FREQ)){
		mic_amp = 0;
		//chprintf((BaseSequentialStream *)&SD3, "amplitude modified to 0\n\n");
		return mic_amp;  // on supprime les valeurs inutiles a l'analyse
	}
	mic_amp = mic_amp_output;
	return mic_amp;
}

void filtre_amp(float* mic_amp_output)
{
	if(mic_amp_output[0] < 8000 || mic_amp_output[1] < 8000) { //semble être suffisant
		mic_amp_output[0] = 0;
		mic_amp_output[1] = 0;
	}
}

float angle_calcul(float dephasage, uint8_t micro_selection) { // angle en degrés
	float angle;
	float conditionLR=dephasage*(346./500.)/(2*M_PI*0.0606);
	float conditionFB=dephasage*(346./500.)/(2*M_PI*0.055);
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

	if(micro_selection==0){ //Right - Left
			//chprintf((BaseSequentialStream *)&SD3, "condition=%f\n", conditionLR);
			angle = 180*asinf(conditionLR)/M_PI;
			//chprintf((BaseSequentialStream *)&SD3, "angle=%f\n", angle);
			if(sec_amps[0]>=sec_amps[1]){ //Back > Front
				if(mid_amps[0]>=mid_amps[1]){	//Right > Left
					angle = 180 - angle;
				} else { //left > Right
					angle = -180 - angle;
				}
			}
		} else if(micro_selection==2) { // Back - Front
			//chprintf((BaseSequentialStream *)&SD3, "condition=%f\n", conditionFB);
			angle = 180*asinf(conditionFB)/M_PI;
			//chprintf((BaseSequentialStream *)&SD3, "angle=%f\n", angle);
			if(sec_amps[0]>=sec_amps[1]){ // Right > Left
				angle = 90 + 4 + angle;
			} else {
				angle = -90 + 4 - angle;
			}
		}

	if(angle>180 || angle<-180){
		angle=180;
	}
	return angle;
}

int get_sound_angle(float* sound_direction){
	*sound_direction = angle_moyenne;
	new_angle_flag?got_new_direction_flag=TRUE:0;
	return new_angle_flag;
}
