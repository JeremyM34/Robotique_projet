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
static float high_amps[2];
static int order_out[2]; // which are the mics with the highest amplitude?
//Array of amplitude values into compare_amp
static float amps_in[2];
static int order_in[2]; //order of mics: Right(0), Left(1), Back(2), Front(3)
//Calcul d'phase
float fsig1=0; //fréquence du signal 1
float fsig2=1; // fréquence du signal 2
static uint16_t compteur = 0;
float phase_final;
static float angle_inter=0;
static float phase_moyenne=0;
static float angle_moyenne=0;
static float phase_old=0;

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
		//micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		//micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		//micBack_cmplx_input[nb_samples] = 0;
		//micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT processing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function.
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		//doFFT_optimized(FFT_SIZE, micBack_cmplx_input);
		//doFFT_optimized(FFT_SIZE, micFront_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		//arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);
		//arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);

		//for(uint16_t j=MAX_FREQ; j<=MAX_FREQ; ++j){ //faire moyenne des phases enregistrés (20) puis les envoyés au moteur	//La résolution en fréquence est déjà de 15,625Hz, on perdrait donc en précision en analysant une bande de fréquence plus large.
				//micRight_output[j] = passe_bande(j, micRight_output[j]); // filtrage en fréquence //N'est plus nécessaire car 1 seule fréquence est regardée
				//micLeft_output[j] = passe_bande(j, micLeft_output[j]);	//car symetrie, on prend seulement les positions positives
				//micBack_output[j] = passe_bande(j, micBack_output[j]);
				//micFront_output[j] = passe_bande(j, micFront_output[j]);

				amps_in[0]= micRight_output[MAX_FREQ];
				amps_in[1]= micLeft_output[MAX_FREQ];
				//amps_in[0]= micBack_output[j];
				//amps_in[1]= micFront_output[j];

				order_in[0]=0;  //probleme 0 et 1?
				order_in[1]=1;

				compare_amp(amps_in, order_in); // OUTPUT: 2 highest amplitudes with corresponding mic number

				filtre_amp(high_amps); // filtrage en amplitude pour eliminer le son parasite

				/*
				if(high_amps[0] !=0){
					phase_final = phase_calcul(MAX_FREQ);
					chprintf((BaseSequentialStream *)&SD3, "phase_final=%f\n", phase_final);
				}
				*/

				if(high_amps[0] !=0){
					phase_final = phase_calcul(MAX_FREQ); // calcul de phase
					phase_moyenne = phase_final+phase_moyenne;	//!!!!!!!!!!!!!!!!!!!!!!!ATTENTION!!!!!!!!!!!!!!!!!!!!!! Pour les tests, angle_moyenne = phase moyenne
					//chprintf((BaseSequentialStream *)&SD3, "accumulation en cours=%f\n", angle_moyenne);
					compteur++;
					//chprintf((BaseSequentialStream *)&SD3, "compteur=%d\n", compteur);
				}

				if((compteur == 20)){	//envoyer angle
					phase_moyenne = phase_moyenne/20;
					chprintf((BaseSequentialStream *)&SD3, "phase moyenne=%f\n", phase_moyenne); //!!!!!!!!!!!!!!!!!!!!!!!ATTENTION!!!!!!!!!!!!!!!!!!!!!! Pour les tests, angle_moyenne = phase moyenne
					angle_moyenne = angle_calcul(phase_moyenne);
					compteur=0;
					phase_moyenne=0; //enlever quand on utilise le return
					new_angle_flag = 1;

				/*
				if((j >= MIN_FREQ) && (j <= MAX_FREQ)){ //**
					for(o=0; o<=1;++o){
						if(high_amps[0] != 0){
							chprintf((BaseSequentialStream *)&SD3, "order=%d\n", order_out[o]); //unsigned integer
							chprintf((BaseSequentialStream *)&SD3, "amplitude=%f\n", high_amps[o]);
							//chprintf((BaseSequentialStream *)&SD3, "\n");
						}
					}
					if(high_amps[0] != 0){
						//phase_final = phase_final*360/(2*M_PI);
						chprintf((BaseSequentialStream *)&SD3, "angle inter=%f\n", angle_inter);
						//chprintf((BaseSequentialStream *)&SD3, "angle moyen=%f\n", angle_moyenne);
						//chprintf((BaseSequentialStream *)&SD3, "phase=%f\n", phase_final);
						chprintf((BaseSequentialStream *)&SD3, "\n");
					}
				}
			//}
			 */
		}
		nb_samples = 0;
	}
}

void compare_amp(float* amplitude_input, int* order_input){
	static float a;
	static int b;
	static int c;
	a=0;
	b=0;
/*
	//Affichage pour debuggage
	if(order_input[0]!=0){
	chprintf((BaseSequentialStream *)&SD3, "Before\n");
	for(c=0;c<=1;++c){
	chprintf((BaseSequentialStream *)&SD3, "order=%d\n", order_input[c]); //unsigned integer
	chprintf((BaseSequentialStream *)&SD3, "amplitude=%f\n", amplitude_input[c]);
	}
	chprintf((BaseSequentialStream *)&SD3, "\n");
	}
	*/
	if(amplitude_input[0]<amplitude_input[1]) {
		a=amplitude_input[0];
		amplitude_input[0]=amplitude_input[1];
		amplitude_input[1]=a;
		b=order_input[0];
		order_input[0]=order_input[1];
		order_input[1]=b;
	}
		order_out[0]=order_input[0];
		high_amps[0]=amplitude_input[0];
		order_out[1]=order_input[1];
		high_amps[1]=amplitude_input[1];


		/*
	//Affichage pour debuggage
	if(high_amps[0]!=0){
	chprintf((BaseSequentialStream *)&SD3, "After\n");
	for(c=0;c<=1;c+=1){
		chprintf((BaseSequentialStream *)&SD3, "order=%d\n", order_out[c]); //unsigned integer
		chprintf((BaseSequentialStream *)&SD3, "amplitude=%f\n", high_amps[c]);
	}
	chprintf((BaseSequentialStream *)&SD3, "\n");
	// si les deux micros sélectionnés sont de faces opposées.
	if (((order_out[0]==0) && (order_out[1]==1)) || ((order_out[0]==1) && (order_out[1]==0))) {
		order_out[1]=order_input[1];
		high_amps[1]=amplitude_input[1];
	}
	else if (((order_out[0]==3) && (order_out[1]==2)) || ((order_out[0]==2) && (order_out[1]==3))) {
		order_out[1]=order_input[1];
		high_amps[1]=amplitude_input[1];
	}
	//Affichage pour debuggage
	chprintf((BaseSequentialStream *)&SD3, "After change\n");
	for(d=0;d>=1;++d){
		chprintf((BaseSequentialStream *)&SD3, "order fin=%d\n", order_out[d]); //unsigned integer
		chprintf((BaseSequentialStream *)&SD3, "amplitude fin=%f\n", high_amps[d]);
	}
	chprintf((BaseSequentialStream *)&SD3, "\n");
	*/
}

float phase_calcul(uint16_t position) { 	//PLAN A: comprendre comment se situe la phase par rapport a l'phase		PLAN B: faire comme romix, 2 micros front et back seuelemtn
	// penser au sens de l'phase, les valeurs de re et im en entrée sont aléatoires
	// calcul de l'phase grace aux FFT
	//if (fsig1==fsig2){ //extraire fsig des FFT! pas encore fait.

	float phase;
	float phase1;
	float phase2;
	float FFT_re1;
	float FFT_im1;
	float FFT_re2;
	float FFT_im2;

	/*
	if(((order_out[0]==0) || (order_out[1]==0)) && ((order_out[0]==3) || (order_out[1]==3))) { // Front - Right
		FFT_re1= micRight_cmplx_input[2*position];
		FFT_im1= micRight_cmplx_input[2*position+1];
		FFT_re2= micFront_cmplx_input[2*position];
		FFT_im2= micFront_cmplx_input[2*position+1];
	} else if(((order_out[0]==2) || (order_out[1]==2)) && ((order_out[0]==0) || (order_out[1]==0))){ //Right - Back
		FFT_re1= micBack_cmplx_input[2*position];
		FFT_im1= micBack_cmplx_input[2*position+1];
		FFT_re2= micRight_cmplx_input[2*position];
		FFT_im2= micRight_cmplx_input[2*position+1];
	} else if(((order_out[0]==1) || (order_out[1]==1)) && ((order_out[0]==3) || (order_out[1]==3))){ // Front - Left
		FFT_re1= micLeft_cmplx_input[2*position];
		FFT_im1= micLeft_cmplx_input[2*position+1];
		FFT_re2= micFront_cmplx_input[2*position];
		FFT_im2= micFront_cmplx_input[2*position+1];
	} else if(((order_out[0]==1) || (order_out[1]==1)) && ((order_out[0]==3) || (order_out[1]==3))){ // Back - Left
		FFT_re1= micBack_cmplx_input[2*position];
		FFT_im1= micBack_cmplx_input[2*position+1];
		FFT_re2= micLeft_cmplx_input[2*position];
		FFT_im2= micLeft_cmplx_input[2*position+1];
	}
	*/
	FFT_re1= micRight_cmplx_input[2*position];
	FFT_im1= micRight_cmplx_input[2*position+1];
	FFT_re2= micLeft_cmplx_input[2*position];
	FFT_im2= micLeft_cmplx_input[2*position+1];

	phase1=atan2f(FFT_im1,FFT_re1);
	phase2=atan2f(FFT_im2,FFT_re2);

	phase=phase1-phase2; // en radians

	//phase = 0.7*phase + 0.3*phase_old; // donne moins d'importance aux oscillations de la valeur non filtrée.

	/*
	if(((order_out[0]==1) && (phase>0)) || ((order_out[0]==0) && (phase<0))){ //il y a des valeurs incohérentes ou la phase est négative alors qu'elle devrait toujours etre positive et vice verca
		high_amps[0]=0;
		high_amps[1]=0;
	}
	*/
	//chprintf((BaseSequentialStream *)&SD3, "unfiltered phase =%f\n", phase);

	if((phase > 5) || (phase < -5)){ // eliminer les phases improbables //J'ai remplacé 0,4 par 5 car il y a seulement des pics au dessus de 5 qui apparaissent, et la pahse peut aller jusqu'a plus de 1 (mais toujours moins de 5) selon les fréquences
		phase = phase_old;
		return phase;
	} else {
		phase_old = phase;
		return phase;
	}

	//phase = phase*360/(2*M_PI); //en degrés
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
	if(mic_amp_output[0] < 4000) { //semble être suffisant
		mic_amp_output[0] = 0;
		mic_amp_output[1] = 0;
	}
}

float angle_calcul(float phase) { // angle en degrés
	float angle;
	phase = 159.09*pow(phase,6) + 38.99*pow(phase,5) - 130.42*pow(phase,4) + 26.067*pow(phase,3) + 11.744*pow(phase,2) + 99.131*phase - 16.93; // tester avec offset 5.909
	//chprintf((BaseSequentialStream *)&SD3, "angle=%f\n", angle);
	return angle;
}

int get_sound_angle(float* sound_direction){
	*sound_direction = angle_moyenne;
	new_angle_flag?new_angle_flag=0:0;
	return new_angle_flag;
}
