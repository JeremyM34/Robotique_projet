#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>

#include <audio/microphone.h>
#include <sound_direction.h>
#include <fft.h>
#include <arm_math.h>

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
// input buffers copy
static float micLeft_cmplx_input_copy[2 * FFT_SIZE];
static float micRight_cmplx_input_copy[2 * FFT_SIZE];
static float micFront_cmplx_input_copy[2 * FFT_SIZE];
static float micBack_cmplx_input_copy[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];
//Array of amplitude values out of compare_amp: Two highest amplitude
static float high_amps[2];
static int order_out[2]; // which are the mics with the highest amplitude?
//Array of amplitude values into compare_amp
static float amps_in[4];
static int order_in[4]; //order of mics: Right(0), Left(1), Back(2), Front(3)
//Calcul d'angle
float fsig1=0; //fréquence du signal 1
float fsig2=1; // fréquence du signal 2
float angle_final;
float FFT_re1;
float FFT_im1;
float FFT_re2;
float FFT_im2;
//filtre passe-bas


#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define MAX_FREQ		29	//453Hz: we don't analyze after this index to not use resources for nothing

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
	static uint8_t mustSend = 0;
	static uint8_t n;
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

		if(nb_samples >= (2 * FFT_SIZE)){

			// Copy buffers
			arm_copy_f32(micRight_cmplx_input, micRight_cmplx_input_copy, FFT_SIZE);
			arm_copy_f32(micLeft_cmplx_input, micLeft_cmplx_input_copy, FFT_SIZE);
			arm_copy_f32(micFront_cmplx_input, micFront_cmplx_input_copy, FFT_SIZE);
			arm_copy_f32(micBack_cmplx_input, micBack_cmplx_input_copy, FFT_SIZE);

			/*	FFT processing
			*
			*	This FFT function stores the results in the input buffer given.
			*	This is an "In Place" function.
			*/

			doFFT_optimized(FFT_SIZE, micRight_cmplx_input_copy);
			doFFT_optimized(FFT_SIZE, micLeft_cmplx_input_copy);
			doFFT_optimized(FFT_SIZE, micFront_cmplx_input_copy);
			doFFT_optimized(FFT_SIZE, micBack_cmplx_input_copy);

			/*	Magnitude processing
			*
			*	Computes the magnitude of the complex numbers and
			*	stores them in a buffer of FFT_SIZE because it only contains
			*	real numbers.
			*
			*/
			arm_cmplx_mag_f32(micRight_cmplx_input_copy, micRight_output, FFT_SIZE);
			arm_cmplx_mag_f32(micLeft_cmplx_input_copy, micLeft_output, FFT_SIZE);
			arm_cmplx_mag_f32(micFront_cmplx_input_copy, micFront_output, FFT_SIZE);
			arm_cmplx_mag_f32(micBack_cmplx_input_copy, micBack_output, FFT_SIZE);

			for(uint16_t j=0; j<=FFT_SIZE/2; ++j){
				//if(j<=FFT_SIZE/2){
					micRight_output[j] = passe_bande(j, micRight_output[j]); // filtrage en fréquence
					micLeft_output[j] = passe_bande(j, micLeft_output[j]);	//car symetrie, on prend seulement les positions positives
					micBack_output[j] = passe_bande(j, micBack_output[j]);
					micFront_output[j] = passe_bande(j, micFront_output[j]);
					/*
					micRight_output[j] = filtre_amp(micRight_output[j]); // filtrage en amplitude pour eliminer le fond sonore
					micLeft_output[j] = filtre_amp(micLeft_output[j]);
					micBack_output[j] = filtre_amp(micBack_output[j]);
					micFront_output[j] = filtre_amp(micFront_output[j]);
					*/
					amps_in[0]= micRight_output[j];
					amps_in[1]= micLeft_output[j];
					amps_in[2]= micBack_output[j];
					amps_in[3]= micFront_output[j];

					for(n=0; n<=3; ++n){
						order_in[n]=n;
					}
					compare_amp(amps_in, order_in); // OUTPUT: 2 highest amplitudes with corresponding mic number
				//}
				//sends only one FFT result over 10 for 1 mic to not flood the computer
				//sends to UART3
				if(mustSend >= 10){
					chprintf((BaseSequentialStream *)&SD3, "Final\n");
					//chprintf((BaseSequentialStream *)&SD3, "Result\n");
					for(o=0; o<=1;++o){
						if(high_amps[o] != 0){
							chprintf((BaseSequentialStream *)&SD3, "order=%d\n", order_out[o]); //unsigned integer
							chprintf((BaseSequentialStream *)&SD3, "amplitude=%f\n", high_amps[o]);
							chprintf((BaseSequentialStream *)&SD3, "\n");
						}
					}
					/*
					if (fsig1==fsig2){ //extraire fsig des FFT! pas encore fait.
						  // Mettre cette partie en commentaire pour tester filtre passe bas a mettre en place. (prioritaire)

							  if((order_out[0]==0) || (order_out[1]==0)) {  //pas de nombres consecutifs, sinon ya un bug...
								  FFT_re1= micRight_cmplx_input_copy[2*i];  // j'assume que le buffer est linéaire
								  FFT_im1= micRight_cmplx_input_copy[2*i+1];
								  if((order_out[0]==2) || (order_out[1]==2)){
									  FFT_re2= micBack_cmplx_input_copy[2*i];
									  FFT_im2= micBack_cmplx_input_copy[2*i+1];
								  }
								  else if((order_out[0]==3) || (order_out[1]==3)){
									  FFT_re2= micFront_cmplx_input_copy[2*i];
									  FFT_im2= micFront_cmplx_input_copy[2*i+1];
								  }
							  }
							  if((order_out[0]==1) || (order_out[1]==1)) {
								   FFT_re1= micLeft_cmplx_input_copy[2*i];
								   FFT_im1= micLeft_cmplx_input_copy[2*i+1];
								   if((order_out[0]==2) || (order_out[1]==2)){
									  FFT_re2= micBack_cmplx_input_copy[2*i];
									  FFT_im2= micBack_cmplx_input_copy[2*i+1];
								   }
								   else if((order_out[0]==3) || (order_out[1]==3)){
									  FFT_re2= micFront_cmplx_input_copy[2*i];
									  FFT_im2= micFront_cmplx_input_copy[2*i+1];
								   }
							  }
							  angle_final = angle_calcul(fsig1, fsig2, FFT_re1, FFT_im1, FFT_re2, FFT_im2);
					  }
					  */

					//mustSend = 0;
					//chprintf((BaseSequentialStream *)&SD3, "mustSend=%d\n", mustSend);
				}
				nb_samples = 0;
				mustSend++;
				//chprintf((BaseSequentialStream *)&SD3, "mustSend2=%d\n", mustSend);
		}
	}
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}

void compare_amp(float* amplitude_input, int* order_input){
	static int k;
	static int l;
	static float a;
	static int b;
	static int c;
	a=0;
	b=0;
	/*
	//Affichage pour debuggage
	chprintf((BaseSequentialStream *)&SD3, "Before\n");
	for(c=0;c<=3;++c){
	chprintf((BaseSequentialStream *)&SD3, "order=%d\n", order_input[c]); //unsigned integer
	chprintf((BaseSequentialStream *)&SD3, "amplitude=%f\n", amplitude_input[c]);
	}
	chprintf((BaseSequentialStream *)&SD3, "\n");
	*/

	for(k=0; k<=3; ++k){
		for(l=0; l<=3; ++l){
			if(k!=l) {
				if(amplitude_input[k]<amplitude_input[l]) {
					a=amplitude_input[k];
					amplitude_input[k]=amplitude_input[l];
					amplitude_input[l]=a;
					b=order_input[k];
					order_input[k]=order_input[l];
					order_input[l]=b;
				}
			}
		}
	}

	order_out[0]=order_input[3];
	high_amps[0]=amplitude_input[3];
	order_out[1]=order_input[2];
	high_amps[1]=amplitude_input[2];
	
	/*
	//Affichage pour debuggage
	chprintf((BaseSequentialStream *)&SD3, "After\n");
	for(c=3;c>=0;c-=1){
		chprintf((BaseSequentialStream *)&SD3, "order=%d\n", order_input[c]); //unsigned integer
		chprintf((BaseSequentialStream *)&SD3, "amplitude=%f\n", amplitude_input[c]);
	}
	chprintf((BaseSequentialStream *)&SD3, "\n");
	*/
}

float angle_calcul(float freq_sig1, float freq_sig2, float re1, float im1, float re2, float im2) {
	// penser au sens de l'angle, les valeurs de re et im en entrée sont aléatoires
	float angle;
	float phase1;
	float phase2;

	phase1=atan2(im1,re1);
	phase2=atan2(im2,re2);

	angle=phase1-phase2;

	return angle;
}

float passe_bande(uint8_t position, float mic_amp_output) {
	float mic_amp;
	//chprintf((BaseSequentialStream *)&SD3, "position=%d\n", position);
	//chprintf((BaseSequentialStream *)&SD3, "amplitude output=%f\n", mic_amp_output);

	if((position < 10) || (position > 29)){
		mic_amp = 0;
		//chprintf((BaseSequentialStream *)&SD3, "amplitude modified to 0\n\n");
		return mic_amp;  // on supprime les valeurs inutiles a l'analyse
	}
	mic_amp = mic_amp_output;
	return mic_amp;
}

float filtre_amp(float mic_amp_output)
{
	if(mic_amp_output < 2000000000) {
		mic_amp_output = 0;
		return mic_amp_output;
	}
	return mic_amp_output;
}
