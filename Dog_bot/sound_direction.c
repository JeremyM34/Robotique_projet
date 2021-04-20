#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>

#include <audio/microphone.h>
#include <sound_direction.h>
#include <fft.h>
#include <arm_math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

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
static float amps_in[4];
static int order_in[4]; //order of mics: Right(0), Left(1), Back(2), Front(3)

#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define FREQ_FORWARD	16	//250Hz
#define FREQ_LEFT		19	//296Hz
#define FREQ_RIGHT		23	//359HZ
#define FREQ_BACKWARD	26	//406Hz
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing

#define FREQ_FORWARD_L		(FREQ_FORWARD-1)
#define FREQ_FORWARD_H		(FREQ_FORWARD+1)
#define FREQ_LEFT_L			(FREQ_LEFT-1)
#define FREQ_LEFT_H			(FREQ_LEFT+1)
#define FREQ_RIGHT_L		(FREQ_RIGHT-1)
#define FREQ_RIGHT_H		(FREQ_RIGHT+1)
#define FREQ_BACKWARD_L		(FREQ_BACKWARD-1)
#define FREQ_BACKWARD_H		(FREQ_BACKWARD+1)

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
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);
		
		amps_in[0]= micRight_output[i];
	    	amps_in[1]= micLeft_output[i];
		amps_in[2]= micBack_output[i];
		amps_in[3]= micFront_output[i];

		for(n=0; n<=3; ++n){
			order_in[n]=n;
		}
		compare_amp(amps_in, order_in); // OUTPUT: 2 highest amplitudes with corresponding mic number

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		if(mustSend >= 10){
			chBSemSignal(&sendToComputer_sem);
	        	chprintf((BaseSequentialStream *)&SD3, "Result\n");
	        	for(o=0; o<=1;++o){
	        	  	chprintf((BaseSequentialStream *)&SD3, "order=%d\n", order_out[o]); //unsigned integer
	        	  	chprintf((BaseSequentialStream *)&SD3, "amplitude=%f\n", high_amps[o]);
	          	}
	          	chprintf((BaseSequentialStream *)&SD3, "\n");
	          	mustSend = 0;

		}
		nb_samples = 0;
		mustSend++;
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
	
	//Affichage pour debuggage
	chprintf((BaseSequentialStream *)&SD3, "Before\n");
	for(c=0;c<=3;++c){
	chprintf((BaseSequentialStream *)&SD3, "order=%d\n", order_input[c]); //unsigned integer
	chprintf((BaseSequentialStream *)&SD3, "amplitude=%f\n", amplitude_input[c]);
	}
	chprintf((BaseSequentialStream *)&SD3, "\n");


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
	
	//Affichage pour debuggage
	chprintf((BaseSequentialStream *)&SD3, "After\n");
	for(c=3;c>=0;c-=1){
		chprintf((BaseSequentialStream *)&SD3, "order=%d\n", order_input[c]); //unsigned integer
		chprintf((BaseSequentialStream *)&SD3, "amplitude=%f\n", amplitude_input[c]);
	}
	chprintf((BaseSequentialStream *)&SD3, "\n");

}

