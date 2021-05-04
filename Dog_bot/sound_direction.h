#ifndef SOUND_DIRECTION_H
#define SOUND_DIRECTION_H


#define FFT_SIZE 	1024

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;

void sound_direction_setUp(void);

void compare_amp(float* amplitude_input, int* order_input);

float phase_calcul(uint16_t position);

int get_sound_angle(float* sound_direction);

float angle_calcul(float phase);

void processAudioData(int16_t *data, uint16_t num_samples);

float passe_bande(uint8_t position, float mic_amp_output);

void filtre_amp(float* mic_amp_output);
/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

#endif /* SOUND_DIRECTION_H */
