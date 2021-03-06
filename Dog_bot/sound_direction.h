#ifndef SOUND_DIRECTION_H
#define SOUND_DIRECTION_H

#define SCAN_FREQUENCY_NUM			32		// position in the FFT array, corresponds to 500Hz
#define SCAN_FREQUENCY				(SCAN_FREQUENCY_NUM * 15.625)

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

bool get_sound_angle(float* sound_direction);


#endif /* SOUND_DIRECTION_H */
