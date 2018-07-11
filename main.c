/**
 * \file
 *
 * \brief Getting Started Application.
 *
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage Getting Started Application
 *
 * \section Purpose
 *
 * The Getting Started example will help new users get familiar with Atmel's
 * SAM family of microcontrollers. This basic application shows the startup
 * sequence of a chip and how to use its core peripherals.
 *
 * \section Requirements
 *
 * This application has been tested on following boards:
 * - SAM D21/R21/L21/L22/C21 Xplained Pro
 * - SAM D10 Xplained Mini
 * - SAMR21ZLL-EK
 *
 * \section Description
 *
 * The program demo how LED,button,delay,interrupt and timer/counter work .
 *
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board.
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 38400 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# The LED(s) should start blinking on the board. In the terminal window, the
 *    following text should appear (values depend on the board and chip used):
 *    \code
 *     -- Getting Started Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 * -# Pressing and release button SW0 should make LED0 on and off
 *    blinking.
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include "asf.h"
#include "stdio_serial.h"
#include "conf_uart_serial.h"
#include "time_tick.h"
#include "EqSource.h"

#define DBG_LOG(string) (printf("%s\n\r", string))

#ifdef __cplusplus
extern "C" {
#endif



/* Global Variables and structures */

static struct usart_module cdc_uart_module;
static struct tc_module tc_instance;
struct dac_module dac_inst;

/* Audio buffer size of this example */
#define AUDIO_BUF_SIZE  1024

/* Standard WAV audio header */
COMPILER_PACK_SET(1)
struct wav_header {
	char ChunkID[4];
	uint32_t ChunkSize;
	char Format[4];
	char Subchunk1ID[4];
	uint32_t Subchunk1Size;
	uint16_t AudioFormat;
	uint16_t NumChannels;
	uint32_t SampleRate;
	uint32_t ByteRate;
	uint16_t BlockAlign;
	uint16_t BitsPerSample;
	char Subchunk2ID[4];
	uint32_t Subchunk2Size;
};
struct wav_header audio_header;
COMPILER_PACK_RESET()

/* Buffer and status variables */
int16_t temp_audio_buf[AUDIO_BUF_SIZE];
uint16_t audio_data_0[AUDIO_BUF_SIZE/2], audio_data_1[AUDIO_BUF_SIZE/2];
volatile uint16_t *audio_data_ptr;
volatile bool current_data_buf = 0, audio_playback_done = false;
uint32_t nb_audio_blocks, block_cnt, buf_cnt, i, j;
unsigned int temp;

//EQUALIZADOR
volatile uint16_t *audio_data_ptrEQ;
EQSTATE *eqs;


/* Variable that selects the required audio file to play */
char test_file_name[] = "0:testro.wav"; //testkl.wav // test_16kHz
/* FAT FS variables */
Ctrl_status status;
FRESULT res;
FATFS fs;
FIL file_object;
TCHAR testd[128];

#define CLOCK_SOURCE GCLK_GENERATOR_0 // clock source for TC and DAC

/* DAC */

/**
 * \brief Configures the DAC in event triggered mode.
 *
 * Configures the DAC to use the module's default configuration, with output
 * channel mode configured for event triggered conversions.
 *
 * \param dev_inst  Pointer to the DAC module software instance to initialize
 */
static void configure_dac(struct dac_module *dac_module)
{
	struct dac_config config;
	struct dac_chan_config channel_config;
		
	/* Get the DAC default configuration */
	dac_get_config_defaults(&config);

	config.clock_source = CLOCK_SOURCE;

	dac_init(dac_module, DAC, &config);

	/* Get the default DAC channel config */
	dac_chan_get_config_defaults(&channel_config);

	/* Set the channel configuration, and enable it */
	dac_chan_set_config(dac_module, DAC_CHANNEL_0, &channel_config);
	dac_chan_enable(dac_module, DAC_CHANNEL_0);

	/* Enable event triggered conversions */
	struct dac_events events = { .on_event_start_conversion = true };

	dac_enable_events(dac_module, &events);

	dac_enable(dac_module);
}


/* DAC END*/

/**
 *  Configure UART console.
 */
static void configure_console(void)
{
	struct usart_config usart_conf;

	usart_get_config_defaults(&usart_conf);
	usart_conf.mux_setting = CONF_STDIO_MUX_SETTING;
	usart_conf.pinmux_pad0 = CONF_STDIO_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = CONF_STDIO_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = CONF_STDIO_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = CONF_STDIO_PINMUX_PAD3;
	usart_conf.baudrate    = CONF_STDIO_BAUDRATE;

	stdio_serial_init(&cdc_uart_module, CONF_STDIO_USART_MODULE, &usart_conf);
	usart_enable(&cdc_uart_module);
}

/* Updates the board LED to the current button state. */
static void update_led_state(void)
{
	bool pin_state = port_pin_get_input_level(BUTTON_0_PIN);
	if (pin_state) {
		port_pin_set_output_level(LED_0_PIN, LED_0_INACTIVE);
	} else {
		port_pin_set_output_level(LED_0_PIN, LED_0_ACTIVE);
	}
}

/** Callback function for the EXTINT driver, called when an external interrupt
 *  detection occurs.
 */
static void extint_callback(void)
{
	static bool dac_out_on = true;
	
	if(dac_out_on)
	{
			dac_out_on = false;
			dac_chan_disable_output_buffer(&dac_inst, DAC_CHANNEL_0);
	}
	else
	{
		dac_out_on = true;
		dac_chan_enable_output_buffer(&dac_inst, DAC_CHANNEL_0);
	}
}

/** Configures and registers the External Interrupt callback function with the
 *  driver.
 */
static void configure_eic_callback(void)
{
	extint_register_callback(extint_callback,
			BUTTON_0_EIC_LINE,
			EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(BUTTON_0_EIC_LINE,
			EXTINT_CALLBACK_TYPE_DETECT);
}

/** Configures the External Interrupt Controller to detect changes in the board
 *  button state.
 */
static void configure_extint(void)
{
	struct extint_chan_conf eint_chan_conf;
	extint_chan_get_config_defaults(&eint_chan_conf);

	eint_chan_conf.gpio_pin           = BUTTON_0_EIC_PIN;
	eint_chan_conf.gpio_pin_mux       = BUTTON_0_EIC_MUX;
	eint_chan_conf.detection_criteria = EXTINT_DETECT_BOTH;
	eint_chan_conf.filter_input_signal = true;
	extint_chan_set_config(BUTTON_0_EIC_LINE, &eint_chan_conf);
}


/** TC Callback function. Used to process match channel 0 interrupt
 */
static void tc_callback_timer_match(struct tc_module *const module_inst)
{

	audio_data_ptrEQ = (uint16_t)do_3band(eqs, *audio_data_ptr++);

	dac_chan_write(&dac_inst, DAC_CHANNEL_0, *audio_data_ptrEQ++);
	buf_cnt++;
	if (buf_cnt == (AUDIO_BUF_SIZE/2))
	{
		buf_cnt = 0;
		audio_playback_done = true;
	}
}

/** Configures  TC function with the  driver.
 */
static void configure_tc(void)
{
	struct tc_config config_tc;
	struct tc_events conf_tc_events = {.generate_event_on_compare_channel[0] = 1};
	//struct tc_events events = { .generate_event_on_overflow = true };

	tc_get_config_defaults(&config_tc);
	
	config_tc.clock_source = CLOCK_SOURCE;
	config_tc.wave_generation = TC_WAVE_GENERATION_MATCH_FREQ;
	config_tc.counter_size    = TC_COUNTER_SIZE_16BIT;
	config_tc.counter_16_bit.compare_capture_channel[0] = 0xFFFF;

	tc_init(&tc_instance, CONF_TC_INSTANCE, &config_tc);
	
	tc_enable_events(&tc_instance, &conf_tc_events);
	tc_enable(&tc_instance);
	
	tc_stop_counter(&tc_instance);
}

/** Registers TC callback function with the  driver.
 */
static void configure_tc_callbacks(void)
{
	tc_register_callback(&tc_instance, tc_callback_timer_match, TC_CALLBACK_CC_CHANNEL0);
	
	tc_enable_callback(&tc_instance, TC_CALLBACK_CC_CHANNEL0);
}

/* EVSYS*/
/* Event System initialization */
static void evsys_init(void)
{
	struct events_resource conf_event_resource;
	struct events_config conf_event;
	
	events_get_config_defaults(&conf_event);
	
	//conf_event.edge_detect = EVENTS_EDGE_DETECT_NONE;
	conf_event.path = EVENTS_PATH_ASYNCHRONOUS;
	//conf_event.generator    = EVSYS_ID_GEN_TC3_OVF; // TC overflow
	conf_event.generator = EVSYS_ID_GEN_TC3_MCX_0; // tc chan0 match
	
	events_allocate(&conf_event_resource, &conf_event);
	events_attach_user(&conf_event_resource, EVSYS_ID_USER_DAC_START);
}

/* EVSYS END*/


/* Function to mount file system, open audio file and read
   audio header */
static void mount_file_system(void)
{		
	/* Wait card present and ready */
	do {
		status = sd_mmc_test_unit_ready(0);
		if (CTRL_FAIL == status) {
			printf("Card install FAIL\n\r");
			printf("Please unplug and re-plug the card.\n\r");
			while (CTRL_NO_PRESENT != sd_mmc_check(0)) {
			}
		}
	} while (CTRL_GOOD != status);
	
	/* Mount the file sytem */
	printf("Mount disk (f_mount)...\r\n");
	memset(&fs, 0, sizeof(FATFS));
	
	res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);
	
	if (FR_INVALID_DRIVE == res) {
		printf("[FAIL] res %d\r\n", res);
		while(1);
	}
	printf("[OK]\r\n");

	/* Open the WAV file */
	printf("Open a file (f_open)...\r\n");
	test_file_name[0] = LUN_ID_SD_MMC_0_MEM + '0';
	res = f_open(&file_object, (char const *)test_file_name, FA_READ);
	if (res != FR_OK) {
		printf("[FAIL] res %d\r\n", res);
		while(1);
	}
	printf("[OK]\r\n");
	
	/* Read audio header */
	printf("Read from file (f_read)...\r\n");
	res = f_read(&file_object, (uint8_t *)&audio_header, \
					sizeof(audio_header), &temp);
	if (res != FR_OK) {
		printf("[FAIL] res %d\r\n", res);
		while(1);
	}
	printf("[OK]\r\n");
}

/**
 *  \brief getting-started Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
*/
int main(void)
{
	

	init_3band_state(eqs, 10,10,20);

	


	struct port_config pin;

	system_init();

	/*Configure UART console.*/
	configure_console();
	
	DBG_LOG("UART DONE");
	
	/*Configures  TC driver*/
	DBG_LOG("Config TC");
	configure_tc();
	DBG_LOG("TC DONE");
	
	/*Configures TC callback*/
	DBG_LOG("Config TC Callback");
	configure_tc_callbacks();
	DBG_LOG("TC Callback DONE");		

	/*Configure and initialize DAC*/
	DBG_LOG("Initializing DAC");
	/* Enable the internal bandgap to use as reference to the DAC */
	system_voltage_reference_enable(SYSTEM_VOLTAGE_REFERENCE_BANDGAP);
	configure_dac(&dac_inst);
	DBG_LOG("DAC DONE");

	/*Configures the External Interrupt*/
	DBG_LOG("Config EXTINT");
	configure_extint();
	DBG_LOG("EXTINT DONE");
	
	/*Configures the External Interrupt callback*/
	DBG_LOG("Config EXTINT Callback");
	configure_eic_callback();
	DBG_LOG("EXTINT Callback DONE");	

	/*Initialize the delay driver*/
	DBG_LOG("Initialize Delay Driver");
	delay_init();
	DBG_LOG("Delay Driver DONE");
	
	/* Initialize event system*/
	DBG_LOG("Initialize EVSYS");
	evsys_init();
	DBG_LOG("EVSYS DONE");

	/*Enable system interrupt*/
	DBG_LOG("Enabling Interrupts");
	system_interrupt_enable_global();
	DBG_LOG("Interrupt DONE");	
	
	/* Initialize SD MMC stack */
	DBG_LOG("Initialize SD");
	sd_mmc_init();
	DBG_LOG("SD DONE");
	
	/*Initialize FAT file system and read .wav*/
	DBG_LOG("Initializing FAT");
	mount_file_system();
	DBG_LOG("FAT DONE");
	
    /*Configures PORT for LED0*/
	port_get_config_defaults(&pin);
	pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED0_PIN, &pin);

	port_pin_set_output_level(LED0_PIN, LED0_INACTIVE);

	for (int i = 0; i < 3; i++) {
		port_pin_toggle_output_level(LED0_PIN);
		delay_s(1);
	}

	for (int i = 0; i < 20; i++) {
		port_pin_toggle_output_level(LED0_PIN);
		delay_ms(100);
	}

	port_pin_set_output_level(LED0_PIN, LED0_INACTIVE);
	
	dac_chan_enable_output_buffer(&dac_inst, DAC_CHANNEL_0);
	
	printf("FILE HEADER PRINT: \n\n\rChunkID: %s\n\rChunkSize: %d\n\rFormat: %s\n\rSubChunk1ID: %s\n\rSubchunk1Size: %d\n\rAudioFormat: %d\n\r",
			audio_header.ChunkID, audio_header.ChunkSize, audio_header.Format, audio_header.Subchunk1ID, audio_header.Subchunk1Size, 
			audio_header.AudioFormat);
			
	printf("NumChannels: %d\n\rSampleRate: %d\n\rByteRate: %d\n\rBlockAlign: %d\n\rBitsPerSample: %d\n\rSubchunk2ID: %s\n\rSubchunk2Size: %d\n\r",
			audio_header.NumChannels, audio_header.SampleRate, audio_header.ByteRate, audio_header.BlockAlign, audio_header.BitsPerSample, 
			audio_header.Subchunk2ID, audio_header.Subchunk2Size);
	
	
	volatile int test = system_gclk_gen_get_hz(CLOCK_SOURCE);
	
	int cValue = (system_gclk_gen_get_hz(CLOCK_SOURCE)/audio_header.SampleRate);
	
	/* Set timer 0 compare value corresponding to wav sample rate */
	tc_set_compare_value(&tc_instance, 0, cValue);
	
	/* Set the timer top value to alter the overflow frequency */
	//tc_set_top_value(&tc_instance, system_gclk_gen_get_hz(GCLK_GENERATOR_0) / audio_header.SampleRate);
	
	/* The input data is processed for block of AUDIO_BUF_SIZE.
	 * Find number of AUDIO_BUF_SIZE data blocks in the input .WAV file. 
	 * Subchunk2Size of the audio header tells the number of audio data in 
	 * the input wave file. 
	 * Note : As it is stereo data both left and right channel data is read. 
	 * But only left channel data is processed. So when reading the audio 
	 * samples the read size is set to twice the AUDIO_BUF_SIZE.
	 */
	nb_audio_blocks = audio_header.Subchunk2Size - \
						(audio_header.Subchunk2Size % (AUDIO_BUF_SIZE*2));
	nb_audio_blocks = nb_audio_blocks/(AUDIO_BUF_SIZE*2);
	
	/* Read audio data from the input wav file in temp_audio_buf */
	res = f_read(&file_object, (uint8_t *)temp_audio_buf, \
					AUDIO_BUF_SIZE*2, &temp);
	if (res != FR_OK) {
		printf("[FAIL] Reading first block!\r\n");
		while(1);
	}
	/* The input file is in 16 bit signed PCM format. But the DAC
	 * does not support signed values, So shift the input data
	 * by 32767 bits to form signed value. Store the resulting value 
	 * to buffer 1 (i.e. audio_data_0).
	 */
	j = 0;
	for (i = 0; i < AUDIO_BUF_SIZE; i = i+2) 
	{
		int32_t temp2 = (int32_t)temp_audio_buf[i] + 32767;
		temp = (uint16_t)(temp2 & 0xFFFF);
		audio_data_0[j++] = (temp >> 6);
	}	
	/* Move the current data pointer to buffer1 */
	audio_data_ptr = audio_data_0;
	current_data_buf = 0;
	/* Start timer to sample first block */
	tc_start_counter(&tc_instance);
	
	/* Read data for nb_audio_blocks-1 times as it is once read before */
	for (block_cnt = 0; block_cnt < nb_audio_blocks-1; block_cnt++)	
	{
		/* Check if the pointer is in buffer1 or 2 via current_data_buf value 
		 * and switch the buffer accordingly. 
		 */
		if (current_data_buf == 0) 
		{
			/* Read audio data */
			res = f_read(&file_object, (uint8_t *)temp_audio_buf, AUDIO_BUF_SIZE*2, &temp);
			if (res != FR_OK)
			{
				printf("[FAIL] Reading block %d\r\n", (int)i);
				while(1);
			}
			
			/* Shift to form unsigned value */
			j = 0;
			for (i = 0; i < AUDIO_BUF_SIZE; i = i+2) 
			{
				temp = (uint16_t)((int32_t)temp_audio_buf[i] + 32768);
				audio_data_1[j++] = (temp >> 6);
			}
			
			/* Wait until it is sampled by DAC. It is set at timer call back
			 * to indicate next set of data can be processed.
			 */
			while(!audio_playback_done);
			audio_playback_done = false;
			audio_data_ptr = audio_data_1;
			current_data_buf = 1;
		}
		else {
			/* Read audio data */
			res = f_read(&file_object, (uint8_t *)temp_audio_buf, AUDIO_BUF_SIZE*2, &temp);
			if (res != FR_OK) {
				printf("[FAIL] Reading block %d\r\n", (int)i);
				while(1);
			}
			/* Shift to form unsigned value */
			j = 0;
			for (i = 0; i < AUDIO_BUF_SIZE; i = i+2) {
				temp = (uint16_t)((int32_t)temp_audio_buf[i] + 32768);
				audio_data_0[j++] = (temp >> 6);
			}
			/* Wait until it is sampled by DAC. It is set at timer call back
			 * to indicate next set of data can be processed.
			 */
			while(!audio_playback_done);
			audio_playback_done = false;
			/* Switch the pointer to next buffer */
			audio_data_ptr = audio_data_0;
			current_data_buf = 0;
		}
	}
	/* Stop the timer once processing all input audio data */
	tc_stop_counter(&tc_instance);
	/* Close the file */
	f_close(&file_object);
	printf("Audio playback done! Closing File.\r\n");
	
	/* Loop until next reset */
	
	/*main loop*/
	while(1)
	{
	}
}

#ifdef __cplusplus
}
#endif
