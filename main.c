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

#include "csc_app.h"
#include "asf.h"
#include "stdio_serial.h"
//#include "time_tick.h"
#include "pt\pt.h"


#define DBG_LOG(string) (printf("%s\n\r", string))

#ifdef __cplusplus
extern "C" {
#endif

/* Global Variables and structures */

static struct tc_module tc_instance;
struct dac_module dac_inst;

/* Audio buffer size of this example */
#define AUDIO_BUF_SIZE  2132

/* Variaveis de controle */
bool pause = false; // play/pause
bool done = false; // finished playing
bool ready = false; // file is open and ready to be read
bool bton = true; // activate/deactivate bt proto


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

/* Variable that selects the required audio file to play */
char audio_file_name[32] = "0:testkl.wav"; //testkl.wav // test_16kHz
/* FAT FS variables */
Ctrl_status status;
FRESULT res;
FATFS fs;
FIL file_object;

#define CLOCK_SOURCE GCLK_GENERATOR_0 // clock source for TC and DAC

/* Func Prototypes*/
static void tc_callback_timer_match(struct tc_module *const module_inst); // timer counter match callback func
static void extint_callback(void); // extint callback func

#pragma region INITIALIZATION

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

	
/* Config and enable TC3 and callbacks*/
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
	
	/* Register and enable callback functions for compare event*/
	tc_register_callback(&tc_instance, tc_callback_timer_match, TC_CALLBACK_CC_CHANNEL0);		
	tc_enable_callback(&tc_instance, TC_CALLBACK_CC_CHANNEL0);
}

/* Config and enable Event System*/
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

/* Configures the External Interrupt Controller and callbacks */
static void configure_extint(void)
{
	struct extint_chan_conf eint_chan_conf;
	extint_chan_get_config_defaults(&eint_chan_conf);

	eint_chan_conf.gpio_pin           = BUTTON_0_EIC_PIN;
	eint_chan_conf.gpio_pin_mux       = BUTTON_0_EIC_MUX;
	eint_chan_conf.detection_criteria = EXTINT_DETECT_BOTH;
	eint_chan_conf.filter_input_signal = true;
	extint_chan_set_config(BUTTON_0_EIC_LINE, &eint_chan_conf);
	
	/* Register and enable callbacks*/
	extint_register_callback(extint_callback, BUTTON_0_EIC_LINE, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(BUTTON_0_EIC_LINE, EXTINT_CALLBACK_TYPE_DETECT);
}

#pragma endregion

/** Callback function for the EXTINT driver, called when an external interrupt
 *  detection occurs.
 */
static void extint_callback(void)
{	
	bton = true;
}

/** TC Callback function. Used to process match channel 0 interrupt
 */
static void tc_callback_timer_match(struct tc_module *const module_inst)
{
	dac_chan_write(&dac_inst, DAC_CHANNEL_0, *audio_data_ptr++);
	
	buf_cnt++;
	if (buf_cnt == (AUDIO_BUF_SIZE/2))
	{
		buf_cnt = 0;
		audio_playback_done = true;
	}
	
}

/* Function to mount FAT file system */
static void mount_file_system(void)
{		
	/* Wait card present and ready */
	do {
		status = sd_mmc_test_unit_ready(0);
		
		if (CTRL_FAIL == status) 
		{
			printf("Card install FAIL\n\r");
			printf("Please unplug and re-plug the card.\n\r");
			while (CTRL_NO_PRESENT != sd_mmc_check(0)) 
			{
			}
		}
	} while (CTRL_GOOD != status);
	
	/* Mount the file sytem */
	printf("Mount disk (f_mount)...\r\n");
	memset(&fs, 0, sizeof(FATFS));
	
	res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);
	
	if (FR_INVALID_DRIVE == res) 
	{
		printf("[FAIL] res %d\r\n", res);
		while(1);
	}
	
	printf("[OK]\r\n");
}

/* Open WAV audio file and read header*/
static void open_wav()
{
	/* Open the WAV file */
	printf("Open file \"%s\"\r\n", audio_file_name);
	audio_file_name[0] = LUN_ID_SD_MMC_0_MEM + '0';
	res = f_open(&file_object, (char const *)audio_file_name, FA_READ);
	if (res != FR_OK)
	{
		printf("[FAIL] res %d\r\n", res);
		while(1);
	}
	printf("[OK]\r\n");
	
	/* Read audio header */
	printf("Reading WAV\r\n");
	res = f_read(&file_object, (uint8_t *)&audio_header, \
	sizeof(audio_header), &temp);
	
	if (res != FR_OK) 
	{
		printf("[FAIL] res %d\r\n", res);
		while(1);
	}
	printf("[OK]\r\n");
	
	ready = true;
}

static void config_led()
{
	struct port_config pin;
	
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
}

void command(char *msg, char size)
{	
	if(strncmp(msg, "pause", size) == 0)
	{
		if(done) // can't pause something that isn't playing dummy!
			return;
		
		if(pause)
		{
			pause = false;
			dac_chan_enable_output_buffer(&dac_inst, DAC_CHANNEL_0);
			tc_start_counter(&tc_instance);

		}
		else
		{
			pause = true;
			dac_chan_disable_output_buffer(&dac_inst, DAC_CHANNEL_0);
			tc_stop_counter(&tc_instance);
		}
		
		return;
	}
	
	if(strncmp(msg, "done", size) == 0)
	{
		done = true;
		dac_chan_disable_output_buffer(&dac_inst, DAC_CHANNEL_0);
		tc_stop_counter(&tc_instance);
		
		return;
	}
	
	if(strncmp(msg, "play", 4) == 0)
	{
		msg[size] = '\0';
		
		sprintf(&audio_file_name[2], "%s", &msg[5]);
		command("done", 4);
		
		return;
	}
	
	if(strncmp(msg, "btoff", size) == 0)
	{
		bton = false;
		return;
	}
}

#pragma region PROTOTHREADS

static struct pt pt1, pt2, pt3; // protothreads state variables

/* Load samples from wav  */
static PT_THREAD(PROTO_SAMPLES(struct pt *pt))
{
	PT_BEGIN(pt);

	while(1)
	{
		if (current_data_buf == 0) 
		{
			/* Read audio data */
			res = f_read(&file_object, (uint8_t *)temp_audio_buf, AUDIO_BUF_SIZE*2, &temp);
			if (res != FR_OK) 
			{
				printf("[FAIL] Reading block %d\r\n", (int)i);
				while(1);
			}
			
			if(temp < AUDIO_BUF_SIZE*2)
			{// EOF
				command("done", 4); // finished playing
			}
			
			/* Shift to form unsigned value */
			j = 0;
			for (i = 0; i < AUDIO_BUF_SIZE; i = i+2) {
				temp = (uint16_t)((int32_t)temp_audio_buf[i] + 32768);
				audio_data_1[j++] = (temp >> 6);
			}
			
			/* Wait until it is sampled by DAC. It is set at timer call back
			 * to indicate next set of data can be processed.
			 */
			PT_YIELD_UNTIL(pt, audio_playback_done);
			audio_playback_done = false;
			audio_data_ptr = audio_data_1;
			current_data_buf = 1;
		}
		else 
		{
			/* Read audio data */
			res = f_read(&file_object, (uint8_t *)temp_audio_buf, AUDIO_BUF_SIZE*2, &temp);
			if (res != FR_OK) 
			{
				printf("[FAIL] Reading block %d\r\n", (int)i);
				while(1);
			}
			
			if(temp < AUDIO_BUF_SIZE*2)
			{// EOF
				command("done", 4); // finished playing
			}
			
			/* Shift to form unsigned value */
			j = 0;
			for (i = 0; i < AUDIO_BUF_SIZE; i = i+2) 
			{
				temp = (uint16_t)((int32_t)temp_audio_buf[i] + 32768);
				audio_data_0[j++] = (temp >> 6);
			}
			
			/* Wait until it is sampled by DAC. It is set at timer call back
			 * to indicate next set of data can be processed.
			 */
			PT_YIELD_UNTIL(pt, audio_playback_done);
			
			audio_playback_done = false;
			/* Switch the pointer to next buffer */
			audio_data_ptr = audio_data_0;
			current_data_buf = 0;
		}
	}

	PT_END(pt);
}

/* Process bluetooth events*/
static PT_THREAD(PROTO_BT(struct pt *pt))
{
  PT_BEGIN(pt);

  while(1)
  {
	ble_event_task();
	
	PT_YIELD(pt);	
  }
  
  PT_END(pt);
}


/* Main music thread*/
static PT_THREAD(PROTO_PLAYER(struct pt *pt))
{
  PT_BEGIN(pt);

  /* We loop forever here. */
  while(1) 
  {
	  do 
	  {
		  open_wav();
	  }  while(!ready); // nothing to play
	  
	dac_chan_enable_output_buffer(&dac_inst, DAC_CHANNEL_0); // enable DAC output
	
	printf("FILE HEADER PRINT: \n\n\rChunkID: %s\n\rChunkSize: %d\n\rFormat: %s\n\rSubChunk1ID: %s\n\rSubchunk1Size: %d\n\rAudioFormat: %d\n\r",
			audio_header.ChunkID, audio_header.ChunkSize, audio_header.Format, audio_header.Subchunk1ID, audio_header.Subchunk1Size, 
			audio_header.AudioFormat);			
	printf("NumChannels: %d\n\rSampleRate: %d\n\rByteRate: %d\n\rBlockAlign: %d\n\rBitsPerSample: %d\n\rSubchunk2ID: %s\n\rSubchunk2Size: %d\n\r",
			audio_header.NumChannels, audio_header.SampleRate, audio_header.ByteRate, audio_header.BlockAlign, audio_header.BitsPerSample, 
			audio_header.Subchunk2ID, audio_header.Subchunk2Size);
			
	/* Set timer 0 compare value corresponding to wav sample rate */
	tc_set_compare_value(&tc_instance, 0, system_gclk_gen_get_hz(CLOCK_SOURCE)/audio_header.SampleRate);
	
	/* Read audio data from the input wav file in temp_audio_buf */
	res = f_read(&file_object, (uint8_t *)temp_audio_buf, \
					AUDIO_BUF_SIZE*2, &temp);
	if (res != FR_OK) 
	{
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
	
	done = false; // not even started yet!
	
	/* Start timer to sample first block */
	tc_start_counter(&tc_instance);
	
	/* While not done, play*/
	do
	{		
		PROTO_SAMPLES(&pt2);	
		
		if(bton)	
			PROTO_BT(&pt3);		
			
	} while(!done);
	
	/* Stop the timer once processing all input audio data */
	tc_stop_counter(&tc_instance);
	/* Close the file */
	f_close(&file_object);
	printf("Audio playback done! Closing File.\r\n");
  }

  PT_END(pt);
}

#pragma endregion


/**
 *  \brief getting-started Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
*/
int main(void)
{
	system_init();

	/* Initialize serial console */
	sio2host_init();		
	
	printf("\033[2J"); //"clear" terminal
	DBG_LOG("UART DONE");
	
	/*Configures TC driver and callbacks*/
	DBG_LOG("Config TC");
	configure_tc();
	DBG_LOG("TC DONE");		

	/*Configure and initialize DAC*/
	DBG_LOG("Initializing DAC");
	/* Enable the internal bandgap to use as reference to the DAC */
	system_voltage_reference_enable(SYSTEM_VOLTAGE_REFERENCE_BANDGAP);
	configure_dac(&dac_inst);
	DBG_LOG("DAC DONE");

	/*Initialize the delay driver*/
	DBG_LOG("Initialize Delay Driver");
	delay_init();
	DBG_LOG("Delay Driver DONE");
	
	/* Initialize event system*/
	DBG_LOG("Initialize EVSYS");
	evsys_init();
	DBG_LOG("EVSYS DONE");
	
	/*Configures the External Interrupt and callbacks*/
	DBG_LOG("Config EXTINT");
	configure_extint();
	DBG_LOG("EXTINT DONE");	

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
	
	/*Initialize BLE*/
	DBG_LOG("Initializing BLUETOOTH");
	BTSTART();
	DBG_LOG("\n\rBLUETOOTH DONE");
	
	/* Initialize the protothread state variables with PT_INIT(). */
	PT_INIT(&pt1);
	PT_INIT(&pt2);
	PT_INIT(&pt3);
	
	/* Sinalize system init DONE*/
	config_led();
	
	/* Protothreads take it from here*/
	while(1)
	{
		PROTO_PLAYER(&pt1); // music
	}
}

#ifdef __cplusplus
}
#endif
