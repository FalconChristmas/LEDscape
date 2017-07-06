/** \file
 * LEDscape for the BeagleBone Black.
 *
 * Drives up to 32 ws281x LED strips using the PRU to have no CPU overhead.
 * Allows easy double buffering of frames.
 */

#ifndef _ledscape_h_
#define _ledscape_h_

#include <stdint.h>

#include "pru.h"

/** The number of strips supported.
 *
 * Changing this also requires changes in ws281x.p to stride the
 * correct number of bytes per row..
 */
#define LEDSCAPE_NUM_STRIPS 32

#define LEDSCAPE_MATRIX 1
#define LEDSCAPE_STRIP 2


typedef struct {
	int x;
	int y;
	int rot; // 0 == none, 1 == left, 2 == right, 3 == flip
} ledscape_matrix_panel_t;

#define LEDSCAPE_MATRIX_OUTPUTS 8 // number of outputs on the cape
#define LEDSCAPE_MATRIX_PANELS 12 // number of panels chained per output

typedef struct {
	int type;
	int width;
	int height;
	int panel_width;
	int panel_height;
	int leds_width;
	int leds_height;
	int panelCount;
    int rowsPerOutput;
	ledscape_matrix_panel_t panels[LEDSCAPE_MATRIX_OUTPUTS][LEDSCAPE_MATRIX_PANELS];
} ledscape_matrix_config_t;


typedef struct {
	int type;
	int leds_width; // length of the longest strip
	int leds_height; // number of output strips
} ledscape_strip_config_t;


typedef union {
	int type;
	ledscape_matrix_config_t matrix_config;
	ledscape_strip_config_t strip_config;
} ledscape_config_t;


/** LEDscape pixel format is BRGA.
 *
 * data is laid out with BRGA format, since that is how it will
 * be translated during the clock out from the PRU.
 */
typedef struct {
	uint8_t b;
	uint8_t r;
	uint8_t g;
	uint8_t a;
} __attribute__((__packed__)) ledscape_pixel_t;


/** LEDscape frame buffer is "strip-major".
 *
 * All 32 strips worth of data for each pixel are stored adjacent.
 * This makes it easier to clock out while reading from the DDR
 * in a burst mode.
 */
typedef struct {
	ledscape_pixel_t strip[LEDSCAPE_NUM_STRIPS];
} __attribute__((__packed__)) ledscape_frame_t;


typedef struct ledscape ledscape_t;


#ifdef __cplusplus
extern "C" {
#endif

extern ledscape_config_t ledscape_matrix_default;

extern ledscape_t *
ledscape_init(
	ledscape_config_t * config,
	int no_init_pru
);

extern ledscape_t *
ledscape_matrix_init(
	ledscape_config_t * config,
	int no_init_pru,
	int pru_number,
	const char * pru_program
);

extern ledscape_t *
ledscape_strip_init(
	ledscape_config_t * config,
	int no_init_pru,
	int pru_number,
	const char * pru_program
);

extern void
ledscape_draw(
	ledscape_t * const leds,
	const void * const rgb // 4-byte rgb data
);


extern void
ledscape_set_color(
	ledscape_frame_t * const frame,
	uint8_t strip,
	uint8_t pixel,
	uint8_t r,
	uint8_t g,
	uint8_t b
);


extern uint32_t
ledscape_wait(
	ledscape_t * const leds
);


extern void
ledscape_close(
	ledscape_t * const leds
);


/** Flip a rectangular frame buffer to map the LED matrices */
void
ledscape_matrix_remap(
	uint32_t * leds_out,
	const uint32_t * fb_in,
	const ledscape_matrix_config_t * config
);


/** Write with a fixed-width 8px font */
void
ledscape_printf(
	uint32_t * px,
	const size_t width,
	const uint32_t color,
	const char * fmt,
	...
);


/** Parse a config file */
ledscape_config_t *
ledscape_config(
	const char * filename
);

    
/***************************************************************************/
/** Structs moved from ledscape.c so that we can bypass the draw routines **/
/** Command structure shared with the PRU.
 *
 * This is mapped into the PRU data RAM and points to the
 * frame buffer in the shared DDR segment.
 *
 * Changing this requires changes in ws281x.p
 */
typedef struct
{
	// in the DDR shared with the PRU
	uintptr_t pixels_dma;

	// Length in pixels of the longest LED strip.
	unsigned num_pixels;

	// write 1 to start, 0xFF to abort. will be cleared when started
	volatile unsigned command;

	// will have a non-zero response written when done
	volatile unsigned response;

	// Panels per output (1-8)
	uint16_t panelCount;

    // Rows to output (8 : 1/8 scan,  16: 1/16 scan, etc...)
    uint16_t rowsPerOutput;
    
    uint32_t statEnable;
    
    // each bit has two values:
    //   1) How many cycles the display should be on
    //   2) How many "extra" cycles to keep the display off (dim)
    uint32_t brightInfo[16];
} __attribute__((__packed__)) ws281x_command_t;

struct ledscape
{
	ws281x_command_t * ws281x;
	pru_t * pru;
	unsigned width;
	unsigned height;
	unsigned panelCount;
    unsigned rowsPerOutput;
	unsigned frame_size;
	ledscape_config_t * config;
};
/***************************************************************************/


#ifdef __cplusplus
};
#endif

#endif
