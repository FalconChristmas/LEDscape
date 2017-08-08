/** \file
 * Userspace interface to the WS281x LED strip driver.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <unistd.h>
#include "ledscape.h"
#include "pru.h"

#define CONFIG_LED_MATRIX


/** GPIO pins used by the LEDscape.
 *
 * The device tree should handle this configuration for us, but it
 * seems horribly broken and won't configure these pins as outputs.
 * So instead we have to repeat them here as well.
 *
 * If these are changed, be sure to check the mappings in
 * ws281x.p!
 *
 * The RGB matrix uses a subset of these pins, although with
 * the HDMI disabled it might use quite a few more for the four
 * output version.
 *
 * \todo: Find a way to unify this with the defines in the .p file
 */
static const uint8_t gpios0[] = {
	23, 27, 22, 10, 9, 8, 26, 11, 30, 31, 5, 3, 4, 2, 14, 15
};

static const uint8_t gpios1[] = {
	13, 15, 12, 14, 29, 16, 17, 28, 18, 19,
};

static const uint8_t gpios2[] = {
	2, 5, 22, 23, 14, 12, 10, 8, 6, 3, 4, 1, 24, 25, 17, 16, 15, 13, 11, 9, 7,
};

static const uint8_t gpios3[] = {
	21, 19, 15, 14, 17, 16, 18, 20
};

#define ARRAY_COUNT(a) ((sizeof(a) / sizeof(*a)))


/*
 * Configure all of our output pins.
 * These must have also been set by the device tree overlay.
 * If they are not, some things will appear to work, but not
 * all the output pins will be correctly configured as outputs.
 */
static void
ledscape_gpio_init(void)
{
	for (unsigned i = 0 ; i < ARRAY_COUNT(gpios0) ; i++)
		pru_gpio(0, gpios0[i], 1, 0);
	for (unsigned i = 0 ; i < ARRAY_COUNT(gpios1) ; i++)
		pru_gpio(1, gpios1[i], 1, 0);
	for (unsigned i = 0 ; i < ARRAY_COUNT(gpios2) ; i++)
		pru_gpio(2, gpios2[i], 1, 0);
	for (unsigned i = 0 ; i < ARRAY_COUNT(gpios3) ; i++)
		pru_gpio(3, gpios3[i], 1, 0);
}


#if 0
/** Retrieve one of the two frame buffers. */
ledscape_frame_t *
ledscape_frame(
	ledscape_t * const leds,
	unsigned int frame
)
{
	if (frame >= 2)
		return NULL;

	return (ledscape_frame_t*)((uint8_t*) leds->pru->ddr + leds->frame_size * frame);
}
#endif

static uint8_t *
ledscape_remap(
	ledscape_t * const leds,
	uint8_t * const frame,
	unsigned x,
	unsigned y
)
{
#undef CONFIG_ZIGZAG
#ifdef CONFIG_ZIGZAG
	(void) leds;

	// each panel is 16x8
	// vertical panel number is y % 8 (which output line)
	// horizontal panel number is y % (16*8)
	// if y % 2 == 1, map backwards
	const unsigned panel_width = 16;
	const unsigned panel_height = 8;
	unsigned panel_num = x / panel_width;
	unsigned output_line = y / panel_height;
	unsigned panel_x = x % panel_width;
	unsigned panel_y = y % panel_height;
	unsigned panel_offset = panel_y * panel_width;

	// the even lines are forwards, the odd lines go backwards
	if (panel_y % 2 == 0)
	{
		panel_offset += panel_x;
	} else {
		panel_offset += panel_width - panel_x - 1;
	}

	return &frame[(panel_num*128 + panel_offset)*48*3 + output_line];
#else
	(void) leds;
	return &frame[x*48*3 + y];
#endif
}


/** Copy a 16x32 region from in to a 32x16 region of out.
 * If rot == 0, rotate -90, else rotate +90.
 */
static void
ledscape_matrix_panel_copy(
	uint8_t * const out,
	const uint32_t * const in,
	const ledscape_matrix_config_t * const config,
	const int rot
)
{
	const size_t row_stride = LEDSCAPE_MATRIX_OUTPUTS*3*2;
	const size_t row_len = config->leds_width*row_stride;

	for (int x = 0 ; x < config->panel_width ; x++)
	{
		for (int y = 0 ; y < config->panel_height ; y++)
		{
			int ix, iy;
			if (rot == 0)
			{
				// no rotation == (0,0) => (0,0)
				ix = x;
				iy = y;
			} else
			if (rot == 1)
			{
				// rotate +90 (0,0) => (0,15)
				ix = config->panel_height-1 - y;
				iy = x;
			} else
			if (rot == 2)
			{
				// rotate -90 (0,0) => (31,0)
				ix = y;
				iy = config->panel_width-1 - x;
			} else
			if (rot == 3)
			{
				// flip == (0,0) => (31,15)
				ix = config->panel_width-1 - x;
				iy = config->panel_height-1 - y;
			} else
			{
				// barf
				ix = iy = 0;
			}

			const uint32_t * const col_ptr = &in[ix + config->width*iy];
			const uint32_t col = *col_ptr;

			// the top half and bottom half of the panels
			// are squished together in the output since
			// they are drawn simultaneously.
			//uint8_t * const pix = &out[x*row_stride + (y/8)*3 + (y%8)*row_len];
			uint8_t * const pix = &out[x*row_stride + (y/(config->panel_height/2))*3 + (y%(config->panel_height/2))*row_len];

			pix[0] = (col >> 16) & 0xFF; // red
			pix[1] = (col >>  8) & 0xFF; // green
			pix[2] = (col >>  0) & 0xFF; // blue
			//printf("%d,%d => %p %p %08x\n", x, y, pix, col_ptr, col);
		}
	}
}

static inline uint8_t mapColor(uint8_t v, uint8_t bits) {
    if (bits == 6 && (v == 3 || v == 2)) {
        return 4;
    }
    if (bits == 7 && v == 1) {
        return 2;
    }
    return v;
}

static void
ledscape_map_matrix_bits(
     ledscape_t * const leds,
     uint8_t * din,
     uint8_t * out
)
{
    const ledscape_matrix_config_t * const config
        = &leds->config->matrix_config;
    
    uint8_t *rowin = din;
    uint8_t *rowout = out;
    int maxRows = config->rowsPerOutput;
    if (maxRows <= 0 || maxRows > 32) {
        maxRows = 8;
    }
    uint8_t bitsToOutput = config->bitsToOutput;
    int interleave = 0;
    int row1 = 1;
    int bytesPerRow = LEDSCAPE_MATRIX_OUTPUTS * LEDSCAPE_MATRIX_PANELS * 3 * 2 * config->panel_width;
    int interleaveOffset = 0;
    if (maxRows * 4 == config->panel_height) {
        //need to interleave 2:1 (32x16 1/4 scan  or  32x32 1/8 scan or 40x20 1/5 scan,  etc...)
        interleave = 1;
        interleaveOffset = bytesPerRow * maxRows;
    } else if (maxRows * 8 == config->panel_height) {
        //need to interleave 4:1 (32x16 1/2 scan  or  32x32 1/4)
        interleave = 3;
        interleaveOffset = bytesPerRow * maxRows * interleave;
    }
    for (int row = 0; row < maxRows; row++, rowin += bytesPerRow) {
        for (int bit = 8; bit > (8 - config->bitsToOutput); ) {
            --bit;
            /*
            if (row == 0 && bit == 7) {
                printf("\nbit %d\n", bit);
            }
             */
            uint8_t mask = 1 << bit;

            uint8_t red1[8];
            uint8_t green1[8];
            uint8_t blue1[8];
            uint8_t red2[8];
            uint8_t green2[8];
            uint8_t blue2[8];
            memset(red1, 0, 8);
            memset(green1, 0, 8);
            memset(blue1, 0, 8);
            memset(red2, 0, 8);
            memset(green2, 0, 8);
            memset(blue2, 0, 8);

            int curout = 0;
            int inbit = 1;
            int curbit = 0;
            int offset = 0;
            /*
            if (config->initialSkip != 0) {
                rowout += config->initialSkip;
                offset = config->initialSkip * 8;
            }
            */
            do {
                
                if (mapColor(rowin[offset + interleaveOffset], bitsToOutput) & mask) {
                    red1[curout] |= inbit;
                }
                if (mapColor(rowin[offset + interleaveOffset + 1], bitsToOutput) & mask) {
                    green1[curout] |= inbit;
                }
                if (mapColor(rowin[offset + interleaveOffset + 2], bitsToOutput) & mask) {
                    blue1[curout] |= inbit;
                }
                if (mapColor(rowin[offset + interleaveOffset + 3], bitsToOutput) & mask) {
                    red2[curout] |= inbit;
                }
                if (mapColor(rowin[offset + interleaveOffset + 4], bitsToOutput) & mask) {
                    green2[curout] |= inbit;
                }
                if (mapColor(rowin[offset + interleaveOffset + 5], bitsToOutput) & mask) {
                    blue2[curout] |= inbit;
                }
                offset += 6;
                curout++;
                if (curout == 8) {
                    curout = 0;
                    curbit++;
                    inbit = 1 << curbit;
                    if (curbit == 8) {
                        if (interleave) {
                            if (!row1) {
                                row1 = interleave;
                                interleaveOffset = bytesPerRow * maxRows * interleave;
                            } else {
                                row1--;
                                interleaveOffset -= bytesPerRow * maxRows;
                                offset -= 6 * 8 * 8;
                            }
                        }

                        
                        for (int x = 0; x < 8; x++) {
                            rowout[0] = red1[x];
                            rowout[1] = green1[x];
                            rowout[2] = blue1[x];
                            rowout[3] = red2[x];
                            rowout[4] = green2[x];
                            rowout[5] = blue2[x];
                            
                            /*
                            if (row == 0 && bit == 7) {
                                printf("%2X ", rowout[0]);
                            }
                             */
                            rowout += 6;
                        }
                        memset(red1, 0, 8);
                        memset(green1, 0, 8);
                        memset(blue1, 0, 8);
                        memset(red2, 0, 8);
                        memset(green2, 0, 8);
                        memset(blue2, 0, 8);
                        
                        curbit = 0;
                        inbit = 1;
                    }
                }
                
            } while (offset < bytesPerRow);
        }
        for (int x = 0; x < (8 - config->bitsToOutput); x++) {
            rowout += 6 * LEDSCAPE_MATRIX_PANELS * LEDSCAPE_MATRIX_OUTPUTS * config->panel_width / 8;
        }
    }
}

static void printStats(uint32_t *stats, int h) {
    FILE *rfile;
    rfile=fopen("/tmp/framerates.txt","w");
    fprintf(rfile, "DDR  %X\n", stats[0]);
    stats++;
    fprintf(rfile, "Width  %X\n", stats[0]);
    stats++;
    fprintf(rfile, "  cmd: %d   response: %d      initialSkip: %d   rowsPerOutput: %d  stats: %d\n", stats[0], stats[1], stats[2] & 0xFFFF, (stats[2] >> 16), stats[3]);
    stats += 4;
    for (int x = 7; x >= 0; x--) {
        fprintf(rfile, "DV: %d    %8X   %8X\n", x, stats[0], stats[1]);
        stats += 2;
    }
    
    for (int x = 0; x < (8 * h / 2); x++) {
        fprintf(rfile, "%2d   %8X   %8X   %8X\n", x, stats[0], stats[1], stats[2]);
        stats += 3;
    }
    fclose(rfile);
}
static void dumpData(uint8_t *data1, uint8_t *data2, size_t sz) {
    static int frame = 0;
    
    if (frame == 0) {
        FILE *rfile;
        rfile=fopen("/tmp/data.dump.orig","wb");
        fwrite(data1, sz, 1, rfile);
        fclose(rfile);

        rfile=fopen("/tmp/data.dump.mapped","wb");
        fwrite(data2, sz, 1, rfile);
        fclose(rfile);
    }
    frame++;
    if (frame == 20) {
        frame = 0;
    }
    
}

static void
ledscape_matrix_draw(
	ledscape_t * const leds,
	const void * const buffer
)
{
	static unsigned frame = 0;
	const uint32_t * const in = buffer;
	uint8_t * const dout = leds->pru->ddr + leds->frame_size * frame;
    uint8_t * out = malloc(leds->frame_size);

	// matrix is re-packed such that a 6-byte read will bring in
	// the brightness values for all six outputs of a given panel.
	// this means that the rows stride 16 * 3 pixels at a time.
	// 
	// this way the PRU can read all sixteen output pixels in
	// one LBBO and clock them out.
	// while there are eight output chains, there are two simultaneous
	// per output chain.
	const ledscape_matrix_config_t * const config
		= &leds->config->matrix_config;

	const size_t panel_stride = config->panel_width*2*3*LEDSCAPE_MATRIX_OUTPUTS;

	for (unsigned i = 0 ; i < LEDSCAPE_MATRIX_OUTPUTS ; i++)
	{
		for (unsigned j = 0 ; j < LEDSCAPE_MATRIX_PANELS ; j++)
		{
			const ledscape_matrix_panel_t * const panel
				= &config->panels[i][j];

			// the start of the panel in the input
			// is defined by the panel's xy coordinate
			// and the width of the input image.
			const uint32_t * const ip
				= &in[panel->x + panel->y*config->width];

			// the start of the panel's output is defined
			// by the current output panel number and the total
			// number of panels in the chain.
			uint8_t * const op = &out[6*i + j*panel_stride];
		
			// copy the top half of this matrix
			ledscape_matrix_panel_copy(
				op,
				ip,
				config,
				panel->rot
			);
		}
	}
    
    uint32_t *stats = (uint32_t*)leds->pru->data_ram;
    if (stats[5]) {
        printStats(stats, config->panel_height);
    }

    ledscape_map_matrix_bits(leds, out, dout);
    //memcpy(dout, out, leds->frame_size);
    //dumpData(out, dout, leds->frame_size);
    
    free(out);
	leds->ws281x->pixels_dma = leds->pru->ddr_addr + leds->frame_size * frame;
	// disable double buffering for now
	//frame = (frame + 1) & 1;
}


/** Translate the RGBA buffer to the correct output type and
 * initiate the transfer of a frame to the LED strips.
 *
 * Matrix drivers shuffle to have consecutive bits, ws281x do bit slicing.
 */
void
ledscape_strip_draw(
	ledscape_t * const leds,
	const void * const buffer
)
{
	static unsigned frame = 0;
	const uint32_t * const in = buffer;
	uint8_t * const out = leds->pru->ddr + leds->frame_size * frame;

	// Translate the RGBA frame into G R B, sliced by color
	// only 48 outputs currently supported
	const unsigned pru_stride = 48;
	for (unsigned y = 0 ; y < leds->height ; y++)
	{
		const uint32_t * const row_in = &in[y*leds->width];
		for (unsigned x = 0 ; x < leds->width ; x++)
		{
			uint8_t * const row_out
				= ledscape_remap(leds, out, x, y);
			uint32_t p = row_in[x];
			row_out[0*pru_stride] = (p >>  8) & 0xFF; // green
			row_out[1*pru_stride] = (p >> 16) & 0xFF; // red
			row_out[2*pru_stride] = (p >>  0) & 0xFF; // blue
		}
	}
	
	// Wait for any current command to have been acknowledged
	while (leds->ws281x->command)
		;

	// Update the pixel data and send the start
	leds->ws281x->pixels_dma
		= leds->pru->ddr_addr + leds->frame_size * frame;
//	frame = (frame + 1) & 1;

	// Send the start command
	leds->ws281x->command = 1;
}


/** Wait for the current frame to finish transfering to the strips.
 * \returns a token indicating the response code.
 */
uint32_t
ledscape_wait(
	ledscape_t * const leds
)
{
	while (1)
	{
		uint32_t response = leds->ws281x->response;
		if (!response)
			continue;
		leds->ws281x->response = 0;
		return response;
	}
}


ledscape_t *
ledscape_matrix_init(
	ledscape_config_t * const config_union,
	int no_pru_init,
	int pru_number,
	const char * pru_program
)
{
	ledscape_matrix_config_t * const config = &config_union->matrix_config;
	pru_t * const pru = pru_init(pru_number);
	const size_t frame_size = config->panel_width * config->panel_height * 3 * LEDSCAPE_MATRIX_OUTPUTS * LEDSCAPE_MATRIX_PANELS;

	ledscape_t * const leds = calloc(1, sizeof(*leds));

	*leds = (ledscape_t) {
		.config		= config_union,
		.pru		= pru,
		.width		= config->leds_width,
		.height		= config->leds_height,
		.initialSkip = config->initialSkip,
        .rowsPerOutput = config->rowsPerOutput,
        .bitsToOutput = config->bitsToOutput,
		.ws281x		= pru->data_ram,
		.frame_size	= frame_size,
	};
    
	*(leds->ws281x) = (ws281x_command_t) {
		.pixels_dma	= 0, // will be set in draw routine
		.num_pixels	= (config->leds_width * 3) * 16,
		.initialSkip	= config->initialSkip,
        .rowsPerOutput = config->rowsPerOutput,
        .bitsToOutput = config->bitsToOutput,
		.command	= 0,
		.response	= 0,
	};

    for (int x = 0; x < 16; x++) {
        leds->ws281x->brightInfo[x] = 0;
    }

	//ledscape_gpio_init();

	// Initiate the PRU program
    if (!no_pru_init) {
		pru_exec(pru, pru_program);

        // Watch for a done response that indicates a proper startup
        // \todo timeout if it fails
        printf("waiting for response\n");
        while (!leds->ws281x->response)
            ;
        printf("got response\n");
    }

	return leds;
}


ledscape_t *
ledscape_strip_init(
	ledscape_config_t * const config_union,
	int no_pru_init,
	int pru_number,
	const char * pru_program
)
{
	ledscape_strip_config_t * const config = &config_union->strip_config;
	pru_t * const pru = pru_init(pru_number);
	const size_t frame_size = 48 * config->leds_width * 8 * 3;

	printf("frame-size %zu, ddr-size=%zu\n", frame_size, pru->ddr_size);
#if 0
	if (2 *frame_size > pru->ddr_size)
		die("Pixel data needs at least 2 * %zu, only %zu in DDR\n",
			frame_size,
			pru->ddr_size
		);
#endif

	ledscape_t * const leds = calloc(1, sizeof(*leds));

	*leds = (ledscape_t) {
		.config		= config_union,
		.pru		= pru,
		.width		= config->leds_width,
		.height		= 48, // only 48 are supported, config->leds_height,
		.ws281x		= pru->data_ram,
		.frame_size	= frame_size,
	};

	// LED strips, not matrix output
	*(leds->ws281x) = (ws281x_command_t) {
		.pixels_dma	= 0, // will be set in draw routine
		.num_pixels	= config->leds_width, // * leds->height,
		.command	= 0,
		.response	= 0,
	};

	printf("pixels: %d\n", leds->ws281x->num_pixels);

	//ledscape_gpio_init();

	// Initiate the PRU program
	if (!no_pru_init)
		pru_exec(pru, pru_program);

	// Watch for a done response that indicates a proper startup
	// \todo timeout if it fails
	printf("waiting for response\n");
	while (!leds->ws281x->response)
		;
	printf("got response\n");

	return leds;
}


ledscape_t *
ledscape_init(
	ledscape_config_t * const config,
	int no_pru_init
)
{
	switch (config->type)
	{
		case LEDSCAPE_MATRIX:
			return ledscape_matrix_init(config, no_pru_init, 0, "./lib/matrix.bin");
		case LEDSCAPE_STRIP:
			return ledscape_strip_init(config, no_pru_init, 0, "./lib/ws281x.bin");
		default:
			fprintf(stderr, "unknown config type %d\n", config->type);
			return NULL;
	}
}


void
ledscape_draw(
	ledscape_t * const leds,
	const void * const buffer
)
{
	switch (leds->config->type)
	{
		case LEDSCAPE_MATRIX:
			ledscape_matrix_draw(leds, buffer);
			break;
		case LEDSCAPE_STRIP:
			ledscape_strip_draw(leds, buffer);
			break;
		default:
			fprintf(stderr, "unknown config type %d\n", leds->config->type);
			break;
	}
}


void
ledscape_close(
	ledscape_t * const leds
)
{
	// Signal a halt command
    switch (leds->config->type)
    {
        case LEDSCAPE_MATRIX:
            leds->ws281x->pixels_dma = 0xFF;
            break;
        case LEDSCAPE_STRIP:
            leds->ws281x->command = 0xFF;
            break;
        default:
            fprintf(stderr, "unknown config type %d\n", leds->config->type);
            break;
    }
	pru_close(leds->pru);
}


void
ledscape_set_color(
	ledscape_frame_t * const frame,
	uint8_t strip,
	uint8_t pixel,
	uint8_t r,
	uint8_t g,
	uint8_t b
)
{
	ledscape_pixel_t * const p = &frame[pixel].strip[strip];
	p->r = r;
	p->g = g;
	p->b = b;
}



extern const uint8_t fixed_font[][5];

void
ledscape_draw_char(
	uint32_t * px,
	const size_t width,
	const uint32_t color,
	char c
)
{
	if (c < 0x20 || c > 127)
		c = '?';

	const uint8_t* const f = fixed_font[c - 0x20];
	for (int i = 0 ; i < 5 ; i++, px++)
	{
		uint8_t bits = f[i];
		for (int j = 0 ; j < 7 ; j++, bits >>= 1)
			px[j*width] = bits & 1 ? color : 0;
	}
}


/** Write with a fixed-width 8px font */
void
ledscape_printf(
	uint32_t * px,
	const size_t width,
	const uint32_t color,
	const char * fmt,
	...
)
{
	char buf[128];
	va_list ap;
	va_start(ap, fmt);
	int len = vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	(void) len;
	uint32_t * start = px;

	//printf("%p => '%s'\n", px, buf);
	for (unsigned i = 0 ; i < sizeof(buf) ; i++)
	{
		char c = buf[i];
		if (!c)
			break;
		if (c == '\n')
		{
			px = start = start + 8 * width;
			continue;
		}

		ledscape_draw_char(px, width, color, c);
		px += 6;
	}
}


/** Default ledscape config */
#define DEFAULT_MATRIX(i) { \
		{ 0*32, i*16, 0 }, \
		{ 1*32, i*16, 0 }, \
		{ 2*32, i*16, 0 }, \
		{ 3*32, i*16, 0 }, \
		{ 4*32, i*16, 0 }, \
		{ 5*32, i*16, 0 }, \
		{ 6*32, i*16, 0 }, \
		{ 7*32, i*16, 0 }, \
	} \

ledscape_config_t ledscape_matrix_default = {
	.matrix_config = {
		.type		= LEDSCAPE_MATRIX,
		.width		= 256,
		.height		= 128,
		.panel_width	= 32,
		.panel_height 	= 16,
		.leds_width	= 256,
		.leds_height	= 128,
		.panels		= {
			DEFAULT_MATRIX(0),
			DEFAULT_MATRIX(1),
			DEFAULT_MATRIX(2),
			DEFAULT_MATRIX(3),
			DEFAULT_MATRIX(4),
			DEFAULT_MATRIX(5),
			DEFAULT_MATRIX(6),
			DEFAULT_MATRIX(7),
		},
	},
};
