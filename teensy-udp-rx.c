/** \file
 *  UDP image packet receiver.
 *
 * Based on the HackRockCity LED Display code:
 * https://github.com/agwn/pyramidTransmitter/blob/master/LEDDisplay.pde
 *
 * Designed to clock data out to USB teensy3 serial devices, which require
 * bitslicing into the raw form for the OctoWS2811 firmware.
 *
 * Since the teensy devices might come and go, this allows them to go into
 * a disconnected state and will attempt to reconnect when the device port
 * re-appears.
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <inttypes.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include "util.h"

static int port = 9999;
static unsigned width = 10;
static unsigned height = 128;


/** Prepare one Teensy3 worth of image data.
 * Each Teensy handles 8 rows of data and needs the bits sliced into
 * each 8 rows.
 *
 * Since some of the pixels might have bad single channels,
 * allow them to be masked out entirely.
 */
void
bitslice(
	uint8_t * const out,
	const uint8_t * in,
	const unsigned width,
	const unsigned y_offset,
	const uint8_t * const bad_pixels
)
{
	// Reorder from RGB in the input to GRB in the output
	static const uint8_t channel_map[] = { 1, 0, 2 };

	// Skip to the starting row that we're processing
	in += y_offset * width * 3;

	for(unsigned x=0 ; x < width ; x++)
	{
		for (unsigned channel = 0 ; channel < 3 ; channel++)
		{
			const uint8_t mapped_channel
				= channel_map[channel];

			for (unsigned bit_num = 0 ; bit_num < 8 ; bit_num++)
			{
				uint8_t b = 0;
				const uint8_t mask = 1 << (7 - bit_num);

				for(unsigned y = 0 ; y < 8 ; y++)
				{
					const uint8_t bit_pos = 1 << y;
					if (bad_pixels[x] & bit_pos)
						continue;
					const uint8_t v
						= in[3*(x + y*width) + channel];
					if (v & mask)
						b |= bit_pos;
				}

				out[24*x + 8*mapped_channel + bit_num] = b;
			}
		}
	}
}


static int
udp_socket(
	const int port
)
{
	const int sock = socket(AF_INET, SOCK_DGRAM, 0);
	struct sockaddr_in addr = {
		.sin_family = AF_INET,
		.sin_port = htons(port),
		.sin_addr.s_addr = INADDR_ANY,
	};

	if (sock < 0)
		return -1;
	if (bind(sock, (const struct sockaddr*) &addr, sizeof(addr)) < 0)
		return -1;

	return sock;
}


/** Write all the bytes to a fd, even if there is a brief interruption.
 * \return number of bytes written or -1 on any fatal error.
 */
static ssize_t
write_all(
	const int fd,
	const void * const buf_ptr,
	const size_t len
)
{
	const uint8_t * const buf = buf_ptr;
	size_t offset = 0;

	while (offset < len)
	{
		const ssize_t rc = write(fd, buf + offset, len - offset);
		if (rc < 0)
		{
			if (errno == EAGAIN)
				continue;
			return -1;
		}

		if (rc == 0)
			return -1;

		offset += rc;
	}

	return len;
}
		

static int
serial_open(
	const char * const dev
)
{
	const int fd = open(dev, O_RDWR | O_NONBLOCK | O_NOCTTY, 0666);
	if (fd < 0)
		return -1;

	// Disable modem control signals
	struct termios attr;
	tcgetattr(fd, &attr);
	attr.c_cflag |= CLOCAL | CREAD;
	tcsetattr(fd, TCSANOW, &attr);

	return fd;
}


typedef struct
{
	const char * dev;
	unsigned id;
	int fd;
} teensy_dev_t;


typedef struct
{
	unsigned id;
	unsigned y_offset;
	teensy_dev_t * dev;
	uint8_t bad[256]; // which pixels are to be masked out
} teensy_strip_t;

// \todo read this from a file at run time
static teensy_strip_t strips[] = {
	{ .id = 14401, .y_offset = 0, .bad = { [2] = 1, } },
	{ .id = 14389, .y_offset = 8, .bad = { [3] = 1, [4] = 1, } },
	{ .id = 8987, .y_offset = 16, .bad = { [0] = 0xFF, } },
	{ .id = 8998, .y_offset = 24, .bad = { [1] = 0x80, } },
};


static teensy_dev_t teensy_devs[] = {
	{ "/dev/ttyACM0", 0, -1 },
	{ "/dev/ttyACM1", 0, -1 },
	{ "/dev/ttyACM2", 0, -1 },
	{ "/dev/ttyACM3", 0, -1 },
};


static const unsigned num_strips = sizeof(strips) / sizeof(*strips);
static const unsigned num_devs = sizeof(teensy_devs) / sizeof(*teensy_devs);
	

static teensy_strip_t *
strip_find(
	teensy_dev_t * const dev
)
{
	// Look through our table to find where this one goes
	for (unsigned j = 0 ; j < num_strips ; j++)
	{
		teensy_strip_t * const leds = &strips[j];
		if (leds->id != dev->id)
			continue;

		if (leds->dev != NULL)
		{
			fprintf(stderr, "FATAL: %s: Duplicate ID %u (%s)\n",
				dev->dev, dev->id, leds->dev->dev);
			return NULL;
		}

		leds->dev = dev;
		return leds;
	}

	fprintf(stderr, "FATAL: %s: Unknown ID %u\n", dev->dev, dev->id);
	return NULL;
}


static void
strip_close(
	teensy_strip_t * const strip
)
{
	teensy_dev_t * const dev = strip->dev;
	if (!dev)
		return;

	if (dev->fd >= 0)
		close(dev->fd);

	dev->fd = -1;
	dev->id = 0;
	strip->dev = NULL;
}


static int
teensy_open(
	teensy_dev_t * const dev
)
{
	dev->fd = serial_open(dev->dev);
	if (dev->fd < 0)
	{
		if (errno == ENOENT)
			return 0;

		warn("%s: Unable to open serial port: %s\n",
			dev->dev,
			strerror(errno)
		);

		// Something else went wrong
		goto fail;
	}

	// Find out the serial number of this teensy
	ssize_t rc;
	rc = write_all(dev->fd, "?", 1);
	if (rc < 0)
	{
		warn("%s: write failed: %s\n", dev->dev, strerror(errno));
		goto fail;
	}

	// \todo: Read until a newline or a timeout
	usleep(100000);
	char response[128];
	rc = read(dev->fd, response, sizeof(response)-1);
	if (rc < 0)
	{
		warn("%s: read failed: %s\n", dev->dev, strerror(errno));
		goto fail;
	}

	response[rc] = '\0';
	if (0) printf("read: '%s'\n", response);

	unsigned this_width;
	if (sscanf(response, "%d,%*d,%*d,%d", &this_width, &dev->id) != 2)
	{
		warn("%s: Unable to parse response: '%s'\n", dev->dev, response);
		goto fail;
	}

	printf("%s: ID %d width %d\n", dev->dev, dev->id, this_width);

	if (dev->id == 0)
	{
		warn("%s: dev id == 0.  NOT GOOD\n", dev->dev);
		goto fail;
	}

	if (this_width != width)
	{
		warn("%s: width %d != expected %d\n", dev->dev, this_width, width);
		goto fail;
	}

	teensy_strip_t * const leds = strip_find(dev);
	if (!leds)
		goto fail;

	return 1;

fail:
	dev->id = 0;
	if (dev->fd >= 0)
		close(dev->fd);
	return -1;
}


static void *
reopen_thread(
	void * const arg
)
{
	(void) arg;

	while (1)
	{
		for (unsigned i = 0 ; i < num_devs ; i++)
		{
			teensy_dev_t * const dev = &teensy_devs[i];
			if (dev->id != 0)
				continue;
			teensy_open(dev);
		}

		sleep(1);
	}

	return NULL;
}




int
main(
	int argc,
	char ** argv
)
{
	const int sock = udp_socket(port);
	if (sock < 0)
		die("socket port %d failed: %s\n", port, strerror(errno));

	pthread_t reopen_tid;
	if (pthread_create(&reopen_tid, NULL, reopen_thread, NULL) < 0)
		die("pthread create failed: %s\n", strerror(errno));

	const size_t image_size = width * height * 3;
	const size_t slice_size = width * 8 * 3 + 3;
	uint8_t * const slice = calloc(1, slice_size);

	uint8_t buf[65536];

	while (1)
	{
		const ssize_t rlen = recv(sock, buf, sizeof(buf), 0);
		if (rlen < 0)
			die("recv failed: %s\n", strerror(errno));

		if (buf[0] == '2')
		{
			// image type
			printf("image type: %.*s\n",
				(int) rlen - 1,
				&buf[1]
			);
			continue;
		}

		if (buf[0] != '1')
		{
			// What is it?
			warn_once("Unknown image type '%c' (%02x)\n",
				buf[0],
				buf[0]
			);
			continue;
		}

		if ((size_t) rlen != image_size + 1)
		{
			warn_once("WARNING: Received packet %zu bytes, expected %zu\n",
				rlen,
				image_size + 1
			);
		}

		// Header for the frame to the teensy indicating that it
		// is to be drawn immediately.
		slice[0] = '$';
		slice[1] = 0;
		slice[2] = 0;

		// Translate the image from packed RGB into sliced 24-bit
		// for each teensy.
		for (unsigned i = 0 ; i < num_strips ; i++)
		{
			teensy_strip_t * const strip = &strips[i];
			teensy_dev_t * const dev = strip->dev;

			// If it hasn't been opened yet, go onto the next one
			if (!dev)
				continue;

			bitslice(
				slice + 3,
				buf + 1,
				width,
				strip->y_offset,
				strip->bad
			);

			const ssize_t rc
				= write_all(dev->fd, slice, slice_size);

			if ((size_t) rc == slice_size)
				continue;

			if (rc < 0)
				warn("%s: write failed: %s\n",
					strip->dev,
					strerror(errno)
				);
			else
				warn("%s: short write %zu != %zu: %s\n",
					strip->dev,
					rc,
					slice_size,
					strerror(errno)
				);

			strip_close(strip);
		}
	}

	return 0;
}