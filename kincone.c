/*
 * Test Kinect/libfreenect program to display a radar-like distance cone.
 * Created Jan. 12, 2011
 * (C)2011 Mike Bourgeous
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>

#include <libfreenect/libfreenect.h>


#define INFO_OUT(...) {\
	printf("%s:%d: %s():\t", __FILE__, __LINE__, __FUNCTION__);\
	printf(__VA_ARGS__);\
}
#define ERROR_OUT(...) {\
	fprintf(stderr, "\e[0;1m%s:%d: %s():\t", __FILE__, __LINE__, __FUNCTION__);\
	fprintf(stderr, __VA_ARGS__);\
	fprintf(stderr, "\e[0m");\
}

#define ARRAY_SIZE(array) (sizeof((array)) / sizeof((array)[0]))


#define SM_HIST_SIZE	64

// Get a depth pixel from an 11-bit buffer stored in uint16_t
#define DPT(buf, x, y) (buf[(y) * FREENECT_FRAME_W + (x)])

// Convert pixel number to coordinates
#define PX_TO_X(pix) ((pix) % FREENECT_FRAME_W)
#define PX_TO_Y(pix) ((pix) / FREENECT_FRAME_W)

// Convert pixel number to grid entry
#define PX_TO_GRIDX(pix) (PX_TO_X(pix) * divisions / FREENECT_FRAME_W)
#define PX_TO_GRIDY(pix) (PX_TO_Y(pix) * divisions / FREENECT_FRAME_H)

// Application state (I wish freenect provided a user data struct for callbacks)
static float depth_lut[2048];
static int out_of_range = 0;
static int divisions = 48; // Grid divisions
static unsigned int frame = 0; // Frame count
static float zmin = 0.5; // Near clipping plane in meters for ASCII art mode
static float zmax = 5.0; // Far clipping plane '' ''
static int ytop = 0; // Top image Y coordinate to consider
static int ybot = FREENECT_FRAME_H; // Bottom image Y coodrinate to consider
static float xworldmax = 3.50104; // == xworld(640, zmax)
static float yworldmax = 2.46573; // == yworld(480, zmax) TODO: correct if yworld is changed

static enum {
	STATS,
	HISTOGRAM,
	ASCII,
} disp_mode = STATS;

static float lutf(float idx)
{
	int idx_int = (int)idx;
	float k = idx_int - idx;

	return depth_lut[idx_int] * k + depth_lut[idx_int + 1] * (1.0f - k);
}

static float xworld(int x, float z)
{
	// tan 35 ~= .70021
	return (float)(x - FREENECT_FRAME_W / 2) * (.70021f / 320.0f) * z;
}

static float yworld(int y, float z)
{
	// TODO: Determine whether a separate function is necessary, or if
	// xworld() is enough for both X and Y.  I'm pretty sure that 26.25 is
	// incorrect for the vertical viewing angle, since the sensor is planar
	// tan 26.25 ~= .49315
	return (float)(y - FREENECT_FRAME_H / 2) * (.49315f / 240.0f) * z;
}

static int xworld_to_grid(float xworld)
{
	// At ximage=640 and z=zmax, xworld is zmax * .70021
	return (int)(xworld * divisions / xworldmax + 0.5f); // +0.5f for rounding
}

static int yworld_to_grid(float yworld)
{
	return (int)(yworld * divisions / yworldmax + 0.5f);
}

static int zworld_to_grid(float z)
{
	int val = (int)((z - zmin) * divisions / zmax + 0.5f);
	if(val < 0) {
		return 0;
	}
	if(val >= divisions) {
		return divisions - 1;
	}
	return val;
}

void depth(freenect_device *kn_dev, void *depthbuf, uint32_t timestamp)
{
	uint16_t *buf = (uint16_t *)depthbuf;
	int gridpop[divisions][divisions]; // Point population count
	int oor_total = 0; // Out of range count
	int popmax = 0; // Used for scaling pixel intensity
	int x, y, u, v, w;
	float z;

	// Initialize data structures
	memset(gridpop, 0, sizeof(gridpop));

	// Fill in cone
	for(y = ytop; y < ybot; y++) {
		for(x = 0; x < FREENECT_FRAME_W; x++) {
			z = depth_lut[DPT(buf, x, y)];
			u = xworld_to_grid(xworld(x, z));
			v = zworld_to_grid(z);
			w = yworld_to_grid(yworld(y, z));

			gridpop[v][u]++;
			if(gridpop[v][u] > popmax) {
				popmax = gridpop[u][v];
			}
		}
	}

	// Display grid containing cone
	printf("\e[H\e[2J");
	INFO_OUT("time: %u frame: %d top: %d bottom: %d out: %d%%\n",
			timestamp, frame, ytop, ybot, oor_total * 100 / FREENECT_FRAME_PIX);

	for(v = 0; v < divisions; v++) {
		for(u = 0; u < divisions; u++) {
			int c = gridpop[v][u] * 5 / popmax;
			putchar(" .-+%8!"[c]); // The exclamation should never be displayed
		}
		putchar('\n');
	}

	fflush(stdout);

	// Make LED red if more than 35% of the image is out of range (can't
	// set LED in callback for some reason)
	out_of_range = oor_total > FREENECT_FRAME_PIX * 35 / 100;

	frame++;
}


static int done = 0;

void intr(int signum)
{
	INFO_OUT("Exiting due to signal %d (%s)\n", signum, strsignal(signum));
	done = 1;

	signal(signum, exit);
}

// http://groups.google.com/group/openkinect/browse_thread/thread/31351846fd33c78/e98a94ac605b9f21#e98a94ac605b9f21
void init_lut()
{
	int i;

	for(i = 0; i < 2048; i++) {
		depth_lut[i] = 0.1236 * tanf(i / 2842.5 + 1.1863);
	}
}

int main(int argc, char *argv[])
{
	freenect_context *kn;
	freenect_device *kn_dev;
	
	int opt;

	// Handle command-line options
	while((opt = getopt(argc, argv, "g:y:Y:z:Z:")) != -1) {
		switch(opt) {
			case 's':
				// Stats mode
				disp_mode = STATS;
				break;
			case 'h':
				// Histogram mode
				disp_mode = HISTOGRAM;
				break;
			case 'a':
				// ASCII art mode
				disp_mode = ASCII;
				break;
			case 'g':
				// Grid divisions
				divisions = atoi(optarg);
				break;
			case 'y':
				// Y top
				ytop = atoi(optarg);
				if(ytop < 0) {
					ytop = 0;
				} else if(ytop >= FREENECT_FRAME_H) {
					ytop = FREENECT_FRAME_H - 1;
				}
				break;
			case 'Y':
				// Y bottom
				ybot = atoi(optarg);
				if(ybot < 0) {
					ybot = 0;
				} else if(ybot >= FREENECT_FRAME_H) {
					ybot = FREENECT_FRAME_H - 1;
				}
				break;
			case 'z':
				// Near clipping
				zmin = atof(optarg);
				break;
			case 'Z':
				// Far clipping
				zmax = atof(optarg);
				break;
			default:
				fprintf(stderr, "Usage: %s -[sh] [-g divisions]\n", argv[0]);
				fprintf(stderr, "Use up to one of:\n");
				fprintf(stderr, "\ts - Stats mode (default)\n");
				fprintf(stderr, "\th - Histogram mode\n");
				fprintf(stderr, "\ta - ASCII art mode\n");
				fprintf(stderr, "Use any of:\n");
				fprintf(stderr, "\tg - Set grid divisions for both dimensions\n");
				fprintf(stderr, "\tz - Set near clipping plane in meters for ASCII art mode (default 0.5)\n");
				fprintf(stderr, "\tZ - Set far clipping plane in meters for ASCII art mode (default 5.0)\n");
				return -1;
		}
	}

	xworldmax = xworld(FREENECT_FRAME_W, zmax);
	yworldmax = yworld(FREENECT_FRAME_H, zmax);

	init_lut();

	if(signal(SIGINT, intr) == SIG_ERR ||
			signal(SIGTERM, intr) == SIG_ERR) {
		ERROR_OUT("Error setting signal handlers\n");
		return -1;
	}

	if(freenect_init(&kn, NULL) < 0) {
		ERROR_OUT("libfreenect init failed.\n");
		return -1;
	}

	INFO_OUT("Found %d Kinect devices.\n", freenect_num_devices(kn));

	if(freenect_num_devices(kn) == 0) {
		ERROR_OUT("No Kinect devices present.\n");
		return -1;
	}

	if(freenect_open_device(kn, &kn_dev, 0)) {
		ERROR_OUT("Error opening Kinect #0.\n");
		return -1;
	}

	freenect_set_tilt_degs(kn_dev, -5);
	freenect_set_led(kn_dev, LED_GREEN);
	freenect_set_depth_callback(kn_dev, depth);
	freenect_set_depth_format(kn_dev, FREENECT_DEPTH_11BIT);

	freenect_start_depth(kn_dev);

	int last_oor = out_of_range;
	while(!done && freenect_process_events(kn) >= 0) {
		if(last_oor != out_of_range) {
			freenect_set_led(kn_dev, out_of_range ? LED_BLINK_RED_YELLOW : LED_GREEN);
			last_oor = out_of_range;
		}
	}

	freenect_stop_depth(kn_dev);
	freenect_set_led(kn_dev, LED_OFF);
	freenect_close_device(kn_dev);
	freenect_shutdown(kn);

	return 0;
}

