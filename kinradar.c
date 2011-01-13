/*
 * Test Kinect/libfreenect program to display a radar-like distance cone.
 * Created Jan. 12, 2011
 * (C)2011 Mike Bourgeous
 * Distributed with no warranty under GPLv2 or later.
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


// Get a depth pixel from an 11-bit buffer stored in uint16_t
#define DPT(buf, x, y) (buf[(y) * FREENECT_FRAME_W + (x)])

// Application state (I wish freenect provided a user data struct for callbacks)
static float depth_lut[2048];
static int out_of_range = 0;
static int udiv = 80; // Grid horizontal (x or y) divisions
static int vdiv = 32; // Grid vertical (z) divisions
static unsigned int frame = 0; // Frame count
static float zmin = 0.5; // Near clipping plane in meters
static float zmax = 5.0; // Far clipping plane '' ''
static int ytop = 0; // Top image Y coordinate to consider
static int ybot = FREENECT_FRAME_H; // Bottom image Y coodrinate to consider
static float xworldmax; // Maximum X coordinate visible on grid
static float yworldmax; // Maximum Y coordinate visible on grid (if Y is shown instead of X)
static int done = 0; // Set to 1 to break main loop


static float xworld(int x, float z)
{
	// tan 35 ~= .70021
	return (float)(FREENECT_FRAME_W / 2 - x) * (.70021f / (FREENECT_FRAME_W / 2)) * z;
}

static float yworld(int y, float z)
{
	// TODO: Determine whether a separate function is necessary, or if
	// xworld() is enough for both X and Y.  I'm pretty sure that 26.25 is
	// incorrect for the vertical viewing angle, since the sensor is
	// planar.  At any rate, this function isn't really used yet.
	// tan 26.25 ~= .49315
	return (float)(y - FREENECT_FRAME_H / 2) * (.49315f / (FREENECT_FRAME_H / 2)) * z;
}

static int xworld_to_grid(float xworld)
{
	// At ximage=640 and z=zmax, xworld is zmax * .70021
	return (int)((xworld + xworldmax) * udiv * 0.5f / xworldmax);
}

static int yworld_to_grid(float yworld)
{
	return (int)((yworld + yworldmax) * udiv * 0.5f / yworldmax);
}

static int zworld_to_grid(float z)
{
	int val = (int)((z - zmin) * vdiv / (zmax - zmin));
	if(val < 0) {
		val = 0;
	}
	if(val >= vdiv) {
		val = vdiv - 1;
	}
	return val;
}

void depth(freenect_device *kn_dev, void *depthbuf, uint32_t timestamp)
{
	const uint16_t *buf = (uint16_t *)depthbuf;
	int barrier1 = 0xf9e8d7b6;
	int gridpop[vdiv][udiv]; // Point population count
	int barrier2 = 0x1337d00d;
	int oor_total = 0; // Out of range count
	int popmax; // Used for scaling pixel intensity
	int x, y, u, v, w;
	float z;

	// Initialize data structures
	memset(gridpop, 0, sizeof(gridpop));
	popmax = 0;

	// Fill in cone
	for(y = ytop; y < ybot; y++) {
		for(x = 0; x < FREENECT_FRAME_W; x++) {
			if(DPT(buf, x, y) == 2047) {
				oor_total++;
				continue;
			}

			z = depth_lut[DPT(buf, x, y)];

			if(z < zmin) {
				continue;
			}
			if(z > zmax) {
				continue;
			}

			u = xworld_to_grid(xworld(x, z));
			w = yworld_to_grid(yworld(y, z));
			v = zworld_to_grid(z);

			// XXX
			if(u < 0 || u >= udiv || v < 0 || v >= vdiv) {
				ERROR_OUT("(u, v) = (%d, %d) is out of range!\n", u, v);
				ERROR_OUT("x=%d xw=%f xwmax=%f z=%f zmax=%f\n",
						x, xworld(x, z), xworldmax, z, zmax);
				done = 1;
			}

			gridpop[v][u]++;
			if(gridpop[v][u] > popmax) {
				popmax = gridpop[v][u];
			}
		}
	}

	// Draw cone borders (this isn't perfect, but it's good enough)
	for(z = zmin; z < zmax; z += (zmax - zmin) / vdiv) {
		u = xworld_to_grid(xworld(0, z));
		v = zworld_to_grid(z);
		gridpop[v][u] = -1;
		u = xworld_to_grid(xworld(FREENECT_FRAME_W - 1, z));
		v = zworld_to_grid(z);
		gridpop[v][u] = -2;
	}

	// Display grid containing cone
	printf("\e[H\e[2J");
	INFO_OUT("time: %u frame: %d top: %d bottom: %d popmax: %d out: %d%%\n",
			timestamp, frame, ytop, ybot, popmax, oor_total * 100 / FREENECT_FRAME_PIX);

	if(popmax) {
		for(v = 0; v < vdiv; v++) {
			for(u = 0; u < udiv; u++) {
				int c = gridpop[v][u] * 20 / popmax;
				if(c > 5) {
					c = 5;
				}

				// Special values for cone borders
				if(gridpop[v][u] == -1) {
					c = 6;
				} else if(gridpop[v][u] == -2) {
					c = 7;
				}
				putchar(" .-+%8/\\"[c]);
			}
			putchar('\n');
		}
	}

	fflush(stdout);

	// Make LED red if more than 35% of the image is out of range (can't
	// set LED in callback for some reason)
	out_of_range = oor_total > FREENECT_FRAME_PIX * 35 / 100;

	frame++;

	if(barrier1 != 0xf9e8d7b6) {
		ERROR_OUT("Stack smashing on barrier 1!\n");
		ERROR_OUT("Stack smashing on barrier 1!\n");
		ERROR_OUT("Stack smashing on barrier 1!\n");
		done = 1;
	}
	if(barrier2 != 0x1337d00d) {
		ERROR_OUT("Stack smashing on barrier 2!\n");
		ERROR_OUT("Stack smashing on barrier 2!\n");
		ERROR_OUT("Stack smashing on barrier 2!\n");
		done = 1;
	}
}


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
	while((opt = getopt(argc, argv, "g:G:y:Y:z:Z:")) != -1) {
		switch(opt) {
			case 'g':
				// Horizontal (x or y) grid divisions
				udiv = atoi(optarg);
				break;
			case 'G':
				// Vertical (z) grid divisions
				vdiv = atoi(optarg);
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
				fprintf(stderr, "Usage: %s [-gG divisions] [-yY pixels] [-zZ distance]\n",
						argv[0]);
				fprintf(stderr, "Use any of:\n");
				fprintf(stderr, "\tg - Set horizontal (x or y) grid divisions\n");
				fprintf(stderr, "\tG - Set vertical (z) grid divisions\n");
				fprintf(stderr, "\ty - Set top of active area in screen pixels (0-%d)\n",
						FREENECT_FRAME_H - 1);
				fprintf(stderr, "\tY - Set bottom of active area in screen pixels (0-%d)\n",
						FREENECT_FRAME_H - 1);
				fprintf(stderr, "\tz - Set near clipping plane in meters (default 0.5)\n");
				fprintf(stderr, "\tZ - Set far clipping plane in meters (default 5.0)\n");
				return -1;
		}
	}

	init_lut();

	xworldmax = xworld(0, zmax);
	yworldmax = yworld(FREENECT_FRAME_H, zmax);

	INFO_OUT("zmax: %f xworldmax: %f zgridmax: %d xgridmin: %d xgridmax: %d\n",
			zmax, xworldmax, zworld_to_grid(zmax),
			xworld_to_grid(xworld(0, zmax)), xworld_to_grid(xworld(639, zmax)));

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

