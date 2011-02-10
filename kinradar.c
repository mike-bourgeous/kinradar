/*
 * Kinect/libfreenect program to display radar-like distance cones.
 * (C)2011 Mike Bourgeous
 * Distributed with no warranty under GPLv2 or later.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <signal.h>
#include <errno.h>
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
#define ERRNO_OUT(...) {\
	fprintf(stderr, "\e[0;1m%s:%d: %s():\t", __FILE__, __LINE__, __FUNCTION__);\
	fprintf(stderr, __VA_ARGS__);\
	fprintf(stderr, ": %d (%s)\e[0m\n", errno, strerror(errno));\
}

// Get a depth pixel from an 11-bit buffer stored in uint16_t
#define DPT(buf, x, y) (buf[(y) * FREENECT_FRAME_W + (x)])

struct grid_info {
	int udiv; // X or Y axis divisions
	int vdiv; // Z axis divisions

	float zmin; // Near clipping plane (millimeters)
	float zmax; // Far clipping plane
	float wmax; // Max X or Y coordinate visible on the grid

	int *gridpop_buffer;
	int **gridpop;
	int popmax;
	int bufsize;
};

struct kinradar_data {
	float depth_lut[2048];
	
	enum {
		SHOW_BOTH,
		SHOW_HORIZ,
		SHOW_VERT,
	} disp_mode;

	unsigned int out_of_range:1; // Whether to flash the LED
	unsigned int done:1; // Set to 1 to break the main loop

	struct grid_info xgrid; // Overhead view
	struct grid_info ygrid; // Side view
	int ytop; // Top image Y coordinate to consider
	int ybot; // Bottom image Y coordinate to consider

	unsigned int frame; // Frame count
};

struct kinradar_data *sigdata;

static float xworld(int x, float z)
{
	// tan 35 ~= .70021
	return (float)(FREENECT_FRAME_W / 2 - x) * (.70021f / (FREENECT_FRAME_W / 2)) * z;
}

static float yworld(int y, float z)
{
	return xworld(y + (FREENECT_FRAME_W - FREENECT_FRAME_H) / 2, z);
}

static int xyworld_to_grid(struct grid_info *grid, float w)
{
	return (int)((w + grid->wmax) * grid->udiv / (2.0f * grid->wmax));
}

static int zworld_to_grid(struct grid_info *grid, float z)
{
	int val = (int)((z - grid->zmin) * grid->vdiv / (grid->zmax - grid->zmin));
	if(val < 0) {
		val = 0;
	}
	if(val >= grid->vdiv) {
		val = grid->vdiv - 1;
	}
	return val;
}

static float zgrid_to_world(struct grid_info *grid, int zg)
{
	return zg * (grid->zmax - grid->zmin) / grid->vdiv + grid->zmin;
}

static void set_bold(int bold)
{
	static int last_bold = -1;

	if(last_bold != !!bold) {
		if(!!bold) {
			printf("\e[1m");
		} else {
			printf("\e[22m");
		}
		last_bold = !!bold;
	}
}

static void set_fgcolor(int fg)
{
	static int last_fg = -1;
	int new_fg = fg % 8 + 30;

	if(new_fg != last_fg) {
		printf("\e[%dm", new_fg);
		last_fg = new_fg;
	}
}

static void set_bgcolor(int bg)
{
	static int last_bg = -1;
	int new_bg = bg % 8 + 40;

	if(new_bg != last_bg) {
		printf("\e[%dm", new_bg);
		last_bg = new_bg;
	}
}

// Prevents having the same foreground and background
static void set_color(int bold, int fgcolor, int bgcolor)
{
	if(!bold && bgcolor == fgcolor) {
		if(bgcolor == 0) {
			fgcolor = 7;
		} else {
			fgcolor = 0;
		}
	}
	set_bold(bold);
	set_fgcolor(fgcolor);
	set_bgcolor(bgcolor);
}

static void reset_color()
{
	printf("\e[0;1;30m");
}

// This should really be done with curses or something
static void putchar_color(int bold, int fgcolor, int bgcolor, int c)
{
	set_color(bold, fgcolor, bgcolor);
	putchar(c);
}

// Prints a single grid cell's character
inline void print_cell(struct kinradar_data *data, int val, int scale)
{
	int c = val * 20 / scale;
	const char charset[] = " .-+%8/\\";
	const int fg[] = { 0, 0, 7, 7, 7, 7, 2, 2 };
	const int bold[] = { 1, 1, 0, 0, 1, 1, 0, 0 };

	if(c > 5) {
		c = 5;
	}

	// Special values for borders
	if(val == -1) {
		c = 6;
	} else if(val == -2) {
		c = 7;
	}

	putchar_color(bold[c], fg[c], 0, charset[c]);
}

// Displays the given grid of characters at the given zero-based cursor
// position.  If x < 0, the grid is printed without horizontal positioning.  If
// y < 0, the grid is printed at the cursor's current vertical position.  Grid
// cells are converted to character values by multiplying by 20 then dividing
// by popmax.  If clear is nonzero, then the remainder of each line to the
// right of the grid is cleared.  If transpose is nonzero, then u and v are
// swapped.
void print_grid(struct kinradar_data *data, struct grid_info *grid, int x, int y, int clear, int transpose)
{
	char prefix[16];
	char *suffix;
	int u, v;
	int scale;

	if(y >= 0) {
		printf("\e[%dH", y + 1);
	}

	if(x >= 0) {
		snprintf(prefix, sizeof(prefix), "\e[%dG", x + 1);
	} else {
		prefix[0] = 0;
	}

	if(grid->popmax) {
		scale = grid->popmax;
	} else {
		scale = 1;
	}

	if(clear) {
		suffix = "\e[K";
	} else {
		suffix = "";
	}

	if(transpose) {
		for(u = 0; u < grid->udiv; u++) {
			printf("%s", prefix);
			for(v = 0; v < grid->vdiv; v++) {
				print_cell(data, grid->gridpop[v][u], scale);
			}
			puts(suffix);
		}
	} else {
		for(v = 0; v < grid->vdiv; v++) {
			printf("%s", prefix);
			for(u = 0; u < grid->udiv; u++) {
				print_cell(data, grid->gridpop[v][u], scale);
			}
			puts(suffix);
		}
	}
}

void clear_grid(struct grid_info *grid)
{
	memset(grid->gridpop_buffer, 0, grid->bufsize);
	grid->popmax = 0;
}

void draw_grid_border(struct grid_info *grid)
{
	int u, v;
	float zw;
	float inc = (grid->zmax - grid->zmin) / grid->vdiv;

	for(v = 0; v < grid->vdiv; v++) {
		zw = zgrid_to_world(grid, v) + inc;
		u = xyworld_to_grid(grid, grid->wmax * zw / grid->zmax);

		if(u >= grid->udiv) {
			u = grid->udiv - 1;
		}

		grid->gridpop[v][u] = -2;

		u = xyworld_to_grid(grid, -grid->wmax * zw / grid->zmax);

		if(u < 0 || u >= grid->udiv) {
			ERROR_OUT("u %d out of range\n", u);
			abort();
		}

		grid->gridpop[v][u] = -1;
	}
}

void depth(freenect_device *kn_dev, void *depthbuf, uint32_t timestamp)
{
	struct kinradar_data *data = freenect_get_user(kn_dev);
	const uint16_t *buf = (uint16_t *)depthbuf;
	int oor_total = 0; // Out of range count
	int x, y, u, v;
	float xw, yw, zw;

	// Initialize data structures
	clear_grid(&data->xgrid);
	clear_grid(&data->ygrid);

	// Fill in cone
	for(y = data->ytop; y < data->ybot; y++) {
		for(x = 0; x < FREENECT_FRAME_W; x++) {
			if(DPT(buf, x, y) == 2047) {
				oor_total++;
				continue;
			}

			zw = data->depth_lut[DPT(buf, x, y)];

			if(zw < data->xgrid.zmin) {
				continue;
			}
			if(zw > data->xgrid.zmax) {
				continue;
			}

			xw = xworld(x, zw);
			yw = yworld(y, zw);

			u = xyworld_to_grid(&data->xgrid, xw);
			v = zworld_to_grid(&data->xgrid, zw);
			data->xgrid.gridpop[v][u]++;
			if(data->xgrid.gridpop[v][u] > data->xgrid.popmax) {
				data->xgrid.popmax = data->xgrid.gridpop[v][u];
			}

			u = xyworld_to_grid(&data->ygrid, yw);
			v = zworld_to_grid(&data->ygrid, zw);
			data->ygrid.gridpop[v][u]++;
			if(data->ygrid.gridpop[v][u] > data->ygrid.popmax) {
				data->ygrid.popmax = data->ygrid.gridpop[v][u];
			}
		}
	}

	// Draw cone borders
	draw_grid_border(&data->xgrid);
	draw_grid_border(&data->ygrid);

	// Display scene info
	printf("\e[H");
	INFO_OUT("\e[Ktime: %u frame: %d top: %d bottom: %d\n",
			timestamp, data->frame, data->ytop, data->ybot);
	INFO_OUT("\e[Kxpopmax: %d ypopmax: %d out: %d%%\n",
			data->xgrid.popmax, data->ygrid.popmax,
			oor_total * 100 / FREENECT_FRAME_PIX);

	// Display grid
	reset_color();
	if(data->disp_mode == SHOW_BOTH || data->disp_mode == SHOW_HORIZ) {
		print_grid(data, &data->xgrid, -1, 2, data->disp_mode == SHOW_HORIZ, 0);
	}
	if(data->disp_mode == SHOW_BOTH || data->disp_mode == SHOW_VERT) {
		print_grid(data, &data->ygrid,
				data->disp_mode == SHOW_VERT ? -1 : data->xgrid.udiv + 1, 2,
				1, 1);
	}
	printf("\e[m\e[K");

	fflush(stdout);

	data->out_of_range = oor_total > FREENECT_FRAME_PIX * 35 / 100;
	data->frame++;
}

// http://groups.google.com/group/openkinect/browse_thread/thread/31351846fd33c78/e98a94ac605b9f21#e98a94ac605b9f21
static void init_lut(float depth_lut[])
{
	int i;

	for(i = 0; i < 2048; i++) {
		depth_lut[i] = 0.1236 * tanf(i / 2842.5 + 1.1863);
	}
}

static void init_data(struct kinradar_data *data)
{
	memset(data, 0, sizeof(struct kinradar_data));

	init_lut(data->depth_lut);

	data->disp_mode = SHOW_BOTH;

	data->xgrid.udiv = 65;
	data->xgrid.vdiv = 32;
	data->xgrid.zmin = 0.0f;
	data->xgrid.zmax = 6.0f;
	data->xgrid.wmax = xworld(0, data->xgrid.zmax);

	data->ygrid.udiv = 32;
	data->ygrid.vdiv = 80;
	data->ygrid.zmin = 0.0f;
	data->ygrid.zmax = 6.0f;
	data->ygrid.wmax = yworld(FREENECT_FRAME_H - 1, data->ygrid.zmax);

	data->ytop = 0;
	data->ybot = FREENECT_FRAME_H;
}

static int alloc_grid(struct grid_info *grid)
{
	int i;

	grid->bufsize = sizeof(int) * grid->udiv * grid->vdiv;

	grid->gridpop_buffer = malloc(grid->bufsize);
	if(grid->gridpop_buffer == NULL) {
		ERRNO_OUT("Error allocating grid population buffer");
		return -1;
	}
	grid->gridpop = malloc(sizeof(int *) * grid->vdiv);
	if(grid->gridpop == NULL) {
		ERRNO_OUT("Error allocating grid population array");
		return -1;
	}

	for(i = 0; i < grid->vdiv; i++) {
		grid->gridpop[i] = grid->gridpop_buffer + i * grid->udiv;
	}

	return 0;
}

static int init_grids(struct kinradar_data *data)
{
	return (alloc_grid(&data->xgrid) || alloc_grid(&data->ygrid)) ? -1 : 0;
}


void intr(int signum)
{
	printf("\e[m");
	INFO_OUT("Exiting due to signal %d (%s)\n", signum, strsignal(signum));
	sigdata->done = 1;

	// Exit the program the next time this signal is received
	signal(signum, exit);
}

int main(int argc, char *argv[])
{
	struct kinradar_data data;
	freenect_context *kn;
	freenect_device *kn_dev;
	int ret = 0, opt;

	init_data(&data);
	sigdata = &data;

	// Handle command-line options
	while((opt = getopt(argc, argv, "g:G:y:Y:z:Z:hv")) != -1) {
		switch(opt) {
			case 'g':
				// Horizontal grid divisions
				data.xgrid.udiv = atoi(optarg);
				data.ygrid.vdiv = data.xgrid.udiv;
				break;
			case 'G':
				// Vertical grid divisions
				data.xgrid.vdiv = atoi(optarg);
				data.ygrid.udiv = data.xgrid.vdiv;
				break;
			case 'y':
				// Y top
				data.ytop = atoi(optarg);
				if(data.ytop < 0) {
					data.ytop = 0;
				} else if(data.ytop >= FREENECT_FRAME_H) {
					data.ytop = FREENECT_FRAME_H - 1;
				}
				break;
			case 'Y':
				// Y bottom
				data.ybot = atoi(optarg);
				if(data.ybot < 0) {
					data.ybot = 0;
				} else if(data.ybot >= FREENECT_FRAME_H) {
					data.ybot = FREENECT_FRAME_H - 1;
				}
				break;
			case 'z':
				// Near clipping
				data.xgrid.zmin = atof(optarg);
				data.ygrid.zmin = data.xgrid.zmin;
				break;
			case 'Z':
				// Far clipping
				data.xgrid.zmax = atof(optarg);
				data.ygrid.zmax = data.xgrid.zmax;
				break;
			case 'h':
				// Horizontal only
				data.disp_mode = SHOW_HORIZ;
				break;
			case 'v':
				// Vertical only
				data.disp_mode = SHOW_VERT;
				break;
			default:
				fprintf(stderr, "Usage: %s [-gG divisions] [-yY pixels] [-zZ distance] [-hv]\n",
						argv[0]);
				fprintf(stderr, "Use any of:\n");
				fprintf(stderr, "\tg - Set horizontal grid divisions\n");
				fprintf(stderr, "\tG - Set vertical grid divisions\n");
				fprintf(stderr, "\ty - Set top of active area in screen pixels (inclusive) (0-%d)\n",
						FREENECT_FRAME_H - 1);
				fprintf(stderr, "\tY - Set bottom of active area in screen pixels (exclusive) (0-%d)\n",
						FREENECT_FRAME_H);
				fprintf(stderr, "\tz - Set near clipping plane in meters (default 0.0)\n");
				fprintf(stderr, "\tZ - Set far clipping plane in meters (default 6.0)\n");
				fprintf(stderr, "\th - Show horizontal (overhead) view only\n");
				fprintf(stderr, "\tv - Show vertical (side) view only\n");
				fprintf(stderr, "Press Ctrl-C (or send SIGINT) to quit.\n");
				return -1;
		}
	}

	data.xgrid.wmax = xworld(0, data.xgrid.zmax);
	data.ygrid.wmax = yworld(FREENECT_FRAME_H - 1, data.ygrid.zmax);

	init_grids(&data);

	INFO_OUT("zmax: %f xworldmax: %f zgridmax: %d xgridmin: %d xgridmax: %d\n",
			data.xgrid.zmax, data.xgrid.wmax, zworld_to_grid(&data.xgrid, data.xgrid.zmax),
			xyworld_to_grid(&data.xgrid, xworld(0, data.xgrid.zmax)),
			xyworld_to_grid(&data.xgrid, xworld(639, data.xgrid.zmax)));

	INFO_OUT("yworldmax: %f ygridmin: %d ygridmax: %d\n", data.ygrid.wmax,
			xyworld_to_grid(&data.ygrid, yworld(479, data.ygrid.zmax)),
			xyworld_to_grid(&data.ygrid, yworld(0, data.ygrid.zmax)));

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

	freenect_set_user(kn_dev, &data);
	freenect_set_tilt_degs(kn_dev, -5);
	freenect_set_led(kn_dev, LED_GREEN);
	freenect_set_depth_callback(kn_dev, depth);
	freenect_set_depth_format(kn_dev, FREENECT_DEPTH_11BIT);

	freenect_start_depth(kn_dev);

	printf("\e[H\e[2J");

	int last_oor = data.out_of_range;
	while(!data.done) {
		ret = freenect_process_events(kn);
		if(ret && ret != LIBUSB_ERROR_INTERRUPTED) {
			// EINTR may occur when profiling
			break;
		}

		if(last_oor != data.out_of_range) {
			freenect_set_led(kn_dev, data.out_of_range ? LED_BLINK_RED_YELLOW : LED_GREEN);
			last_oor = data.out_of_range;
		}
	}

	freenect_stop_depth(kn_dev);
	freenect_set_led(kn_dev, LED_OFF);
	freenect_close_device(kn_dev);
	freenect_shutdown(kn);

	return 0;
}

