/*
 * Test Kinect/libfreenect program to display grid-based stats.
 * Created Jan. 11, 2011
 * (C)2011 Mike Bourgeous
 *
 * Some ideas for more work:
 * 3D grid occupation - consider a 3D grid box as "occupied" if 20% or more of
 * the pixels in that 3D grid box's corresponding image-space 2D box are within
 * the range of that 3D grid box.  In this case the 3D grid boxes would
 * actually be pyramid sections in true 3D space, not cubes.
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
#define DPT(buf, x, y) (buf[(x) * FREENECT_FRAME_W + (y)])

// Convert pixel number to coordinates
#define PX_TO_X(pix) ((pix) % FREENECT_FRAME_W)
#define PX_TO_Y(pix) ((pix) / FREENECT_FRAME_W)

// Convert pixel number to grid entry
#define PX_TO_GRIDX(pix) (PX_TO_X(pix) * divisions / FREENECT_FRAME_W)
#define PX_TO_GRIDY(pix) (PX_TO_Y(pix) * divisions / FREENECT_FRAME_H)

// Application state (I wish freenect provided a user data struct for callbacks)
static float depth_lut[2048];
static int out_of_range = 0;
static int divisions = 6; // Grid divisions
static int boxwidth = 10; // Display grid box width, less border and padding
static int histrows = 8; // Number of histogram rows per grid box
static unsigned int frame = 0; // Frame count
static float zmin = 0.5; // Near clipping plane in meters for ASCII art mode
static float zmax = 5.0; // Far clipping plane '' ''

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

void repeat_char(int c, int count)
{
	int i;
	for(i = 0; i < count; i++) {
		putchar(c);
	}
}

// Prints horizontal border between grid rows
void grid_hline()
{
	int i;
	for(i = 0; i < divisions; i++) {
		putchar('+');
		repeat_char('-', boxwidth + 2);
	}
	puts("+");
}

// Prints a single row in a single grid box
void grid_box_row(const char *text)
{
	printf("| %*s ", boxwidth, text);
}

// Prints a formatted single row in a single grid box
void __attribute__((format(printf, 1, 2))) grid_entry(const char *format, ...)
{
	char buf[boxwidth + 1];
	va_list args;

	va_start(args, format);
	vsnprintf(buf, boxwidth + 1, format, args);
	va_end(args);

	grid_box_row(buf);
}

// Prints a horizontal bar chart element in a grid box
void grid_bar(int c, int percent)
{
	int charcount;

	if(percent > 100) {
		percent = 100;
	} else if(percent < 0) {
		percent = 0;
	}

	charcount = percent * boxwidth / 100;

	printf("| ");
	repeat_char(c, charcount);
	repeat_char(' ', boxwidth - charcount);
	putchar(' ');
}

void depth(freenect_device *kn_dev, void *depthbuf, uint32_t timestamp)
{
	uint16_t *buf = (uint16_t *)depthbuf;
	volatile int small_histogram[divisions][divisions][SM_HIST_SIZE];
	volatile int total[divisions][divisions];
	volatile uint16_t min[divisions][divisions];
	volatile uint16_t max[divisions][divisions];
	volatile uint16_t median[divisions][divisions];
	volatile float avg[divisions][divisions];
	volatile int oor_count[divisions][divisions];
	volatile int div_pix[divisions][divisions];
	int oor_total = 0; // Out of range count
	volatile int i, j, medcount, histcount;

	// Initialize data structures
	memset(small_histogram, 0, sizeof(small_histogram));
	memset(total, 0, sizeof(total));
	memset(min, 0xff, sizeof(min));
	memset(max, 0, sizeof(max));
	memset(oor_count, 0, sizeof(oor_count));
	memset(div_pix, 0, sizeof(oor_count));

	// Fill in grid stats
	for(i = 0; i < FREENECT_FRAME_PIX; i++) {
		int gridx = PX_TO_GRIDX(i);
		int gridy = PX_TO_GRIDY(i);

		div_pix[gridy][gridx]++; // TODO: Calculate this only once
		if(buf[i] == 2047) {
			oor_count[gridy][gridx]++;
			oor_total++;
			continue;
		}

		small_histogram[gridy][gridx][buf[i] * SM_HIST_SIZE / 1024]++;

		if(buf[i] < min[gridy][gridx]) {
			min[gridy][gridx] = buf[i];
		}
		if(buf[i] > max[gridy][gridx]) {
			max[gridy][gridx] = buf[i];
		}
		total[gridy][gridx] += buf[i];
	}

	// Calculate grid averages
	for(i = 0; i < divisions; i++) {
		for(j = 0; j < divisions; j++) {
			if(oor_count[i][j] < div_pix[i][j]) {
				avg[i][j] = (double)total[i][j] / (double)(div_pix[i][j] - oor_count[i][j]);

				// FIXME: Something is wrong with median calculation
				for(medcount = 0, histcount = 0; histcount < SM_HIST_SIZE; histcount++) {
					medcount += small_histogram[i][j][histcount];
					if(medcount >= (div_pix[i][j] - oor_count[i][j]) / 2) {
						break;
					}
				}
				median[i][j] = (histcount * 1024 + (SM_HIST_SIZE / 2)) / SM_HIST_SIZE;
			} else {
				min[i][j] = 2047;
				max[i][j] = 2047;
				avg[i][j] = 2047;
				median[i][j] = 2047;
			}
		}
	}

	// Display grid stats
	printf("\e[H\e[2J");
	INFO_OUT("time: %u frame: %d out: %d%%\n", timestamp, frame, oor_total * 100 / FREENECT_FRAME_PIX);
	for(i = 0; i < divisions; i++) {
		if(disp_mode != ASCII) {
			grid_hline();
		}

		switch(disp_mode) {
			case STATS:
				// This would be an interesting use of lambdas to return the
				// value for a given column, allowing a "grid_row" function to
				// be produced:
				// grid_row("Pix %d", int lambda(int j) { return div_pix[i][j]; })

				for(j = 0; j < divisions; j++) {
					grid_entry("Pix %d", div_pix[i][j]);
				}
				puts("|");

				for(j = 0; j < divisions; j++) {
					grid_entry("Avg %f", lutf(avg[i][j]));
				}
				puts("|");

				for(j = 0; j < divisions; j++) {
					grid_entry("Min %f", depth_lut[min[i][j]]);
				}
				puts("|");

				for(j = 0; j < divisions; j++) {
					grid_entry("Med ~%f", depth_lut[median[i][j]]);
				}
				puts("|");

				for(j = 0; j < divisions; j++) {
					grid_entry("Max %f", depth_lut[max[i][j]]);
				}
				puts("|");

				for(j = 0; j < divisions; j++) {
					grid_entry("Out %d%%", oor_count[i][j] * 100 / div_pix[i][j]);
				}
				puts("|");
				break;

			case HISTOGRAM:
				for(histcount = 0; histcount < histrows; histcount++) {
					for(j = 0; j < divisions; j++) {
						int l, val = 0;
						if(i == 2 && j == 4 && histcount == 0) { // XXX : this block is for debugging
							printf("\n");
							for(l = 0; l < SM_HIST_SIZE; l++) {
								INFO_OUT("%d (%f): %d\n",
										l * 1024 / SM_HIST_SIZE,
										depth_lut[l * 1024 / SM_HIST_SIZE],
										small_histogram[i][j][l]);
							}
							printf("\n");
						}
						for(l = 0; l < SM_HIST_SIZE / histrows; l++) {
							val += small_histogram[i][j][histcount + l];
						}
						grid_bar('*', val * 40 * histrows / div_pix[i][j]);
					}
					puts("|");
				}
				break;

			case ASCII:
				for(i = 0; i < divisions; i++) {
					for(j = 0; j < divisions; j++) {
						int c = (int)((depth_lut[min[i][j]] - zmin) * 4.0f / zmax);
						if(c > 5) {
							c = 5;
						} else if(c < 0) {
						       c = 0;
						}
						if(min[i][j] == 2047) {
							c = 6;
						}

						// 1st character is closest, 5th character farthest
						// 6th character is shown for out-of-range areas
						putchar("8%+-._ "[c]);
					}
					putchar('\n');
				}
				break;
		}
	}
	if(disp_mode != ASCII) {
		grid_hline();
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
	
	int rows = 40, cols = 96; // terminal size
	int opt;

	if(getenv("LINES")) {
		rows = atoi(getenv("LINES"));
	}
	if(getenv("COLUMNS")) {
		cols = atoi(getenv("COLUMNS"));
	}

	// Handle command-line options
	while((opt = getopt(argc, argv, "shag:z:Z:")) != -1) {
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

	boxwidth = (cols - 1) / divisions - 3;
	if(boxwidth < 10) {
		boxwidth = 10;
	}
	histrows = (rows - 2) / divisions - 1;
	
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

