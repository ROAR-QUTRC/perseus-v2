/*
 * Mandelbrot Set - CPU Implementation
 * ASCII visualization with ANSI 256-color support
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <math.h>

// Default parameters
#define DEFAULT_WIDTH 80
#define DEFAULT_HEIGHT 40
#define DEFAULT_MAX_ITER 100

// Mandelbrot view bounds
#define X_MIN -2.5
#define X_MAX 1.0
#define Y_MIN -1.0
#define Y_MAX 1.0

// Character palette for density rendering (10 levels)
static const char *CHARS = " .:-=+*#%@";

// Get elapsed time in milliseconds
double get_time_ms(struct timeval *start, struct timeval *end) {
    return (end->tv_sec - start->tv_sec) * 1000.0 +
           (end->tv_usec - start->tv_usec) / 1000.0;
}

// Map iteration count to ANSI 256-color code
int iter_to_color(int iter, int max_iter) {
    if (iter == max_iter) {
        return 0;  // Black for inside the set
    }

    // Normalize iteration to 0-1 range
    double t = (double)iter / max_iter;

    // Color gradient: deep blue -> cyan -> green -> yellow -> white
    if (t < 0.1) {
        // Dark blue (colors 17-21)
        return 17 + (int)(t * 10 * 4);
    } else if (t < 0.3) {
        // Blue to cyan (colors 21-51)
        return 21 + (int)((t - 0.1) * 5 * 30);
    } else if (t < 0.5) {
        // Cyan to green (colors 51-46)
        return 51 - (int)((t - 0.3) * 5 * 5);
    } else if (t < 0.7) {
        // Green to yellow (colors 46-226)
        return 46 + (int)((t - 0.5) * 5 * 180);
    } else {
        // Yellow to white (colors 226-231)
        return 226 + (int)((t - 0.7) * 3.33 * 5);
    }
}

// Map iteration count to character
char iter_to_char(int iter, int max_iter) {
    if (iter == max_iter) {
        return ' ';  // Inside the set
    }
    int idx = (iter * 9) / max_iter;
    if (idx > 9) idx = 9;
    return CHARS[idx];
}

// Compute Mandelbrot escape iterations for a point
int mandelbrot(double x0, double y0, int max_iter) {
    double x = 0.0, y = 0.0;
    int iter = 0;

    while (x*x + y*y <= 4.0 && iter < max_iter) {
        double xtemp = x*x - y*y + x0;
        y = 2.0*x*y + y0;
        x = xtemp;
        iter++;
    }

    return iter;
}

// Render a single line and return timing
void render_line(int py, int width, int height, int max_iter, int use_color,
                 double x_scale, double y_scale) {
    double y0 = Y_MAX - py * y_scale;

    for (int px = 0; px < width; px++) {
        double x0 = X_MIN + px * x_scale;
        int iter = mandelbrot(x0, y0, max_iter);

        if (use_color) {
            int color = iter_to_color(iter, max_iter);
            char c = iter_to_char(iter, max_iter);
            printf("\033[38;5;%dm%c\033[0m", color, c);
        } else {
            printf("%c", iter_to_char(iter, max_iter));
        }
    }
}

// Compute-only mode (for timing without rendering overhead)
void compute_mandelbrot(int width, int height, int max_iter, int *results) {
    double x_scale = (X_MAX - X_MIN) / width;
    double y_scale = (Y_MAX - Y_MIN) / height;

    for (int py = 0; py < height; py++) {
        double y0 = Y_MAX - py * y_scale;

        for (int px = 0; px < width; px++) {
            double x0 = X_MIN + px * x_scale;
            results[py * width + px] = mandelbrot(x0, y0, max_iter);
        }
    }
}

void print_usage(const char *prog_name) {
    printf("Usage: %s [OPTIONS]\n", prog_name);
    printf("\nOptions:\n");
    printf("  -w, --width N      Set width (default: %d)\n", DEFAULT_WIDTH);
    printf("  -h, --height N     Set height (default: %d)\n", DEFAULT_HEIGHT);
    printf("  -i, --iterations N Set max iterations (default: %d)\n", DEFAULT_MAX_ITER);
    printf("  -n, --no-color     Disable color output\n");
    printf("  -t, --timing-only  Only output timing (for benchmarks)\n");
    printf("  -s, --stream       Stream output line by line (for race mode)\n");
    printf("  --help             Show this help message\n");
}

int main(int argc, char *argv[]) {
    int width = DEFAULT_WIDTH;
    int height = DEFAULT_HEIGHT;
    int max_iter = DEFAULT_MAX_ITER;
    int use_color = 1;
    int timing_only = 0;
    int stream_mode = 0;

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-w") == 0 || strcmp(argv[i], "--width") == 0) {
            if (i + 1 < argc) width = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--height") == 0) {
            if (i + 1 < argc) height = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--iterations") == 0) {
            if (i + 1 < argc) max_iter = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-n") == 0 || strcmp(argv[i], "--no-color") == 0) {
            use_color = 0;
        } else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--timing-only") == 0) {
            timing_only = 1;
        } else if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--stream") == 0) {
            stream_mode = 1;
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    double x_scale = (X_MAX - X_MIN) / width;
    double y_scale = (Y_MAX - Y_MIN) / height;

    struct timeval start, end, line_end;
    gettimeofday(&start, NULL);

    if (stream_mode) {
        // Stream mode: output each line immediately with timing
        for (int py = 0; py < height; py++) {
            render_line(py, width, height, max_iter, use_color, x_scale, y_scale);
            gettimeofday(&line_end, NULL);
            double elapsed = get_time_ms(&start, &line_end);
            // Output line terminator with current elapsed time
            printf("|%.1f\n", elapsed);
            fflush(stdout);
        }
        gettimeofday(&end, NULL);
        printf("TIME_MS:%.2f\n", get_time_ms(&start, &end));
    } else if (timing_only) {
        // Compute only, no rendering
        int *results = (int *)malloc(width * height * sizeof(int));
        if (!results) {
            fprintf(stderr, "Error: Failed to allocate memory\n");
            return 1;
        }
        compute_mandelbrot(width, height, max_iter, results);
        gettimeofday(&end, NULL);
        printf("TIME_MS:%.2f\n", get_time_ms(&start, &end));
        free(results);
    } else {
        // Standard mode: compute all, then render
        int *results = (int *)malloc(width * height * sizeof(int));
        if (!results) {
            fprintf(stderr, "Error: Failed to allocate memory\n");
            return 1;
        }

        compute_mandelbrot(width, height, max_iter, results);
        gettimeofday(&end, NULL);

        // Render from computed results
        for (int py = 0; py < height; py++) {
            for (int px = 0; px < width; px++) {
                int iter = results[py * width + px];

                if (use_color) {
                    int color = iter_to_color(iter, max_iter);
                    char c = iter_to_char(iter, max_iter);
                    printf("\033[38;5;%dm%c\033[0m", color, c);
                } else {
                    printf("%c", iter_to_char(iter, max_iter));
                }
            }
            printf("\n");
        }

        printf("TIME_MS:%.2f\n", get_time_ms(&start, &end));
        free(results);
    }

    return 0;
}
