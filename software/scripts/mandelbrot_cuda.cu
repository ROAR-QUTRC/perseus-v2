/*
 * Mandelbrot Set - CUDA Implementation
 * ASCII visualization with ANSI 256-color support
 * GPU-accelerated parallel computation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <cuda_runtime.h>

// Default parameters
#define DEFAULT_WIDTH 80
#define DEFAULT_HEIGHT 40
#define DEFAULT_MAX_ITER 100

// Mandelbrot view bounds
#define X_MIN -2.5
#define X_MAX 1.0
#define Y_MIN -1.0
#define Y_MAX 1.0

// CUDA block dimensions
#define BLOCK_SIZE 256

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

// CUDA kernel - compute Mandelbrot for a single row
__global__ void mandelbrot_row_kernel(int *results, int width, int py, int max_iter,
                                       double x_min, double x_max, double y0) {
    int px = blockIdx.x * blockDim.x + threadIdx.x;

    if (px >= width) return;

    double x_scale = (x_max - x_min) / width;
    double x0 = x_min + px * x_scale;

    double x = 0.0, y = 0.0;
    int iter = 0;

    while (x*x + y*y <= 4.0 && iter < max_iter) {
        double xtemp = x*x - y*y + x0;
        y = 2.0*x*y + y0;
        x = xtemp;
        iter++;
    }

    results[px] = iter;
}

// CUDA kernel - compute Mandelbrot for entire image
__global__ void mandelbrot_kernel(int *results, int width, int height, int max_iter,
                                   double x_min, double x_max, double y_min, double y_max) {
    int px = blockIdx.x * blockDim.x + threadIdx.x;
    int py = blockIdx.y * blockDim.y + threadIdx.y;

    if (px >= width || py >= height) return;

    double x_scale = (x_max - x_min) / width;
    double y_scale = (y_max - y_min) / height;

    double x0 = x_min + px * x_scale;
    double y0 = y_max - py * y_scale;

    double x = 0.0, y = 0.0;
    int iter = 0;

    while (x*x + y*y <= 4.0 && iter < max_iter) {
        double xtemp = x*x - y*y + x0;
        y = 2.0*x*y + y0;
        x = xtemp;
        iter++;
    }

    results[py * width + px] = iter;
}

// CPU fallback for a single row
void compute_row_cpu(int width, int py, int height, int max_iter, int *results) {
    double x_scale = (X_MAX - X_MIN) / width;
    double y_scale = (Y_MAX - Y_MIN) / height;
    double y0 = Y_MAX - py * y_scale;

    for (int px = 0; px < width; px++) {
        double x0 = X_MIN + px * x_scale;

        double x = 0.0, y = 0.0;
        int iter = 0;

        while (x*x + y*y <= 4.0 && iter < max_iter) {
            double xtemp = x*x - y*y + x0;
            y = 2.0*x*y + y0;
            x = xtemp;
            iter++;
        }

        results[px] = iter;
    }
}

// CPU fallback - compute Mandelbrot on CPU
void compute_mandelbrot_cpu(int width, int height, int max_iter, int *results) {
    double x_scale = (X_MAX - X_MIN) / width;
    double y_scale = (Y_MAX - Y_MIN) / height;

    for (int py = 0; py < height; py++) {
        double y0 = Y_MAX - py * y_scale;

        for (int px = 0; px < width; px++) {
            double x0 = X_MIN + px * x_scale;

            double x = 0.0, y = 0.0;
            int iter = 0;

            while (x*x + y*y <= 4.0 && iter < max_iter) {
                double xtemp = x*x - y*y + x0;
                y = 2.0*x*y + y0;
                x = xtemp;
                iter++;
            }

            results[py * width + px] = iter;
        }
    }
}

// Render a line from results
void render_line(int *results, int width, int max_iter, int use_color) {
    for (int px = 0; px < width; px++) {
        int iter = results[px];
        if (use_color) {
            int color = iter_to_color(iter, max_iter);
            char c = iter_to_char(iter, max_iter);
            printf("\033[38;5;%dm%c\033[0m", color, c);
        } else {
            printf("%c", iter_to_char(iter, max_iter));
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

    // Check for CUDA device
    int device_count = 0;
    cudaError_t err = cudaGetDeviceCount(&device_count);
    int use_gpu = (err == cudaSuccess && device_count > 0);

    struct timeval start, end, line_end;
    gettimeofday(&start, NULL);

    if (stream_mode) {
        // Stream mode: output each line immediately with timing
        double y_scale = (Y_MAX - Y_MIN) / height;

        if (use_gpu) {
            // GPU streaming mode
            int *d_results, *h_results;
            h_results = (int *)malloc(width * sizeof(int));
            cudaMalloc(&d_results, width * sizeof(int));

            int blocks = (width + BLOCK_SIZE - 1) / BLOCK_SIZE;

            for (int py = 0; py < height; py++) {
                double y0 = Y_MAX - py * y_scale;

                mandelbrot_row_kernel<<<blocks, BLOCK_SIZE>>>(
                    d_results, width, py, max_iter, X_MIN, X_MAX, y0);
                cudaDeviceSynchronize();
                cudaMemcpy(h_results, d_results, width * sizeof(int), cudaMemcpyDeviceToHost);

                render_line(h_results, width, max_iter, use_color);
                gettimeofday(&line_end, NULL);
                printf("|%.1f\n", get_time_ms(&start, &line_end));
                fflush(stdout);
            }

            cudaFree(d_results);
            free(h_results);
        } else {
            // CPU fallback streaming
            int *h_results = (int *)malloc(width * sizeof(int));

            for (int py = 0; py < height; py++) {
                compute_row_cpu(width, py, height, max_iter, h_results);
                render_line(h_results, width, max_iter, use_color);
                gettimeofday(&line_end, NULL);
                printf("|%.1f\n", get_time_ms(&start, &line_end));
                fflush(stdout);
            }

            free(h_results);
        }

        gettimeofday(&end, NULL);
        printf("TIME_MS:%.2f\n", get_time_ms(&start, &end));
        printf("GPU_USED:%d\n", use_gpu);

    } else if (timing_only) {
        // Timing only mode
        int *h_results = (int *)malloc(width * height * sizeof(int));

        if (use_gpu) {
            int *d_results;
            cudaMalloc(&d_results, width * height * sizeof(int));

            dim3 block(16, 16);
            dim3 grid((width + 15) / 16, (height + 15) / 16);

            cudaDeviceSynchronize();
            mandelbrot_kernel<<<grid, block>>>(d_results, width, height, max_iter,
                                               X_MIN, X_MAX, Y_MIN, Y_MAX);
            cudaDeviceSynchronize();

            cudaMemcpy(h_results, d_results, width * height * sizeof(int), cudaMemcpyDeviceToHost);
            cudaFree(d_results);
        } else {
            compute_mandelbrot_cpu(width, height, max_iter, h_results);
        }

        gettimeofday(&end, NULL);
        printf("TIME_MS:%.2f\n", get_time_ms(&start, &end));
        printf("GPU_USED:%d\n", use_gpu);
        free(h_results);

    } else {
        // Standard mode
        int *h_results = (int *)malloc(width * height * sizeof(int));

        if (use_gpu) {
            cudaDeviceProp prop;
            cudaGetDeviceProperties(&prop, 0);
            printf("GPU: %s\n", prop.name);

            int *d_results;
            cudaMalloc(&d_results, width * height * sizeof(int));

            dim3 block(16, 16);
            dim3 grid((width + 15) / 16, (height + 15) / 16);

            cudaDeviceSynchronize();
            mandelbrot_kernel<<<grid, block>>>(d_results, width, height, max_iter,
                                               X_MIN, X_MAX, Y_MIN, Y_MAX);
            cudaDeviceSynchronize();

            cudaMemcpy(h_results, d_results, width * height * sizeof(int), cudaMemcpyDeviceToHost);
            cudaFree(d_results);
        } else {
            printf("NO_GPU: Using CPU fallback\n");
            compute_mandelbrot_cpu(width, height, max_iter, h_results);
        }

        gettimeofday(&end, NULL);

        // Render
        for (int py = 0; py < height; py++) {
            for (int px = 0; px < width; px++) {
                int iter = h_results[py * width + px];
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
        printf("GPU_USED:%d\n", use_gpu);
        free(h_results);
    }

    return 0;
}
