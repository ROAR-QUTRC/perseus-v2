/*
 * Matrix Multiplication - CUDA Implementation
 * Naïve O(n³) algorithm parallelized on GPU
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <cuda_runtime.h>

#define DEFAULT_SIZE 512
#define BLOCK_SIZE 16

// Get elapsed time in milliseconds
double get_time_ms(struct timeval *start, struct timeval *end) {
    return (end->tv_sec - start->tv_sec) * 1000.0 +
           (end->tv_usec - start->tv_usec) / 1000.0;
}

// Initialize matrix with random values
void init_matrix(float *M, int n) {
    for (int i = 0; i < n * n; i++) {
        M[i] = (float)(rand() % 100) / 100.0f;
    }
}

// CUDA kernel - one thread per output element
__global__ void matrix_multiply_kernel(float *A, float *B, float *C, int n) {
    int row = blockIdx.y * blockDim.y + threadIdx.y;
    int col = blockIdx.x * blockDim.x + threadIdx.x;

    if (row < n && col < n) {
        float sum = 0.0f;
        for (int k = 0; k < n; k++) {
            sum += A[row * n + k] * B[k * n + col];
        }
        C[row * n + col] = sum;
    }
}

// CUDA kernel - compute a batch of rows (for streaming progress)
__global__ void matrix_multiply_rows_kernel(float *A, float *B, float *C, int n,
                                             int start_row, int num_rows) {
    int local_row = blockIdx.y * blockDim.y + threadIdx.y;
    int col = blockIdx.x * blockDim.x + threadIdx.x;
    int row = start_row + local_row;

    if (local_row < num_rows && row < n && col < n) {
        float sum = 0.0f;
        for (int k = 0; k < n; k++) {
            sum += A[row * n + k] * B[k * n + col];
        }
        C[row * n + col] = sum;
    }
}

// CPU fallback - naïve O(n³) matrix multiplication
void matrix_multiply_cpu(float *A, float *B, float *C, int n, int stream_mode) {
    struct timeval start, now;
    gettimeofday(&start, NULL);

    int last_percent = -1;

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            float sum = 0.0f;
            for (int k = 0; k < n; k++) {
                sum += A[i * n + k] * B[k * n + j];
            }
            C[i * n + j] = sum;
        }

        if (stream_mode) {
            int percent = ((i + 1) * 100) / n;
            if (percent != last_percent) {
                gettimeofday(&now, NULL);
                printf("PROGRESS:%d|%.1f\n", percent, get_time_ms(&start, &now));
                fflush(stdout);
                last_percent = percent;
            }
        }
    }
}

// Verify result (compute checksum)
float checksum(float *M, int n) {
    float sum = 0.0f;
    for (int i = 0; i < n * n; i++) {
        sum += M[i];
    }
    return sum;
}

void print_usage(const char *prog_name) {
    printf("Usage: %s [OPTIONS]\n", prog_name);
    printf("\nOptions:\n");
    printf("  -n, --size N       Matrix size NxN (default: %d)\n", DEFAULT_SIZE);
    printf("  -s, --stream       Stream progress updates (for race mode)\n");
    printf("  -t, --timing-only  Only output timing (for benchmarks)\n");
    printf("  --help             Show this help message\n");
}

int main(int argc, char *argv[]) {
    int n = DEFAULT_SIZE;
    int stream_mode = 0;
    int timing_only = 0;

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-n") == 0 || strcmp(argv[i], "--size") == 0) {
            if (i + 1 < argc) n = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--stream") == 0) {
            stream_mode = 1;
        } else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--timing-only") == 0) {
            timing_only = 1;
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    // Check for CUDA device
    int device_count = 0;
    cudaError_t err = cudaGetDeviceCount(&device_count);
    int use_gpu = (err == cudaSuccess && device_count > 0);

    // Allocate host matrices
    float *h_A = (float *)malloc(n * n * sizeof(float));
    float *h_B = (float *)malloc(n * n * sizeof(float));
    float *h_C = (float *)malloc(n * n * sizeof(float));

    if (!h_A || !h_B || !h_C) {
        fprintf(stderr, "Error: Failed to allocate memory for %dx%d matrices\n", n, n);
        return 1;
    }

    // Initialize with deterministic seed for reproducibility
    srand(42);
    init_matrix(h_A, n);
    init_matrix(h_B, n);

    struct timeval start, end, now;
    gettimeofday(&start, NULL);

    if (use_gpu) {
        // Allocate device matrices
        float *d_A, *d_B, *d_C;
        cudaMalloc(&d_A, n * n * sizeof(float));
        cudaMalloc(&d_B, n * n * sizeof(float));
        cudaMalloc(&d_C, n * n * sizeof(float));

        // Copy input matrices to device
        cudaMemcpy(d_A, h_A, n * n * sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(d_B, h_B, n * n * sizeof(float), cudaMemcpyHostToDevice);

        if (stream_mode) {
            // Stream mode: compute in batches and report progress
            int batch_size = n / 10;  // 10% at a time
            if (batch_size < 1) batch_size = 1;

            int last_percent = -1;

            for (int start_row = 0; start_row < n; start_row += batch_size) {
                int rows_this_batch = batch_size;
                if (start_row + rows_this_batch > n) {
                    rows_this_batch = n - start_row;
                }

                dim3 block(BLOCK_SIZE, BLOCK_SIZE);
                dim3 grid((n + BLOCK_SIZE - 1) / BLOCK_SIZE,
                          (rows_this_batch + BLOCK_SIZE - 1) / BLOCK_SIZE);

                matrix_multiply_rows_kernel<<<grid, block>>>(d_A, d_B, d_C, n,
                                                              start_row, rows_this_batch);
                cudaDeviceSynchronize();

                int percent = ((start_row + rows_this_batch) * 100) / n;
                if (percent != last_percent) {
                    gettimeofday(&now, NULL);
                    printf("PROGRESS:%d|%.1f\n", percent, get_time_ms(&start, &now));
                    fflush(stdout);
                    last_percent = percent;
                }
            }
        } else {
            // Full computation
            dim3 block(BLOCK_SIZE, BLOCK_SIZE);
            dim3 grid((n + BLOCK_SIZE - 1) / BLOCK_SIZE,
                      (n + BLOCK_SIZE - 1) / BLOCK_SIZE);

            cudaDeviceSynchronize();
            matrix_multiply_kernel<<<grid, block>>>(d_A, d_B, d_C, n);
            cudaDeviceSynchronize();
        }

        // Copy result back
        cudaMemcpy(h_C, d_C, n * n * sizeof(float), cudaMemcpyDeviceToHost);

        cudaFree(d_A);
        cudaFree(d_B);
        cudaFree(d_C);
    } else {
        // CPU fallback
        matrix_multiply_cpu(h_A, h_B, h_C, n, stream_mode);
    }

    gettimeofday(&end, NULL);
    double elapsed = get_time_ms(&start, &end);

    if (stream_mode || timing_only) {
        printf("TIME_MS:%.2f\n", elapsed);
        printf("GPU_USED:%d\n", use_gpu);
        printf("CHECKSUM:%.2f\n", checksum(h_C, n));
    } else {
        if (use_gpu) {
            cudaDeviceProp prop;
            cudaGetDeviceProperties(&prop, 0);
            printf("GPU: %s\n", prop.name);
        } else {
            printf("NO_GPU: Using CPU fallback\n");
        }
        printf("Matrix size: %d x %d\n", n, n);
        printf("Operations: %.2f million (n³)\n", (double)n * n * n / 1000000.0);
        printf("Time: %.2f ms\n", elapsed);
        printf("GFLOPS: %.2f\n", (2.0 * n * n * n) / (elapsed * 1000000.0));
        printf("TIME_MS:%.2f\n", elapsed);
        printf("GPU_USED:%d\n", use_gpu);
        printf("Checksum: %.2f\n", checksum(h_C, n));
    }

    free(h_A);
    free(h_B);
    free(h_C);

    return 0;
}
