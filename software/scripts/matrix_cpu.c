/*
 * Matrix Multiplication - CPU Implementation
 * Naïve O(n³) algorithm for CUDA vs CPU comparison
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#define DEFAULT_SIZE 512

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

// Naïve O(n³) matrix multiplication: C = A * B
void matrix_multiply(float *A, float *B, float *C, int n, int stream_mode) {
    struct timeval start, now;
    gettimeofday(&start, NULL);

    int total_rows = n;
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
            int percent = ((i + 1) * 100) / total_rows;
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

    // Allocate matrices
    float *A = (float *)malloc(n * n * sizeof(float));
    float *B = (float *)malloc(n * n * sizeof(float));
    float *C = (float *)malloc(n * n * sizeof(float));

    if (!A || !B || !C) {
        fprintf(stderr, "Error: Failed to allocate memory for %dx%d matrices\n", n, n);
        return 1;
    }

    // Initialize with deterministic seed for reproducibility
    srand(42);
    init_matrix(A, n);
    init_matrix(B, n);

    struct timeval start, end;
    gettimeofday(&start, NULL);

    // Perform multiplication
    matrix_multiply(A, B, C, n, stream_mode);

    gettimeofday(&end, NULL);
    double elapsed = get_time_ms(&start, &end);

    if (stream_mode || timing_only) {
        printf("TIME_MS:%.2f\n", elapsed);
        printf("CHECKSUM:%.2f\n", checksum(C, n));
    } else {
        printf("Matrix size: %d x %d\n", n, n);
        printf("Operations: %.2f million (n³)\n", (double)n * n * n / 1000000.0);
        printf("Time: %.2f ms\n", elapsed);
        printf("GFLOPS: %.2f\n", (2.0 * n * n * n) / (elapsed * 1000000.0));
        printf("TIME_MS:%.2f\n", elapsed);
        printf("Checksum: %.2f\n", checksum(C, n));
    }

    free(A);
    free(B);
    free(C);

    return 0;
}
