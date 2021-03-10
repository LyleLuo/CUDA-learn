#include "core.h"

__global__ void kernel(int width, int height, float *input, float *output) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;

    if (idx < width && idy < height) {
        int cnt[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        for (int pos_y = idy - 2; pos_y <= idy + 2; ++pos_y) {
            if (pos_y >= 0 && pos_y < height) {
                for (int pos_x = idx - 2; pos_x <= idx + 2; ++pos_x) {
                    if (pos_x >= 0 && pos_x < width) {
                        cnt[(int)input[pos_x + pos_y*width]]++;
                    }
                }
            }
        }
        
        double n = (min(idx, 2) + 1 + min(width - idx, 2)) * (min(idy, 2) + 1 + min(height - idy, 2));
        double n_inv = 1.0 / n;
        double ans = log(n);

        for (int i = 0; i < 16; ++i) {
            if (cnt[i]) {
                ans -= log((double)cnt[i]) * cnt[i] * n_inv;
            }
        }

        output[idy * width + idx] = ans;
    }
}

void cudaCallback(int width, int height, float *sample, float **result) {
    int size = width * height;
    float *input_d, *output_d;

    // Allocate device memory and copy data from host to device
    CHECK(cudaMalloc((void **)&input_d, sizeof(float)*size));
    CHECK(cudaMalloc((void **)&output_d, sizeof(float)*size));
    CHECK(cudaMemcpy(input_d, sample, sizeof(float)*size, cudaMemcpyHostToDevice));

    // Invoke the device function

    const dim3 blockDim(32, 32), gridDim(divup(width, 32), divup(height, 32));

    kernel<<< gridDim, blockDim >>>(width, height, input_d, output_d);
    cudaDeviceSynchronize();

    // Copy back the results and de-allocate the device memory
    *result = (float *)malloc(sizeof(float)*size);
    CHECK(cudaMemcpy(*result, output_d, sizeof(float)*size, cudaMemcpyDeviceToHost));
    CHECK(cudaFree(input_d));
    CHECK(cudaFree(output_d))
    
    // Note that you don't have to free sample and *result by yourself
}
