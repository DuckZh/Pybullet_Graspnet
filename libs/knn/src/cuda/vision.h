#pragma once
#include <torch/extension.h>
#include <cuda_runtime.h>

void knn_device(float* ref_dev, int ref_width,
    float* query_dev, int query_width,
    int height, int k, float* dist_dev, long* ind_dev, cudaStream_t stream);
