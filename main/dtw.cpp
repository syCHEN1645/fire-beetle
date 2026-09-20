#include "dtw.h"
#include <cmath>
#include <algorithm>

DTW::DTW() {
    this->ref = nullptr;
    this->ref_r = 0;
    this->ref_c = 0;
}

DTW::~DTW() {
}

void DTW::set_reference(const float* ref, size_t ref_r, size_t ref_c) {
    this->ref = ref;
    this->ref_r = ref_r;
    this->ref_c = ref_c;
}

/// @brief Computes the Dynamic Time Warping (DTW) distance between two sequences.
/// @param seq The sequence of float values to be compared against the reference sequence.
/// @param seq_r The number of rows in the sequence.
/// @param seq_c The number of columns in the sequence.
/// @return The DTW distance between the two sequences as a float.
/// @note The individual samples should have the same size (col). 
float DTW::compute(const float *seq, size_t seq_r, size_t seq_c) {
    if (seq == nullptr || ref == nullptr) {
        return std::numeric_limits<float>::infinity();
    }

    // Check if the individual samples (inner array) have the same size.
    // Before passing into DTW, the sequences should be preprocessed before passing here. 
    if (seq_c != ref_c) {
        return std::numeric_limits<float>::infinity();
    }

    if (seq_r == 0 || ref_r == 0) {
        return std::numeric_limits<float>::infinity();
    }

    float *cost_matrix = new float[seq_r * ref_r];

    // the matrix has seq_r rows and ref_r columns
    for (size_t i = 0; i < seq_r * ref_r; ++i) {
        cost_matrix[i] = std::numeric_limits<float>::infinity();
    }
    cost_matrix[0] = sample_dist(ref, seq, ref_c);

    // fill in the first row and first column of the cost matrix
    for (size_t i = 1; i < seq_r; ++i) {
        cost_matrix[i * ref_r] = cost_matrix[(i - 1) * ref_r] + sample_dist(ref, seq + i * ref_c, ref_c);
    }
    for (size_t j = 1; j < ref_r; ++j) {
        cost_matrix[j] = cost_matrix[j - 1] + sample_dist(ref + j * ref_c, seq, ref_c);
    }

    for (size_t i = 1; i < seq_r; ++i) {
        for (size_t  j = 1; j < ref_r; ++j) {
            float min_cost = std::min({cost_matrix[(i - 1) * ref_r + j], cost_matrix[i * ref_r + j - 1], cost_matrix[(i - 1) * ref_r + j - 1]});
            cost_matrix[i * ref_r + j] = sample_dist(ref + j * ref_c, seq + i * ref_c, ref_c) + min_cost;
        }
    }

    float result = cost_matrix[seq_r * ref_r - 1];
    delete[] cost_matrix;
    return result;
}

/// @brief Computes the Euclidean distance between a sample from the sequence and a sample from the reference sequence.
/// @param ref_start Pointer to the start of the reference sample.
/// @param sample_start Pointer to the start of the sequence sample.
/// @param len The length of the samples (number of columns).
/// @return The Euclidean distance between the specified sample and the corresponding reference sample.
float DTW::sample_dist(const float *ref_start, const float *sample_start, size_t len) {
    float sum = 0.0;
    for (size_t i = 0; i < len; ++i) {
        sum += (sample_start[i] - ref_start[i]) * (sample_start[i] - ref_start[i]);
    }
    return std::sqrt(sum);
}