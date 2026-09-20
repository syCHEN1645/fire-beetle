#include <stdio.h>

class DTW {
public:
    DTW();
    ~DTW();

    void set_reference(const float* ref, size_t ref_r, size_t ref_c);

    float compute(const float* seq, size_t seq_r, size_t seq_c);
    
private:
    const float* ref;
    size_t ref_r;
    size_t ref_c;

    float sample_dist(const float* ref_start, const float* sample_start, size_t len);
};