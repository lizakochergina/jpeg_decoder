#pragma once

#include <cstddef>
#include <vector>
#include <cmath>
#include <fftw3.h>

class DctCalculator {
public:
    // input and output are width by width matrices, first row, then
    // the second row.
    DctCalculator(size_t width, std::vector<double> *input, std::vector<double> *output);
    ~DctCalculator();
    void Inverse();

private:
    size_t width_;
    std::vector<double> *input_;
    std::vector<double> *output_;
    fftw_plan plan_;
    const double sqrt_const_ = sqrt(2);
};
