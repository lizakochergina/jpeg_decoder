#include "fft.h"

#include <fftw3.h>
#include <stdexcept>

DctCalculator::DctCalculator(size_t width, std::vector<double> *input, std::vector<double> *output)
    : width_(width), input_(input), output_(output) {

    if (width * width != input->size() || input->size() != output->size()) {
        throw std::invalid_argument("dimensions are not correct");
    }

    plan_ = fftw_plan_r2r_2d(width_, width_, &input_->at(0), &output_->at(0), FFTW_REDFT01,
                             FFTW_REDFT01, FFTW_ESTIMATE);
}

void DctCalculator::Inverse() {
    input_->at(0) = input_->at(0) * 0.5 * 4;
    for (size_t j = 1; j != width_; ++j) {
        size_t k = j;
        size_t m = j * width_;
        input_->at(k) = input_->at(k) * sqrt_const_;
        input_->at(m) = input_->at(m) * sqrt_const_;
    }
    fftw_execute(plan_);

    for (auto &el : *output_) {
        el *= 0.0625;
    }
}
DctCalculator::~DctCalculator() {
    fftw_destroy_plan(plan_);
}
