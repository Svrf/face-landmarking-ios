//
//  OneEuroFaceFilter.c
//  DisplayLiveSamples
//
//  Created by Jesse Boyes on 7/30/19.
//  Copyright © 2019 Svtrf. All rights reserved.
//

#include "OneEuroFaceFilter.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif


// MARK: - Low-pass filter (One-euro filter) for point smoothing

static const unsigned long NUM_POINTS = (68*2) + 3;

double min_cutoff = 1.0;
double beta = 0.0;
double d_cutoff = 1.0;
double initial_noise_threshold = 0.5;

double prev_x[NUM_POINTS];
double prev_dx[NUM_POINTS];
double prev_t[NUM_POINTS];

double smoothingFactor(double time, double cutoff) {
    double r = 2*M_PI*cutoff*time;
    return r;
}

double exponentialSmoothing(double a, double x, double prevX) {
    return a*x + (1-a)*prevX;
}

double filter(double x, double t, unsigned int index) {
    double t_e = t - prev_t[index];

    // The filtered derivative of the signal
    double a_d = smoothingFactor(t_e, d_cutoff);
    double dx = (x - prev_x[index]);
    double dx_hat = exponentialSmoothing(a_d, dx, prev_dx[index]);

    // The filtered signal
    double cutoff = min_cutoff + beta*fabs(dx_hat);
    double a = smoothingFactor(t_e, cutoff);
    double x_hat = exponentialSmoothing(a, x, prev_x[index]);

    // Store previous values
    prev_x[index] = x_hat;
    prev_dx[index] = dx_hat;
    prev_t[index] = t;

    // Don't start returning filtered data 'til it's ready
    if (t > initial_noise_threshold) {
        return x_hat;
    } else {
        return x;
    }
}

#ifdef __cplusplus
}
#endif
