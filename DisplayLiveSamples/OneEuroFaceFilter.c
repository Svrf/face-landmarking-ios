//
//  OneEuroFaceFilter.c
//  DisplayLiveSamples
//
//  Created by Jesse Boyes on 7/30/19.
//  Copyright © 2019 ZweiGraf. All rights reserved.
//

#include "OneEuroFaceFilter.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif


// MARK: - Low-pass filter (One-euro filter) for point smoothing

/*
 def smoothing_factor(t_e, cutoff):
 r = 2 * math.pi * cutoff * t_e
 return r / (r + 1)
 */

static const unsigned long NUM_POINTS = (68*2) + 3;

double min_cutoff = 1.0;
double beta = 0.0;
double d_cutoff = 1.0;

double prev_x[NUM_POINTS];
double prev_dx[NUM_POINTS];
double prev_t[NUM_POINTS];

double smoothingFactor(double time, double cutoff) {
    double r = 2*M_PI*cutoff*time;
    return r;
}

/*


 def exponential_smoothing(a, x, x_prev):
 return a * x + (1 - a) * x_prev
 */

double exponentialSmoothing(double a, double x, double prevX) {
    return a*x + (1-a)*prevX;
}

/*
 class OneEuroFilter:
 def __init__(self, t0, x0, dx0=0.0, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
 """Initialize the one euro filter."""
 # The parameters.
 self.min_cutoff = float(min_cutoff)
 self.beta = float(beta)
 self.d_cutoff = float(d_cutoff)
 # Previous values.
 self.x_prev = float(x0)
 self.dx_prev = float(dx0)
 self.t_prev = float(t0)

 def __call__(self, t, x):
 """Compute the filtered signal."""
 t_e = t - self.t_prev

 # The filtered derivative of the signal.
 a_d = smoothing_factor(t_e, self.d_cutoff)
 dx = (x - self.x_prev) / t_e
 dx_hat = exponential_smoothing(a_d, dx, self.dx_prev)

 # The filtered signal.
 cutoff = self.min_cutoff + self.beta * abs(dx_hat)
 a = smoothing_factor(t_e, cutoff)
 x_hat = exponential_smoothing(a, x, self.x_prev)

 # Memorize the previous values.
 self.x_prev = x_hat
 self.dx_prev = dx_hat
 self.t_prev = t

 return x_hat
 */

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

    return x_hat;
}

#ifdef __cplusplus
}
#endif
