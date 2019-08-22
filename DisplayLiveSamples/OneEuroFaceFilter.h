//
//  OneEuroFaceFilter.h
//  DisplayLiveSamples
//
//  Created by Jesse Boyes on 7/30/19.
//  Copyright © 2019 ZweiGraf. All rights reserved.
//

#ifndef OneEuroFaceFilter_h
#define OneEuroFaceFilter_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

    extern double min_cutoff;
    extern double beta;
    extern double d_cutoff;
    extern double initial_noise_threshold;

double filter(double x, double t, unsigned int index);

#ifdef __cplusplus
}
#endif

#endif /* OneEuroFaceFilter_h */

