//
//  DlibWrapper.h
//  DisplayLiveSamples
//
//  Created by Luis Reisewitz on 16.05.16.
//  Copyright © 2016 ZweiGraf. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <CoreMedia/CoreMedia.h>
#import <SceneKit/SceneKit.h>

@interface DlibWrapper : NSObject

- (instancetype)init;
- (void)doWorkOnSampleBuffer:(CMSampleBufferRef)sampleBuffer inRects:(NSArray<NSValue *> *)rects;
- (void)prepare;

@property (assign) SCNVector3 headPoseAngle;
// Head position is:
// x + y: pixel position on camera input image
// z: a number between 0 (front plane) and 1 (horizon)
@property (assign) SCNVector3 headPosition;

@property (assign) CGSize cameraBufferSize;

@property (assign) double slider1Value;
@property (assign) double slider2Value;

// For the camera FOV matrix
@property (assign) double cameraFx;
@property (assign) double cameraFy;
@property (assign) double cameraCx;
@property (assign) double cameraCy;

@end
