//
//  DlibWrapper.m
//  DisplayLiveSamples
//
//  Created by Luis Reisewitz on 16.05.16.
//  Copyright © 2016 ZweiGraf. All rights reserved.
//

#import "DlibWrapper.h"
#import <UIKit/UIKit.h>

#include <dlib/image_processing.h>
#include <dlib/image_io.h>
//#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/opencv.h>
#include <math.h>
#include <opencv2/calib3d.hpp>

#include "OneEuroFaceFilter.h"
#include "DisplayLiveSamples-Bridging-Header.h"

// How much larger to expand the discovered face rectangle
const static unsigned int FACE_RECT_OVERFLOW=10;

@interface DlibWrapper ()

@property (assign) BOOL prepared;

+ (std::vector<dlib::rectangle>)convertCGRectValueArray:(NSArray<NSValue *> *)rects;

@end
@implementation DlibWrapper {
    dlib::shape_predictor sp;
//    dlib::frontal_face_detector detector;

    // 3d representation of face points for reverse-projection
    std::vector<cv::Point3d> object_pts;

    // Counter for low pass filter
    double frame_number;
}


- (instancetype)init {
    self = [super init];
    if (self) {
        _prepared = NO;
        frame_number = 0;

        //fill in 3D ref points(world coordinates), model referenced from http://aifi.isr.uc.pt/Downloads/OpenGL/glAnthropometric3DModel.cpp
        object_pts.push_back(cv::Point3d(6.825897, 6.760612, 4.402142));     //#33 left brow left corner
        object_pts.push_back(cv::Point3d(1.330353, 7.122144, 6.903745));     //#29 left brow right corner
        object_pts.push_back(cv::Point3d(-1.330353, 7.122144, 6.903745));    //#34 right brow left corner
        object_pts.push_back(cv::Point3d(-6.825897, 6.760612, 4.402142));    //#38 right brow right corner
        object_pts.push_back(cv::Point3d(5.311432, 5.485328, 3.987654));     //#13 left eye left corner
        object_pts.push_back(cv::Point3d(1.789930, 5.393625, 4.413414));     //#17 left eye right corner
        object_pts.push_back(cv::Point3d(-1.789930, 5.393625, 4.413414));    //#25 right eye left corner
        object_pts.push_back(cv::Point3d(-5.311432, 5.485328, 3.987654));    //#21 right eye right corner
        object_pts.push_back(cv::Point3d(2.005628, 1.409845, 6.165652));     //#55 nose left corner
        object_pts.push_back(cv::Point3d(-2.005628, 1.409845, 6.165652));    //#49 nose right corner
        object_pts.push_back(cv::Point3d(2.774015, -2.080775, 5.048531));    //#43 mouth left corner
        object_pts.push_back(cv::Point3d(-2.774015, -2.080775, 5.048531));   //#39 mouth right corner
        object_pts.push_back(cv::Point3d(0.000000, -3.116408, 6.097667));    //#45 mouth central bottom corner
        object_pts.push_back(cv::Point3d(0.000000, -7.415691, 4.070434));    //#6 chin corner
    }
    return self;
}

- (void)prepare {
    NSString *modelFileName = [[NSBundle mainBundle] pathForResource:@"shape_predictor_68_face_landmarks" ofType:@"dat"];
    std::string modelFileNameCString = [modelFileName UTF8String];
    
    dlib::deserialize(modelFileNameCString) >> sp;
    
    // FIXME: test this stuff for memory leaks (cpp object destruction)
    self.prepared = YES;
}

- (void)doWorkOnSampleBuffer:(CMSampleBufferRef)sampleBuffer inRects:(NSArray<NSValue *> *)rects {
    
    if (!self.prepared) {
        [self prepare];
    }
    
    dlib::array2d<dlib::bgr_pixel> img;
    
    // MARK: magic
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    CVPixelBufferLockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);

    size_t width = CVPixelBufferGetWidth(imageBuffer)+8; //// TODO I had to add 8 here, y tho
    size_t height = CVPixelBufferGetHeight(imageBuffer);
    char *baseBuffer = (char *)CVPixelBufferGetBaseAddress(imageBuffer);
    _cameraBufferSize = CGSizeMake(width, height);
    
    // set_size expects rows, cols format
    img.set_size(height, width);
    
    // copy samplebuffer image data into dlib image format
    img.reset();
    long position = 0;
    while (img.move_next()) {
        dlib::bgr_pixel& pixel = img.element();

        // assuming bgra format here
        long bufferLocation = position * 4; //(row * width + column) * 4;
        char b = baseBuffer[bufferLocation];
        char g = baseBuffer[bufferLocation + 1];
        char r = baseBuffer[bufferLocation + 2];
        //        we do not need this
        //        char a = baseBuffer[bufferLocation + 3];
        
        dlib::bgr_pixel newpixel(b, g, r);
        pixel = newpixel;
        
        position++;
    }
    
    // unlock buffer again until we need it again
    CVPixelBufferUnlockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);
    
    // convert the face bounds list to dlib format
    std::vector<dlib::rectangle> convertedRectangles = [DlibWrapper convertCGRectValueArray:rects];

//    std::vector<dlib::rectangle> faces = detector(img);

    // for the first detected face
    if (convertedRectangles.size() > 0) {
        dlib::rectangle oneFaceRect = convertedRectangles[0];
        std::vector<dlib::point> smoothed_points;

        // detect all landmarks
        dlib::full_object_detection shape = sp(img, oneFaceRect);

        // and draw them into the image (samplebuffer)
        for (unsigned int k = 0; k < shape.num_parts(); k++) {
            dlib::point p = shape.part(k);
            double smooth_x = filter((double)p.x(), (double)frame_number, k*2);
            double smooth_y = filter((double)p.y(), (double)frame_number, k*2+1);
            dlib::point smooth_p((unsigned long)smooth_x, (unsigned long)smooth_y);
            smoothed_points.push_back(smooth_p);
            draw_solid_circle(img, smooth_p, 3, color_for_feature(k));
        }

        // reverse-project the face points to determine pose
        [self updateHeadPose:smoothed_points width:width height:height];
    } else {
        self.headPosition = SCNVector3Zero;
    }

    // Use this to calibrate Jesse's wtf hack / TODO remove this (See above TODO)
    dlib::rectangle testRect(0,0,100,100);
    fill_rect(img, testRect, dlib::rgb_pixel(255, 0, 0));

    // lets put everything back where it belongs
    CVPixelBufferLockBaseAddress(imageBuffer, 0);

    // copy dlib image data back into samplebuffer
    img.reset();
    position = 0;
    while (img.move_next()) {
        dlib::bgr_pixel& pixel = img.element();
        
        // assuming bgra format here
        long bufferLocation = position * 4; //(row * width + column) * 4;
        baseBuffer[bufferLocation] = pixel.blue;
        baseBuffer[bufferLocation + 1] = pixel.green;
        baseBuffer[bufferLocation + 2] = pixel.red;
        //        we do not need this
        //        char a = baseBuffer[bufferLocation + 3];
        
        position++;
    }
    CVPixelBufferUnlockBaseAddress(imageBuffer, 0);

    frame_number += 0.09;
}

dlib::rgb_pixel color_for_feature(unsigned long index) {
    if (index < 17) { // jawline
        return dlib::rgb_pixel(0, 255, 255);
    } else if (index < 22) { // left eyebrow
        return dlib::rgb_pixel(255, 0, 0);
    } else if (index < 27) { // right eyebrow
        return dlib::rgb_pixel(0, 255, 0);
    } else if (index < 36) { // nose
        return dlib::rgb_pixel(255, 255, 0);
    } else if (index < 42) { // left eye
        return dlib::rgb_pixel(0, 0, 255);
    } else if (index < 48) { // right eye
        return dlib::rgb_pixel(0, 0, 0);
    } else {
        return dlib::rgb_pixel(255, 255, 255);
    }
}

+ (std::vector<dlib::rectangle>)convertCGRectValueArray:(NSArray<NSValue *> *)rects {
    std::vector<dlib::rectangle> myConvertedRects;
    for (NSValue *rectValue in rects) {
        CGRect rect = [rectValue CGRectValue];
        long left = rect.origin.x - FACE_RECT_OVERFLOW;
        long top = rect.origin.y - FACE_RECT_OVERFLOW;
        long right = left + rect.size.width + FACE_RECT_OVERFLOW*2;
        long bottom = top + rect.size.height + FACE_RECT_OVERFLOW*2;
        dlib::rectangle dlibRect(left, top, right, bottom);

        myConvertedRects.push_back(dlibRect);
    }
    return myConvertedRects;
}

// MARK: - Head pose

- (void)updateHeadPose:(std::vector<dlib::point> &)shape width:(size_t)width height:(size_t)height
{
    // ------------------------------------------
    // TODO: Move this to an initialization step

    //Intrisics can be calculated using opencv sample code under opencv/sources/samples/cpp/tutorial_code/calib3d
    //Normally, you can also apprximate fx and fy by image width, cx by half image width, cy by half image height instead
//    double K[9] = { 6.5308391993466671e+002, 0.0, 3.1950000000000000e+002, 0.0, 6.5308391993466671e+002, 2.3950000000000000e+002, 0.0, 0.0, 1.0 };
    /*
     Camera Matrix:
     [ fx,  0, cx
        0, fy, cy
        0,  0,  1]

     iphone X: FX: 1435.0 FY: 1568.0573 CX: 960.0 CY: 540.0

     Dist Coefficients:
     [ k1, k2, p1, p2 [, k3 [, k4, k5, k6]]] (4, 5, or 8 elements)
     */
//    double K[9] = {
//        1435, 0.0, 960,
//        0.0, 1538.0573, 540,
//        0.0, 0.0, 1.0 };

    double K[9] = {
        self.cameraFx, 0.0, self.cameraCx,
        0.0, self.cameraFy, self.cameraCy,
        0.0, 0.0, 1.0 };
    double D[5] = { 7.0834633684407095e-002, 6.9140193737175351e-002, 0.0, 0.0, -1.3073460323689292e+000 };
    //fill in cam intrinsics and distortion coefficients
    cv::Mat cam_matrix = cv::Mat(3, 3, CV_64FC1, K);
    cv::Mat dist_coeffs = cv::Mat(5, 1, CV_64FC1, D);

    //2D ref points(image coordinates), referenced from detected facial feature
    std::vector<cv::Point2d> image_pts;

    //result
    cv::Mat rotation_vec;                           //3 x 1
    cv::Mat rotation_mat;                           //3 x 3 R
    cv::Mat translation_vec;                        //3 x 1 T
    cv::Mat pose_mat = cv::Mat(3, 4, CV_64FC1);     //3 x 4 R | T
    cv::Mat euler_angle = cv::Mat(3, 1, CV_64FC1);

    //temp buf for decomposeProjectionMatrix()
    cv::Mat out_intrinsics = cv::Mat(3, 3, CV_64FC1);
    cv::Mat out_rotation = cv::Mat(3, 3, CV_64FC1);
    cv::Mat out_translation = cv::Mat(3, 1, CV_64FC1);

    // -----------------------------------------------------------------

    //fill in 2D ref points, annotations follow https://ibug.doc.ic.ac.uk/resources/300-W/
    image_pts.push_back(cv::Point2d(shape[17].x(), shape[17].y())); //#17 left brow left corner
    image_pts.push_back(cv::Point2d(shape[21].x(), shape[21].y())); //#21 left brow right corner
    image_pts.push_back(cv::Point2d(shape[22].x(), shape[22].y())); //#22 right brow left corner
    image_pts.push_back(cv::Point2d(shape[26].x(), shape[26].y())); //#26 right brow right corner
    image_pts.push_back(cv::Point2d(shape[36].x(), shape[36].y())); //#36 left eye left corner
    image_pts.push_back(cv::Point2d(shape[39].x(), shape[39].y())); //#39 left eye right corner
    image_pts.push_back(cv::Point2d(shape[42].x(), shape[42].y())); //#42 right eye left corner
    image_pts.push_back(cv::Point2d(shape[45].x(), shape[45].y())); //#45 right eye right corner
    image_pts.push_back(cv::Point2d(shape[31].x(), shape[31].y())); //#31 nose left corner
    image_pts.push_back(cv::Point2d(shape[35].x(), shape[35].y())); //#35 nose right corner
    image_pts.push_back(cv::Point2d(shape[48].x(), shape[48].y())); //#48 mouth left corner
    image_pts.push_back(cv::Point2d(shape[54].x(), shape[54].y())); //#54 mouth right corner
    image_pts.push_back(cv::Point2d(shape[57].x(), shape[57].y())); //#57 mouth central bottom corner
    image_pts.push_back(cv::Point2d(shape[8].x(), shape[8].y()));   //#8 chin corner

    //calc pose
    cv::solvePnP(object_pts, image_pts, cam_matrix, cv::noArray(), rotation_vec, translation_vec, false, cv::SOLVEPNP_EPNP);

//    std::vector<cv::Point3d> reprojectsrc;
//    std::vector<cv::Point2d> reprojectdst;
//    reprojectdst.resize(1);
//    reprojectsrc.push_back(cv::Point3d(0.0, 0.0, 0.0));
//    cv::projectPoints(reprojectsrc, rotation_vec, translation_vec, cam_matrix, dist_coeffs, reprojectdst);

    //calc euler angle

    // Convert rotation vector into rotation matrix
    cv::Rodrigues(rotation_vec, rotation_mat);

    // Combine rotation vector and translation vector into pose matrix
    cv::hconcat(rotation_mat, translation_vec, pose_mat);

    // Get the hacked up position vector
    SCNVector3 position = [self estimatePositionFromShape:shape width:width height:height];

    // Extract translation and euler angle from pose matrix
    cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angle);
    NSLog(@"R: %0.2f, %0.2f, %0.2f, T: %0.2f, %0.2f, %0.2f, T2: %0.2f, %0.2f, %0.2f, T3: %0.2f, %0.2f, %0.2f",
          euler_angle.at<double>(0),
          euler_angle.at<double>(1),
          euler_angle.at<double>(2),
          out_translation.at<double>(0),
          out_translation.at<double>(1),
          out_translation.at<double>(2),
          translation_vec.at<double>(0),
          translation_vec.at<double>(1),
          translation_vec.at<double>(2),
          position.x, position.y, position.z
          );
/*
    NSLog(@"Camera matrix:\n[%0.1f, %0.1f, %0.1f,\n%0.1f, %0.1f, %0.1f,\n%0.1f, %0.1f, %0.1f]",
          out_intrinsics.at<double>(0),
          out_intrinsics.at<double>(1),
          out_intrinsics.at<double>(2),
          out_intrinsics.at<double>(3),
          out_intrinsics.at<double>(4),
          out_intrinsics.at<double>(5),
          out_intrinsics.at<double>(6),
          out_intrinsics.at<double>(7),
          out_intrinsics.at<double>(8)
          );
*/

    // Correct for front camera mirroring

//    double smooth_euler_0 = filter(fmodf(euler_angle.at<double>(0), 360), frame_number, (2*68)) * M_PI/180;
//    double smooth_euler_1 = filter(fmodf(euler_angle.at<double>(1), 360), frame_number, (2*68)+1) * M_PI/180;
//    double smooth_euler_2 = filter(fmodf(euler_angle.at<double>(2), 360), frame_number, (2*68)+2) * M_PI/180;
//    self.headPoseAngle = SCNVector3Make(M_PI + smooth_euler_0, M_PI + smooth_euler_1, -smooth_euler_2);

    self.headPoseAngle =
    SCNVector3Make(M_PI + (euler_angle.at<double>(0) * M_PI / 180),
                   M_PI + (euler_angle.at<double>(1) * M_PI / 180),
                   -euler_angle.at<double>(2) * M_PI / 180);
//    SCNVector3Make(fmodf(M_PI + (euler_angle.at<double>(0) * M_PI / 180), 2*M_PI),
//                   fmodf(M_PI + (euler_angle.at<double>(1) * M_PI / 180), 2*M_PI),
//                   -fmodf(euler_angle.at<double>(2) * M_PI / 180, 2*M_PI));
    self.headPosition = position;
//    SCNVector3Make(out_translation.at<double>(0),
//                   out_translation.at<double>(1),
//                   out_translation.at<double>(2));
}

/* this is a total hack just to see if I can get anywhere close
 */
- (SCNVector3)estimatePositionFromShape:(std::vector<dlib::point> &)shape
                                  width:(size_t)width
                                 height:(size_t)height
{
    static const double downscale_factor_z = 750;

    double x = 0;
    double y = 0;
    double min_x = 1000000;
    double max_x = 0;

    // Take the average of points for eyebrows + nose
    for (int i=17; i < 68; i++) {
        x += shape[i].x();
        y += shape[i].y();
        max_x = MAX(shape[i].x(), max_x);
        min_x = MIN(shape[i].x(), min_x);
    }
    x /= (68-17);
    y /= (68-17);

//    x = shape[28].x();
//    y = shape[28].y();

    double divisor = 1.5;// self.slider1Value * 3; // 1.44
    double downscale_factor = 722;// self.slider2Value * 1000; // 689
    double z = ((double)width/divisor) - (max_x - min_x);

    NSLog(@"divisor: %0.2f / downscale factor: %0.1f", divisor, downscale_factor);
    NSLog(@"I think Z is: %0.2f", z/downscale_factor);

    return SCNVector3Make(x, y, z/downscale_factor);
}

@end
