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
#include <dlib/opencv.h>
#include <opencv2/calib3d.hpp>

@interface DlibWrapper ()

@property (assign) BOOL prepared;

+ (std::vector<dlib::rectangle>)convertCGRectValueArray:(NSArray<NSValue *> *)rects;

@end
@implementation DlibWrapper {
    dlib::shape_predictor sp;
}


- (instancetype)init {
    self = [super init];
    if (self) {
        _prepared = NO;
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
    NSLog(@"Image dimensions: %zu / %zu", width, height);
    char *baseBuffer = (char *)CVPixelBufferGetBaseAddress(imageBuffer);
    
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
    
    // for every detected face
    if (convertedRectangles.size() > 0) {
        dlib::rectangle oneFaceRect = convertedRectangles[0];
        
        // detect all landmarks
        dlib::full_object_detection shape = sp(img, oneFaceRect);
        
        // and draw them into the image (samplebuffer)
        for (unsigned long k = 0; k < shape.num_parts(); k++) {
            dlib::point p = shape.part(k);
            draw_solid_circle(img, p, 3, color_for_feature(k));
        }
        
        // reverse-project the face points to determine pose
        [self updateHeadPose:shape];
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
}

- (void)updateHeadPose:(dlib::full_object_detection &)shape
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
    double K[9] = {
        1435, 0.0, 960,
        0.0, 1538.0573, 540,
        0.0, 0.0, 1.0 };
    double D[5] = { 7.0834633684407095e-002, 6.9140193737175351e-002, 0.0, 0.0, -1.3073460323689292e+000 };
    //fill in cam intrinsics and distortion coefficients
    cv::Mat cam_matrix = cv::Mat(3, 3, CV_64FC1, K);
    cv::Mat dist_coeffs = cv::Mat(5, 1, CV_64FC1, D);

    //fill in 3D ref points(world coordinates), model referenced from http://aifi.isr.uc.pt/Downloads/OpenGL/glAnthropometric3DModel.cpp
    std::vector<cv::Point3d> object_pts;
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
    image_pts.push_back(cv::Point2d(shape.part(17).x(), shape.part(17).y())); //#17 left brow left corner
    image_pts.push_back(cv::Point2d(shape.part(21).x(), shape.part(21).y())); //#21 left brow right corner
    image_pts.push_back(cv::Point2d(shape.part(22).x(), shape.part(22).y())); //#22 right brow left corner
    image_pts.push_back(cv::Point2d(shape.part(26).x(), shape.part(26).y())); //#26 right brow right corner
    image_pts.push_back(cv::Point2d(shape.part(36).x(), shape.part(36).y())); //#36 left eye left corner
    image_pts.push_back(cv::Point2d(shape.part(39).x(), shape.part(39).y())); //#39 left eye right corner
    image_pts.push_back(cv::Point2d(shape.part(42).x(), shape.part(42).y())); //#42 right eye left corner
    image_pts.push_back(cv::Point2d(shape.part(45).x(), shape.part(45).y())); //#45 right eye right corner
    image_pts.push_back(cv::Point2d(shape.part(31).x(), shape.part(31).y())); //#31 nose left corner
    image_pts.push_back(cv::Point2d(shape.part(35).x(), shape.part(35).y())); //#35 nose right corner
    image_pts.push_back(cv::Point2d(shape.part(48).x(), shape.part(48).y())); //#48 mouth left corner
    image_pts.push_back(cv::Point2d(shape.part(54).x(), shape.part(54).y())); //#54 mouth right corner
    image_pts.push_back(cv::Point2d(shape.part(57).x(), shape.part(57).y())); //#57 mouth central bottom corner
    image_pts.push_back(cv::Point2d(shape.part(8).x(), shape.part(8).y()));   //#8 chin corner

    double maxX = 0;
    double minX = 1000000;
    double maxY = 0;
    double minY = 1000000;
    for (unsigned long i = 0; i < image_pts.size(); i++) {
        maxX = MAX(image_pts[i].x, maxX);
        minX = MIN(image_pts[i].x, minX);
        maxY = MAX(image_pts[i].y, maxY);
        minY = MIN(image_pts[i].y, minY);
    }
    NSLog(@"Rect: (%0.2f, %0.2f, %0.2f, %0.2f)", minX, minY, maxX, maxY);

    //calc pose
    cv::solvePnP(object_pts, image_pts, cam_matrix, cv::noArray()/*dist_coeffs*/, rotation_vec, translation_vec, false, cv::SOLVEPNP_EPNP);
    NSLog(@"Rotation Vector: %0.2f, %0.2f, %0.2f",
          rotation_vec.at<double>(0),
          rotation_vec.at<double>(1),
          rotation_vec.at<double>(2));

    //calc euler angle
    cv::Rodrigues(rotation_vec, rotation_mat);
    cv::hconcat(rotation_mat, translation_vec, pose_mat);
    cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angle);
    NSLog(@"R: %0.2f, %0.2f, %0.2f, T: %0.2f, %0.2f, %0.2f",
          euler_angle.at<double>(0),
          euler_angle.at<double>(1),
          euler_angle.at<double>(2),
          out_translation.at<double>(0),
          out_translation.at<double>(1),
          out_translation.at<double>(2)
          );

    self.headPoseAngle =
    SCNVector3Make(euler_angle.at<double>(0) * M_PI / 180,
                   euler_angle.at<double>(1) * M_PI / 180,
                   euler_angle.at<double>(2) * M_PI / 180);

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
        long left = rect.origin.x;
        long top = rect.origin.y;
        long right = left + rect.size.width;
        long bottom = top + rect.size.height;
        dlib::rectangle dlibRect(left, top, right, bottom);

        myConvertedRects.push_back(dlibRect);
    }
    return myConvertedRects;
}

@end
