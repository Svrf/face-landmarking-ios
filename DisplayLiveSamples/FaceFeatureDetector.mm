//
//  DlibWrapper.m
//  DisplayLiveSamples
//
//  Created by Luis Reisewitz on 16.05.16.
//  Copyright © 2016 ZweiGraf. All rights reserved.
//

#import "FaceFeatureDetector.h"
#import <UIKit/UIKit.h>

#include <dlib/image_processing.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>
#include <math.h>
#include <opencv2/calib3d.hpp>
#include <dlib/image_processing/frontal_face_detector.h>

#include "OneEuroFaceFilter.h"
#include "DisplayLiveSamples-Bridging-Header.h"

// How much larger to expand the discovered face rectangle
static unsigned int FACE_RECT_OVERFLOW=10;
const static bool SMOOTH_POINTS = true; /* Filter face detection points */
const static bool SMOOTH_PROJECTION = false; /* Filter projection points */
const static double FRAME_ADVANCE = 0.07; /* higher = more responsive, more noise */
const static bool DRAW_FACE_DETECTION_POINTS = true; /* Points for face and rect around face */

@interface FaceFeatureDetector ()

@property (assign) BOOL prepared;

+ (std::vector<dlib::rectangle>)convertCGRectValueArray:(NSArray<NSValue *> *)rects;

@end
@implementation FaceFeatureDetector {
    dlib::shape_predictor sp;
    dlib::frontal_face_detector detector;

    // 3d representation of face points for reverse-projection
    std::vector<cv::Point3d> object_pts;

    // Counter for low pass filter
    double frame_number;
}

- (instancetype)init {
    self = [super init];
    if (self) {
        _prepared = NO;
        detector = dlib::get_frontal_face_detector();
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

- (void)setSlider1Value:(double)slider1Value {
    _slider1Value = slider1Value;
    FACE_RECT_OVERFLOW = 5 + (unsigned int)(_slider1Value*100);
    NSLog(@"Face rect overflow: %ul", FACE_RECT_OVERFLOW);
}

- (void)resetFrameNumber {
    frame_number = 0.0;
}

//- (double)slider1Value {
//    return _slider1Value;
//}

- (void)populateImageArray:(dlib::array2d<dlib::bgr_pixel> &)img
                     width:(size_t *)width
                    height:(size_t *)height
                withBuffer:(CVImageBufferRef)imageBuffer
{
    CVPixelBufferLockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);

    *width = CVPixelBufferGetWidth(imageBuffer)+8; //// TODO: I had to add 8 here, y tho
    *height = CVPixelBufferGetHeight(imageBuffer);
    char *baseBuffer = (char *)CVPixelBufferGetBaseAddress(imageBuffer);
    _cameraBufferSize = CGSizeMake(*width, *height);

    // set_size expects rows, cols format
    img.set_size(*height, *width);

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
}

- (void)populateSampleBuffer:(CVImageBufferRef)imageBuffer withImageArray:(dlib::array2d<dlib::bgr_pixel> &)img
{
    char *baseBuffer = (char *)CVPixelBufferGetBaseAddress(imageBuffer);

    // copy dlib image data back into samplebuffer
    img.reset();
    size_t position = 0;
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

- (void)doWorkOnSampleBuffer:(CMSampleBufferRef)sampleBuffer inRects:(NSArray<NSValue *> *)rects {
    
    if (!self.prepared) {
        [self prepare];
    }
    
    // MARK: magic
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);

    dlib::array2d<dlib::bgr_pixel> img;
    size_t width, height;
    [self populateImageArray:img width:&width height:&height withBuffer:imageBuffer];

    // convert the face bounds list to dlib format
    std::vector<dlib::rectangle> convertedRectangles = [FaceFeatureDetector convertCGRectValueArray:rects];

//    std::vector<dlib::rectangle> faces = detector(img);

    // for the first detected face
    if (convertedRectangles.size() > 0 && convertedRectangles[0].left() > 0 && convertedRectangles[0].right() < width-1) {
        dlib::rectangle oneFaceRect = convertedRectangles[0];
        std::vector<dlib::point> smoothed_points;

        // detect all landmarks
        dlib::full_object_detection shape = sp(img, oneFaceRect);

        // and draw them into the image (samplebuffer)
        for (unsigned int k = 0; k < shape.num_parts(); k++) {
            dlib::point p = shape.part(k);
            double smooth_x = SMOOTH_POINTS ? filter((double)p.x(), (double)frame_number, k*2) : (double)p.x();
            double smooth_y = SMOOTH_POINTS ? filter((double)p.y(), (double)frame_number, k*2+1) : (double)p.y();
            dlib::point smooth_p((unsigned long)smooth_x, (unsigned long)smooth_y);
            smoothed_points.push_back(smooth_p);
            if (DRAW_FACE_DETECTION_POINTS) {
                draw_solid_circle(img, smooth_p, 3, color_for_feature(k));
            }
        }

        if (DRAW_FACE_DETECTION_POINTS) {
            draw_rectangle(img, oneFaceRect, dlib::rgb_pixel(0, 128, 0));
        }

        // reverse-project the face points to determine pose
        [self updateHeadPose_v3:smoothed_points image:img width:width height:height];
    } else {
        self.headPosition = SCNVector3Zero;
    }

    // Use this to calibrate Jesse's wtf hack / TODO remove this (See above TODO)
    dlib::rectangle testRect(0,0,100,100);
    fill_rect(img, testRect, dlib::rgb_pixel(255, 0, 0));

    // lets put everything back where it belongs
    CVPixelBufferLockBaseAddress(imageBuffer, 0);

    [self populateSampleBuffer:imageBuffer withImageArray:img];

    frame_number += FRAME_ADVANCE;
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

- (void)updateHeadPose_v2:(std::vector<dlib::point> &)shape image:(dlib::array2d<dlib::bgr_pixel> &)img width:(size_t)width height:(size_t)height
{
    std::vector<double> rv(3), tv(3);
    cv::Mat rvec(rv),tvec(tv);
    cv::Vec3d eav;

    // Labelling the 3D Points derived from a 3D model of human face.
    // You may replace these points as per your custom 3D head model if any
    std::vector<cv::Point3f > modelPoints;
    modelPoints.push_back(cv::Point3f(2.37427,110.322,21.7776));    // l eye (v 314)
    modelPoints.push_back(cv::Point3f(70.0602,109.898,20.8234));    // r eye (v 0)
    modelPoints.push_back(cv::Point3f(36.8301,78.3185,52.0345));    //nose (v 1879)
    modelPoints.push_back(cv::Point3f(14.8498,51.0115,30.2378));    // l mouth (v 1502)
    modelPoints.push_back(cv::Point3f(58.1825,51.0115,29.6224));    // r mouth (v 695)
    modelPoints.push_back(cv::Point3f(-61.8886f,127.797,-89.4523f));  // l ear (v 2011)
    modelPoints.push_back(cv::Point3f(127.603,126.9,-83.9129f));     // r ear (v 1138)

    // labelling the position of corresponding feature points on the input image.
    std::vector<cv::Point2f> srcImagePoints = {
        cv::Point2f(shape[45].x(), shape[45].y()), // left eye
        cv::Point2f(shape[36].x(), shape[36].y()), // right eye
        cv::Point2f(shape[33].x(), shape[33].y()), // nose
        cv::Point2f(shape[54].x(), shape[54].y()), // left lip corner
        cv::Point2f(shape[48].x(), shape[48].y()), // right lip corner
        cv::Point2f(shape[16].x(), shape[16].y()), // left ear
        cv::Point2f(shape[0].x(), shape[0].y()), // right ear
    };

    cv::Mat ip(srcImagePoints);

    cv::Mat op = cv::Mat(modelPoints);
//    cv::Scalar m = mean(cv::Mat(modelPoints));

    rvec = cv::Mat(rv);
    double _d[9] = {1,0,0,
        0,-1,0,
        0,0,-1};
    Rodrigues(cv::Mat(3,3,CV_64FC1,_d),rvec);
    tv[0]=0;tv[1]=0;tv[2]=1;
    tvec = cv::Mat(tv);

    double _cm[9] = {
        self.cameraFx, 0.0, self.cameraCx,
        0.0, self.cameraFy, self.cameraCy,
        0.0, 0.0, 1.0 };

    cv::Mat camMatrix = cv::Mat(3,3,CV_64FC1, _cm);

    double _dc[] = {0,0,0,0};
    solvePnP(op,ip,camMatrix,cv::Mat(1,4,CV_64FC1,_dc),rvec,tvec,false,cv::SOLVEPNP_EPNP);

    double rot[9] = {0};
    cv::Mat rotM(3,3,CV_64FC1,rot);
    Rodrigues(rvec,rotM);
    double* _r = rotM.ptr<double>();
    printf("rotation mat: \n %.3f %.3f %.3f\n%.3f %.3f %.3f\n%.3f %.3f %.3f\n",
           _r[0],_r[1],_r[2],_r[3],_r[4],_r[5],_r[6],_r[7],_r[8]);

    printf("trans vec: \n %.3f %.3f %.3f\n",tv[0],tv[1],tv[2]);

    double _pm[12] = {_r[0],_r[1],_r[2],tv[0],
        _r[3],_r[4],_r[5],tv[1],
        _r[6],_r[7],_r[8],tv[2]};

    cv::Mat tmp,tmp1,tmp2,tmp3,tmp4,tmp5;
    cv::decomposeProjectionMatrix(cv::Mat(3,4,CV_64FC1,_pm),tmp,tmp1,tmp2,tmp3,tmp4,tmp5,eav);

    self.headPoseAngle =
    SCNVector3Make((float)(eav[0] * M_PI / 180),
                   -(float)(eav[1] * M_PI / 180),
                   -(float)(eav[2] * M_PI / 180) + M_PI);

    printf("Face Rotation Angle:  %.5f %.5f %.5f\n",eav[0],eav[1],eav[2]);

    SCNVector3 position = [self estimatePositionFromShape:shape image:img width:width height:height angle: self.headPoseAngle];
    //    position.z = out_translation.at<double>(2);
    self.headPosition = position;

}

- (void)updateHeadPose_v3:(std::vector<dlib::point> &)shape image:(dlib::array2d<dlib::bgr_pixel> &)img width:(size_t)width height:(size_t)height
{
    double K[9] = {
        self.cameraFx, 0.0, self.cameraCx,
        0.0, self.cameraFy, self.cameraCy,
        0.0, 0.0, 1.0 };

    /* Calibrated parameters!!!
     iPhone X:
     double D[5] = { 1.7479705014455854e-01, -7.3389958871609140e-01,
     -1.1315715905407971e-03, -2.3005306869031618e-03,
     8.9658222837817847e-01 };
     */

    /* iPhone 7: */
    double D[5] = { 1.6306670740348619e-01, -6.9679269326948057e-01,
        -1.1975503089872291e-04, -3.7197654044547885e-03,
        7.6833717412969704e-01 };

    //fill in cam intrinsics and distortion coefficients
    cv::Mat cam_matrix = cv::Mat(3, 3, CV_64FC1, K);
    cv::Mat dist_coeffs = cv::Mat(5, 1, CV_64FC1, D);

    //2D ref points(image coordinates), referenced from detected facial feature
    std::vector<cv::Point2d> image_pts;
    std::vector<cv::Point2d> p3p_image_pts;
    std::vector<cv::Point3d> p3p_object_pts;

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

    // Labelling the 3D Points derived from a 3D model of human face.
    // You may replace these points as per your custom 3D head model if any
    std::vector<cv::Point3f > model_points = {
        cv::Point3f(2.37427,110.322,21.7776),    // l eye (v 314)
        cv::Point3f(70.0602,109.898,20.8234),    // r eye (v 0)
        cv::Point3f(36.8301,78.3185,52.0345),    //nose (v 1879)
        cv::Point3f(14.8498,51.0115,30.2378),    // l mouth (v 1502)
        cv::Point3f(58.1825,51.0115,29.6224),    // r mouth (v 695)
        cv::Point3f(-61.8886f,127.797,-89.4523f),  // l ear (v 2011)
        cv::Point3f(127.603,126.9,-83.9129f)
    };     // r ear (v 1138)

    // labelling the position of corresponding feature points on the input image.
    std::vector<cv::Point2f> src_image_points = {
        cv::Point2f((shape[42].x() + shape[45].x())/2, (shape[42].y() + shape[45].y())/2), // left eye
        cv::Point2f((shape[39].x() + shape[36].x())/2, (shape[39].y() + shape[36].y())/2), // right eye
        cv::Point2f(shape[30].x(), shape[30].y()), // nose
        cv::Point2f(shape[54].x(), shape[54].y()), // left lip corner
        cv::Point2f(shape[48].x(), shape[48].y()), // right lip corner
        cv::Point2f(shape[16].x(), shape[16].y()), // left ear
        cv::Point2f(shape[0].x(), shape[0].y()), // right ear
    };

    //calc pose
    cv::solvePnP(model_points, src_image_points, cam_matrix, dist_coeffs, rotation_vec, translation_vec, false, cv::SOLVEPNP_DLS);

    //calc euler angle

    // Convert rotation vector into rotation matrix
    cv::Rodrigues(rotation_vec, rotation_mat);

    // Combine rotation vector and translation vector into pose matrix
    cv::hconcat(rotation_mat, translation_vec, pose_mat);

    // Extract translation and euler angle from pose matrix
    cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angle);

    // Correct for front camera mirroring

    self.headPoseAngle =
    SCNVector3Make(M_PI + ((15 + euler_angle.at<double>(0)) * M_PI / 180),
                   M_PI + ((10 + euler_angle.at<double>(1)) * M_PI / 180),
                   -(-5 + euler_angle.at<double>(2)) * M_PI / 180);

    // Get the hacked up position vector
    SCNVector3 position = [self estimatePositionFromShape:shape image:img width:width height:height angle: self.headPoseAngle];
//    double newZ = (out_translation.at<double>(2)+1);
//    NSLog(@"%%%% Z: %0.2f vs %0.2f", newZ, position.z);
//    position.z = newZ;

    self.headPosition = position;
}

- (void)updateHeadPose_v1:(std::vector<dlib::point> &)shape image:(dlib::array2d<dlib::bgr_pixel> &)img width:(size_t)width height:(size_t)height
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
        self.cameraFx, 0.0, self.cameraCx,
        0.0, self.cameraFy, self.cameraCy,
        0.0, 0.0, 1.0 };

/*
    double K[9] = {
        1.4551780026980582e+03, 0., 9.5902575796350970e+02, 0.,
        1.4576596237871390e+03, 5.4933833052386569e+02, 0., 0., 1.
    };*/

    /* Calibrated parameters!!!
     iPhone X:
    double D[5] = { 1.7479705014455854e-01, -7.3389958871609140e-01,
        -1.1315715905407971e-03, -2.3005306869031618e-03,
        8.9658222837817847e-01 };
     */

    /* iPhone 7: */
    double D[5] = { 1.6306670740348619e-01, -6.9679269326948057e-01,
        -1.1975503089872291e-04, -3.7197654044547885e-03,
        7.6833717412969704e-01 };

    //fill in cam intrinsics and distortion coefficients
    cv::Mat cam_matrix = cv::Mat(3, 3, CV_64FC1, K);
    cv::Mat dist_coeffs = cv::Mat(5, 1, CV_64FC1, D);

    //2D ref points(image coordinates), referenced from detected facial feature
    std::vector<cv::Point2d> image_pts;
    std::vector<cv::Point2d> p3p_image_pts;
    std::vector<cv::Point3d> p3p_object_pts;

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
    cv::solvePnP(object_pts, image_pts, cam_matrix, dist_coeffs, rotation_vec, translation_vec, false, cv::SOLVEPNP_DLS);

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

    // Extract translation and euler angle from pose matrix
    cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angle);
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

    if (SMOOTH_PROJECTION) {
        double smooth_euler_0 = filter(euler_angle.at<double>(0), frame_number, (2*68)+0) * M_PI/180;
        double smooth_euler_1 = filter(euler_angle.at<double>(1), frame_number, (2*68)+1) * M_PI/180;
        double smooth_euler_2 = filter(fmodf(euler_angle.at<double>(2) + 180, 360), frame_number, (2*68)+2) * M_PI/180;
        self.headPoseAngle = SCNVector3Make(M_PI + smooth_euler_0, M_PI + smooth_euler_1, -smooth_euler_2 - M_PI);//euler_angle.at<double>(2) * M_PI / 180);
    } else {
        self.headPoseAngle =
        SCNVector3Make(M_PI + (euler_angle.at<double>(0) * M_PI / 180),
                       M_PI + (euler_angle.at<double>(1) * M_PI / 180),
                       -euler_angle.at<double>(2) * M_PI / 180);
    }

    // Get the hacked up position vector
    SCNVector3 position = [self estimatePositionFromShape:shape image:img width:width height:height angle: self.headPoseAngle];
//    position.z = out_translation.at<double>(2);
    self.headPosition = position;

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
}

/* this is a total hack just to see if I can get anywhere close
 */
- (SCNVector3)estimatePositionFromShape:(std::vector<dlib::point> &)shape
                                  image:(dlib::array2d<dlib::bgr_pixel> &)img
                                  width:(size_t)width
                                 height:(size_t)height
                                  angle:(SCNVector3)angle
{
    double x = 0;
    double y = 0;
    double min_x = 1000000;
    double max_x = 0;

    // Take the average of points for eyebrows + nose
    static int startPoint = 17;
    static int endPoint = 67;
    for (int i=startPoint; i <= endPoint; i++) {
        x += shape[i].x();
        y += shape[i].y();
        max_x = MAX(shape[i].x(), max_x);
        min_x = MIN(shape[i].x(), min_x);
    }
    x /= (endPoint-startPoint);
    y /= (endPoint-startPoint);

    if (DRAW_FACE_DETECTION_POINTS) {
        draw_solid_circle(img, dlib::point(x, y), 6, dlib::rgb_pixel(255, 0, 0));
    }

//    x = shape[28].x();
//    y = shape[28].y();

    double divisor = self.slider1Value * 3; // 1.5
    double downscale_factor = self.slider2Value * 1000; // 722

    NSLog(@"divisor: %0.2f / downscale factor: %0.1f", divisor, downscale_factor);
    double z = ((double)width/divisor) - abs((max_x - min_x)/cosf(angle.y));
//    double z = ((double)width/divisor) - (max_x - min_x);
    NSLog(@"z: %0.1f / y angle: %0.2f / cos y %0.2f", z, angle.y, cosf(angle.y));
    z = z/downscale_factor;
    NSLog(@"I think Z is: %0.2f", z);

    // Return tip of nose
    return SCNVector3Make(shape[33].x(), shape[33].y(), z);
}

@end
