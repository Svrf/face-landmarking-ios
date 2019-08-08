//
//  SessionHandler.swift
//  DisplayLiveSamples
//
//  Created by Luis Reisewitz on 15.05.16.
//  Copyright © 2016 ZweiGraf. All rights reserved.
//

import AVFoundation
import SceneKit

class SessionHandler : NSObject, AVCaptureVideoDataOutputSampleBufferDelegate, AVCaptureMetadataOutputObjectsDelegate {
    var refNode: SCNNode?
    var scnView: SCNView?

    var session = AVCaptureSession()
    var hfov: Float?
    var vfov: Float?
    let layer = AVSampleBufferDisplayLayer()
    let sampleQueue = DispatchQueue(label: "com.zweigraf.DisplayLiveSamples.sampleQueue", attributes: [])
    let faceQueue = DispatchQueue(label: "com.zweigraf.DisplayLiveSamples.faceQueue", attributes: [])
    let wrapper = DlibWrapper()

    var yawWarning: UILabel?
    var rollWarning: UILabel?
    
    var currentMetadata: [AnyObject]

    var metadataOutput: AVCaptureMetadataOutput?
    
    override init() {
        currentMetadata = []
        super.init()
    }
    
    func openSession() {
        let device = AVCaptureDevice.devices(for: AVMediaType.video)
            .map { $0 }
            .filter { $0.position == .front}
            .first!
        
        let input = try! AVCaptureDeviceInput(device: device)
        
        let output = AVCaptureVideoDataOutput()
        output.setSampleBufferDelegate(self, queue: sampleQueue)
        
        let metaOutput = AVCaptureMetadataOutput()
        metaOutput.setMetadataObjectsDelegate(self, queue: faceQueue)

        // Calc fx/fy/cx/cy
        let format = device.activeFormat;

        let fDesc = format.formatDescription;
        let dim = CMVideoFormatDescriptionGetPresentationDimensions(fDesc, true, true)

        let cx = Float(dim.width) / 2.0
        let cy = Float(dim.height) / 2.0

        let HFOV = format.videoFieldOfView
        self.hfov = HFOV
        let VFOV = ((HFOV)/cx)*cy
        self.vfov = VFOV

        let fx = abs(Float(dim.width) / (2 * tan(HFOV / 180 * Float.pi / 2)))
        let fy = abs(Float(dim.height) / (2 * tan(VFOV / 180 * Float.pi / 2)))
        print("FX: \(fx) FY: \(fy) CX: \(cx) CY: \(cy)")
        wrapper?.cameraFx = Double(fx)
        wrapper?.cameraFy = Double(fy)
        wrapper?.cameraCx = Double(cx)
        wrapper?.cameraCy = Double(cy)
        // TODO: Pass these through Session / dlib helper

        session.beginConfiguration()
        
        if session.canAddInput(input) {
            session.addInput(input)
        }
        if session.canAddOutput(output) {
            session.addOutput(output)
            if output.connections.count > 0 {
                output.connections[0].videoOrientation = .portrait
                output.connections[0].isVideoMirrored = true
            }
        }
        if session.canAddOutput(metaOutput) {
            session.addOutput(metaOutput)
        }
        metadataOutput = metaOutput
        
        session.commitConfiguration()
        
        let settings: [AnyHashable: Any] = [kCVPixelBufferPixelFormatTypeKey as AnyHashable: Int(kCVPixelFormatType_32BGRA)]
        output.videoSettings = settings as? [String : Any]
    
        // availableMetadataObjectTypes change when output is added to session.
        // before it is added, availableMetadataObjectTypes is empty
        metaOutput.metadataObjectTypes = [AVMetadataObject.ObjectType.face]
        
        wrapper?.prepare()
        
        session.startRunning()
    }

    func updateSlider1(_ newValue: Float) {
        wrapper?.slider1Value = Double(newValue)
    }

    func updateSlider2(_ newValue: Float) {
        wrapper?.slider2Value = Double(newValue)
    }

    // MARK: AVCaptureVideoDataOutputSampleBufferDelegate
    func captureOutput(_ output: AVCaptureOutput,
                       didOutput sampleBuffer: CMSampleBuffer,
                       from connection: AVCaptureConnection) {

        if !currentMetadata.isEmpty {
            var yawMetadata:CGFloat?
            var rollMetadata:CGFloat?
            let boundsArray = currentMetadata
                .compactMap { $0 as? AVMetadataFaceObject }
                .map { (faceObject) -> NSValue in
                    let convertedObject = output.transformedMetadataObject(for: faceObject, connection: connection)!
                    var rect = convertedObject.bounds
                    if (faceObject.hasYawAngle) {
                        yawMetadata = faceObject.yawAngle
                        print("Yaw metadata: \(String(describing: yawMetadata))")
                        if (yawMetadata! > 180.0 && yawMetadata! <= 315.0) {
                            rect = CGRect(x: rect.minX,// - (rect.width/4),
                                          y: rect.minY,
                                          width: rect.width * 4/5,
                                          height: rect.height)
                        }
                        if (yawMetadata! >= 45.0 && yawMetadata! < 180.0) {
                            rect = CGRect(x: rect.minX + (rect.width * 1/5),
                                y: rect.minY,
                                width: rect.width * 4/5,
                                height: rect.height)
                        }
                    }
                    if (faceObject.hasRollAngle) {
                        rollMetadata = faceObject.rollAngle
                    }

                    return NSValue(cgRect: rect)
            }
            
            wrapper?.doWork(on: sampleBuffer, inRects: boundsArray)
            if var angle = wrapper?.headPoseAngle, let position = wrapper?.headPosition,
                (position.x != 0 || position.y != 0 || position.z != 0)  {
                DispatchQueue.main.async { 
                    self.refNode?.isHidden = false
                    var validAngle = true
                    // TODO: Use these as references to de-noise the data
                    if let roll = rollMetadata {
                        let adjustedRoll = (Float(roll)*Float.pi/180) - Float.pi/2
                        let rollDiff = min(fabs(adjustedRoll - angle.z), fabs(adjustedRoll - angle.z - 2*Float.pi))
                        if rollDiff > Float.pi/2 {
                            self.rollWarning?.isHidden = false
                            self.rollWarning?.text = String(format:"Roll off: %0.2f", rollDiff)
//                            validAngle = false
                        } else {
                            self.rollWarning?.isHidden = true
                        }
                    }
                    if let yaw = yawMetadata {
                        let adjustedYaw = -(Float(yaw)*Float.pi/180) + Float.pi
                        let yawDiff = min(fabs(adjustedYaw - angle.y), fabs(adjustedYaw - angle.y + 2*Float.pi))
                        if yawDiff > Float.pi/3 {
                            self.yawWarning?.isHidden = false
                            self.yawWarning?.text = String(format:"Yaw off: %0.2f", yawDiff)
                            self.refNode?.isHidden = true
  //                          validAngle = false
                        } else {
                            self.refNode?.isHidden = false

                            self.yawWarning?.isHidden = true
                        }
                    }
                    if (validAngle) {
                        self.refNode?.eulerAngles = angle
                    }
                    let scaledPosition = SCNVector3(x: position.x * Float(self.scnView!.frame.size.width/self.wrapper!.cameraBufferSize.width),
                                                    y: position.y * Float(self.scnView!.frame.size.height/self.wrapper!.cameraBufferSize.height),
                                                    z: position.z)
                    self.refNode?.position = self.scnView!.unprojectPoint(scaledPosition)
                    print("Position: \(position) - unprojected: \(self.refNode!.position)")
                }
            } else {
                DispatchQueue.main.async {
                    self.refNode?.isHidden = true
                }
            }
        } else {
            self.refNode?.isHidden = true
        }

        layer.enqueue(sampleBuffer)
    }
    
    func captureOutput(_ output: AVCaptureOutput, didDrop sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        print("DidDropSampleBuffer")
    }
    
    // MARK: AVCaptureMetadataOutputObjectsDelegate
    
    func metadataOutput(_ output: AVCaptureMetadataOutput, didOutput metadataObjects: [AVMetadataObject], from connection: AVCaptureConnection) {
        currentMetadata = metadataObjects as [AnyObject]
    }
}
