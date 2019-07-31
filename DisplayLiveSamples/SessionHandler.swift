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
    
    var currentMetadata: [AnyObject]
    
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
            let boundsArray = currentMetadata
                .compactMap { $0 as? AVMetadataFaceObject }
                .map { (faceObject) -> NSValue in
                    let convertedObject = output.transformedMetadataObject(for: faceObject, connection: connection)
                    return NSValue(cgRect: convertedObject!.bounds)
            }
            
            wrapper?.doWork(on: sampleBuffer, inRects: boundsArray)
            if let angle = wrapper?.headPoseAngle, let position = wrapper?.headPosition {
                DispatchQueue.main.async { 
                    self.refNode?.eulerAngles = angle
                    let scaledPosition = SCNVector3(x: position.x * Float(self.scnView!.frame.size.width/self.wrapper!.cameraBufferSize.width),
                                                    y: position.y * Float(self.scnView!.frame.size.height/self.wrapper!.cameraBufferSize.height),
                                                    z: position.z)
                    self.refNode?.position = self.scnView!.unprojectPoint(scaledPosition)
                    print("Position: \(position) - unprojected: \(self.refNode!.position)")
                }
            }
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
