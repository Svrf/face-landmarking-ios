

import SvrfFaceFilterKitPrivate
import AVFoundation
import SceneKit

public class FaceTrackingHandler : NSObject, AVCaptureVideoDataOutputSampleBufferDelegate, AVCaptureMetadataOutputObjectsDelegate {
    public var refNode: SCNNode? {
        didSet {
            if let occluderNode = refNode?.childNode(withName: "Occluder",
                                                      recursively: true) {

                // Get bounding rectangle and add insets to find the nose tip, align model to it for handling positioning
                let (minBox, maxBox) = occluderNode.boundingBox
                let width = maxBox.x - minBox.x
                let height = maxBox.y - minBox.y
                let depth = maxBox.z - minBox.z
                // These constants are specific to our particular occluder.
                let nose = SCNVector3Make((maxBox.x + minBox.x)/2 - width*0.1, ((maxBox.y + minBox.y)/2) - height*0.1, maxBox.z + depth*1.7)
                print("Nose from bounding box: \(nose)")
                refNode?.pivot = SCNMatrix4MakeTranslation(nose.x*occluderNode.scale.z,
                                                           nose.y*occluderNode.scale.z,
                                                           nose.z*occluderNode.scale.z)
            }
       }
    }
    public var scnView: SCNView?

    public var session = AVCaptureSession()
    public var hfov: Float?
    public var vfov: Float?
    public let layer = AVSampleBufferDisplayLayer()
    let sampleQueue = DispatchQueue(label: "com.svrf.sampleQueue", attributes: [])
    let faceQueue = DispatchQueue(label: "com.svrf.faceQueue", attributes: [])
    let detector = FaceFeatureDetector()

    public var yawWarning: UILabel?
    public var rollWarning: UILabel?
    
    var currentMetadata: [AnyObject]

    var metadataOutput: AVCaptureMetadataOutput?
    
    override public init() {
        currentMetadata = []
        super.init()
    }
    
    public func openSession() {
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
        let dim = CMVideoFormatDescriptionGetPresentationDimensions(fDesc, usePixelAspectRatio: true, useCleanAperture: true)

        let cx = Float(dim.width) / 2.0
        let cy = Float(dim.height) / 2.0

        let HFOV = format.videoFieldOfView
        self.hfov = HFOV
        let VFOV = ((HFOV)/cx)*cy
        self.vfov = VFOV

        let fx = abs(Float(dim.width) / (2 * tan(HFOV / 180 * Float.pi / 2)))
        let fy = abs(Float(dim.height) / (2 * tan(VFOV / 180 * Float.pi / 2)))
        print("FX: \(fx) FY: \(fy) CX: \(cx) CY: \(cy)")
        detector?.cameraFx = Double(fx)
        detector?.cameraFy = Double(fy)
        detector?.cameraCx = Double(cx)
        detector?.cameraCy = Double(cy)
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
        
        detector?.prepare()
        
        session.startRunning()
    }

    public func updateSlider1(_ newValue: Float) {
        detector?.slider1Value = Double(newValue)
    }

    public func updateSlider2(_ newValue: Float) {
        detector?.slider2Value = Double(newValue)
    }

    // MARK: AVCaptureVideoDataOutputSampleBufferDelegate
    public func captureOutput(_ output: AVCaptureOutput,
                              didOutput sampleBuffer: CMSampleBuffer,
                              from connection: AVCaptureConnection) {

        if !currentMetadata.isEmpty {
            var yawMetadata:CGFloat?
            var rollMetadata:CGFloat?
            let boundsArray = currentMetadata
                .compactMap { $0 as? AVMetadataFaceObject }
                .map { (faceObject) -> NSValue in
                    let convertedObject = output.transformedMetadataObject(for: faceObject, connection: connection)!
                    let rect = convertedObject.bounds
                    if (faceObject.hasYawAngle) {
                        yawMetadata = faceObject.yawAngle
                        print("Yaw metadata: \(String(describing: yawMetadata))")
/*                        if (yawMetadata! > 180.0 && yawMetadata! <= 315.0) {
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
                        }*/
                    }
                    if (faceObject.hasRollAngle) {
                        rollMetadata = faceObject.rollAngle
                    }

                    return NSValue(cgRect: rect)
            }
            
            detector?.doWork(on: sampleBuffer, inRects: boundsArray)
            if let angle = detector?.headPoseAngle, let position = detector?.headPosition,
                (position.x != 0 || position.y != 0 || position.z != 0)  {
                DispatchQueue.main.async { 
                    self.refNode?.isHidden = false
//                    var validAngle = true
                    // TODO: Use these as references to de-noise the data
                    if let roll = rollMetadata {
                        let adjustedRoll = (Float(roll)*Float.pi/180) - Float.pi/2
                        let rollDiff = min(abs(adjustedRoll - angle.z), abs(adjustedRoll - angle.z - 2*Float.pi))
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
                        let yawDiff = min(abs(adjustedYaw - angle.y), abs(adjustedYaw - angle.y + 2*Float.pi))
                        if yawDiff > Float.pi/3 {
                            self.yawWarning?.isHidden = false
                            self.yawWarning?.text = String(format:"Yaw off: %0.2f", yawDiff)
                            self.hideFaceFilters()
  //                          validAngle = false
                        } else {
                            self.refNode?.isHidden = false

                            self.yawWarning?.isHidden = true
                        }
                    }
//                    if (validAngle) {
                        self.refNode?.eulerAngles = angle
//                    }
                    let scaledPosition = SCNVector3(x: position.x * Float(self.scnView!.frame.size.width/self.detector!.cameraBufferSize.width),
                                                    y: position.y * Float(self.scnView!.frame.size.height/self.detector!.cameraBufferSize.height),
                                                    z: position.z)
                    let noseTip = self.scnView!.unprojectPoint(scaledPosition)
//                    let (_, sphereRadius) = self.refNode!.boundingSphere
//                    let xCenter = cos(Float.pi/2 - angle.x)*sphereRadius
//                    let yCenter = sin(Float.pi/2 - angle.y)*sphereRadius
//                    let zCenter = cos(Float.pi/2 - angle.z)*sphereRadius
//                    let centerPoint = SCNVector3Make(noseTip.x + xCenter, noseTip.y + yCenter, noseTip.z + zCenter)
                    self.refNode?.position = noseTip //self.scnView!.unprojectPoint(scaledPosition)
                    print("Position: \(position) - unprojected: \(self.refNode!.position)")
                }
            } else {
                DispatchQueue.main.async {
                    self.hideFaceFilters()
                }
            }
        } else {
            hideFaceFilters()
        }

        layer.enqueue(sampleBuffer)
    }
    
    public func captureOutput(_ output: AVCaptureOutput, didDrop sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        print("DidDropSampleBuffer")
    }
    
    // MARK: AVCaptureMetadataOutputObjectsDelegate
    
    public func metadataOutput(_ output: AVCaptureMetadataOutput, didOutput metadataObjects: [AVMetadataObject], from connection: AVCaptureConnection) {
        currentMetadata = metadataObjects as [AnyObject]
    }

    private func hideFaceFilters() {
        self.refNode?.isHidden = true
        detector?.resetFrameNumber()
    }
}
