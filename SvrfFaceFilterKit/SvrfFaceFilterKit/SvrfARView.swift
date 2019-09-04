//
//  SvrfARView.swift
//  SvrfFaceFilterKit
//
//  Created by Jesse Boyes on 9/4/19.
//  Copyright © 2019 Svrf, Inc. All rights reserved.
//

import UIKit
import ARKit

open class SvrfARView: UIView {
    enum RenderMode {
        case ARKit
        case Svrf
    }

    private var renderMode: RenderMode!

    // Common
    private var faceRootNode: SCNNode?

    // ARKit face tracking
    private var arSceneView: ARSCNView?

    // Svrf face tracking
    private var sessionHandler: FaceTrackingHandler?
    private var sceneView: SCNView?
    private var camera: SCNCamera?

    override init(frame: CGRect) {
        super.init(frame: frame)
        setup()
    }

    required public init?(coder aDecoder: NSCoder) {
        super.init(coder: aDecoder)
        setup()
    }

    public func addChildNode(_ node: SCNNode) {
        switch renderMode! {
        case .Svrf:
            sessionHandler?.refNode?.addChildNode(node)
        case .ARKit:
            faceRootNode?.addChildNode(node)
            print("TODO");
        }
    }

    public func updateSlider1(_ value: Float) {
        sessionHandler?.updateSlider1(value)
    }

    public func updateSlider2(_ value: Float) {
        sessionHandler?.updateSlider2(value)
    }

    public func calibrateNodes() {
        sessionHandler?.calibrateNodes()
    }

    override open func layoutSubviews() {
        super.layoutSubviews()
        if renderMode == .ARKit {
            arSceneView?.frame = self.bounds
        } else {
            sceneView?.frame = self.bounds
            sessionHandler?.layer.frame = self.bounds
        }
    }

    public func setWarningLabels(yaw: UILabel, roll: UILabel) {
        sessionHandler?.yawWarning = yaw
        sessionHandler?.rollWarning = roll
    }

    // MARK: - Private methods

    private func setup() {
        if ARFaceTrackingConfiguration.isSupported {
            renderMode = .ARKit

            arSceneView = ARSCNView(frame: self.bounds)
            arSceneView?.delegate = self
            let configuration = ARFaceTrackingConfiguration()
            arSceneView?.session.run(configuration)

            self.addSubview(arSceneView!)
        } else {
            renderMode = .Svrf

            sessionHandler = FaceTrackingHandler()
            sceneView = SCNView(frame: self.bounds)
            if let sessionHandler = sessionHandler, let sceneView = sceneView {
                self.addSubview(sceneView)
                sessionHandler.scnView = sceneView
                sessionHandler.openSession()

                let scene = SCNScene()
                sceneView.backgroundColor = .clear
                sceneView.scene = scene
                sceneView.autoenablesDefaultLighting = true
                //        sceneView.alpha = 0.5

                let camera = SCNCamera()
                let cameraNode = SCNNode()
                cameraNode.camera = camera
                self.camera = camera
                camera.fieldOfView = fieldOfView()
                cameraNode.position = SCNVector3(x: 0.0, y: 0.0, z: 1)

                //        let light = SCNLight()
                //        light.type = .omni
                //        let lightNode = SCNNode()
                //        lightNode.light = light
                //        lightNode.position = SCNVector3(x: 1.5, y: 1.5, z: 1.5)
                //
                //        scene.rootNode.addChildNode(lightNode)
                scene.rootNode.addChildNode(cameraNode)

                let layer = sessionHandler.layer
                layer.frame = self.bounds
                self.layer.insertSublayer(layer, below: sceneView.layer)

                let refNode = SCNNode()
                sessionHandler.refNode = refNode
                sceneView.scene?.rootNode.addChildNode(refNode)
                faceRootNode = refNode
            }
        }
    }

    private func fieldOfView() -> CGFloat {
        // iPhone 7: 55
        // iPhone X: 65

        //        if let fov = sessionHandler.hfov {
        //            camera.fieldOfView = CGFloat(fov/2)
        //            camera.projectionDirection = .horizontal
        //        }

        print("Device model: \(UIDevice.modelName)")
        if UIDevice.modelName == .iPhone7 {
            return 65
        } else {
            return 55
        }
    }
}

extension SvrfARView : ARSCNViewDelegate {

    // ARNodeTracking
    public func renderer(_ renderer: SCNSceneRenderer, didAdd node: SCNNode, for anchor: ARAnchor) {

        // Hold onto the `faceNode` so that the session does not need to be restarted when switching face filters.
        faceRootNode = node

        // Put code into async thread
//        serialQueue.async {
//
//            // Setup face node content
//            self.setupFaceNodeContent()
//        }
    }

    // ARFaceGeometryUpdate
    public func renderer(_ renderer: SCNSceneRenderer, didUpdate node: SCNNode, for anchor: ARAnchor) {

        // FaceAnchor unwrapping
        guard let faceAnchor = anchor as? ARFaceAnchor, let device = renderer.device else { return }

        // Update virtualFaceNode with FaceAnchor and MTLDevice
        // TODO: propagate blend shapes
//        virtualFaceNode?.update(withFaceAnchor: faceAnchor, andMTLDevice: device)
    }

}
