//
//  ViewController.swift
//  DisplayLiveSamples
//
//  Created by Luis Reisewitz on 15.05.16.
//  Copyright © 2016 ZweiGraf. All rights reserved.
//

import UIKit
import AVFoundation
import SceneKit
import SvrfGLTFSceneKit

class ViewController: UIViewController {
    let sessionHandler = FaceTrackingHandler()
    
    @IBOutlet weak var preview: UIView!
    @IBOutlet weak var sceneView: SCNView!

    @IBOutlet weak var yawWarning: UILabel!
    @IBOutlet weak var rollWarning: UILabel!

    override func viewDidLoad() {
        super.viewDidLoad()

        sessionHandler.scnView = sceneView
        sessionHandler.openSession()

        sessionHandler.yawWarning = yawWarning
        sessionHandler.rollWarning = rollWarning

        let scene = SCNScene()
        sceneView.scene = scene
        sceneView.autoenablesDefaultLighting = true
        sceneView.alpha = 0.5

        let camera = SCNCamera()
        let cameraNode = SCNNode()
        cameraNode.camera = camera
        if let fov = sessionHandler.hfov {
            camera.fieldOfView = CGFloat(fov)
            camera.projectionDirection = .horizontal
        }
        cameraNode.position = SCNVector3(x: 0.0, y: 0.0, z: 1)

//        let light = SCNLight()
//        light.type = .omni
//        let lightNode = SCNNode()
//        lightNode.light = light
//        lightNode.position = SCNVector3(x: 1.5, y: 1.5, z: 1.5)
//
//        scene.rootNode.addChildNode(lightNode)
        scene.rootNode.addChildNode(cameraNode)
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }

    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)

        let layer = sessionHandler.layer
        layer.frame = preview.bounds
        preview.layer.insertSublayer(layer, below: sceneView.layer)

        let model = try! loadFaceNode("headwear1.glb")
        model.scale = SCNVector3(10,10,10)//3.2,3.2,3.2) // TODO why does this need to be scaled up so much?

//        let cubeGeometry = SCNBox(width: 0.1, height: 0.1, length: 0.1, chamferRadius: 0.0)
//        let cubeNode = SCNNode(geometry: cubeGeometry)

        sessionHandler.refNode = model
        sceneView.scene?.rootNode.addChildNode(sessionHandler.refNode!)

        view.layoutIfNeeded()
    }

    @IBAction func slider1Changed(_ sender: UISlider) {
        sessionHandler.updateSlider1(sender.value)
    }

    @IBAction func slider2Changed(_ sender: UISlider) {
        sessionHandler.updateSlider2(sender.value)
    }


    // MARK: - Model management

    private func loadFaceNode(_ glbFile: String) throws -> SCNNode {
        let modelSource = try GLTFSceneSource(path: glbFile)

        let faceFilterNode = SCNNode()
        let sceneNode = try modelSource.scene().rootNode

        if let occluderNode = sceneNode.childNode(withName: "Occluder",
                                                  recursively: true) {
            faceFilterNode.addChildNode(occluderNode)
//            setOccluderNode(node: occluderNode)
        }

        if let headNode = sceneNode.childNode(withName: "Head", recursively: true) {
            faceFilterNode.addChildNode(headNode)
        }

        faceFilterNode.morpher?.calculationMode = SCNMorpherCalculationMode.normalized

        return faceFilterNode
    }

    /**
     Sets a node to have all of its children set as an occluder.
     - Parameters:
     - node: A *SCNNode* likely named *Occluder*.
     */
    private func setOccluderNode(node: SCNNode) {

        // Any child of this node should be occluded
        node.enumerateHierarchy { (childNode, _) in
            childNode.geometry?.firstMaterial?.colorBufferWriteMask = []
            childNode.renderingOrder = -1
        }
    }

}

