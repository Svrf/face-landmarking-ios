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
import SvrfFaceFilterKit

class ViewController: UIViewController {
    let sessionHandler = FaceTrackingHandler()
    
    @IBOutlet weak var preview: UIView!
    @IBOutlet weak var sceneView: SCNView!

    @IBOutlet weak var yawWarning: UILabel!
    @IBOutlet weak var rollWarning: UILabel!

    var headbandNode: SCNNode?
    var glassesNode: SCNNode?
    var earringsNode: SCNNode?
    var mustacheNode: SCNNode?
    var camera: SCNCamera?

    override func viewDidLoad() {
        super.viewDidLoad()

        sessionHandler.scnView = sceneView
        sessionHandler.openSession()

        sessionHandler.yawWarning = yawWarning
        sessionHandler.rollWarning = rollWarning

        let scene = SCNScene()
        sceneView.scene = scene
        sceneView.autoenablesDefaultLighting = true
//        sceneView.alpha = 0.5

        let camera = SCNCamera()
        let cameraNode = SCNNode()
        cameraNode.camera = camera
        self.camera = camera
        // iPhone 7: 55
        // iPhone X: 65
        camera.fieldOfView = 65
//        if let fov = sessionHandler.hfov {
//            camera.fieldOfView = CGFloat(fov/3)
//            camera.projectionDirection = .horizontal
//        }
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
        let refNode = SCNNode()

        let headwear = try! loadFaceNode("headwear1.glb")
        headwear.scale = SCNVector3(6,6,6)//3.2,3.2,3.2) // TODO why does this need to be scaled up so much?
        headbandNode = headwear
        refNode.addChildNode(headwear)

        let glasses = try! loadFaceNode("eyewear1.glb")
        glasses.scale = SCNVector3(6,6,6)//3.2,3.2,3.2) // TODO why does this need to be scaled up so much?
        glassesNode = glasses
        refNode.addChildNode(glasses)

        let earrings = try! loadFaceNode("earrings.glb")
        earrings.scale = SCNVector3(6,6,6)//3.2,3.2,3.2) // TODO why does this need to be scaled up so much?
        earringsNode = earrings
        refNode.addChildNode(earrings)

        let mustache = try! loadFaceNode("mustache.glb")
        mustache.scale = SCNVector3(6,6,6)//3.2,3.2,3.2) // TODO why does this need to be scaled up so much?
        mustacheNode = mustache
        refNode.addChildNode(mustache)

//        let cubeGeometry = SCNBox(width: 0.1, height: 0.1, length: 0.1, chamferRadius: 0.0)
//        let cubeNode = SCNNode(geometry: cubeGeometry)

        sessionHandler.refNode = refNode
        sceneView.scene?.rootNode.addChildNode(sessionHandler.refNode!)

        view.layoutIfNeeded()
    }

    @IBAction func slider1Changed(_ sender: UISlider) {
        sessionHandler.updateSlider1(sender.value)
    }

    @IBAction func slider2Changed(_ sender: UISlider) {
        sessionHandler.updateSlider2(sender.value)
    }

    @IBAction func slider3Changed(_ sender: UISlider) {
        camera?.fieldOfView = CGFloat(sender.value*180)
        print("Field of view: \(String(describing: camera?.fieldOfView))")
    }

    @IBAction func headbandTapped(_ sender: UIBarButtonItem) {
        headbandNode?.isHidden = !headbandNode!.isHidden
    }

    @IBAction func glassesTapped(_ sender: UIBarButtonItem) {
        glassesNode?.isHidden = !glassesNode!.isHidden
    }

    @IBAction func earringsTapped(_ sender: UIBarButtonItem) {
        earringsNode?.isHidden = !earringsNode!.isHidden
    }

    @IBAction func mustacheTapped(_ sender: UIBarButtonItem) {
        mustacheNode?.isHidden = !mustacheNode!.isHidden
    }


    // MARK: - Model management

    private func loadFaceNode(_ glbFile: String) throws -> SCNNode {
        let modelSource = try GLTFSceneSource(path: glbFile)

        let faceFilterNode = SCNNode()
        let sceneNode = try modelSource.scene().rootNode

        if let occluderNode = sceneNode.childNode(withName: "Occluder",
                                                  recursively: true) {
            faceFilterNode.addChildNode(occluderNode)
            setOccluderNode(node: occluderNode)
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

