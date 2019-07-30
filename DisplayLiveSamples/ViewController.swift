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

class ViewController: UIViewController {
    let sessionHandler = SessionHandler()
    
    @IBOutlet weak var preview: UIView!
    @IBOutlet weak var sceneView: SCNView!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.view.backgroundColor = .blue
        sceneView.allowsCameraControl = true

        let scene = SCNScene()
        sceneView.scene = scene

        let camera = SCNCamera()
        let cameraNode = SCNNode()
        cameraNode.camera = camera
        cameraNode.position = SCNVector3(x: 0.0, y: 0.0, z: 3.0)

        let light = SCNLight()
        light.type = .omni
        let lightNode = SCNNode()
        lightNode.light = light
        lightNode.position = SCNVector3(x: 1.5, y: 1.5, z: 1.5)

        scene.rootNode.addChildNode(lightNode)
        scene.rootNode.addChildNode(cameraNode)
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }

    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        
        sessionHandler.openSession()

        let layer = sessionHandler.layer
        layer.frame = preview.bounds
        preview.layer.insertSublayer(layer, below: sceneView.layer)

        let cubeGeometry = SCNBox(width: 1.0, height: 1.0, length: 1.0, chamferRadius: 0.0)
        let cubeNode = SCNNode(geometry: cubeGeometry)

        sessionHandler.refNode = cubeNode
        sceneView.scene?.rootNode.addChildNode(cubeNode)

        view.layoutIfNeeded()
    }

}

