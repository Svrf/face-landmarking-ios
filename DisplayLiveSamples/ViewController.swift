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
    
    @IBOutlet weak var arView: SvrfARView!

    @IBOutlet weak var yawWarning: UILabel!
    @IBOutlet weak var rollWarning: UILabel!

    let headwearFiles = [
        "headwear1.glb",
        "headwear2.glb",
        "headwear3.glb",
        "headwear4.glb",
        "headwear5.glb",
        nil,
    ]
    var headwearIndex = 0

    let mustacheFiles = [
        "mustache1.glb",
        "mustache2.glb",
        "mustache3.glb",
        nil,
    ]
    var mustacheIndex = 0

    let glassesFiles = [
        "glasses1.glb",
        "glasses2.glb",
        "glasses3.glb",
        "glasses4.glb",
        "glasses5.glb",
        nil,
    ]
    var glassesIndex = 0

    let earringsFiles = [
        "earrings1.glb",
        "earrings2.glb",
        "earrings3.glb",
        nil,
    ]
    var earringsIndex = 0

    let skinFiles = [
        "skin1.glb",
        "skin2.glb",
        "skin3.glb",
        nil,
    ]
    var skinIndex = 0

    var headwearNode: SCNNode?
    var glassesNode: SCNNode?
    var earringsNode: SCNNode?
    var mustacheNode: SCNNode?
    var camera: SCNCamera?

    override func viewDidLoad() {
        super.viewDidLoad()
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }

    func addNode(_ name: String) -> SCNNode {
        let node = try! loadFaceNode(name)
        arView.addChildNode(node)
//        sessionHandler.refNode!.addChildNode(node)

        return node
    }

    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)

        headwearNode = addNode("headwear1.glb")
        glassesNode = addNode("eyewear1.glb")
        earringsNode = addNode("earrings1.glb")
        mustacheNode = addNode("mustache.glb")

        arView.calibrateNodes()
        arView.setWarningLabels(yaw: yawWarning, roll: rollWarning)

//        let cubeGeometry = SCNBox(width: 0.1, height: 0.1, length: 0.1, chamferRadius: 0.0)
//        let cubeNode = SCNNode(geometry: cubeGeometry)

//        sceneView.scene?.rootNode.addChildNode(sessionHandler.refNode!)

        view.layoutIfNeeded()
    }

    @IBAction func slider1Changed(_ sender: UISlider) {
        arView.updateSlider1(sender.value)
    }

    @IBAction func slider2Changed(_ sender: UISlider) {
        arView.updateSlider2(sender.value)
    }

    @IBAction func slider3Changed(_ sender: UISlider) {
        camera?.fieldOfView = CGFloat(sender.value*180)
        print("Field of view: \(String(describing: camera?.fieldOfView))")
    }

    @IBAction func headbandTapped(_ sender: UIBarButtonItem) {
        headwearIndex += 1
        headwearIndex %= headwearFiles.count
        if let file = headwearFiles[headwearIndex] {
            headwearNode?.removeFromParentNode()
            headwearNode = addNode(file)
            headwearNode?.isHidden = false
        } else {
            headwearNode?.isHidden = true
        }
    }

    @IBAction func glassesTapped(_ sender: UIBarButtonItem) {
        glassesIndex += 1
        glassesIndex %= glassesFiles.count
        if let file = glassesFiles[glassesIndex] {
            glassesNode?.removeFromParentNode()
            glassesNode = addNode(file)
            glassesNode?.isHidden = false
        } else {
            glassesNode?.isHidden = true
        }
    }

    @IBAction func earringsTapped(_ sender: UIBarButtonItem) {
        earringsIndex += 1
        earringsIndex %= earringsFiles.count
        if let file = earringsFiles[earringsIndex] {
            earringsNode?.removeFromParentNode()
            earringsNode = addNode(file)
            earringsNode?.isHidden = false
        } else {
            earringsNode?.isHidden = true
        }
    }

    @IBAction func mustacheTapped(_ sender: UIBarButtonItem) {
        mustacheIndex += 1
        mustacheIndex %= mustacheFiles.count
        if let file = mustacheFiles[mustacheIndex] {
            mustacheNode?.removeFromParentNode()
            mustacheNode = addNode(file)
            mustacheNode?.isHidden = false
        } else {
            mustacheNode?.isHidden = true
        }
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

