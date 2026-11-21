import SwiftUI
import SceneKit
import CoreFoundation

/// A bounded visualization of the Pi's accumulated local map; no capture or control.
struct PointCloudView: View {
    let map: [String: Any]
    var compact = false
    @State private var showSurfaces = true
    @State private var resetView = 0

    var body: some View {
        let cloud = CloudSnapshot(map, showSurfaces: showSurfaces)
        if compact {
            CloudSceneView(cloud: cloud, reset: resetView)
                .overlay(alignment: .topTrailing) {
                    Button(showSurfaces ? "Points" : "Surfaces") { showSurfaces.toggle() }
                        .font(.caption).buttonStyle(.bordered).background(.thinMaterial, in: Capsule()).padding(4)
                }
        } else {
        VStack(alignment: .leading, spacing: 10) {
            HStack {
                Text("3D surroundings").font(.title2.bold())
                Spacer()
                Button("Recenter") { resetView &+= 1 }.buttonStyle(.bordered)
            }
            HStack {
                Text(cloud.live ? "LIVE" : "STALE")
                    .font(.caption.bold()).foregroundStyle(cloud.live ? Color.green : Color.orange)
                Text("Tracking: \(cloud.tracking)").font(.footnote)
            }
            CloudSceneView(cloud: cloud, reset: resetView)
                .frame(height: 340)
                .clipShape(RoundedRectangle(cornerRadius: 12))
                .overlay(alignment: .bottomLeading) {
                    Text(cloud.measured.isEmpty && cloud.estimated.isEmpty && cloud.distant.isEmpty ? "Waiting for 3D points" : "Drag to orbit · pinch to zoom")
                        .font(.caption).foregroundStyle(.white)
                        .padding(8).background(.black.opacity(0.7), in: RoundedRectangle(cornerRadius: 8))
                        .padding(8).allowsHitTesting(false)
                }
                .accessibilityLabel("Interactive 3D surroundings up to 20 meters. Warm nearby points, cool distant points. Bright measured points, faint current inferred points, small violet dots for uncertain visual memory. Labels mark remembered objects.")
            HStack(spacing: 8) {
                Text("Near 0 m")
                LinearGradient(colors: [.red, .orange, .yellow, .green, .cyan, .blue, .purple], startPoint: .leading, endPoint: .trailing)
                    .frame(height: 8).clipShape(Capsule())
                Text("Far 20 m")
            }.font(.caption)
            Text("Distance from phone · bright: \(cloud.measured.count) measured · faint: \(cloud.estimated.count) inferred")
                .font(.caption)
            Text("\(cloud.activeInferredCount) active inferred obstacle points · \(cloud.inferenceLive ? "fresh" : "expired / unavailable")")
                .font(.caption).foregroundStyle(.purple)
            Text("Violet memory: \(cloud.distant.count) uncertain points · \(Int(cloud.memoryConfidence * 100))% mean display weight")
                .font(.caption).foregroundStyle(.purple)
            Text("Remembered geometry fades over 120 s; oldest \(Int(cloud.memoryAge)) s. It suggests surfaces up to 20 m, but never drives avoidance or fills in walls.")
                .font(.caption).foregroundStyle(.secondary)
            Text("\(cloud.objects.count) remembered objects · labels show last-seen locations, which can change.")
                .font(.caption).foregroundStyle(.secondary)
            Text(cloud.floorAssumed
                 ? "Mount height estimates the floor. Tap Set floor with the rover on the ground to confirm. Grid spacing is 1 m."
                 : (cloud.floor == nil ? "White marker: estimated phone pose. Waiting for floor reference."
                    : "Level floor at calibrated height · grid: 1 m · white marker: estimated phone pose."))
                .font(.caption).foregroundStyle(cloud.floorAssumed ? Color.orange : Color.secondary)
            Text("Fresh calibrated inferred points may add obstacles or stop motion, never establish safe free space. Faint history points are not necessarily active obstacles. This local map can drift.")
                .font(.caption).foregroundStyle(.secondary)
        }
    }
    }
}

private struct CloudPoint: Hashable {
    let x: Float
    let y: Float
    let z: Float
    var vector: SCNVector3 { SCNVector3(x, y, z) }
}

private struct CloudObject: Equatable {
    let id: String
    let label: String
    let point: CloudPoint
    let age: Float?
    let recentlySeen: Bool
    let selected: Bool
    var caption: String {
        if recentlySeen { return label + " · recent" }
        return label + (age.map { " · \(Int(min($0, 9999)))s ago" } ?? " · saved")
    }
}

private struct CloudSnapshot {
    let session: String
    let splats: Data
    let measured: [CloudPoint]
    let estimated: [CloudPoint]
    let distant: [CloudPoint]
    let distantConfidence: [Float]
    let memoryConfidence: Float
    let memoryAge: Float
    let objects: [CloudObject]
    let activeInferredCount: Int
    let inferenceLive: Bool
    let pose: [Float]?
    let floor: Float?
    let floorAssumed: Bool
    let tracking: String
    let live: Bool

    init(_ map: [String: Any], showSurfaces: Bool = true) {
        if showSurfaces, let encoded = map["surface_splats"] as? String, encoded.utf8.count <= 26000,
           let data = Data(base64Encoded: encoded), data.count.isMultiple(of: 13) {
            splats = data
        } else { splats = Data() }
        session = String((map["session"] as? String ?? "").prefix(128))
        measured = Self.points(map["points3d"], limit: 2000)
        estimated = Self.points(map["estimated3d"], limit: 1000)
        let memoryPoints = map["distant3d"] as? [Any] ?? []
        let memoryStrengths = map["distant_confidence"] as? [Any] ?? []
        let remembered: [(CloudPoint, Float)] = memoryPoints.prefix(1500).enumerated().compactMap { index, value in
            guard index < memoryStrengths.count,
                  let point = Self.points([value], limit: 1).first,
                  let strength = Self.number(memoryStrengths[index]), (0...1).contains(strength) else { return nil }
            return (point, strength)
        }
        distant = remembered.map { $0.0 }; distantConfidence = remembered.map { $0.1 }
        let memory = map["distant_memory"] as? [String: Any] ?? [:]
        memoryConfidence = min(1, max(0, Self.number(memory["mean_confidence"]) ?? 0))
        memoryAge = min(120, max(0, Self.number(memory["oldest_age_s"]) ?? 0))
        inferenceLive = (map["inference_live"] as? Bool ?? false) && (map["connected"] as? Bool ?? true)
        activeInferredCount = inferenceLive ? Self.points(map["active_estimated3d"], limit: 256).count : 0
        tracking = String((map["tracking"] as? String ?? "waiting").prefix(32))
        live = (map["tracking_live"] as? Bool ?? map["live"] as? Bool ?? false)
            && tracking == "normal" && (map["connected"] as? Bool ?? true)
        if let items = map["pose"] as? [Any], items.count == 4 {
            let numbers = items.compactMap { Self.number($0) }
            pose = numbers.count == 4 ? numbers : nil
        } else { pose = nil }
        floorAssumed = map["floor_assumed"] as? Bool ?? false
        floor = (map["floor_ready"] as? Bool ?? false) || floorAssumed ? Self.number(map["floor_y"]) : nil
        let selected = map["selected"] as? String
        var seen = Set<String>()
        objects = Array((map["objects"] as? [[String: Any]] ?? []).prefix(100)).compactMap { item in
            guard let id = item["id"] as? String, !id.isEmpty, id.count <= 96, seen.insert(id).inserted,
                  let label = item["label"] as? String, let position = item["position"],
                  let point = Self.points([position], limit: 1).first else { return nil }
            let age = Self.number(item["age_s"]).flatMap { $0 >= 0 ? $0 : nil }
            return CloudObject(id: id, label: String(label.prefix(40)), point: point, age: age,
                recentlySeen: (item["currently_seen"] as? Bool ?? age.map { $0 <= 1.5 } ?? false), selected: id == selected)
        }.sorted { ($0.age ?? .greatestFiniteMagnitude) < ($1.age ?? .greatestFiniteMagnitude) }
    }

    private static func number(_ value: Any?) -> Float? {
        guard let number = value as? NSNumber, CFGetTypeID(number) != CFBooleanGetTypeID() else { return nil }
        let value = number.doubleValue
        return value.isFinite && abs(value) <= 1_000_000 ? Float(value) : nil
    }

    private static func points(_ value: Any?, limit: Int) -> [CloudPoint] {
        guard let values = value as? [Any] else { return [] }
        return values.prefix(limit).compactMap { item in
            guard let xyz = item as? [Any], xyz.count == 3,
                  let x = number(xyz[0]), let y = number(xyz[1]), let z = number(xyz[2]),
                  abs(x) <= 1000, abs(y) <= 1000, abs(z) <= 1000 else { return nil }
            return CloudPoint(x: x, y: y, z: z)
        }
    }
}

private struct CloudSceneView: UIViewRepresentable {
    let cloud: CloudSnapshot
    let reset: Int

    func makeCoordinator() -> Coordinator { Coordinator() }

    func makeUIView(context: Context) -> SCNView {
        let view = SCNView(frame: .zero)
        view.backgroundColor = UIColor(red: 0.025, green: 0.04, blue: 0.065, alpha: 1)
        view.scene = context.coordinator.scene
        view.pointOfView = context.coordinator.camera
        view.preferredFramesPerSecond = 30
        view.antialiasingMode = .multisampling2X
        view.allowsCameraControl = true
        view.defaultCameraController.interactionMode = .orbitTurntable
        view.defaultCameraController.inertiaEnabled = true
        view.defaultCameraController.automaticTarget = false
        view.cameraControlConfiguration.allowsTranslation = true
        view.accessibilityLabel = "3D local map. Drag to orbit, pinch to zoom. Warm near points, cool far points."
        context.coordinator.update(cloud, reset: reset, in: view)
        return view
    }

    func updateUIView(_ view: SCNView, context: Context) { context.coordinator.update(cloud, reset: reset, in: view) }

    final class Coordinator {
        let scene = SCNScene()
        let camera = SCNNode()
        private let surfaces = SCNNode()
        private var previousSplats = Data()
        private let measured = SCNNode()
        private let estimated = SCNNode()
        private let distant = SCNNode()
        private let marker = SCNNode()
        private let grid = SCNNode()
        private let floorPlane = SCNNode()
        private let objectNodes = SCNNode()
        private var priorMeasured: [CloudPoint] = []
        private var priorEstimated: [CloudPoint] = []
        private var priorDistant: [CloudPoint] = []
        private var priorConfidence: [Float] = []
        private var priorObjects: [CloudObject] = []
        private var priorColorPose: CloudPoint?
        private var priorFloor: Float?
        private var priorAssumed = false
        private var priorGridCenter: CloudPoint?
        private var priorSession: String?
        private var priorReset = -1

        init() {
            camera.camera = SCNCamera()
            camera.camera?.zNear = 0.05; camera.camera?.zFar = 100; camera.camera?.fieldOfView = 60
            for node in [camera, floorPlane, surfaces, measured, estimated, distant, grid, marker, objectNodes] { scene.rootNode.addChildNode(node) }
            let corners: [SCNVector3] = [SCNVector3(-0.18, -0.12, -0.4), SCNVector3(0.18, -0.12, -0.4),
                                       SCNVector3(0.18, 0.12, -0.4), SCNVector3(-0.18, 0.12, -0.4)]
            var lines = [SCNVector3]()
            for i in 0..<4 { lines += [SCNVector3Zero, corners[i], corners[i], corners[(i + 1) % 4]] }
            marker.geometry = Self.lines(lines, color: .white)
            floorPlane.geometry = SCNPlane(width: 20, height: 20)
            floorPlane.eulerAngles.x = -.pi / 2
        }

        func update(_ cloud: CloudSnapshot, reset: Int, in view: SCNView) {
            let newSession = priorSession != cloud.session
            let colorPose = cloud.pose.map { CloudPoint(x: $0[0], y: $0[1], z: $0[2]) }
            let moved = priorColorPose != colorPose
            SCNTransaction.begin(); SCNTransaction.animationDuration = 0
            if newSession || previousSplats != cloud.splats {
                surfaces.geometry = Self.surfaceGeometry(cloud.splats)
                previousSplats = cloud.splats
            }
            measured.isHidden = !cloud.splats.isEmpty
            if newSession || moved || priorMeasured != cloud.measured {
                measured.geometry = Self.points(cloud.measured, origin: colorPose, inferred: false)
                priorMeasured = cloud.measured
            }
            if newSession || moved || priorEstimated != cloud.estimated {
                estimated.geometry = Self.points(cloud.estimated, origin: colorPose, inferred: true)
                priorEstimated = cloud.estimated
            }
            if newSession || moved || priorDistant != cloud.distant || priorConfidence != cloud.distantConfidence {
                distant.geometry = Self.points(cloud.distant, origin: colorPose, inferred: true,
                                               memory: true, confidence: cloud.distantConfidence)
                priorDistant = cloud.distant; priorConfidence = cloud.distantConfidence
            }
            priorColorPose = colorPose
            measured.opacity = cloud.live ? 1 : 0.45
            estimated.opacity = cloud.live ? 0.45 : 0.2
            distant.opacity = cloud.live ? 0.8 : 0.4
            marker.opacity = cloud.live ? 1 : 0.45
            objectNodes.opacity = cloud.live ? 1 : 0.5
            marker.isHidden = cloud.pose == nil
            if let pose = cloud.pose {
                marker.position = SCNVector3(pose[0], pose[1], pose[2])
                marker.eulerAngles = SCNVector3(0, pose[3] + Float.pi, 0)
            }
            let center = CloudPoint(x: floor((cloud.pose?[0] ?? 0) / 2) * 2, y: cloud.floor ?? 0,
                                    z: floor((cloud.pose?[2] ?? 0) / 2) * 2)
            if newSession || priorFloor != cloud.floor || priorAssumed != cloud.floorAssumed || priorGridCenter != center {
                var lines = [SCNVector3]()
                for offset in -10...10 {
                    let v = Float(offset)
                    lines += [SCNVector3(center.x + v, center.y, center.z - 10), SCNVector3(center.x + v, center.y, center.z + 10),
                              SCNVector3(center.x - 10, center.y, center.z + v), SCNVector3(center.x + 10, center.y, center.z + v)]
                }
                grid.geometry = Self.lines(lines, color: cloud.floorAssumed ? UIColor(red: 0.5, green: 0.3, blue: 0.12, alpha: 1) : UIColor(white: 0.3, alpha: 1))
                grid.isHidden = cloud.floor == nil; floorPlane.isHidden = cloud.floor == nil
                floorPlane.position = SCNVector3(center.x, center.y - 0.005, center.z)
                floorPlane.geometry?.materials = [Self.material(cloud.floorAssumed ? .orange : .cyan)]
                floorPlane.opacity = cloud.floorAssumed ? 0.035 : 0.06
                priorGridCenter = center; priorAssumed = cloud.floorAssumed
            }
            if newSession || priorObjects != cloud.objects || priorFloor != cloud.floor {
                objectNodes.childNodes.forEach { $0.removeFromParentNode() }
                // Bound text geometry even if memory contains many objects.
                for (index, object) in cloud.objects.enumerated() {
                    objectNodes.addChildNode(Self.objectNode(object, floor: cloud.floor, showLabel: index < 30 || object.selected))
                }
                priorObjects = cloud.objects
            }
            priorFloor = cloud.floor
            if newSession || reset != priorReset {
                let target = SCNVector3(cloud.pose?[0] ?? 0, cloud.floor ?? cloud.pose?[1] ?? 0, cloud.pose?[2] ?? 0)
                let distance: Float = cloud.distant.isEmpty ? 4 : 12
                camera.position = SCNVector3(target.x + distance, target.y + distance * 0.75, target.z + distance)
                camera.look(at: target)
                view.defaultCameraController.stopInertia(); view.pointOfView = camera
                view.defaultCameraController.target = target; priorReset = reset
            }
            priorSession = cloud.session
            SCNTransaction.commit()
        }

        private static func objectNode(_ object: CloudObject, floor: Float?, showLabel: Bool) -> SCNNode {
            let root = SCNNode(); root.position = object.point.vector
            let color: UIColor = object.selected ? .cyan : (object.recentlySeen ? .yellow : .lightGray)
            let dot = SCNSphere(radius: 0.07); dot.segmentCount = 8; dot.materials = [material(color)]
            root.addChildNode(SCNNode(geometry: dot))
            if showLabel {
            let text = SCNText(string: object.caption, extrusionDepth: 0)
            text.font = .systemFont(ofSize: 8, weight: .semibold); text.flatness = 0.4
            text.materials = [material(color)]
            let label = SCNNode(geometry: text); label.scale = SCNVector3(0.025, 0.025, 0.025)
            label.position = SCNVector3(0, 0.12, 0)
            let (low, high) = text.boundingBox
            label.pivot = SCNMatrix4MakeTranslation((low.x + high.x) / 2, low.y, 0)
            label.constraints = [SCNBillboardConstraint()]; root.addChildNode(label)
            }
            if let floor {
                root.addChildNode(SCNNode(geometry: lines([SCNVector3Zero, SCNVector3(0, floor - object.point.y, 0)], color: color.withAlphaComponent(0.4))))
            }
            return root
        }

        // One triangle mesh per snapshot. Three concentric rings approximate
        // Gaussian opacity on each patch, oriented with the measured surface.
        private static func surfaceGeometry(_ data: Data) -> SCNGeometry? {
            guard !data.isEmpty else { return nil }
            var vertices = [SCNVector3](), colors = [Float](), triangles = [UInt32]()
            let sides = 8
            let circle = (0..<sides).map { i -> SIMD2<Float> in
                let a = Float(i)*2 * .pi / Float(sides)
                return SIMD2(cos(a), sin(a))
            }
            for i in stride(from: 0, to: data.count, by: 13) {
                func meters(_ j: Int) -> Float {
                    Float(Int16(bitPattern: UInt16(data[i+j]) | UInt16(data[i+j+1]) << 8)) / 100
                }
                let p = SIMD3(meters(0), meters(2), meters(4))
                func signed(_ j: Int) -> Float { Float(Int8(bitPattern: data[i+j])) }
                let raw = SIMD3(signed(6), signed(7), signed(8))
                guard simd_length(raw) > 1 else { continue }
                let n = simd_normalize(raw)
                let tangent = simd_normalize(simd_cross(n, abs(n.y) < 0.9 ? SIMD3(0,1,0) : SIMD3(1,0,0)))
                let bitangent = simd_cross(n, tangent)
                let radius = Float(data[i+12])/100
                let base = UInt32(vertices.count)
                let rgb = (9...11).map { Float(data[i+$0])/255 }
                func vertex(_ point: SIMD3<Float>, _ alpha: Float) {
                    vertices.append(SCNVector3(point)); colors.append(contentsOf: rgb); colors.append(alpha)
                }
                vertex(p, 1)
                for (fraction, alpha): (Float, Float) in [(0.45,0.82), (0.75,0.36), (1,0)] {
                    for c in circle { vertex(p + (c.x*tangent + c.y*bitangent)*radius*fraction, alpha) }
                }
                for j in 0..<sides {
                    let a = base+1+UInt32(j), b = base+1+UInt32((j+1)%sides)
                    triangles += [base,a,b]
                    for r in 0..<2 {
                        let x = a+UInt32(r*sides), y = b+UInt32(r*sides), step = UInt32(sides)
                        triangles += [x,x+step,y, y,x+step,y+step]
                    }
                }
            }
            let colorSource = SCNGeometrySource(data: colors.withUnsafeBufferPointer { Data(buffer: $0) },
                semantic: .color, vectorCount: vertices.count, usesFloatComponents: true,
                componentsPerVector: 4, bytesPerComponent: 4, dataOffset: 0, dataStride: 16)
            let geometry = SCNGeometry(sources: [SCNGeometrySource(vertices: vertices), colorSource],
                                       elements: [SCNGeometryElement(indices: triangles, primitiveType: .triangles)])
            let surface = material(.white)
            surface.blendMode = .alpha
            geometry.materials = [surface]
            return geometry
        }

        private static func points(_ points: [CloudPoint], origin: CloudPoint?, inferred: Bool,
                                   memory: Bool = false, confidence: [Float] = []) -> SCNGeometry? {
            guard !points.isEmpty else { return nil }
            let coordinates = points.flatMap { [$0.x, $0.y, $0.z] }
            let data = coordinates.withUnsafeBufferPointer { Data(buffer: $0) }
            let source = SCNGeometrySource(data: data, semantic: .vertex, vectorCount: points.count,
                usesFloatComponents: true, componentsPerVector: 3, bytesPerComponent: 4, dataOffset: 0, dataStride: 12)
            let colors: [Float] = points.enumerated().flatMap { index, point -> [Float] in
                guard let origin else { return [0.6, 0.6, 0.6, 1] }
                let dx = point.x-origin.x, dy = point.y-origin.y, dz = point.z-origin.z
                let distance = sqrt(dx*dx + dy*dy + dz*dz)
                let hue = memory ? 0.72 + CGFloat(min(1, distance / 20)) * 0.08
                    : CGFloat(min(1, distance / 20)) * 0.78
                let color = UIColor(hue: hue, saturation: memory ? 0.55 : 0.9, brightness: 1, alpha: 1)
                var red: CGFloat = 0, green: CGFloat = 0, blue: CGFloat = 0, alpha: CGFloat = 0
                color.getRed(&red, green: &green, blue: &blue, alpha: &alpha)
                return [Float(red), Float(green), Float(blue), memory ? max(0.08, confidence[index]) : 1]
            }
            let colorData = colors.withUnsafeBufferPointer { Data(buffer: $0) }
            let colorSource = SCNGeometrySource(data: colorData, semantic: .color, vectorCount: points.count,
                usesFloatComponents: true, componentsPerVector: 4, bytesPerComponent: 4, dataOffset: 0, dataStride: 16)
            let indices = (0..<points.count).map { UInt32($0) }
            let indexData = indices.withUnsafeBufferPointer { Data(buffer: $0) }
            let element = SCNGeometryElement(data: indexData, primitiveType: .point, primitiveCount: points.count, bytesPerIndex: 4)
            element.pointSize = memory ? 0.02 : (inferred ? 0.025 : 0.035)
            element.minimumPointScreenSpaceRadius = inferred ? 1 : 1.5
            element.maximumPointScreenSpaceRadius = memory ? 2 : (inferred ? 3 : 4)
            let geometry = SCNGeometry(sources: [source, colorSource], elements: [element])
            geometry.materials = [material(.white)]
            return geometry
        }

        private static func lines(_ vertices: [SCNVector3], color: UIColor) -> SCNGeometry {
            let source = SCNGeometrySource(vertices: vertices)
            let element = SCNGeometryElement(indices: (0..<vertices.count).map { UInt32($0) }, primitiveType: .line)
            let geometry = SCNGeometry(sources: [source], elements: [element]); geometry.materials = [material(color)]
            return geometry
        }

        private static func material(_ color: UIColor) -> SCNMaterial {
            let material = SCNMaterial(); material.lightingModel = .constant
            material.diffuse.contents = color; material.isDoubleSided = true
            return material
        }
    }
}
