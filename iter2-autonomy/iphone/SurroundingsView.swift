import SwiftUI

/// A view of the map returned by the Pi. It never issues motor commands.
struct SurroundingsView: View {
    let map: [String: Any]
    let connected: Bool
    let onCommand: ([String: Any]) -> Void

    var compact = false
    @State private var span: Double = 8
    @State private var selectedID: String?
    @State private var pan = CGSize.zero
    @GestureState private var drag = CGSize.zero
    @GestureState private var magnification: CGFloat = 1
    private var visibleSpan: Double { max(2, min(30, span / Double(magnification))) }

    private let background = Color(red: 0.035, green: 0.055, blue: 0.09)
    private let freeColor = Color(red: 0.19, green: 0.31, blue: 0.40)
    private let obstacleColor = Color.orange

    var body: some View {
        let snapshot = MapSnapshot(map)
        let usable = connected && snapshot.valid && snapshot.live && snapshot.tracking == "normal"
        let targetsEnabled = usable && snapshot.floorReady

        if compact { floorCanvas(snapshot, usable: usable) } else {
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                Text("Surroundings").font(.title2.bold())
                Spacer()
                Text(usable ? "LIVE" : (connected && snapshot.valid && snapshot.tracking == "normal" && !snapshot.floorReady ? "SET FLOOR" : "STALE"))
                    .font(.caption.bold())
                    .foregroundStyle(usable ? Color.green : Color.orange)
            }
            Text(statusText(snapshot, usable: usable))
                .font(.footnote)
                .foregroundStyle(usable && snapshot.floorReady ? Color.secondary : Color.orange)
            if !snapshot.message.isEmpty {
                Text("\(connected ? "Pi" : "Last Pi message"): \(snapshot.message)")
                    .font(.footnote)
                    .accessibilityLabel("Pi map status: \(snapshot.message)")
            }

            Picker("Map width", selection: $span) {
                Text("4 m").tag(4.0)
                Text("8 m").tag(8.0)
                Text("20 m").tag(20.0)
            }.pickerStyle(.segmented)
                .accessibilityLabel("Map width in meters")

            HStack {
                Button { span = max(2, span / 1.5) } label: { Image(systemName: "plus.magnifyingglass") }
                    .accessibilityLabel("Zoom map in")
                Button { span = min(30, span * 1.5) } label: { Image(systemName: "minus.magnifyingglass") }
                    .accessibilityLabel("Zoom map out")
                Spacer()
                Text(String(format: "%.1f m wide", visibleSpan)).font(.caption.monospacedDigit())
                Button("Recenter") { pan = .zero; span = 8 }
            }.buttonStyle(.bordered)

            floorCanvas(snapshot, usable: usable)
            .frame(height: 320)
            .clipShape(RoundedRectangle(cornerRadius: 12))
            .overlay(alignment: .bottomLeading) {
                Text("Pinch to zoom · drag to pan")
                    .font(.caption).foregroundStyle(.white)
                    .padding(8).background(.black.opacity(0.7), in: RoundedRectangle(cornerRadius: 8)).padding(8)
                    .allowsHitTesting(false)
            }
            .accessibilityLabel("Zoomable surroundings map. Orange is a measured obstacle, purple is an inferred obstacle, blue is measured free space, dark is unknown. Cyan line is a path preview.")

            HStack(spacing: 12) {
                legend("Unknown", color: background)
                legend("Free", color: freeColor)
                legend("Obstacle", color: obstacleColor)
                legend("Inferred", color: .purple)
            }.font(.caption)
            HStack(spacing: 12) {
                legend("Robot", color: .white)
                legend("Object", color: .yellow)
                legend("Path preview", color: .cyan)
            }.font(.caption)

            Text("\(connected ? "Path preview" : "Last path preview status"): \(snapshot.planStatus)")
                .font(.footnote)
            Text("Inferred blockers: \(map["inferred_obstacle_cells"] as? Int ?? 0) · \((map["inference_live"] as? Bool ?? false) ? "fresh" : "unavailable")")
                .font(.caption).foregroundStyle(.purple)
            Text("\(snapshot.frames) scene frames · \(snapshot.voxels) stored 3D cells")
                .font(.caption).foregroundStyle(.secondary)
            if let coverage = snapshot.scanDegrees {
                Text("Heading coverage \(Int(coverage.rounded()))° · orientation only, not map completeness")
                    .font(.caption).foregroundStyle(.secondary)
            }

            HStack {
                Text("Detected targets").font(.headline)
                Spacer()
                Button("Clear map") {
                    selectedID = nil
                    onCommand(["op": "map_clear"])
                }.buttonStyle(.bordered)
                    .disabled(!connected)
            }
            if snapshot.objects.isEmpty {
                Text("No located objects yet. Slowly look around with the rear camera.")
                    .font(.footnote).foregroundStyle(.secondary)
            } else {
                ForEach(snapshot.objects.prefix(12)) { object in
                    let fresh = object.age <= 5
                    Button {
                        selectedID = object.id
                        onCommand(["op": "select_target", "id": object.id])
                    } label: {
                        HStack {
                            Image(systemName: selectedID == object.id ? "scope" : "mappin.circle")
                            VStack(alignment: .leading, spacing: 2) {
                                Text(object.label).font(.body.weight(.medium))
                                Text(targetDetails(object, pose: snapshot.pose))
                                    .font(.caption).foregroundStyle(.secondary)
                            }
                            Spacer()
                            Text(fresh ? "Preview path" : "Last seen")
                                .font(.caption)
                        }
                    }
                    .buttonStyle(.bordered)
                    .disabled(!targetsEnabled || !fresh)
                }
            }
            Text("Selecting a target only previews a route. It does not start the motors.")
                .font(.caption).foregroundStyle(.secondary)
        }
        .onChange(of: snapshot.session) { _, _ in selectedID = nil; pan = .zero }
    }

    }

    private func floorCanvas(_ snapshot: MapSnapshot, usable: Bool) -> some View {
            GeometryReader { geometry in
                Canvas { context, size in
                    var drawingContext = context
                    draw(snapshot, context: &drawingContext, size: size, usable: usable)
                }
                .background(background)
                .contentShape(Rectangle())
                .gesture(DragGesture(minimumDistance: 4)
                    .updating($drag) { value, state, _ in state = value.translation }
                    .onEnded { value in
                        let scale = Double(geometry.size.width) / visibleSpan
                        guard scale > 0 else { return }
                        pan.width -= value.translation.width / scale
                        pan.height -= value.translation.height / scale
                    })
                .simultaneousGesture(MagnifyGesture()
                    .updating($magnification) { value, state, _ in state = value.magnification }
                    .onEnded { value in span = max(2, min(30, span / Double(value.magnification))) })
            }
    }

    private func legend(_ label: String, color: Color) -> some View {
        HStack(spacing: 4) {
            Circle().fill(color).overlay(Circle().stroke(.gray, lineWidth: 0.5))
                .frame(width: 9, height: 9)
            Text(label)
        }
    }

    private func statusText(_ snapshot: MapSnapshot, usable: Bool) -> String {
        if !connected { return "Pi disconnected — map is not updating." }
        if !snapshot.valid { return "Waiting for a valid surroundings map from the Pi." }
        if snapshot.tracking == "normal" && !snapshot.floorReady {
            return "With the rover on the floor, tap Set floor to use the mount height."
        }
        if !usable { return "Tracking \(snapshot.tracking) — map is not current; paths are disabled." }
        return "Tracking active · floor calibrated · unknown areas stay blocked."
    }

    private func targetDetails(_ object: MapObject, pose: [Double]?) -> String {
        let confidence = Int(object.confidence * 100)
        let age = object.age < 1 ? "just seen" : "seen \(Int(min(object.age, 9999))) s ago"
        if let pose {
            let distance = hypot(object.x - pose[0], object.z - pose[2])
            return String(format: "%.1f m · %d%% confidence · %@", distance, confidence, age)
        }
        return "\(confidence)% confidence · \(age)"
    }

    private func draw(_ snapshot: MapSnapshot, context: inout GraphicsContext, size: CGSize, usable: Bool) {
        guard size.width > 0, size.height > 0 else { return }
        let scale = Double(size.width) / visibleSpan
        let centerX = (snapshot.pose?[0] ?? 0) + Double(pan.width) - Double(drag.width) / scale
        let centerZ = (snapshot.pose?[2] ?? 0) + Double(pan.height) - Double(drag.height) / scale
        func screen(_ x: Double, _ z: Double) -> CGPoint {
            CGPoint(x: (x - centerX) * scale + Double(size.width) / 2,
                    y: (z - centerZ) * scale + Double(size.height) / 2)
        }
        context.clip(to: Path(CGRect(origin: .zero, size: size)))
        if !usable { context.opacity = 0.55 }

        // Contiguous row runs avoid issuing one draw call per cell.
        if snapshot.valid {
            let halfZ = Double(size.height) / scale / 2
            let firstX = max(0, min(snapshot.size, Int(floor((centerX - visibleSpan / 2 - snapshot.originX) / snapshot.resolution))))
            let lastX = max(0, min(snapshot.size, Int(ceil((centerX + visibleSpan / 2 - snapshot.originX) / snapshot.resolution))))
            let firstZ = max(0, min(snapshot.size, Int(floor((centerZ - halfZ - snapshot.originZ) / snapshot.resolution))))
            let lastZ = max(0, min(snapshot.size, Int(ceil((centerZ + halfZ - snapshot.originZ) / snapshot.resolution))))
            for z in firstZ..<max(firstZ, lastZ) {
                var x = firstX
                while x < lastX {
                    let value = snapshot.grid[z * snapshot.size + x]
                    let begin = x
                    repeat { x += 1 } while x < lastX && snapshot.grid[z * snapshot.size + x] == value
                    guard value != 48 else { continue }
                    let origin = screen(snapshot.originX + Double(begin) * snapshot.resolution,
                                        snapshot.originZ + Double(z) * snapshot.resolution)
                    let rect = CGRect(x: origin.x, y: origin.y,
                                      width: Double(x - begin) * snapshot.resolution * scale + 0.25,
                                      height: snapshot.resolution * scale + 0.25)
                    context.fill(Path(rect), with: .color(value == 50 ? obstacleColor : (value == 51 ? .purple : freeColor)))
                }
            }
        }

        if usable && snapshot.floorReady && snapshot.path.count > 1 {
            var path = Path()
            path.move(to: screen(snapshot.path[0][0], snapshot.path[0][1]))
            for point in snapshot.path.dropFirst() { path.addLine(to: screen(point[0], point[1])) }
            context.stroke(path, with: .color(.cyan), style: StrokeStyle(lineWidth: 3, lineCap: .round, lineJoin: .round))
        }

        for object in snapshot.objects {
            let point = screen(object.x, object.z)
            guard point.x >= 0, point.y >= 0, point.x <= size.width, point.y <= size.height else { continue }
            let selected = object.id == selectedID
            let radius = selected ? 7.0 : 4.0
            let dot = Path(ellipseIn: CGRect(x: point.x - radius, y: point.y - radius, width: radius * 2, height: radius * 2))
            context.fill(dot, with: .color(object.age <= 5 ? (selected ? .cyan : .yellow) : .gray))
            if selected {
                context.draw(Text(object.label).font(.caption.bold()).foregroundColor(.white),
                             at: CGPoint(x: point.x, y: point.y - 16), anchor: .bottom)
            }
        }
        if let pose = snapshot.pose {
            let center = screen(pose[0], pose[2])
            let heading = pose[3]
            func arrowPoint(_ distance: Double, _ angle: Double) -> CGPoint {
                CGPoint(x: center.x + sin(angle) * distance, y: center.y + cos(angle) * distance)
            }
            var arrow = Path()
            arrow.move(to: arrowPoint(13, heading))
            arrow.addLine(to: arrowPoint(9, heading + 2.5))
            arrow.addLine(to: arrowPoint(9, heading - 2.5))
            arrow.closeSubpath()
            context.fill(arrow, with: .color(.white))
            context.stroke(arrow, with: .color(.black), lineWidth: 1.5)
        }
    }
}

private struct MapObject: Identifiable {
    let id: String
    let label: String
    let x: Double
    let z: Double
    let confidence: Double
    let age: Double
}

private struct MapSnapshot {
    let session: String
    let tracking: String
    let live: Bool
    let floorReady: Bool
    let size: Int
    let resolution: Double
    let originX: Double
    let originZ: Double
    let grid: [UInt8]
    let pose: [Double]?
    let objects: [MapObject]
    let path: [[Double]]
    let planStatus: String
    let message: String
    let scanDegrees: Double?
    let frames: Int
    let voxels: Int
    let valid: Bool

    init(_ data: [String: Any]) {
        session = data["session"] as? String ?? ""
        tracking = String((data["tracking"] as? String ?? "unavailable").prefix(40))
        live = data["live"] as? Bool ?? false
        floorReady = data["floor_ready"] as? Bool ?? false
        let n = data["size"] as? Int ?? 0
        let resolutionValue = Self.number(data["resolution"])
        let origin = Self.vector(data["origin"], count: 2)
        let bytes = Array((data["grid"] as? String ?? "").utf8.prefix(65_537))
        valid = (1...256).contains(n) && bytes.count == n * n && bytes.allSatisfy { (48...51).contains($0) }
            && resolutionValue.map { $0 >= 0.01 && $0 <= 10 } == true && origin != nil
        size = valid ? n : 0
        resolution = valid ? resolutionValue! : 0.1
        originX = origin?[0] ?? 0
        originZ = origin?[1] ?? 0
        grid = valid ? bytes : []
        pose = Self.vector(data["pose"], count: 4)
        planStatus = String((data["plan_status"] as? String ?? "waiting for map status").prefix(180))
        message = String((data["message"] as? String ?? "").prefix(500))
        scanDegrees = Self.number(data["scan_degrees"]).flatMap { (0...360).contains($0) ? $0 : nil }
        frames = max(0, data["frames"] as? Int ?? 0)
        voxels = max(0, data["voxels"] as? Int ?? 0)
        let rawPath = data["path"] as? [Any] ?? []
        let parsedPath = Array(rawPath.prefix(4096)).compactMap { Self.vector($0, count: 2) }
        path = rawPath.count <= 4096 && parsedPath.count == rawPath.count ? parsedPath : []
        var seen = Set<String>()
        objects = Array((data["objects"] as? [[String: Any]] ?? []).prefix(100)).compactMap { item in
            guard let id = item["id"] as? String, !id.isEmpty, id.count <= 96,
                  seen.insert(id).inserted, let label = item["label"] as? String,
                  let position = Self.vector(item["position"], count: 3),
                  let confidence = Self.number(item["confidence"]), (0...1).contains(confidence) else { return nil }
            let age = Self.number(item["age_s"]) ?? 9999
            guard age >= 0 else { return nil }
            return MapObject(id: id, label: String(label.prefix(48)), x: position[0], z: position[2],
                             confidence: confidence, age: age)
        }.sorted { $0.age < $1.age }
    }

    private static func number(_ value: Any?) -> Double? {
        guard let number = value as? Double, number.isFinite, abs(number) <= 1_000_000 else { return nil }
        return number
    }

    private static func vector(_ value: Any?, count: Int) -> [Double]? {
        guard let array = value as? [Any], array.count == count else { return nil }
        let values = array.compactMap(number)
        return values.count == count ? values : nil
    }
}
