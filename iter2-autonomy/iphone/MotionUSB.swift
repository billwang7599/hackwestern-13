import SwiftUI
import CoreMotion
import Network
import AVFoundation
import CoreImage

#if DEBUG
private let isInspectionFixture = ProcessInfo.processInfo.arguments.contains("--preview-fixture")
#else
private let isInspectionFixture = false
#endif

@main
struct MotionUSBApp: App {
    @StateObject private var server = MotionServer()
    @StateObject private var stereo = StereoPreview()
    @State private var stereoMode = false
    @StateObject private var camera = CameraServer()
    @StateObject private var pi = PiStatusServer()
    @StateObject private var speech = SpeechLink()
    @State private var inspectionExpanded = false
    @State private var activeTab = isInspectionFixture && ProcessInfo.processInfo.arguments.contains("--map") ? "map" : "camera"

    private var motor: [String: Any] { pi.latestStatus["motor"] as? [String: Any] ?? [:] }

    private var cloud: [String: Any] {
        var value = pi.surroundings
        value["connected"] = pi.connected
        return value
    }

    private var currentGuardConnection: Bool {
        pi.connected && (isInspectionFixture || ProcessInfo.processInfo.systemUptime - pi.statusReceived < 1.25)
    }

    private var safetyBackground: some View {
        TimelineView(.periodic(from: .now, by: 0.2)) { _ in
            let appearance = ObstaclePresentation(status: pi.latestStatus, connected: currentGuardConnection)
            ZStack { Color(uiColor: .systemBackground); appearance.color.opacity(0.14) }.ignoresSafeArea()
        }
    }

    @AppStorage("speakRobotActions") private var speakActions = true
    @State private var controls = false

    private func feed(_ title: String, _ image: UIImage?, _ timestamp: Double) -> some View {
        Color.black
            .aspectRatio(16.0 / 9.0, contentMode: .fit)
            .overlay {
                if let cg = image?.cgImage {
                    GeometryReader { tile in
                        Image(uiImage: UIImage(cgImage: cg, scale: 1, orientation: .left))
                            .resizable().scaledToFill()
                            .frame(width: tile.size.width, height: tile.size.height)
                            .clipped()
                    }
                }
            }
            .overlay(alignment: .topLeading) {
                TimelineView(.periodic(from: .now, by: 0.5)) { _ in
                    Text(title + (image == nil ? " · waiting" : ProcessInfo.processInfo.systemUptime - timestamp > 2 ? " · stale" : ""))
                        .font(.caption2.bold()).foregroundStyle(.white)
                        .padding(.horizontal, 4).padding(.vertical, 2).background(.black.opacity(0.65))
                }
            }
    }

    private func axes(_ key: String) -> String {
        guard pi.connected, pi.latestStatus["motion_live"] as? Bool == true,
              let values = pi.latestStatus[key] as? [NSNumber], values.count == 3 else { return "—" }
        return values.map { String(format: "%.1f", $0.doubleValue) }.joined(separator: "  ")
    }

    private var motionReadout: some View {
        Grid(alignment: .leading, horizontalSpacing: 6, verticalSpacing: 1) {
            GridRow { Text("Accel m/s²"); Text(axes("accel_m_s2")) }
            GridRow { Text("Gyro rad/s"); Text(axes("gyro_rad_s")) }
        }
        .font(.caption.monospacedDigit()).lineLimit(1).minimumScaleFactor(0.75)
        .accessibilityHint("Values in X, Y, Z order")
    }

    private var primaryControls: some View {
        VStack(alignment: .leading, spacing: 2) {
            HStack(spacing: 4) {
                Button(server.running ? "Stop" : "Start") {
                    if server.running { server.stop(); camera.stop() }
                    else { server.start(); if server.running { camera.start() } }
                }.buttonStyle(.borderedProminent)
                Button("Set floor") { camera.setFloor() }
                    .accessibilityHint("Uses the mounted phone height with the rover resting on the floor.")
                Button { controls.toggle() } label: {
                    Image(systemName: controls ? "xmark" : "slider.horizontal.3")
                }.accessibilityLabel(controls ? "Close controls" : "More controls")
            }
            HStack(spacing: 4) {
                Button("Save map") { camera.saveMap { pi.command(["op":"map_save"]) } }
                Button("Load map") { camera.loadMap { pi.command(["op":"map_load"]) } }
            }
        }.buttonStyle(.bordered).controlSize(.small).font(.caption)
    }

    private var sensorColumn: some View {
        VStack(alignment: .leading, spacing: 2) {
            if !controls {
                feed("YOLO", camera.qnxInspectionImage, camera.qnxInspectionTimestamp)
                feed("LiDAR", camera.inspectionImage, camera.inspectionTimestamp)
            }
            ScrollView {
                VStack(alignment: .leading, spacing: 3) {
                    motionReadout
                    primaryControls
                    let keyboard = pi.latestStatus["keyboard"] as? [String: Any]
                    let key = (keyboard?["last_press"] as? [String])?.joined(separator: "+") ?? ""
                    Text((pi.connected ? "Pi online" : "Pi offline") + " · " +
                         (pi.connected && keyboard?["connected"] as? Bool == true ? "Keys " + (key.isEmpty ? "ready" : key) : "Keys offline"))
                    TimelineView(.periodic(from: .now, by: 5)) { _ in
                        Text("Wi-Fi " + (wifiAddress() ?? "off")).font(.caption.monospaced())
                    }
                    TimelineView(.periodic(from: .now, by: 0.3)) { _ in
                        let guardState = ObstaclePresentation(status: pi.latestStatus, connected: currentGuardConnection)
                        Text(guardState.title).foregroundStyle(guardState.color).fontWeight(.bold)
                    }
                    if !speech.lastSpoken.isEmpty {
                        Label(speech.lastSpoken, systemImage: "speaker.wave.2.fill").lineLimit(2)
                    }
                    Text(camera.roomTagStatus).lineLimit(2)
                    if controls {
                        Text(camera.status)
                        if let rgb = pi.surroundings["camera_height_m"] as? NSNumber,
                           let lidar = pi.surroundings["lidar_height_m"] as? NSNumber {
                            let assumed = pi.surroundings["floor_assumed"] as? Bool == true
                            Text(String(format: "%@ · RGB %.1f cm · LiDAR %.1f cm",
                                        assumed ? "Mount estimate" : "Above floor", rgb.doubleValue * 100, lidar.doubleValue * 100))
                        }
                        Toggle("Speak actions", isOn: $speakActions)
                        ManualDrivePanel(pi: pi)
                        Button("Reset scan") { camera.resetMap() }
                        ForEach(["start","stop"], id: \.self) { op in
                            Button("Pi " + op + " streams") { pi.command(["op":op]) }
                        }
                    }
                }.font(.caption2).padding(.horizontal, 3)
                 .frame(maxWidth: .infinity, alignment: .leading)
            }
        }
    }

    var body: some Scene {
        WindowGroup {
            GeometryReader { g in
                // The mount covers the middle half of the physical display below its midpoint.
                let insets = g.safeAreaInsets
                let quarter = (g.size.width + insets.leading + insets.trailing) / 4
                let left = max(0, quarter - insets.leading)
                let right = max(0, quarter - insets.trailing)
                let top = (g.size.height + insets.bottom - insets.top) / 2
                HStack(alignment: .top, spacing: 2) {
                    sensorColumn.frame(width: left)
                    VStack(alignment: .trailing, spacing: 2) {
                        PointCloudView(map: cloud, compact: true)
                            .frame(maxWidth: .infinity).frame(height: top)
                            .clipped().accessibilityLabel("3D surroundings")
                        SurroundingsView(map: pi.surroundings, connected: pi.connected, onCommand: pi.command, compact: true)
                            .frame(width: right).frame(maxHeight: .infinity)
                            .clipped().accessibilityLabel("Floor minimap")
                    }
                }
            }
            .overlay {
                TimelineView(.periodic(from: .now, by: 0.08)) { _ in
                    if let angle = server.floorTiltDegrees, angle <= 45 {
                        let level = angle <= 1
                        let tint = level ? Color.mint : Color(hue: 0.23 * (1 - angle / 45),
                                                             saturation: 0.88, brightness: 0.92)
                        ZStack(alignment: .bottomTrailing) {
                            tint.opacity(level ? 0.22 : 0.10)
                            Rectangle().strokeBorder(tint, lineWidth: level ? 5 : 3)
                            Text(level ? "Level" : String(format: "%.1f°", angle))
                                .font(.caption.bold().monospacedDigit()).foregroundStyle(.white)
                                .padding(4).background(.black.opacity(0.72), in: Capsule()).padding(5)
                        }
                    }
                }.allowsHitTesting(false)
            }.statusBarHidden().background { safetyBackground }
            .onAppear {
                speech.start()
                #if DEBUG
                if isInspectionFixture { camera.loadInspectionFixture(); pi.loadTelemetryFixture(); return }
                #endif
                pi.start()
            }
            .onChange(of: speakActions) { _, _ in speech.resetNarration() }
            .onChange(of: pi.connected) { _, connected in if !connected { speech.resetNarration() } }
            .onReceive(pi.$latestStatus) { value in
                guard pi.connected else { return }
                if speakActions { speech.narrate(value) }
                if let result = value["ai"] as? [String: Any], result["live"] as? Bool == true { if stereoMode { stereo.result(result) } else { camera.applyQNXResult(result) } }
                if let result = value["depth"] as? [String: Any], result["live"] as? Bool == true { camera.applyQNXDepthResult(result) }
            }
            .onReceive(NotificationCenter.default.publisher(for: UIApplication.didBecomeActiveNotification)) { _ in speech.start(); if !isInspectionFixture { pi.start() } }
            .onReceive(Timer.publish(every: 1, on: .main, in: .common).autoconnect()) { _ in if !isInspectionFixture { pi.checkFreshness() } }
            .onReceive(NotificationCenter.default.publisher(for: UIApplication.didEnterBackgroundNotification)) { _ in
                server.stop(); camera.stop(); stereo.stop(); pi.stop(); speech.stop()
            }
        }
    }
}

// The image already contains source-frame-aligned boxes and depth. SwiftUI
// scales the entire pair with aspect-fit, so overlays never drift with layout.
struct CameraInspectionPanel: View {
    let image: UIImage?
    let timestamp: TimeInterval
    var title = "Live camera + LiDAR"
    var source = "Phone sensors · independent of AI"
    var body: some View {
        TimelineView(.periodic(from: .now, by: 0.25)) { _ in
            let age = max(0, ProcessInfo.processInfo.systemUptime - timestamp)
            let live = isInspectionFixture || (image != nil && timestamp > 0 && age < 1.5)
            VStack(alignment: .leading, spacing: 6) {
                HStack {
                    Text(title).font(.headline)
                    Spacer()
                    Text(isInspectionFixture ? "FIXTURE" : (live ? "LIVE" : "STALE"))
                        .font(.caption.bold()).foregroundStyle(live ? Color.green : Color.orange)
                }
                if let image {
                    Image(uiImage: image).resizable().scaledToFit()
                        .opacity(live ? 1 : 0.45)
                        .clipShape(RoundedRectangle(cornerRadius: 10))
                    Text(isInspectionFixture ? "Synthetic test frame" : String(format: "Matched camera + depth frame · %.1f s old", min(age, 9999)))
                        .font(.caption).foregroundStyle(.secondary)
                } else {
                    ContentUnavailableView("Waiting for camera and LiDAR", systemImage: "camera.viewfinder",
                        description: Text("Press Start with the rear camera facing the room."))
                        .frame(minHeight: 220)
                }
                Text(source)
                    .font(.caption).foregroundStyle(.secondary)
            }
        }
    }
}

/// The image contains the exact RGB source and its QNX prediction, not the live camera.
struct QNXDepthPanel: View {
    let image: UIImage?
    let timestamp: TimeInterval
    let calibrated: Bool
    let mode: String
    let model: String
    let inferenceMS: Double?
    let fitSummary: String
    let status: String
    let connected: Bool

    var body: some View {
        TimelineView(.periodic(from: .now, by: 0.25)) { _ in
            let fixtureStale = ProcessInfo.processInfo.arguments.contains("--depth-stale")
            let age = isInspectionFixture ? (fixtureStale ? 8.0 : 0.9)
                : max(0, ProcessInfo.processInfo.systemUptime - timestamp)
            let available = image != nil && timestamp > 0
            let live = connected && available && age <= 2
            VStack(alignment: .leading, spacing: 8) {
                HStack {
                    Text("QNX RGB depth estimation").font(.headline)
                    Spacer()
                    Text(isInspectionFixture && !fixtureStale ? "FIXTURE" : (live ? "LIVE" : "STALE"))
                        .font(.caption.bold()).foregroundStyle(live ? Color.green : Color.orange)
                }
                Text(calibrated ? "Estimated depth · \(mode) scale · 0.2–20 m" : "Relative depth · no meter scale")
                    .font(.subheadline.bold()).foregroundStyle(calibrated && mode == "direct" ? Color.cyan : Color.orange)
                if let image {
                    Image(uiImage: image).resizable().scaledToFit().opacity(live ? 1 : 0.45)
                        .clipShape(RoundedRectangle(cornerRadius: 10))
                    Text(String(format: "Matched RGB source · %.1f s old", min(age, 9999)))
                        .font(.caption.monospacedDigit()).foregroundStyle(live ? Color.secondary : Color.orange)
                } else {
                    ContentUnavailableView("Waiting for QNX depth", systemImage: "square.3.layers.3d",
                        description: Text("Start sensors to send RGB to the Pi. Relative depth can display without LiDAR calibration."))
                }
                Text("\(model) · inference \(inferenceMS.map { String(format: "%.0f ms", $0) } ?? "—")")
                    .font(.caption.monospacedDigit())
                Text(fitSummary).font(.caption).foregroundStyle(.secondary)
                Text(status).font(.caption)
                Text(calibrated
                    ? "Warm = near; cool = far; black = invalid/out of range. Far-range accuracy is unvalidated. Direct current-LiDAR fits alone may add veto obstacles up to8 m. Remembered/cached scales are approximate visual memory only; they cannot control motion."
                    : "Warm = relatively near; cool = relatively far. Colors normalize each frame. Scan nearby textured surfaces at varied distances to seed measured anchors, then look farther. A single distance is insufficient to fit scale and offset.")
                    .font(.caption).foregroundStyle(.secondary)
                if available && !live {
                    Text("Stale result: displayed for inspection; expired estimates cannot act as current obstacle input.")
                        .font(.caption).foregroundStyle(.orange)
                }
            }.frame(maxWidth: .infinity, alignment: .leading)
        }
    }
}

/// The phone's Wi-Fi IPv4 address: what the Pi needs in state/phone-host.
func wifiAddress() -> String? {
    var list: UnsafeMutablePointer<ifaddrs>?
    guard getifaddrs(&list) == 0, let first = list else { return nil }
    defer { freeifaddrs(list) }
    for entry in sequence(first: first, next: { $0.pointee.ifa_next }) {
        let ifa = entry.pointee
        guard String(cString: ifa.ifa_name) == "en0", let addr = ifa.ifa_addr,
              addr.pointee.sa_family == UInt8(AF_INET) else { continue }
        var host = [CChar](repeating: 0, count: Int(NI_MAXHOST))
        if getnameinfo(addr, socklen_t(addr.pointee.sa_len), &host, socklen_t(host.count),
                       nil, 0, NI_NUMERICHOST) == 0 { return String(cString: host) }
    }
    return nil
}

final class PiStatusServer: ObservableObject {
    @Published var summary = "Waiting for Pi status"
    @Published var motionValues = "Waiting for motion values received by Pi"
    @Published var connected = false
    @Published var surroundings: [String: Any] = [:]
    @Published var latestStatus: [String: Any] = [:]
    @Published private(set) var statusReceived: TimeInterval = 0
    private var listener: NWListener?
    private var client: NWConnection?
    private var buffer = Data()
    private var received = Date.distantPast

    #if DEBUG
    func loadTelemetryFixture() {
        connected = true
        statusReceived = ProcessInfo.processInfo.systemUptime
        let fixtureObjects: [[String: Any]] = [
            ["id": "1", "label": "chair", "position": [0.5, 0.5, 1.5], "confidence": 0.88, "age_s": 0.2, "currently_seen": true],
            ["id": "2", "label": "bottle", "position": [-0.8, 0.7, 3.0], "confidence": 0.76, "age_s": 18.0, "currently_seen": false],
            ["id": "3", "label": "backpack", "position": [1.0, 0.4, 5.0], "confidence": 0.71, "age_s": 42.0, "currently_seen": false]]
        var grid = Array(repeating: Character("0"), count: 200 * 200)
        for z in 85..<120 { for x in 85..<115 { grid[z * 200 + x] = (z == 115 && x > 100) ? "2" : "1" } }
        // Plain loops (same points, same order) instead of nested flatMap
        // closures inside the [String: Any] literal: those exceeded the Swift
        // type-checker's limit and broke every Debug build (Xcode 26.1.1).
        var points3d: [[Double]] = []
        for z in 1...5 { for x in -10...10 { for y in 0...5 { points3d.append([Double(x) * 0.12, Double(y) * 0.16, Double(z)]) } } }
        var estimated3d: [[Double]] = []
        for x in -10...10 { for y in 0...10 { estimated3d.append([Double(x) * 0.15, Double(y) * 0.15, 4.5]) } }
        surroundings = ["session": "fixture", "live": true, "tracking_live": true, "tracking": "normal", "floor_ready": true,
                        "floor_assumed": false, "floor_y": 0.0, "pose": [0.0, 0.8, 0.0, 0.0],
                        "size": 200, "resolution": 0.1, "origin": [-10.0, -10.0], "grid": String(grid),
                        "objects": fixtureObjects, "frames": 1188, "voxels": 5400, "selected": "1",
                        "points3d": points3d, "estimated3d": estimated3d]
        let args = ProcessInfo.processInfo.arguments
        let inferredStop = args.contains("--inferred-stop")
        let blocked = args.contains("--blocked") || inferredStop, clear = args.contains("--clear"), unknown = args.contains("--unknown")
        let guardState: [String: Any] = ["live": !args.contains("--stale"), "verdict": unknown ? "UNKNOWN" : (blocked ? "BLOCKED" : "CLEAR"),
            "permit": !blocked && !unknown, "nearest_obstacle_m": blocked ? 0.4 : (clear ? 2.5 : 0.85),
            "stop_source": inferredStop ? "inferred" : (blocked ? "lidar" : "none"),
            "lidar_verdict": inferredStop ? "CLEAR" : (blocked ? "BLOCKED" : "CLEAR"),
            "inferred_live": inferredStop, "inferred_nearest_m": inferredStop ? 0.4 : NSNull(),
            "stop_distance_m": 0.6, "warning_distance_m": 1.2, "tick_hz": 100, "sample_age_ms": 12,
            "frames": 4830, "late_ticks": 0, "missed_ticks": 0, "max_lateness_ms": 0.5, "priority": 20, "runmask": 8]
        latestStatus = ["type": "status", "mode": "headless", "streaming": true,
                        "motion_hz": 99.8, "camera_fps": 10.0, "samples": 24150,
                        "gaps": 3, "frames": 483, "motion_live": true, "camera_live": true,
                        "receipt_age_ms": 8, "accel_m_s2": [0.13, -0.21, 9.79],
                        "gyro_rad_s": [0.012, -0.045, 0.008], "motor_mode": "stub",
                        "motor": ["backend": "stub", "mode": "stub", "connected": true,
                                  "inhibited": blocked || unknown, "reason": blocked ? "LiDAR blocked" : "Awaiting drive command",
                                  "requested": [0.0, 0.0], "output": [0.0, 0.0], "status_age_ms": 10.0],
                        "left": 0.0, "right": 0.0, "safety": guardState, "surroundings": surroundings]
    }
    #endif

    func start() {
        guard listener == nil else { return }
        do {
            let listener = try NWListener(using: .tcp, on: 5559)
            self.listener = listener
            listener.stateUpdateHandler = { [weak self] state in
                if case .failed(let error) = state {
                    self?.stop(); self?.summary = "Status listener: \(error.localizedDescription)"
                }
            }
            listener.newConnectionHandler = { [weak self] connection in
                guard let self else { return }
                self.client?.cancel()
                self.client = connection
                self.buffer = Data()
                self.connected = false
                connection.stateUpdateHandler = { [weak self, weak connection] state in
                    guard let self, let connection, self.client === connection else { return }
                    switch state {
                    case .ready: self.receive(connection)
                    case .failed, .cancelled:
                        self.connected = false; self.summary = "Pi disconnected"; self.client = nil
                    default: break
                    }
                }
                connection.start(queue: .main)
            }
            listener.start(queue: .main)
        } catch { summary = "Status listener: \(error.localizedDescription)" }
    }

    private func receive(_ connection: NWConnection) {
        connection.receive(minimumIncompleteLength: 1, maximumLength: 4096) { [weak self] data, _, complete, error in
            guard let self, self.client === connection else { return }
            if let data { self.buffer.append(data) }
            while let end = self.buffer.firstIndex(of: 10) {
                let line = self.buffer.prefix(upTo: end)
                guard line.count <= 262144 else { connection.cancel(); return }
                self.buffer.removeSubrange(...end)
                guard let value = try? JSONSerialization.jsonObject(with: line) as? [String: Any],
                      value["type"] as? String == "status" else { continue }
                self.statusReceived = ProcessInfo.processInfo.systemUptime
                self.latestStatus = value
                self.surroundings = value["surroundings"] as? [String: Any] ?? [:]
                self.received = Date()
                self.connected = true
                let motion = value["motion_hz"] as? Double ?? 0
                let fps = value["camera_fps"] as? Double ?? 0
                let gaps = value["gaps"] as? Int ?? 0
                let motionLive = value["motion_live"] as? Bool ?? false
                let cameraLive = value["camera_live"] as? Bool ?? false
                let streaming = value["streaming"] as? Bool ?? false
                let left = value["left"] as? Double ?? 0
                let right = value["right"] as? Double ?? 0
                if let accel = value["accel_m_s2"] as? [Double], accel.count == 3,
                   let gyro = value["gyro_rad_s"] as? [Double], gyro.count == 3 {
                    self.motionValues = String(format: "PI-RECEIVED MOTION\nAccel (m/s²)\nX %+.3f  Y %+.3f  Z %+.3f\nGyro (rad/s)\nX %+.3f  Y %+.3f  Z %+.3f",
                        accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2])
                } else { self.motionValues = "Waiting for motion values received by Pi" }
                self.summary = String(format: "Pi connected • %@\nMotion: %.1f Hz (%@) • gaps %d\nCamera: %.1f fps (%@)\nDrive command: L %.2f / R %.2f",
                    streaming ? "running" : "stopped", motion, motionLive ? "live" : "stale", gaps,
                    fps, cameraLive ? "live" : "stale", left, right)
            }
            // Limit each complete line and the unfinished tail independently.
            // A valid large line plus the next line's prefix may exceed the line limit together.
            guard self.buffer.count <= 262144 else { connection.cancel(); return }
            if complete || error != nil { connection.cancel() }
            else { self.receive(connection) }
        }
    }

    func command(_ value: [String: Any]) {
        guard connected, let client, client.state == .ready,
              var data = try? JSONSerialization.data(withJSONObject: value) else { return }
        data.append(10)
        client.send(content: data, completion: .contentProcessed { error in
            if error != nil { client.cancel() }
        })
    }

    func checkFreshness() {
        if connected && Date().timeIntervalSince(received) > 3 {
            connected = false
            summary = "Pi status stale; controls unavailable"
            client?.cancel()
        }
    }

    func stop() {
        listener?.stateUpdateHandler = nil
        listener?.cancel(); listener = nil
        client?.stateUpdateHandler = nil
        client?.cancel(); client = nil
        connected = false
        summary = "Pi status stopped"
    }
}

final class MotionServer: ObservableObject {
    @Published var status = "Press Start after connecting USB"
    @Published var running = false
    @Published var sent: UInt64 = 0
    private let motion = CMMotionManager()
    var floorTiltDegrees: Double? {
        guard running, let sample = motion.deviceMotion,
              ProcessInfo.processInfo.systemUptime - sample.timestamp < 0.4 else { return nil }
        let g = sample.gravity
        return atan2(hypot(g.x, g.y), -g.z) * 180 / .pi
    }
    private var listener: NWListener?
    private var client: NWConnection?
    private var accel: CMAccelerometerData?
    private var sending = false
    private var sequence: UInt64 = 0

    func start() {
        guard !running else { return }
        guard motion.isAccelerometerAvailable && motion.isGyroAvailable else {
            status = "Accelerometer or gyroscope unavailable"; return
        }
        do {
            let listener = try NWListener(using: .tcp, on: 5557)
            self.listener = listener
            listener.stateUpdateHandler = { [weak self] state in
                guard let self else { return }
                if case .ready = state { self.status = "Listening on port 5557; waiting for Pi" }
                if case .failed(let error) = state {
                    self.stop(); self.status = "Listener: \(error.localizedDescription)"
                }
            }
            listener.newConnectionHandler = { [weak self] connection in self?.accept(connection) }
            listener.start(queue: .main)
            running = true
            sent = 0
            sequence = 0
            UIApplication.shared.isIdleTimerDisabled = true
            if motion.isDeviceMotionAvailable {
                motion.deviceMotionUpdateInterval = 0.05
                motion.startDeviceMotionUpdates()
            }
            motion.accelerometerUpdateInterval = 0.01
            motion.gyroUpdateInterval = 0.01
            motion.startAccelerometerUpdates(to: .main) { [weak self] data, error in
                guard let self else { return }
                if let error { self.stop(); self.status = error.localizedDescription; return }
                self.accel = data
            }
            motion.startGyroUpdates(to: .main) { [weak self] data, error in
                guard let self else { return }
                if let error { self.stop(); self.status = error.localizedDescription; return }
                if let data { self.publish(data) }
            }
        } catch { status = error.localizedDescription }
    }

    private func accept(_ connection: NWConnection) {
        client?.cancel()
        client = connection
        sending = false
        connection.stateUpdateHandler = { [weak self, weak connection] state in
            guard let self, let connection, self.client === connection else { return }
            switch state {
            case .ready:
                self.status = "Client connected; streaming IMU"
                self.watchDisconnect(connection)
            case .failed, .cancelled:
                self.client = nil
                self.sending = false
                self.status = "Disconnected; waiting for Pi"
            default: break
            }
        }
        connection.start(queue: .main)
    }

    private func watchDisconnect(_ connection: NWConnection) {
        connection.receive(minimumIncompleteLength: 1, maximumLength: 256) { [weak self] _, _, complete, error in
            guard let self, self.client === connection else { return }
            if complete || error != nil { connection.cancel() }
            else { self.watchDisconnect(connection) }
        }
    }

    private func publish(_ gyro: CMGyroData) {
        sequence += 1
        guard let accel, abs(gyro.timestamp - accel.timestamp) < 0.05,
              let client, client.state == .ready, !sending else { return }
        let a = accel.acceleration
        let g = gyro.rotationRate
        let sample: [String: Any] = [
            "version": 1, "seq": sequence,
            "accel_time_s": accel.timestamp, "gyro_time_s": gyro.timestamp,
            "accel_m_s2": [a.x * 9.80665, a.y * 9.80665, a.z * 9.80665],
            "gyro_rad_s": [g.x, g.y, g.z]
        ]
        guard var data = try? JSONSerialization.data(withJSONObject: sample) else { return }
        data.append(0x0a)
        sending = true
        client.send(content: data, completion: .contentProcessed { [weak self] error in
            guard let self, self.client === client else { return }
            self.sending = false
            if error != nil { client.cancel() }
            else { self.sent += 1 }
        })
    }

    func stop() {
        motion.stopDeviceMotionUpdates()
        motion.stopAccelerometerUpdates()
        motion.stopGyroUpdates()
        listener?.stateUpdateHandler = nil
        listener?.cancel()
        listener = nil
        client?.stateUpdateHandler = nil
        client?.cancel()
        client = nil
        accel = nil
        sending = false
        running = false
        UIApplication.shared.isIdleTimerDisabled = false
        status = "Stopped"
    }
}

// Bench capture using the rear virtual camera's native stereo disparity stream.
final class StereoPreview: NSObject, ObservableObject, AVCaptureDepthDataOutputDelegate, AVCaptureVideoDataOutputSampleBufferDelegate {
    @Published var status = "Stop sensors to switch modes, then tap Start sensors."
    @Published var image: UIImage?
    private let queue = DispatchQueue(label: "robot.stereo")
    private let context = CIContext()
    private var session: AVCaptureSession?
    private var last = 0.0
    @Published var detected: UIImage?
    @Published var detectedAt = 0.0
    private lazy var rgb = ScenePacketStream(queue: queue, limit: 2_000_012)
    private var sources = [UInt64: CGImage]()
    private var nextRGB = 0.0
    private var label = ""
    private func report(_ value: String) { DispatchQueue.main.async { self.status = value } }
    func start() {
        queue.async {
            guard self.session == nil else { return }
            for type in [AVCaptureDevice.DeviceType.builtInDualWideCamera, .builtInDualCamera] {
                guard let device = AVCaptureDevice.default(type, for: .video, position: .back),
                      let input = try? AVCaptureDeviceInput(device: device) else { continue }
                let capture = AVCaptureSession()
                capture.beginConfiguration()
                guard capture.canAddInput(input) else { capture.commitConfiguration(); continue }
                capture.addInput(input)
                guard let format = device.formats.filter({ !$0.supportedDepthDataFormats.isEmpty }).min(by: {
                    CMVideoFormatDescriptionGetDimensions($0.formatDescription).width < CMVideoFormatDescriptionGetDimensions($1.formatDescription).width
                }), let depth = format.supportedDepthDataFormats.last else { capture.commitConfiguration(); continue }
                do {
                    try device.lockForConfiguration()
                    device.activeFormat = format
                    device.activeDepthDataFormat = depth
                    device.unlockForConfiguration()
                } catch { capture.commitConfiguration(); continue }
                let output = AVCaptureDepthDataOutput()
                guard capture.canAddOutput(output) else { capture.commitConfiguration(); continue }
                capture.addOutput(output)
                output.alwaysDiscardsLateDepthData = true
                output.setDelegate(self, callbackQueue: self.queue)
                output.connection(with: .depthData)?.isEnabled = true
                let video = AVCaptureVideoDataOutput()
                video.alwaysDiscardsLateVideoFrames = true
                guard capture.canAddOutput(video) else { capture.commitConfiguration(); continue }
                capture.addOutput(video)
                video.setSampleBufferDelegate(self, queue: self.queue)
                capture.commitConfiguration()
                do { try self.rgb.start(port: 5563) { self.report($0) } }
                catch { self.report(error.localizedDescription); return }
                self.nextRGB = 0
                self.session = capture
                self.label = device.localizedName
                self.last = 0
                capture.startRunning()
                self.report("Waiting for rear stereo depth from " + self.label)
                return
            }
            self.report("No rear dual-camera streaming depth format available on this device.")
        }
    }
    func stop() {
        queue.async {
            self.session?.stopRunning(); self.session = nil
            self.rgb.stop(); self.sources.removeAll()
            DispatchQueue.main.async { self.image = nil; self.detected = nil }
        }
    }
    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        let now = ProcessInfo.processInfo.systemUptime
        guard session != nil, now >= nextRGB, rgb.available,
              let buffer = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        nextRGB = now + 0.2
        let raw = CIImage(cvPixelBuffer: buffer)
        let scale = min(1, 640 / raw.extent.width)
        let reduced = raw.transformed(by: .init(scaleX: scale, y: scale))
        guard let cg = context.createCGImage(reduced, from: reduced.extent),
              let jpeg = UIImage(cgImage: cg).jpegData(compressionQuality: 0.7) else { return }
        let id = UInt64(now * 1e6)
        var be = id.bigEndian
        var packet = Data("RGB1".utf8)
        withUnsafeBytes(of: &be) { packet.append(contentsOf: $0) }
        packet.append(jpeg)
        if rgb.send(packet) {
            sources[id] = cg
            if sources.count > 15, let oldest = sources.keys.min() { sources.removeValue(forKey: oldest) }
        }
    }
    func result(_ value: [String: Any]) {
        queue.async {
            let id = (value["frame_id"] as? NSNumber)?.uint64Value ?? UInt64(value["frame_id"] as? String ?? "")
            guard let id, let cg = self.sources.removeValue(forKey: id),
                  let objects = value["objects"] as? [[String: Any]] else { return }
            let size = CGSize(width: cg.width, height: cg.height)
            let image = UIGraphicsImageRenderer(size: size).image { renderer in
                UIImage(cgImage: cg).draw(in: CGRect(origin: .zero, size: size))
                UIColor.systemYellow.setStroke()
                for obj in objects {
                    guard let box = obj["bbox"] as? [Double], box.count == 4 else { continue }
                    let rect = CGRect(x: box[0]*size.width, y: box[1]*size.height,
                                      width: box[2]*size.width, height: box[3]*size.height)
                    renderer.cgContext.stroke(rect, width: 2)
                    (obj["label"] as? String ?? "").draw(at: rect.origin, withAttributes: [.foregroundColor: UIColor.white, .backgroundColor: UIColor.black])
                }
            }
            DispatchQueue.main.async { self.detected = image; self.detectedAt = Double(id)/1e6 }
        }
    }
    func depthDataOutput(_ output: AVCaptureDepthDataOutput, didOutput depthData: AVDepthData,
                         timestamp: CMTime, connection: AVCaptureConnection) {
        guard session != nil else { return }
        let disparity = depthData.converting(toDepthDataType: kCVPixelFormatType_DisparityFloat32)
        let frame = CIImage(cvPixelBuffer: disparity.depthDataMap)
        guard let cg = context.createCGImage(frame, from: frame.extent) else { return }
        let now = CMTimeGetSeconds(timestamp)
        let fps = last > 0 && now > last ? 1 / (now-last) : 0
        last = now
        let info = String(format: "%@ · %d × %d · %.1f fps", label, cg.width, cg.height, fps)
        DispatchQueue.main.async { self.image = UIImage(cgImage: cg); self.status = info }
    }
}
