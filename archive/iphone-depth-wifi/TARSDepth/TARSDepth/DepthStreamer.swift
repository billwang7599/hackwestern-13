// DepthStreamer.swift: ARKit session -> DepthScan -> UDP JSON lines to the Pi.
// The phone is only a sensor; the Pi (QNX) decides. See iphone/PROTOCOL.md.

import ARKit
import Foundation
import Network
import UIKit

final class DepthStreamer: NSObject, ObservableObject, ARSessionDelegate {
    // UI-facing state (updated on main, throttled to ~4 Hz)
    @Published var running = false
    @Published var error: String? = nil
    @Published var track = "na"
    @Published var sendHz: Double = 0
    @Published var minAhead: Float? = nil
    @Published var ranges: [Float?] = []
    @Published var fov: (Float, Float) = (-30, 30)
    @Published var poseText = "-"
    @Published var mountWarning: String? = nil
    @Published var sent: UInt64 = 0
    @Published var sendErrors: Int = 0

    var rateHz: Double = 12
    var params = ScanParams()

    private let session = ARSession()
    private let queue = DispatchQueue(label: "tars.depth", qos: .userInteractive)
    private var conn: NWConnection?
    private var seq: UInt64 = 0
    private var lastSend: TimeInterval = 0
    private var lastUI: TimeInterval = 0
    private var hzWindow: [TimeInterval] = []
    private var errCount = 0

    static var lidarAvailable: Bool {
        ARWorldTrackingConfiguration.isSupported &&
        ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)
    }

    func start(host: String, port: UInt16) {
        guard Self.lidarAvailable else {
            error = "This device has no LiDAR (needs iPhone 12 Pro or later Pro, or iPad Pro). Nothing is sent."
            return
        }
        let h = host.trimmingCharacters(in: .whitespaces)
        guard !h.isEmpty, let p = NWEndpoint.Port(rawValue: port) else {
            error = "Enter the Pi's IP address and port."
            return
        }
        error = nil
        queue.async { [weak self] in self?.openConnection(host: h, port: p) }

        let cfg = ARWorldTrackingConfiguration()
        cfg.frameSemantics = [.sceneDepth]
        cfg.worldAlignment = .gravity
        cfg.planeDetection = []
        session.delegate = self
        session.delegateQueue = queue
        session.run(cfg, options: [.resetTracking])   // pose origin = where it starts
        UIApplication.shared.isIdleTimerDisabled = true
        running = true
    }

    func stop() {
        session.pause()
        queue.async { [weak self] in self?.conn?.cancel(); self?.conn = nil }
        UIApplication.shared.isIdleTimerDisabled = false
        running = false
    }

    private func openConnection(host: String, port: NWEndpoint.Port) {
        conn?.cancel()
        let c = NWConnection(host: NWEndpoint.Host(host), port: port, using: .udp)
        c.stateUpdateHandler = { [weak self] st in
            if case .failed(let e) = st {
                DispatchQueue.main.async { self?.error = "UDP: \(e.localizedDescription)" }
            }
        }
        c.start(queue: queue)
        conn = c
    }

    // MARK: ARSessionDelegate (runs on `queue`)

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let now = frame.timestamp
        if now - lastSend < 1.0 / max(1, rateHz) { return }
        lastSend = now

        let pose = PoseMath.planar(frame.camera.transform)
        let track: String
        switch frame.camera.trackingState {
        case .normal: track = "normal"
        case .limited: track = "limited"
        case .notAvailable: track = "na"
        }

        guard let sd = frame.sceneDepth else { return }
        let scan = Self.scan(sceneDepth: sd, camera: frame.camera, pose: pose, params: params)

        seq += 1
        let line = DepthMessage.encode(seq: seq, t: now, scan: scan,
                                       pose: track == "na" ? nil : pose, track: track) + "\n"
        conn?.send(content: line.data(using: .utf8), completion: .contentProcessed { [weak self] e in
            if e != nil { self?.errCount += 1 }
        })

        hzWindow.append(now)
        hzWindow.removeAll { now - $0 > 2 }
        if now - lastUI > 0.25 {
            lastUI = now
            let hz = Double(max(0, hzWindow.count - 1)) / max(0.001, (hzWindow.last ?? now) - (hzWindow.first ?? now))
            let ahead = scan.minRange(lo: -15, hi: 15)
            let warn: String? = pose.portrait
                ? "Phone looks PORTRAIT. Mount it landscape (rear camera forward) or bearings are wrong."
                : (abs(pose.pitch) > 15 ? String(format: "Phone is pitched %.0f°. Mount it level.", pose.pitch) : nil)
            let txt = String(format: "x %.2f  y %.2f  yaw %.1f°  pitch %.1f°%@",
                             pose.x, pose.y, pose.yaw, pose.pitch, pose.flipped ? "  (upside-down, handled)" : "")
            let s = seq, errs = errCount
            DispatchQueue.main.async {
                self.track = track; self.sendHz = hz; self.minAhead = ahead
                self.ranges = scan.ranges; self.fov = (scan.fovLo, scan.fovHi)
                self.poseText = txt; self.mountWarning = warn; self.sent = s; self.sendErrors = errs
            }
        }
    }

    func session(_ session: ARSession, didFailWithError error: Error) {
        DispatchQueue.main.async { self.error = "ARKit: \(error.localizedDescription)"; self.running = false }
    }

    func sessionWasInterrupted(_ session: ARSession) {
        DispatchQueue.main.async { self.error = "ARKit interrupted (app in background?). Pi will see stale data and treat it as no lidar." }
    }

    func sessionInterruptionEnded(_ session: ARSession) {
        DispatchQueue.main.async { self.error = nil }
    }

    // MARK: depth map -> scan

    static func scan(sceneDepth sd: ARDepthData, camera: ARCamera, pose: PlanarPose,
                     params: ScanParams) -> ScanResult {
        let dm = sd.depthMap
        CVPixelBufferLockBaseAddress(dm, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(dm, .readOnly) }
        let w = CVPixelBufferGetWidth(dm), h = CVPixelBufferGetHeight(dm)
        let dStride = CVPixelBufferGetBytesPerRow(dm) / MemoryLayout<Float32>.size
        guard let dBase = CVPixelBufferGetBaseAddress(dm) else {
            return ScanResult(fovLo: -30, fovHi: 30, ranges: Array(repeating: nil, count: params.bins), used: 0)
        }
        let depth = UnsafePointer(dBase.assumingMemoryBound(to: Float32.self))

        var confPtr: UnsafePointer<UInt8>? = nil
        var cStride = 0
        let cm = sd.confidenceMap
        if let cm = cm {
            CVPixelBufferLockBaseAddress(cm, .readOnly)
            if let cb = CVPixelBufferGetBaseAddress(cm) {
                confPtr = UnsafePointer(cb.assumingMemoryBound(to: UInt8.self))
                cStride = CVPixelBufferGetBytesPerRow(cm)
            }
        }
        defer { if let cm = cm { CVPixelBufferUnlockBaseAddress(cm, .readOnly) } }

        // ARKit intrinsics are for the captured image; scale to the depth map
        let k = camera.intrinsics
        let sx = Float(w) / Float(camera.imageResolution.width)
        let sy = Float(h) / Float(camera.imageResolution.height)
        let K = Intrinsics(fx: k[0][0] * sx, fy: k[1][1] * sy, cx: k[2][0] * sx, cy: k[2][1] * sy)

        return DepthScan.scan(depth: depth, depthStride: dStride, conf: confPtr, confStride: cStride,
                              width: w, height: h, K: K, flipped: pose.flipped,
                              pitchDeg: pose.pitch, p: params)
    }
}
