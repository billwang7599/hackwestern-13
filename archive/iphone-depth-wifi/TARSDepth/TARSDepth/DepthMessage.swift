// DepthMessage.swift: the one-line JSON the phone sends to the Pi (UDP 5559).
// Format is specified in iphone/PROTOCOL.md; keep both in sync. Pure Foundation
// so the macOS test can check it against the Python validator and C parser.

import Foundation

enum DepthMessage {
    static let version = 1

    /// track: "normal", "limited", "na" (ARKit tracking state)
    static func encode(seq: UInt64, t: Double, scan: ScanResult, pose: PlanarPose?,
                       track: String) -> String {
        var s = "{\"v\":\(version),\"seq\":\(seq),\"t\":\(fmt(t, 3))"
        s += ",\"fov\":[\(fmt(Double(scan.fovLo), 1)),\(fmt(Double(scan.fovHi), 1))]"
        s += ",\"ranges\":["
        for (i, r) in scan.ranges.enumerated() {
            if i > 0 { s += "," }
            if let r = r { s += fmt(Double(r), 2) } else { s += "null" }
        }
        s += "]"
        if let p = pose {
            s += ",\"pose\":{\"x\":\(fmt(Double(p.x), 3)),\"y\":\(fmt(Double(p.y), 3))"
            s += ",\"yaw\":\(fmt(Double(p.yaw), 1)),\"pitch\":\(fmt(Double(p.pitch), 1))"
            s += ",\"ok\":\(track == "normal" ? "true" : "false")}"
        } else {
            s += ",\"pose\":null"
        }
        s += ",\"track\":\"\(track)\"}"
        return s
    }

    /// Fixed decimals, never exponent notation, "-0.0" normalised to "0.0".
    static func fmt(_ x: Double, _ dp: Int) -> String {
        guard x.isFinite else { return "null" }
        let s = String(format: "%.\(dp)f", x)
        if s.hasPrefix("-"), Double(s) == 0 { return String(s.dropFirst()) }
        return s
    }
}
