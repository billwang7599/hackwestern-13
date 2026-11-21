// DepthScan.swift: pure logic, no ARKit. Turns a LiDAR depth map into a
// "virtual 2D lidar" (min horizontal range per bearing bin) and an ARKit
// camera transform into a planar pose. Compiled into the app AND into the
// macOS test (Tests/scan_test_main.swift), so keep it Foundation + simd only.
//
// Conventions (see iphone/PROTOCOL.md):
//   bearing: degrees, 0 = straight ahead, + = RIGHT (same as goto's camera
//            bearing and tars_motor's TURN + = right)
//   pose:    x right, y forward (of the heading when the app started), metres;
//            yaw degrees, + = turned right (clockwise seen from above)

import Foundation
import simd

struct ScanParams {
    var bins: Int = 61               // angular bins across the horizontal FOV
    var bandHalfFrac: Float = 0.15   // rows used: centre row ± this × image height
    var camHeight: Float = 0.12      // phone lens height above the floor (m)
    var floorMargin: Float = 0.04    // points lower than camHeight - this below the lens = floor
    var minDepth: Float = 0.05       // m
    var maxRange: Float = 5.0        // LiDAR is good to ~5 m
    var minConfidence: UInt8 = 1     // ARConfidenceLevel: 0 low, 1 medium, 2 high
}

/// Pinhole intrinsics in DEPTH-MAP pixels (scale ARKit's, which are for the
/// captured image, by depthWidth / imageWidth).
struct Intrinsics {
    var fx: Float, fy: Float, cx: Float, cy: Float
}

struct ScanResult {
    var fovLo: Float          // bearing of the left image edge (deg, negative)
    var fovHi: Float          // bearing of the right image edge (deg, positive)
    var ranges: [Float?]      // per bin, horizontal range in m, nil = no valid return
    var used: Int             // valid pixels used (debug)

    /// Min range over bins whose centre bearing is within [lo, hi] degrees.
    func minRange(lo: Float, hi: Float) -> Float? {
        let n = ranges.count
        guard n > 0 else { return nil }
        let w = (fovHi - fovLo) / Float(n)
        var best: Float? = nil
        for i in 0..<n {
            let b = fovLo + (Float(i) + 0.5) * w
            if b < lo || b > hi { continue }
            if let r = ranges[i], best == nil || r < best! { best = r }
        }
        return best
    }
}

enum DepthScan {
    /// - depth: Float32 metres (ARKit sceneDepth.depthMap), row-major, `depthStride` floats per row
    /// - conf:  UInt8 ARConfidenceLevel per pixel (may be nil = treat all as high)
    /// - flipped: phone mounted upside-down landscape (image rotated 180°)
    /// - pitchDeg: optical axis below horizontal (+ = looking down). Used for
    ///   the floor test and to project range onto the floor plane.
    static func scan(depth: UnsafePointer<Float32>, depthStride: Int,
                     conf: UnsafePointer<UInt8>?, confStride: Int,
                     width: Int, height: Int, K: Intrinsics,
                     flipped: Bool, pitchDeg: Float, p: ScanParams) -> ScanResult {
        let n = max(1, p.bins)
        let sgn: Float = flipped ? -1 : 1
        let edgeL = atanf((0 - K.cx) / K.fx) * 180 / .pi
        let edgeR = atanf((Float(width) - K.cx) / K.fx) * 180 / .pi
        let fovLo = flipped ? -edgeR : edgeL
        let fovHi = flipped ? -edgeL : edgeR
        let span = max(fovHi - fovLo, 1e-3)

        // per-column tangent and bin (columns are the cheap axis to precompute)
        var colTan = [Float](repeating: 0, count: width)
        var colBin = [Int](repeating: 0, count: width)
        for u in 0..<width {
            let xr = (Float(u) + 0.5 - K.cx) / K.fx
            colTan[u] = xr
            let bearing = sgn * atanf(xr) * 180 / .pi
            colBin[u] = min(n - 1, max(0, Int((bearing - fovLo) / span * Float(n))))
        }

        let half = Int(Float(height) * p.bandHalfFrac)
        let cRow = Int(K.cy.rounded())
        let r0 = max(0, cRow - half), r1 = min(height - 1, cRow + half)
        let th = pitchDeg * .pi / 180
        let cs = cosf(th), sn = sinf(th)
        let floorDrop = p.camHeight - p.floorMargin

        var best = [Float](repeating: .infinity, count: n)
        var used = 0
        if r0 <= r1 {
            for v in r0...r1 {
                // image-down tangent; when flipped, image-down is world-up
                let yd = sgn * (Float(v) + 0.5 - K.cy) / K.fy
                let drow = depth + v * depthStride
                let crow = conf.map { $0 + v * confStride }
                for u in 0..<width {
                    let d = drow[u]
                    if !(d >= p.minDepth && d <= p.maxRange) { continue }   // also rejects NaN
                    if let c = crow, c[u] < p.minConfidence { continue }
                    // camera frame: X right, Ydown, Z forward (depth is along Z)
                    let X = colTan[u] * d, Yd = yd * d
                    let down = Yd * cs + d * sn          // metres below the lens, world
                    if down > floorDrop { continue }     // floor
                    let fwd = d * cs - Yd * sn
                    let r = sqrtf(X * X + fwd * fwd)
                    let b = colBin[u]
                    if r < best[b] { best[b] = r }
                    used += 1
                }
            }
        }
        let ranges: [Float?] = best.map { $0.isFinite && $0 <= p.maxRange ? $0 : nil }
        return ScanResult(fovLo: fovLo, fovHi: fovHi, ranges: ranges, used: used)
    }
}

struct PlanarPose {
    var x: Float, y: Float, yaw: Float   // m, m, deg (+ = right)
    var pitch: Float                     // deg, + = looking down
    var flipped: Bool                    // landscape upside-down
    var portrait: Bool                   // wrong mount: image columns aren't horizontal
}

enum PoseMath {
    /// ARKit world (gravity aligned): +Y up, camera starts looking along -Z.
    /// Ours: x = ARKit x, y = -ARKit z, yaw 0 = initial heading, + = right.
    static func planar(_ t: simd_float4x4) -> PlanarPose {
        let f = -simd_make_float3(t.columns.2)          // optical axis (camera looks along -Z)
        let camUp = simd_make_float3(t.columns.1)       // image "up" in world
        let camRight = simd_make_float3(t.columns.0)
        let yaw = atan2f(f.x, -f.z) * 180 / .pi
        let pitch = asinf(max(-1, min(1, -f.y))) * 180 / .pi
        return PlanarPose(x: t.columns.3.x, y: -t.columns.3.z, yaw: yaw, pitch: pitch,
                          flipped: camUp.y < 0,
                          portrait: abs(camRight.y) > abs(camUp.y))
    }
}
