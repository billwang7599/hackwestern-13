// macOS test for the app's pure logic (DepthScan.swift + DepthMessage.swift).
// Build/run: make -C iphone test-swift
// Prints "SWIFT OK n/n" and, with --emit, one sample JSON line per scene on
// stdout for the Python validator / C parser cross-check.

import Foundation
import simd

var pass = 0, fail = 0
func check(_ ok: Bool, _ what: String) {
    if ok { pass += 1 } else { fail += 1; FileHandle.standardError.write("FAIL: \(what)\n".data(using: .utf8)!) }
}
func near(_ a: Float?, _ b: Float, _ tol: Float) -> Bool { a.map { abs($0 - b) <= tol } ?? false }

// iPhone-like depth map: 256x192, ~65° HFOV
let W = 256, H = 192
let K = Intrinsics(fx: 200, fy: 200, cx: 128, cy: 96)
let camH: Float = 0.12

/// Synthetic scene, level camera at camH: floor everywhere, a wall at z = wallZ
/// (z-depth), plus an optional box covering columns [u0, u1) at depth boxZ.
func render(wallZ: Float, box: (Int, Int, Float)? = nil, lowConfCols: Range<Int>? = nil) -> ([Float32], [UInt8]) {
    var d = [Float32](repeating: 0, count: W * H), c = [UInt8](repeating: 2, count: W * H)
    for v in 0..<H {
        let yd = (Float(v) + 0.5 - K.cy) / K.fy
        for u in 0..<W {
            var z = wallZ
            if yd > 0 { z = min(z, camH / yd) }                    // floor
            if let b = box, u >= b.0, u < b.1, yd * b.2 < camH { z = min(z, b.2) }
            d[v * W + u] = z
            if let lc = lowConfCols, lc.contains(u) { c[v * W + u] = 0 }
        }
    }
    return (d, c)
}

func runScan(_ dc: ([Float32], [UInt8]), flipped: Bool = false, pitch: Float = 0, bins: Int = 61) -> ScanResult {
    var p = ScanParams(); p.bins = bins; p.camHeight = camH
    return dc.0.withUnsafeBufferPointer { dp in
        dc.1.withUnsafeBufferPointer { cp in
            DepthScan.scan(depth: dp.baseAddress!, depthStride: W, conf: cp.baseAddress!, confStride: W,
                           width: W, height: H, K: K, flipped: flipped, pitchDeg: pitch, p: p)
        }
    }
}

let emit = CommandLine.arguments.contains("--emit")
var lines: [String] = []

// 1. open room: wall at 3 m, floor ignored
do {
    let s = runScan(render(wallZ: 3))
    let half = atanf(128 / 200) * 180 / .pi
    check(near(s.fovLo, -half, 0.01) && near(s.fovHi, half, 0.01), "fov ±\(half) got \(s.fovLo),\(s.fovHi)")
    check(s.ranges.count == 61, "61 bins")
    check(near(s.minRange(lo: -2, hi: 2), 3.0, 0.01), "centre 3 m, got \(String(describing: s.minRange(lo: -2, hi: 2)))")
    // edge bin: horizontal range = z / cos(bearing) at its closest column
    let edge = s.ranges[0]!
    check(edge > 3.4 && edge < 3.6, "edge range ≈ 3/cos(31°)=3.5, got \(edge)")
    check(s.ranges.allSatisfy { $0 != nil && $0! > 2.99 }, "floor never shows up as an obstacle")
    lines.append(DepthMessage.encode(seq: 1, t: 12.5, scan: s,
                                     pose: PlanarPose(x: 0, y: 0, yaw: 0, pitch: 0, flipped: false, portrait: false),
                                     track: "normal"))
}
// 2. wall at 0.4 m
do {
    let s = runScan(render(wallZ: 0.4))
    check(near(s.minRange(lo: -15, hi: 15), 0.4, 0.01), "wall 0.4 ahead")
    lines.append(DepthMessage.encode(seq: 2, t: 12.6, scan: s, pose: nil, track: "na"))
}
// 3. box on the RIGHT (columns 200..240) at 0.5 m; flipped mount puts it on the LEFT
do {
    let dc = render(wallZ: 4, box: (200, 240, 0.5))
    let s = runScan(dc)
    check(near(s.minRange(lo: 15, hi: 40), 0.5 / cosf(atanf(72.5 / 200)), 0.03), "box on right, got \(String(describing: s.minRange(lo: 15, hi: 40)))")
    check((s.minRange(lo: -40, hi: -5) ?? 0) > 3.9, "left clear")
    // an upside-down phone sees the same scene rotated 180° in the image (cy, cx are central)
    let f = runScan((dc.0.reversed(), dc.1.reversed()), flipped: true)
    check(near(f.minRange(lo: 15, hi: 40), s.minRange(lo: 15, hi: 40) ?? -1, 0.03), "flipped mount: box still on the right")
    check((f.minRange(lo: -40, hi: -5) ?? 0) > 3.9, "flipped mount: left clear, floor rejected")
    // and if the flip flag were wrong, the box would land on the left
    let wrong = runScan((dc.0.reversed(), dc.1.reversed()), flipped: false)
    check((wrong.minRange(lo: -40, hi: -15) ?? 9) < 3, "sanity: unflipped reading of a flipped image is mirrored")
    lines.append(DepthMessage.encode(seq: 3, t: 12.7, scan: s,
                                     pose: PlanarPose(x: 0.1, y: 0.25, yaw: -12.3, pitch: 1, flipped: false, portrait: false),
                                     track: "limited"))
}
// 4. low-confidence columns are dropped (nil), not reported as close
do {
    let s = runScan(render(wallZ: 1, lowConfCols: 0..<40))
    check(s.ranges[0] == nil && s.ranges[5] == nil, "low confidence -> null")
    check(near(s.minRange(lo: -2, hi: 2), 1, 0.01), "rest still 1 m")
}
// 5. pitched down 10°: floor must still be rejected (scene re-rendered for a pitched camera)
do {
    let th: Float = 10 * .pi / 180
    var d = [Float32](repeating: 0, count: W * H), c = [UInt8](repeating: 2, count: W * H)
    for v in 0..<H { for u in 0..<W {
        let yd = (Float(v) + 0.5 - K.cy) / K.fy
        // ray (X, Ydown, 1) in camera frame -> world: down = Yd cos + sin, fwd = cos - Yd sin
        let down = yd * cosf(th) + sinf(th), fwd = cosf(th) - yd * sinf(th)
        var z: Float = 2.0 / fwd                         // wall at 2 m horizontal
        if down > 0 { z = min(z, camH / down) }          // floor
        d[v * W + u] = z; c[v * W + u] = 2
    } }
    let s = runScan((d, c), pitch: 10)
    check(near(s.minRange(lo: -2, hi: 2), 2.0, 0.02), "pitched: wall 2 m, got \(String(describing: s.minRange(lo: -2, hi: 2)))")
    let s0 = runScan((d, c), pitch: 0)   // ignoring pitch would see floor as obstacle
    check((s0.minRange(lo: -2, hi: 2) ?? 9) < 1.5, "sanity: without pitch correction the floor leaks in")
}
// 6. pose math
do {
    let p0 = PoseMath.planar(matrix_identity_float4x4)
    check(abs(p0.yaw) < 1e-4 && abs(p0.pitch) < 1e-4 && !p0.flipped && !p0.portrait, "identity pose")
    // turn RIGHT 30° = rotate about +Y by -30°
    var m = simd_float4x4(simd_quatf(angle: -30 * .pi / 180, axis: SIMD3(0, 1, 0)))
    m.columns.3 = SIMD4(0.5, 0, -2, 1)       // 0.5 m right, 2 m forward
    let p1 = PoseMath.planar(m)
    check(abs(p1.yaw - 30) < 0.01, "right turn -> yaw +30, got \(p1.yaw)")
    check(abs(p1.x - 0.5) < 1e-5 && abs(p1.y - 2) < 1e-5, "x right, y forward")
    let down = simd_float4x4(simd_quatf(angle: -10 * .pi / 180, axis: SIMD3(1, 0, 0)))
    check(abs(PoseMath.planar(down).pitch - 10) < 0.01, "pitch down +10")
    let flip = simd_float4x4(simd_quatf(angle: .pi, axis: SIMD3(0, 0, 1)))
    check(PoseMath.planar(flip).flipped, "upside-down detected")
    let portrait = simd_float4x4(simd_quatf(angle: .pi / 2, axis: SIMD3(0, 0, 1)))
    check(PoseMath.planar(portrait).portrait, "portrait detected")
}
// 7. message formatting
do {
    check(DepthMessage.fmt(-0.0001, 2) == "0.00", "no -0.00")
    check(DepthMessage.fmt(1e-9, 3) == "0.000", "no exponent")
    check(DepthMessage.fmt(.nan, 2) == "null", "nan -> null")
    for l in lines {
        let ok = (try? JSONSerialization.jsonObject(with: l.data(using: .utf8)!)) != nil
        check(ok && !l.contains("\n"), "valid single-line JSON: \(l.prefix(80))")
    }
}

if emit { for l in lines { print(l) } }
FileHandle.standardError.write("SWIFT \(fail == 0 ? "OK" : "FAIL") \(pass)/\(pass + fail)\n".data(using: .utf8)!)
exit(fail == 0 ? 0 : 1)
