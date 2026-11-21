#!/usr/bin/env python3
"""Run the real Foundation/simd depth helpers on macOS without iOS/AR hardware."""
from pathlib import Path
import subprocess
import tempfile

source = Path(__file__).with_name('SceneCapture.swift').read_text()
calibration = source[source.index('struct RelativeDepthCalibration {'):source.index('// BEGIN REMEMBERED_DEPTH_MATH')]
math = source.split('// BEGIN REMEMBERED_DEPTH_MATH', 1)[1].split('// END REMEMBERED_DEPTH_MATH', 1)[0]
math = math[math.index('\n') + 1:]
checks = r'''
var checks = 0
func check(_ condition: @autoclosure () -> Bool, _ name: String) {
    precondition(condition(), "FAILED: " + name); checks += 1
}
func close(_ a: Double,_ b: Double,_ tolerance: Double = 0.0001) -> Bool { abs(a-b)<tolerance }
let k = simd_float3x3(SIMD3(500,0,0),SIMD3(0,500,0),SIMD3(320,240,1))
let camera = DepthCameraGeometry(intrinsics:k,transform:matrix_identity_float4x4,resolution:SIMD2(640,480))
let uv = SIMD2<Float>(0.25,0.73)
check(simd_distance(camera.levelCorner(uv)!, uv) < 1e-5, "level camera preserves pixels")
let yaw = simd_quatf(angle:0.8,axis:SIMD3<Float>(0,1,0))
let pitch = simd_quatf(angle:0.14,axis:SIMD3<Float>(1,0,0))
let roll = simd_quatf(angle:-0.11,axis:SIMD3<Float>(0,0,1))
var r = simd_float4x4(yaw * pitch * roll); r.columns.3 = SIMD4(1,2,3,1)
let corrected = DepthCameraGeometry(intrinsics:k,transform:r,resolution:SIMD2(640,480))
for side: Float in [-0.3, 0.3] {
    // A world horizon ray must land at principal-point height despite pitch/roll/yaw.
    let ray = (yaw * pitch * roll).inverse.act(yaw.act(SIMD3<Float>(side,0,-1)))
    let source = SIMD2(0.5 + 500/640 * ray.x / -ray.z, 0.5 + 500/480 * ray.y / -ray.z)
    let pixel = corrected.levelCorner(source)!
    check(abs(pixel.y-0.5) < 1e-5 && abs(pixel.x-(0.5+500/640*side)) < 1e-5, "horizon rectified")
}
let vertical = DepthCameraGeometry(intrinsics:k,transform:simd_float4x4(simd_quatf(angle:.pi/2,axis:SIMD3<Float>(1,0,0))),resolution:SIMD2(640,480))
check(vertical.levelCorner(uv) == nil, "floor calibration retains its downward view")
let world = camera.world(u:0.3,v:0.6,meters:3)!
let roundtrip = camera.project(world)!
check(close(roundtrip.u,0.3) && close(roundtrip.v,0.6) && close(roundtrip.meters,3), "portrait reprojection roundtrip")
var transform = simd_float4x4(simd_quatf(angle:0.4,axis:SIMD3(0,1,0)))
transform.columns.3 = SIMD4(1,0.5,-2,1)
let moved = DepthCameraGeometry(intrinsics:k,transform:transform,resolution:SIMD2(640,480))
let movedPoint = moved.world(u:0.7,v:0.4,meters:5)!
let movedProjection = moved.project(movedPoint)!
check(close(movedProjection.u,0.7) && close(movedProjection.v,0.4) && close(movedProjection.meters,5), "rotated translated roundtrip")
var shiftedTransform = matrix_identity_float4x4; shiftedTransform.columns.3.x = 0.1
let shifted = DepthCameraGeometry(intrinsics:k,transform:shiftedTransform,resolution:SIMD2(640,480))
check(shifted.project(world)!.v < roundtrip.v, "old world anchor reprojects to new pixel")
check(camera.project(SIMD3(0,0,1)) == nil, "behind camera rejected")
check(camera.project(SIMD3(10,0,-1)) == nil, "outside frustum rejected")
check(camera.project(SIMD3(0,0,-21)) == nil, "projection beyond display range rejected")
check(camera.project(SIMD3(.nan,0,-1)) == nil, "nonfinite worldpoint rejected")
check(camera.world(u:0.5,v:0.5,meters:20) != nil && camera.world(u:0.5,v:0.5,meters:21) == nil, "world range is bounded, not clamped")
check(close(shifted.motion(from:camera).translation,0.1) && close(shifted.motion(from:camera).degrees,0), "motion translation measured")
check(close(moved.motion(from:camera).degrees,0.4*180 / .pi,0.001), "motion rotation measured")
let pairs: [(Double,Double)] = (0..<100).map { i in
    let d = 0.4+Double(i)/99*3.3
    return ((1/d-0.08)/1.7,d)
}
let fit = try RelativeDepthCalibration.fit(pairs)
check(close(fit.scale,1.7) && close(fit.offset,0.08), "robust metric scale and offset")
let farRelative = (1.0/15-0.08)/1.7
check(fit.meters(farRelative) == nil && close(fit.meters(farRelative,maximum:20)!,15), "safety 8m versus visual 20m ranges")
check(fit.meters((1.0/40-0.08)/1.7,maximum:20) == nil, "distant inverse not clamped to 20m")
check(fit.meters(-10,maximum:20) == nil && fit.meters(.nan,maximum:20) == nil, "negative/nonfinite inverse rejected")
let farPairs: [(Double,Double)] = (0..<100).map { i in
    let d = 4.2+Double(i)/99*3.5
    return ((1/d-0.08)/1.7,d)
}
check((try? RelativeDepthCalibration.fit(farPairs)) == nil, "direct fit rejects beyond measured range")
check((try? RelativeDepthCalibration.fit(farPairs,maximumAnchor:20)) != nil, "reprojected anchors can have farther current depth")
check((try? RelativeDepthCalibration.fit(Array(pairs.prefix(39)))) == nil, "single/few anchors insufficient")
check((try? RelativeDepthCalibration.fit(Array(repeating:(0.5,2.0),count:100))) == nil, "single depth cannot fit metric affine scale")
check(RememberedDepthMath.cacheEligible(age:3,translation:0.12,degrees:6), "bounded cache edge accepted")
check(!RememberedDepthMath.cacheEligible(age:3.01,translation:0,degrees:0), "old cache rejected")
check(!RememberedDepthMath.cacheEligible(age:-0.1,translation:0,degrees:0), "future cache rejected")
check(!RememberedDepthMath.cacheEligible(age:1,translation:0.13,degrees:0), "translated cache rejected")
check(!RememberedDepthMath.cacheEligible(age:1,translation:0,degrees:6.1), "rotated cache rejected")
let predictions = (0..<4096).map { 0.1+Double($0)/4095 }
check(RememberedDepthMath.predictionsAgree(predictions,predictions), "identical predictions agree")
check(!RememberedDepthMath.predictionsAgree(predictions,Array(predictions.reversed())), "same distribution with changed image layout rejected")
check(!RememberedDepthMath.predictionsAgree(predictions,predictions.map { $0+0.3 }), "changed prediction scale rejected")
check(!RememberedDepthMath.predictionsAgree(Array(repeating:1,count:4096),Array(repeating:1,count:4096)), "flat predictions cannot validate cache")
let field = (0..<4096).map { Double($0%64)+2*Double($0/64) }
check(close(RememberedDepthMath.sample(field,u:0.5,v:0.5),94.5), "portrait bilinear sample")
let patch: [Float] = (0..<27).map { (($0/3)%2 == 0) ? 0.2 : 0.8 }
check(DepthRGBTexture.agrees(patch,patch), "textured correspondence accepted")
check(DepthRGBTexture.agrees(patch,patch.map { $0+0.03 }), "small illumination change accepted")
check(!DepthRGBTexture.agrees(patch,patch.map { 1-$0 }), "changed texture rejected")
check(!DepthRGBTexture.agrees(Array(repeating:0.5,count:27),Array(repeating:0.5,count:27)), "flat-color false correspondence rejected")
check(RememberedDepthMath.unoccluded(anchorMeters:2,measuredMeters:nil), "missing LiDAR leaves appearance gate responsible")
check(RememberedDepthMath.unoccluded(anchorMeters:2,measuredMeters:2.1), "consistent measured surface accepted")
check(!RememberedDepthMath.unoccluded(anchorMeters:2,measuredMeters:1), "nearer LiDAR occluder rejects old anchor")
check(!RememberedDepthMath.unoccluded(anchorMeters:2,measuredMeters:3), "disappeared measured surface rejects old anchor")
// Verify the exact explicit RGBA CGContext conversion used after UIImage normalization.
let cornerBytes: [UInt8] = [255,0,0,255, 0,255,0,255, 0,0,255,255, 255,255,0,255]
let cornerImage = CGImage(width:2,height:2,bitsPerComponent:8,bitsPerPixel:32,bytesPerRow:8,
    space:CGColorSpaceCreateDeviceRGB(),bitmapInfo:CGBitmapInfo(rawValue:CGImageAlphaInfo.premultipliedLast.rawValue | CGBitmapInfo.byteOrder32Big.rawValue),
    provider:CGDataProvider(data:Data(cornerBytes) as CFData)!,decode:nil,shouldInterpolate:false,intent:.defaultIntent)!
var converted = [UInt8](repeating:0,count:16)
converted.withUnsafeMutableBytes { storage in
    let c = CGContext(data:storage.baseAddress,width:2,height:2,bitsPerComponent:8,bytesPerRow:8,space:CGColorSpaceCreateDeviceRGB(),
        bitmapInfo:CGImageAlphaInfo.premultipliedLast.rawValue | CGBitmapInfo.byteOrder32Big.rawValue)!
    c.draw(cornerImage,in:CGRect(x:0,y:0,width:2,height:2))
}
check(converted == cornerBytes, "asymmetric RGB corners preserve top-left scanline order")
print("Passed \(checks) depth geometry/anchor/cache checks")
'''
with tempfile.TemporaryDirectory(prefix='hw13-depth-math-') as directory:
    directory = Path(directory)
    program = directory / 'main.swift'
    program.write_text('import Foundation\nimport simd\nimport CoreGraphics\n' + calibration + math + checks)
    binary = directory / 'depth_geometry_checks'
    subprocess.run(['xcrun', 'swiftc', '-module-cache-path', str(directory / 'module-cache'),
                    str(program), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
