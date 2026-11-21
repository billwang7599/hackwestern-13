import SwiftUI
import ARKit
import SceneKit
import AVFoundation
import Network
import CoreImage
import UIKit

struct DetectedObject: Identifiable {
    let id = UUID()
    let label: String
    let confidence: Float
    // Vision coordinates: normalized, origin at bottom left of portrait image.
    let boundingBox: CGRect
    let position: SIMD3<Float>?
}

struct SceneCameraPreview: UIViewRepresentable {
    let session: ARSession
    func makeUIView(context: Context) -> ARSCNView {
        let view = ARSCNView(frame: .zero)
        view.session = session
        view.automaticallyUpdatesLighting = false
        view.rendersContinuously = true
        return view
    }
    func updateUIView(_ view: ARSCNView, context: Context) {
        if view.session !== session { view.session = session }
    }
}

// All instances are owned by CameraServer's capture queue. Acknowledgements
// bound each stream to one outstanding packet; a slow receiver drops frames.
final class ScenePacketStream {
    private let queue: DispatchQueue
    private let limit: Int
    private let acknowledgementTimeout: TimeInterval
    private var listener: NWListener?
    private var client: NWConnection?
    private var busy = false
    private var sequence: UInt64 = 0
    var available: Bool { client?.state == .ready && !busy }
    init(queue: DispatchQueue, limit: Int, acknowledgementTimeout: TimeInterval = 2) {
        self.queue = queue; self.limit = limit; self.acknowledgementTimeout = acknowledgementTimeout
    }

    func start(port: UInt16, error: @escaping (String) -> Void) throws {
        let listener = try NWListener(using: .tcp, on: NWEndpoint.Port(rawValue: port)!)
        self.listener = listener
        listener.stateUpdateHandler = { state in
            if case .failed(let reason) = state { error("Port \(port): \(reason.localizedDescription)") }
        }
        listener.newConnectionHandler = { [weak self] connection in
            guard let self else { connection.cancel(); return }
            self.client?.stateUpdateHandler = nil
            self.client?.cancel()
            self.client = connection
            self.busy = false
            connection.stateUpdateHandler = { [weak self, weak connection] state in
                guard let self, let connection, self.client === connection else { return }
                switch state {
                case .failed, .cancelled: self.client = nil; self.busy = false
                default: break
                }
            }
            connection.start(queue: self.queue)
        }
        listener.start(queue: queue)
    }

    @discardableResult func send(_ payload: Data) -> Bool {
        guard available, !payload.isEmpty, payload.count <= limit, let client else { return false }
        var length = UInt32(payload.count).bigEndian
        var packet = Data(bytes: &length, count: 4)
        packet.append(payload)
        busy = true
        sequence &+= 1
        let token = sequence
        client.send(content: packet, completion: .contentProcessed { [weak self] error in
            guard let self, self.client === client else { return }
            guard error == nil else { client.cancel(); return }
            client.receive(minimumIncompleteLength: 1, maximumLength: 1) { [weak self] data, _, complete, error in
                guard let self, self.client === client else { return }
                if error != nil || complete || data != Data([1]) { client.cancel() }
                else { self.busy = false }
            }
        })
        queue.asyncAfter(deadline: .now() + acknowledgementTimeout) { [weak self] in
            guard let self, self.client === client, self.busy, self.sequence == token else { return }
            client.cancel()
        }
        return true
    }

    func stop() {
        listener?.stateUpdateHandler = nil
        listener?.cancel(); listener = nil
        client?.stateUpdateHandler = nil
        client?.cancel(); client = nil
        busy = false
    }
}

// Immutable copy keeps QNX results and capture tied to exactly the same depth/pose
// without retaining ARKit's pool of depth buffers during inference.
private struct SceneDepthGrid {
    let width: Int
    let height: Int
    let values: [Float]
    let confidence: [UInt8]
    let intrinsics: simd_float3x3
    let transform: simd_float4x4

    init?(_ frame: ARFrame) {
        guard let depth = frame.sceneDepth, let confidenceMap = depth.confidenceMap else { return nil }
        let buffer = depth.depthMap
        width = CVPixelBufferGetWidth(buffer); height = CVPixelBufferGetHeight(buffer)
        guard CVPixelBufferGetWidth(confidenceMap) == width,
              CVPixelBufferGetHeight(confidenceMap) == height,
              CVPixelBufferGetPixelFormatType(confidenceMap) == kCVPixelFormatType_OneComponent8,
              CVPixelBufferGetPixelFormatType(buffer) == kCVPixelFormatType_DepthFloat32 else { return nil }
        CVPixelBufferLockBaseAddress(buffer, .readOnly)
        CVPixelBufferLockBaseAddress(confidenceMap, .readOnly)
        defer {
            CVPixelBufferUnlockBaseAddress(buffer, .readOnly)
            CVPixelBufferUnlockBaseAddress(confidenceMap, .readOnly)
        }
        guard let base = CVPixelBufferGetBaseAddress(buffer),
              let conf = CVPixelBufferGetBaseAddress(confidenceMap) else { return nil }
        var values = [Float](); values.reserveCapacity(width * height)
        var confidence = [UInt8](); confidence.reserveCapacity(width * height)
        for y in 0..<height {
            let row = base.advanced(by: y * CVPixelBufferGetBytesPerRow(buffer)).assumingMemoryBound(to: Float.self)
            let c = conf.advanced(by: y * CVPixelBufferGetBytesPerRow(confidenceMap)).assumingMemoryBound(to: UInt8.self)
            values.append(contentsOf: UnsafeBufferPointer(start: row, count: width))
            confidence.append(contentsOf: UnsafeBufferPointer(start: c, count: width))
        }
        self.values = values; self.confidence = confidence
        var k = frame.camera.intrinsics
        let sx = Float(width) / Float(frame.camera.imageResolution.width)
        let sy = Float(height) / Float(frame.camera.imageResolution.height)
        k.columns.0.x *= sx; k.columns.1.y *= sy
        k.columns.2.x *= sx; k.columns.2.y *= sy
        intrinsics = k; transform = frame.camera.transform
    }

    func depth(x: Int, y: Int, allowMedium: Bool = false) -> Float? {
        guard x >= 0, y >= 0, x < width, y < height else { return nil }
        let i = y * width + x, d = values[i]
        guard d.isFinite, d >= 0.2, d <= 4 else { return nil }
        if confidence[i] == UInt8(ARConfidenceLevel.high.rawValue) { return d }
        guard allowMedium, confidence[i] == UInt8(ARConfidenceLevel.medium.rawValue) else { return nil }
        // Require two adjacent medium/high readings on the same surface.
        var agreeing = 0
        for (u,v) in [(x-1,y),(x+1,y),(x,y-1),(x,y+1)] where u >= 0 && v >= 0 && u < width && v < height {
            let j = v * width + u
            if confidence[j] >= UInt8(ARConfidenceLevel.medium.rawValue),
               (0.2...4).contains(values[j]), abs(values[j]-d) <= max(0.05, 0.03*d) { agreeing += 1 }
            if agreeing >= 2 { break }
        }
        guard agreeing >= 2 else { return nil }
        return d
    }

    func project(x: Int, y: Int, depth d: Float) -> SIMD3<Float> {
        let p = transform * SIMD4<Float>((Float(x) - intrinsics.columns.2.x) * d / intrinsics.columns.0.x,
            -(Float(y) - intrinsics.columns.2.y) * d / intrinsics.columns.1.y, -d, 1)
        return SIMD3(p.x, p.y, p.z)
    }

    func centerPosition(x: Int, y: Int) -> SIMD3<Float>? {
        // Median of a small patch rejects individual depth holes/outliers.
        var samples = [Float]()
        for v in (y - 2)...(y + 2) {
            for u in (x - 2)...(x + 2) { if let d = depth(x: u, y: v) { samples.append(d) } }
        }
        guard samples.count >= 5, x >= 0, y >= 0, x < width, y < height else { return nil }
        samples.sort()
        return project(x: x, y: y, depth: samples[samples.count / 2])
    }

    // Sample native, unrotated camera planes at the same normalized coordinates
    // as ARKit scene depth. Geometry, normals and color come from one ARFrame.
    func surfaceSamples(_ image: CVPixelBuffer) -> [[Int]] {
        let format = CVPixelBufferGetPixelFormatType(image)
        guard format == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange,
              CVPixelBufferGetPlaneCount(image) == 2,
              CVPixelBufferLockBaseAddress(image, .readOnly) == kCVReturnSuccess else { return [] }
        defer { CVPixelBufferUnlockBaseAddress(image, .readOnly) }
        guard let yPlane = CVPixelBufferGetBaseAddressOfPlane(image, 0)?.assumingMemoryBound(to: UInt8.self),
              let uvPlane = CVPixelBufferGetBaseAddressOfPlane(image, 1)?.assumingMemoryBound(to: UInt8.self) else { return [] }
        let w = CVPixelBufferGetWidthOfPlane(image, 0), h = CVPixelBufferGetHeightOfPlane(image, 0)
        let ys = CVPixelBufferGetBytesPerRowOfPlane(image, 0), uvs = CVPixelBufferGetBytesPerRowOfPlane(image, 1)
        func byte(_ value: Float) -> Int { Int(min(255, max(0, value)).rounded()) }
        var result = [[Int]]()
        for y in stride(from: 1, to: height-2, by: max(1, height/16)) {
            for x in stride(from: 1, to: width-2, by: max(1, width/24)) {
                guard let d = depth(x: x, y: y, allowMedium: true),
                      let dx = depth(x: x+2, y: y, allowMedium: true),
                      let dy = depth(x: x, y: y+2, allowMedium: true),
                      max(abs(dx-d), abs(dy-d)) < 0.05 else { continue }
                let p = project(x: x, y: y, depth: d)
                let cross = simd_cross(project(x: x+2, y: y, depth: dx)-p, project(x: x, y: y+2, depth: dy)-p)
                guard simd_length(cross) > 1e-7, simd_length(p) < 300 else { continue }
                let n = simd_normalize(cross)
                let u = x*w/width, v = y*h/height, ci = (v/2)*uvs + (u/2)*2
                let l = Float(yPlane[v*ys+u]), cb = Float(uvPlane[ci])-128, cr = Float(uvPlane[ci+1])-128
                result.append([Int((100*p.x).rounded()), Int((100*p.y).rounded()), Int((100*p.z).rounded()),
                               Int((127*n.x).rounded()), Int((127*n.y).rounded()), Int((127*n.z).rounded()),
                               byte(l+1.402*cr), byte(l-0.344136*cb-0.714136*cr), byte(l+1.772*cb),
                               Int(min(16, max(5, d*Float(width)/intrinsics.columns.0.x*5)).rounded())])
            }
        }
        return result
    }

    func points(limit: Int = 1500, allowMedium: Bool = false) -> [[Float]] {
        var points = [[Float]]()
        for y in stride(from: height / 48, to: height, by: max(1, height / 24)) {
            for x in stride(from: width / 64, to: width, by: max(1, width / 32)) {
                guard points.count < limit, let d = depth(x: x, y: y, allowMedium: allowMedium) else { continue }
                let p = project(x: x, y: y, depth: d)
                points.append([p.x, p.y, p.z])
            }
        }
        return points
    }
}


// A single raster is used by both the phone and optional Pi viewer. UIKit draws
// into this private bitmap on the inference queue; no view is touched off-main.
private enum InspectionRenderer {
    static func heatColor(_ value: Float) -> (UInt8, UInt8, UInt8) {
        let q = max(0, min(1, (value - 0.2) / 3.8)) * 4
        let r = q <= 1 ? 1 : max(0, 2 - q)
        let g = q <= 1 ? q : min(1, 4 - q)
        let b = max(0, min(1, q - 2))
        return (UInt8(r * 255), UInt8(g * 255), UInt8(b * 255))
    }

    static func depthImage(_ depth: SceneDepthGrid, context: CIContext) -> UIImage? {
        var bytes = [UInt8](repeating: 0, count: depth.width * depth.height * 4)
        for y in 0..<depth.height {
            for x in 0..<depth.width {
                let i = (y * depth.width + x) * 4
                bytes[i + 3] = 255
                let index = y * depth.width + x
                let raw = depth.values[index]
                guard raw.isFinite && raw > 0 else { continue }
                guard let d = depth.depth(x: x, y: y, allowMedium: true) else {
                    if (0.2...4).contains(raw) { bytes[i] = 185; bytes[i+2] = 185 }
                    else { bytes[i] = 95; bytes[i+1] = 95; bytes[i+2] = 95 }
                    continue
                }
                let rgb = heatColor(d)
                let divisor: UInt8 = depth.confidence[index] == UInt8(ARConfidenceLevel.high.rawValue) ? 1 : 2
                bytes[i] = rgb.0/divisor; bytes[i+1] = rgb.1/divisor; bytes[i+2] = rgb.2/divisor
            }
        }
        guard let provider = CGDataProvider(data: Data(bytes) as CFData),
              let raw = CGImage(width: depth.width, height: depth.height, bitsPerComponent: 8,
                bitsPerPixel: 32, bytesPerRow: depth.width * 4, space: CGColorSpaceCreateDeviceRGB(),
                bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.last.rawValue), provider: provider,
                decode: nil, shouldInterpolate: false, intent: .defaultIntent) else { return nil }
        let portrait = CIImage(cgImage: raw).oriented(.right)
        guard let image = context.createCGImage(portrait, from: portrait.extent) else { return nil }
        return UIImage(cgImage: image)
    }

    static func render(frame: ARFrame, depth: SceneDepthGrid?, detections: [DetectedObject],
                       caption: String, context: CIContext) -> UIImage? {
        let portrait = CIImage(cvPixelBuffer: frame.capturedImage).oriented(.right)
        // Materialize only the pixels that can appear in the 320 px panel.
        let scale = min(1, 320 / portrait.extent.width)
        let resized = portrait.transformed(by: CGAffineTransform(scaleX: scale, y: scale))
        guard let rgb = context.createCGImage(resized, from: resized.extent) else { return nil }
        let depthView = depth.flatMap { depthImage($0, context: context) }
        return render(rgb: rgb, depthView: depthView, detections: detections, caption: caption)
    }

    static func render(rgb: CGImage, depthView: UIImage?, detections: [DetectedObject], caption: String) -> UIImage {
        let panelWidth: CGFloat = 320
        let panelHeight = panelWidth * CGFloat(rgb.height) / CGFloat(rgb.width)
        let gap: CGFloat = 12, margin: CGFloat = 12, top: CGFloat = 42, footer: CGFloat = 72
        let left = CGRect(x: margin, y: top, width: panelWidth, height: panelHeight)
        let right = left.offsetBy(dx: panelWidth + gap, dy: 0)
        let size = CGSize(width: margin * 2 + panelWidth * 2 + gap, height: top + panelHeight + footer)
        let format = UIGraphicsImageRendererFormat()
        format.scale = 1
        format.opaque = true
        let renderer = UIGraphicsImageRenderer(size: size, format: format)
        return renderer.image { output in
            let cg = output.cgContext
            UIColor(red: 0.035, green: 0.055, blue: 0.09, alpha: 1).setFill()
            output.fill(CGRect(origin: .zero, size: size))
            let font = UIFont.systemFont(ofSize: 22, weight: .semibold)
            let small = UIFont.systemFont(ofSize: 17, weight: .medium)
            func label(_ text: String, at point: CGPoint, font: UIFont, color: UIColor = .white) {
                (text as NSString).draw(at: point, withAttributes: [.font: font, .foregroundColor: color])
            }
            label("CAMERA + TARGETS", at: CGPoint(x: left.minX, y: 9), font: font)
            label("LiDAR DEPTH", at: CGPoint(x: right.minX, y: 9), font: font)
            UIImage(cgImage: rgb).draw(in: left)
            UIColor.black.setFill(); cg.fill(right)
            cg.interpolationQuality = .none
            depthView?.draw(in: right)
            if depthView == nil {
                label("Depth unavailable", at: CGPoint(x: right.minX + 12, y: right.midY), font: font)
            }
            cg.saveGState()
            cg.clip(to: left)
            for object in detections {
                // Vision is bottom-left; the bitmap renderer is top-left.
                let b = object.boundingBox
                let box = CGRect(x: left.minX + b.minX * left.width,
                                 y: left.minY + (1 - b.maxY) * left.height,
                                 width: b.width * left.width, height: b.height * left.height)
                let tint: UIColor = object.position == nil ? .systemYellow : .systemGreen
                cg.setStrokeColor(tint.cgColor); cg.setLineWidth(2.5); cg.stroke(box)
                let name = "\(object.label) \(Int(object.confidence * 100))%" + (object.position == nil ? " • no depth" : "")
                let paragraph = NSMutableParagraphStyle(); paragraph.lineBreakMode = .byTruncatingTail
                let textAttributes: [NSAttributedString.Key: Any] = [.font: small, .foregroundColor: UIColor.black,
                                                                    .paragraphStyle: paragraph]
                let width = min(left.width, (name as NSString).size(withAttributes: textAttributes).width + 10)
                let tag = CGRect(x: min(box.minX, left.maxX - width), y: max(left.minY, box.minY - 23), width: width, height: 23)
                tint.setFill(); cg.fill(tag)
                (name as NSString).draw(in: tag.insetBy(dx: 5, dy: 1), withAttributes: textAttributes)
            }
            cg.restoreGState()
            let legendY = left.maxY + 9
            for x in 0..<Int(right.width) {
                let rgb = heatColor(0.2 + Float(x) / Float(right.width - 1) * 3.8)
                cg.setFillColor(UIColor(red: CGFloat(rgb.0) / 255, green: CGFloat(rgb.1) / 255,
                                       blue: CGFloat(rgb.2) / 255, alpha: 1).cgColor)
                cg.fill(CGRect(x: right.minX + CGFloat(x), y: legendY, width: 1, height: 9))
            }
            label("0.2 m near", at: CGPoint(x: right.minX, y: legendY + 12), font: small)
            label("4 m far", at: CGPoint(x: right.maxX - 64, y: legendY + 12), font: small)
            label("Dim = medium · purple = rejected · gray = range", at: CGPoint(x: right.minX, y: legendY + 33), font: small)
            let paragraph = NSMutableParagraphStyle(); paragraph.lineBreakMode = .byWordWrapping
            (caption as NSString).draw(in: CGRect(x: left.minX, y: legendY, width: left.width, height: 54),
                withAttributes: [.font: small, .foregroundColor: UIColor.white, .paragraphStyle: paragraph])
        }
    }

#if DEBUG
    static func fixture() -> UIImage {
        let size = CGSize(width: 320, height: 426)
        let format = UIGraphicsImageRendererFormat()
        format.scale = 1; format.opaque = true
        let renderer = UIGraphicsImageRenderer(size: size, format: format)
        let boxes = [CGRect(x: 0.15, y: 0.45, width: 0.4, height: 0.4),
                     CGRect(x: 0.6, y: 0.08, width: 0.32, height: 0.24)]
        func rect(_ box: CGRect) -> CGRect {
            CGRect(x: box.minX * size.width, y: (1 - box.maxY) * size.height,
                   width: box.width * size.width, height: box.height * size.height)
        }
        let photo = renderer.image { output in
            UIColor.darkGray.setFill(); output.fill(CGRect(origin: .zero, size: size))
            output.cgContext.setStrokeColor(UIColor.gray.cgColor)
            for x in stride(from: 0, through: 320, by: 40) {
                output.cgContext.move(to: CGPoint(x: x, y: 0))
                output.cgContext.addLine(to: CGPoint(x: x, y: 426))
            }
            for y in stride(from: 0, through: 426, by: 40) {
                output.cgContext.move(to: CGPoint(x: 0, y: y))
                output.cgContext.addLine(to: CGPoint(x: 320, y: y))
            }
            output.cgContext.strokePath()
            UIColor.systemBlue.setFill(); output.fill(rect(boxes[0]).insetBy(dx: 3, dy: 3))
            UIColor.systemPurple.setFill(); output.fill(rect(boxes[1]).insetBy(dx: 3, dy: 3))
            ("SYNTHETIC QA" as NSString).draw(at: CGPoint(x: 8, y: 8),
                withAttributes: [.font: UIFont.boldSystemFont(ofSize: 24), .foregroundColor: UIColor.white])
        }
        let heatmap = renderer.image { output in
            for y in 0..<426 {
                let rgb = heatColor(0.2 + Float(y) / 425 * 3.8)
                UIColor(red: CGFloat(rgb.0) / 255, green: CGFloat(rgb.1) / 255,
                        blue: CGFloat(rgb.2) / 255, alpha: 1).setFill()
                output.fill(CGRect(x: 0, y: y, width: 320, height: 1))
            }
            UIColor.black.setFill(); output.fill(rect(boxes[1]))
        }
        let objects = [DetectedObject(label: "chair", confidence: 0.94, boundingBox: boxes[0], position: SIMD3(0, 0, -1)),
                       DetectedObject(label: "bottle", confidence: 0.81, boundingBox: boxes[1], position: nil)]
        return render(rgb: photo.cgImage!, depthView: heatmap, detections: objects,
                      caption: "DEBUG fixture • synthetic RGB/depth")
    }
#endif
}

// Runs only on the bounded depth-fusion queue. Relative predictions remain visible
// even when metric calibration is unavailable; they never become metric geometry.
private enum RGBDepthRenderer {
    static func render(rgb: CGImage, inverse: [Double],
                       calibration: RelativeDepthCalibration?, caption: String) -> UIImage? {
        guard inverse.count == 4096, inverse.allSatisfy({ $0.isFinite }) else { return nil }
        let sorted = inverse.sorted()
        let low = sorted[204], high = sorted[3891]
        let span = high - low
        var bytes = [UInt8](repeating: 0, count: 64 * 64 * 4)
        for row in 0..<64 { for column in 0..<64 {
            let i = row * 64 + column
            bytes[i * 4 + 3] = 255
            let fraction: Double
            if let calibration {
                // Diagnostic image shows model pixels with the fitted scale.
                // Measured LiDAR overrides inferred points only in the fused map.
                guard let meters = calibration.meters(inverse[i], maximum: 20) else { continue }
                fraction = (meters - 0.2) / 19.8
            } else if span > 1e-8 {
                // MiDaS outputs inverse depth: larger means relatively nearer.
                // Percentile normalization is per frame, with no metric scale.
                fraction = 1 - (inverse[i] - low) / span
            } else {
                bytes[i * 4] = 110; bytes[i * 4 + 1] = 110; bytes[i * 4 + 2] = 110
                continue
            }
            let rgb = InspectionRenderer.heatColor(0.2 + Float(max(0, min(1, fraction))) * 3.8)
            bytes[i * 4] = rgb.0; bytes[i * 4 + 1] = rgb.1; bytes[i * 4 + 2] = rgb.2
        } }
        guard let provider = CGDataProvider(data: Data(bytes) as CFData),
              let heat = CGImage(width: 64, height: 64, bitsPerComponent: 8, bitsPerPixel: 32,
                bytesPerRow: 64 * 4, space: CGColorSpaceCreateDeviceRGB(),
                bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.last.rawValue), provider: provider,
                decode: nil, shouldInterpolate: false, intent: .defaultIntent) else { return nil }
        let width: CGFloat = 320, margin: CGFloat = 12, gap: CGFloat = 12, top: CGFloat = 42
        let height = width * CGFloat(rgb.height) / CGFloat(rgb.width)
        let left = CGRect(x: margin, y: top, width: width, height: height)
        let right = left.offsetBy(dx: width + gap, dy: 0)
        let size = CGSize(width: 2 * width + 2 * margin + gap, height: top + height + 88)
        let format = UIGraphicsImageRendererFormat(); format.scale = 1; format.opaque = true
        return UIGraphicsImageRenderer(size: size, format: format).image { output in
            let cg = output.cgContext
            UIColor(red: 0.035, green: 0.055, blue: 0.09, alpha: 1).setFill()
            output.fill(CGRect(origin: .zero, size: size))
            let heading = UIFont.systemFont(ofSize: 20, weight: .semibold)
            let small = UIFont.systemFont(ofSize: 16, weight: .medium)
            func text(_ value: String, _ point: CGPoint, font: UIFont) {
                (value as NSString).draw(at: point, withAttributes: [.font: font, .foregroundColor: UIColor.white])
            }
            text("MATCHED RGB", CGPoint(x: left.minX, y: 10), font: heading)
            text(calibration == nil ? "RELATIVE DEPTH" : "ESTIMATED DEPTH (m)", CGPoint(x: right.minX, y: 10), font: heading)
            UIImage(cgImage: rgb).draw(in: left)
            cg.interpolationQuality = .none
            UIImage(cgImage: heat).draw(in: right)
            let legendY = right.maxY + 9
            for column in 0..<320 {
                let color = InspectionRenderer.heatColor(0.2 + Float(column) / 319 * 3.8)
                cg.setFillColor(UIColor(red: CGFloat(color.0) / 255, green: CGFloat(color.1) / 255,
                    blue: CGFloat(color.2) / 255, alpha: 1).cgColor)
                cg.fill(CGRect(x: right.minX + CGFloat(column), y: legendY, width: 1, height: 9))
            }
            text(calibration == nil ? "Relatively near" : "0.2 m near", CGPoint(x: right.minX, y: legendY + 12), font: small)
            text(calibration == nil ? "Far" : "20 m far", CGPoint(x: right.maxX - 58, y: legendY + 12), font: small)
            text(calibration == nil ? "Per-frame colors · no meter scale" : "LiDAR-calibrated scale · model pixels", CGPoint(x: right.minX, y: legendY + 33), font: small)
            let paragraph = NSMutableParagraphStyle(); paragraph.lineBreakMode = .byWordWrapping
            (caption as NSString).draw(in: CGRect(x: left.minX, y: legendY, width: left.width, height: 72),
                withAttributes: [.font: small, .foregroundColor: UIColor.white, .paragraphStyle: paragraph])
        }
    }
}

// Calibration stays independent of ARKit and UI so its acceptance rules can be
// exercised with deterministic data. Inputs are (relative inverse depth, meters).
struct RelativeDepthCalibration {
    let scale: Double
    let offset: Double
    let medianError: Double
    let p90Error: Double
    let samples: Int

    struct Failure: LocalizedError {
        let reason: String
        var errorDescription: String? { reason }
    }

    func meters(_ relative: Double, maximum: Double = 8) -> Double? {
        let inverse = scale * relative + offset
        guard inverse.isFinite, inverse > 0 else { return nil }
        let value = 1 / inverse
        return maximum.isFinite && maximum <= 20 && value >= 0.2 && value <= maximum ? value : nil
    }

    private static func quantile(_ values: [Double], _ fraction: Double) -> Double {
        let sorted = values.sorted()
        return sorted[min(sorted.count - 1, Int(Double(sorted.count - 1) * fraction))]
    }

    static func fit(_ pairs: [(Double, Double)], maximumAnchor: Double = 4) throws -> RelativeDepthCalibration {
        let pairs = pairs.filter { $0.0.isFinite && $0.1.isFinite && $0.1 >= 0.2 && $0.1 <= min(20, maximumAnchor) }
        guard pairs.count >= 40 else { throw Failure(reason: "Need at least 40 reliable LiDAR/model pairs") }
        let distances = pairs.map { $0.1 }, relatives = pairs.map { $0.0 }
        guard quantile(distances, 0.9) - quantile(distances, 0.1) >= 0.35,
              quantile(relatives, 0.9) - quantile(relatives, 0.1) > 0.00001 else {
            throw Failure(reason: "Depth variation too small to calibrate")
        }
        let heldout = pairs.enumerated().filter { $0.offset % 5 == 0 }.map { $0.element }
        let training = pairs.enumerated().filter { $0.offset % 5 != 0 }.map { $0.element }
        guard training.count >= 32, heldout.count >= 8 else { throw Failure(reason: "Insufficient independent validation samples") }
        func regression(_ values: [(Double, Double)]) throws -> (Double, Double) {
            let n = Double(values.count)
            let mx = values.reduce(0) { $0 + $1.0 } / n
            let my = values.reduce(0) { $0 + 1 / $1.1 } / n
            let variance = values.reduce(0) { $0 + pow($1.0 - mx, 2) }
            let covariance = values.reduce(0) { $0 + ($1.0 - mx) * (1 / $1.1 - my) }
            guard variance > 1e-12 else { throw Failure(reason: "Degenerate inverse-depth predictions") }
            let a = covariance / variance, b = my - a * mx
            guard a.isFinite, b.isFinite, a > 0 else { throw Failure(reason: "Inverse-depth scale is not positive") }
            return (a, b)
        }
        var selected = training
        var fit = try regression(selected)
        for _ in 0..<4 {
            let residuals = training.map { 1 / $0.1 - (fit.0 * $0.0 + fit.1) }
            let center = quantile(residuals, 0.5)
            let mad = quantile(residuals.map { abs($0 - center) }, 0.5)
            let limit = max(0.02, 3 * 1.4826 * mad)
            selected = zip(training, residuals).filter { abs($0.1 - center) <= limit }.map { $0.0 }
            guard selected.count >= 24, selected.count * 2 >= training.count else {
                throw Failure(reason: "LiDAR and model predictions disagree")
            }
            fit = try regression(selected)
        }
        var errors = [Double]()
        for (relative, measured) in heldout {
            let inverse = fit.0 * relative + fit.1
            guard inverse.isFinite, inverse > 0 else { throw Failure(reason: "Held-out depth is invalid") }
            let predicted = 1 / inverse
            guard (0.2...max(8, min(20, maximumAnchor))).contains(predicted) else { throw Failure(reason: "Held-out depth is outside accepted range") }
            errors.append(abs(predicted - measured))
        }
        let median = quantile(errors, 0.5), p90 = quantile(errors, 0.9)
        guard median <= 0.15, p90 <= 0.25 else {
            throw Failure(reason: String(format: "Calibration rejected: held-out error %.2f m", p90))
        }
        return RelativeDepthCalibration(scale: fit.0, offset: fit.1, medianError: median,
                                        p90Error: p90, samples: pairs.count)
    }
}

// BEGIN REMEMBERED_DEPTH_MATH — Foundation + simd only; exercised by test_depth_geometry.py.
struct DepthCameraGeometry {
    let intrinsics: simd_float3x3
    let transform: simd_float4x4
    let resolution: SIMD2<Float>

    struct Projection {
        let u: Double
        let v: Double
        let meters: Double
    }

    func project(_ world: SIMD3<Float>) -> Projection? {
        let local = transform.inverse * SIMD4<Float>(world, 1)
        let meters = -local.z
        guard meters.isFinite, meters >= 0.2, meters <= 20,
              resolution.x > 0, resolution.y > 0 else { return nil }
        let x = intrinsics.columns.0.x * local.x / meters + intrinsics.columns.2.x
        let y = intrinsics.columns.2.y - intrinsics.columns.1.y * local.y / meters
        let u = 1 - Double(y / resolution.y), v = Double(x / resolution.x)
        guard u.isFinite, v.isFinite, u >= 0.04, u <= 0.96, v >= 0.04, v <= 0.96 else { return nil }
        return Projection(u: u, v: v, meters: Double(meters))
    }

    func world(u: Double, v: Double, meters: Double) -> SIMD3<Float>? {
        guard u.isFinite, v.isFinite, (0...1).contains(u), (0...1).contains(v),
              meters.isFinite, (0.2...20).contains(meters),
              intrinsics.columns.0.x > 0, intrinsics.columns.1.y > 0 else { return nil }
        let x = Float(v) * resolution.x, y = Float(1-u) * resolution.y, d = Float(meters)
        let p = transform * SIMD4<Float>((x-intrinsics.columns.2.x)*d/intrinsics.columns.0.x,
            -(y-intrinsics.columns.2.y)*d/intrinsics.columns.1.y, -d, 1)
        guard p.x.isFinite, p.y.isFinite, p.z.isFinite else { return nil }
        return SIMD3<Float>(p.x, p.y, p.z)
    }

    func motion(from other: DepthCameraGeometry) -> (translation: Double, degrees: Double) {
        let a = SIMD3<Float>(transform.columns.3.x, transform.columns.3.y, transform.columns.3.z)
        let b = SIMD3<Float>(other.transform.columns.3.x, other.transform.columns.3.y, other.transform.columns.3.z)
        let rotation = simd_quatf(transform), previous = simd_quatf(other.transform)
        let cosine = min(1, max(0, abs(simd_dot(rotation.vector, previous.vector))))
        return (Double(simd_distance(a, b)), Double(2 * acos(cosine)) * 180 / .pi)
    }
}

struct DepthRGBTexture {
    // Top-left, portrait-oriented 64x64 RGB, matching the model's input layout.
    let pixels: [Float]

    func patch(u: Double, v: Double) -> [Float]? {
        guard pixels.count == 64*64*3, u.isFinite, v.isFinite else { return nil }
        let x = Int((u * 63).rounded()), y = Int((v * 63).rounded())
        guard x >= 2, x <= 61, y >= 2, y <= 61 else { return nil }
        var result = [Float]()
        for dy in [-2, 0, 2] { for dx in [-2, 0, 2] {
            let offset = ((y+dy)*64+x+dx)*3
            result.append(contentsOf: pixels[offset..<offset+3])
        } }
        return result
    }

    static func agrees(_ first: [Float], _ second: [Float]) -> Bool {
        guard first.count == 27, second.count == 27 else { return false }
        func means(_ values: [Float]) -> [Float] {
            (0..<3).map { c in stride(from: c, to: 27, by: 3).reduce(Float(0)) { $0+values[$1] } / 9 }
        }
        let a = means(first), b = means(second)
        let texture = (0..<27).reduce(Float(0)) { $0 + pow(first[$1]-a[$1%3], 2) } / 27
        let normalizedError = (0..<27).reduce(Float(0)) { $0 + pow((first[$1]-a[$1%3])-(second[$1]-b[$1%3]), 2) } / 27
        let colorError = zip(a,b).reduce(Float(0)) { $0 + abs($1.0-$1.1) } / 3
        // Reject untextured patches: matching a flat wall's color is not correspondence.
        return texture >= 0.0009 && normalizedError <= 0.0036 && colorError <= 0.12
    }

    func similar(to previous: DepthRGBTexture) -> Bool {
        guard pixels.count == 64*64*3, previous.pixels.count == pixels.count else { return false }
        let errors = zip(pixels, previous.pixels).map { abs($0-$1) }.sorted()
        return errors.reduce(0,+)/Float(errors.count) <= 0.07 && errors[errors.count*9/10] <= 0.16
    }
}

struct RememberedDepthMath {
    static func sample(_ values: [Double], u: Double, v: Double) -> Double {
        let x = max(0, min(63, u*64-0.5)), y = max(0, min(63, v*64-0.5))
        let x0 = Int(x), y0 = Int(y), x1 = min(63,x0+1), y1 = min(63,y0+1)
        let fx = x-Double(x0), fy = y-Double(y0)
        let a = values[y0*64+x0]*(1-fx)+values[y0*64+x1]*fx
        let b = values[y1*64+x0]*(1-fx)+values[y1*64+x1]*fx
        return a*(1-fy)+b*fy
    }

    static func unoccluded(anchorMeters: Double, measuredMeters: Double?) -> Bool {
        guard anchorMeters.isFinite, (0.2...20).contains(anchorMeters) else { return false }
        guard let measuredMeters else { return true }
        // Reject both nearer occluders and a former surface that has moved away.
        return measuredMeters.isFinite && abs(measuredMeters-anchorMeters) <= 0.2
    }

    static func predictionsAgree(_ current: [Double], _ previous: [Double]) -> Bool {
        guard current.count == 4096, previous.count == 4096,
              current.allSatisfy({ $0.isFinite }), previous.allSatisfy({ $0.isFinite }) else { return false }
        let a = current.sorted(), b = previous.sorted()
        let span = b[3686]-b[409]
        guard span > 1e-5 else { return false }
        for i in [409,2048,3686] { if abs(a[i]-b[i]) > span*0.1 { return false } }
        let errors = zip(current,previous).map { abs($0-$1)/span }.sorted()
        return errors[2048] <= 0.06 && errors[3686] <= 0.15
    }

    static func cacheEligible(age: Double, translation: Double, degrees: Double) -> Bool {
        age.isFinite && translation.isFinite && degrees.isFinite && (0...3).contains(age)
            && (0...0.12).contains(translation) && (0...6).contains(degrees)
    }
}
// END REMEMBERED_DEPTH_MATH

private extension DepthRGBTexture {
    static func make(_ image: CGImage) -> DepthRGBTexture? {
        // UIKit establishes top-left portrait pixels; then explicit RGBA byte order.
        let format = UIGraphicsImageRendererFormat(); format.scale = 1; format.opaque = true
        let small = UIGraphicsImageRenderer(size: CGSize(width: 64,height: 64),format: format).image { _ in
            UIImage(cgImage: image).draw(in: CGRect(x: 0,y: 0,width: 64,height: 64))
        }
        guard let cgImage = small.cgImage else { return nil }
        var bytes = [UInt8](repeating: 0,count: 64*64*4)
        let okay = bytes.withUnsafeMutableBytes { storage -> Bool in
            guard let context = CGContext(data: storage.baseAddress,width: 64,height: 64,bitsPerComponent: 8,
                bytesPerRow: 64*4,space: CGColorSpaceCreateDeviceRGB(),
                bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue | CGBitmapInfo.byteOrder32Big.rawValue) else { return false }
            // CGImage scanlines already match top-left image coordinates; do not flip them.
            context.draw(cgImage,in: CGRect(x: 0,y: 0,width: 64,height: 64))
            return true
        }
        guard okay else { return nil }
        var pixels = [Float](); pixels.reserveCapacity(64*64*3)
        for i in 0..<4096 { for channel in 0..<3 { pixels.append(Float(bytes[i*4+channel])/255) } }
        return DepthRGBTexture(pixels: pixels)
    }
}

final class CameraServer: NSObject, ObservableObject, ARSessionDelegate {
    let session = ARSession()
    @Published var status = "Rear camera stopped"
    @Published var detections: [DetectedObject] = []
    @Published var inspectionImage: UIImage?
    @Published var inspectionTimestamp: TimeInterval = 0
    @Published var inspectionStatus = "Waiting for camera and LiDAR"
    @Published var inspectionPerformance = "Measuring local preview rate"
    // These counters are owned by the main queue and count image publications,
    // not requested sensor rates or JPEG acknowledgements.
    private var performanceStarted: TimeInterval = 0
    private var performanceFrames = 0
    private var performanceProcessing: TimeInterval = 0
    @Published var qnxInspectionImage: UIImage?
    @Published var qnxInspectionTimestamp: TimeInterval = 0
    @Published var qnxStatusString = "Waiting for QNX detections"
    @Published var depthStatusString = "Waiting for QNX relative depth"
    @Published var qnxDepthImage: UIImage?
    @Published var qnxDepthTimestamp: TimeInterval = 0
    @Published var qnxDepthCalibrated = false
    @Published var qnxDepthModel = "Waiting for MiDaS"
    @Published var qnxDepthInferenceMS: Double?
    @Published var qnxDepthFitSummary = "Scan nearby surfaces at varied distances to seed depth anchors, then look farther."
    @Published var qnxDepthMode = "relative"
    @Published var qnxDepthConfidence: Double = 0
    @Published var qnxDepthAnchorAge: Double?
    private let queue = DispatchQueue(label: "MotionUSB.scene.capture", qos: .userInteractive)
    private let previewQueue = DispatchQueue(label: "MotionUSB.preview", qos: .userInitiated)
    private let sceneQueue = DispatchQueue(label: "MotionUSB.scene.mapping", qos: .userInitiated)
    private let safetyQueue = DispatchQueue(label: "MotionUSB.lidar.guard", qos: .userInteractive)
    private let rgbQueue = DispatchQueue(label: "MotionUSB.rgb.inference", qos: .utility)
    private let resultQueue = DispatchQueue(label: "MotionUSB.qnx.results", qos: .utility)
    private let fusionQueue = DispatchQueue(label: "MotionUSB.depth.fusion", qos: .utility)
    private let previewContext = CIContext()
    private let rgbContext = CIContext()
    private var nextAnchorRGB = -Double.infinity // Owned only by rgbQueue; caps anchor extraction at 5 Hz.
    private lazy var video = ScenePacketStream(queue: previewQueue, limit: 2_000_000)
    private lazy var scene = ScenePacketStream(queue: sceneQueue, limit: 512_000)
    private lazy var safety = ScenePacketStream(queue: safetyQueue, limit: 12_332, acknowledgementTimeout: 0.15)
    private lazy var rgb = ScenePacketStream(queue: rgbQueue, limit: 2_000_012, acknowledgementTimeout: 5)
    private var wanted = false
    private var epoch = UUID()
    private let generationLock = NSLock()
    private var activeGeneration: UUID?
    private var sessionID = UUID().uuidString
    private var floorY: Float?
    private var explicitFloor = false
    private var latestFrame: ARFrame?
    private var previewBusy = false
    private var safetyBusy = false
    private var sceneBusy = false
    private var rgbBusy = false
    private var resultBusy = false
    private var fusionBusy = false
    private var nextPreview = -Double.infinity
    private var nextSafety = -Double.infinity
    private var nextScene = -Double.infinity
    private var safetySequence: UInt32 = 0
    private var lastTracking = ""
    private var loadedMap: ARWorldMap?
    private var relocalizing = false
    private var awaitingFirstFrame = false
    private var runRequestedAt = -Double.infinity
    private struct RGBSource {
        let frameID: UInt64
        let timestamp: TimeInterval
        let epoch: UUID
        let image: CGImage
        let depth: SceneDepthGrid?
        let tracking: String
        let geometry: DepthCameraGeometry
        let texture: DepthRGBTexture?
        let session: String
    }
    private struct AnchorCell: Hashable { let x: Int; let y: Int; let z: Int }
    private struct MeasuredAnchor {
        let position: SIMD3<Float>
        let timestamp: TimeInterval
        let session: String
        let epoch: UUID
        let observer: DepthCameraGeometry
        let patch: [Float]
    }
    private struct CachedDepthFit {
        let fit: RelativeDepthCalibration
        let source: RGBSource
        let inverse: [Double]
        let mode: String
        let anchorTimestamp: TimeInterval
        let confidence: Double
    }
    private var measuredAnchors: [AnchorCell: MeasuredAnchor] = [:]
    private var cachedDepthFit: CachedDepthFit?
    private var persistentPoints: [[Float]] = []
    private var persistentCalibration: [String: Any]?
    private var persistentTimestamp = -Double.infinity
    private var sources: [UInt64: RGBSource] = [:]
    private var sourceOrder: [UInt64] = []
    private var lastQNXFrame: UInt64 = 0
    private var locatedObjects: [[String: Any]] = []
    private var locatedTimestamp = -Double.infinity
    private var estimatedPoints: [[Float]] = []
    private var estimatedTimestamp = -Double.infinity
    private var depthCalibration: [String: Any]?
    private var lastDepthFrame: UInt64 = 0

    override init() {
        super.init()
        session.delegateQueue = queue
        session.delegate = self
        // Vision/Core ML intentionally does not load here. QNX owns inference.
    }

    private func advanceGeneration() {
        epoch = UUID()
        generationLock.lock()
        activeGeneration = wanted ? epoch : nil
        generationLock.unlock()
        sources.removeAll(); sourceOrder.removeAll(); lastQNXFrame = 0
        locatedObjects.removeAll(); locatedTimestamp = -.infinity
        estimatedPoints.removeAll(); estimatedTimestamp = -.infinity
        depthCalibration = nil; lastDepthFrame = 0
        measuredAnchors.removeAll(); cachedDepthFit = nil
        persistentPoints.removeAll(); persistentCalibration = nil; persistentTimestamp = -.infinity
    }

    private func current(_ token: UUID) -> Bool {
        generationLock.lock()
        defer { generationLock.unlock() }
        return activeGeneration == token
    }

    private func nextDeadline(_ previous: Double, at timestamp: Double, interval: Double) -> Double {
        guard previous.isFinite else { return timestamp + interval }
        return previous + (floor(max(0, timestamp - previous) / interval) + 1) * interval
    }

#if DEBUG
    // Explicit launch-argument visual QA only. Never starts real capture/streams.
    func loadInspectionFixture() {
        inspectionImage = InspectionRenderer.fixture()
        inspectionTimestamp = ProcessInfo.processInfo.systemUptime
        inspectionStatus = "DEBUG fixture — synthetic inputs; sensors are off"
        status = inspectionStatus
        let args = ProcessInfo.processInfo.arguments
        let fixtureMode = args.contains("--depth-cached") ? "cached" : (args.contains("--depth-anchors") ? "anchors" : "direct")
        let calibrated = args.contains("--depth-metric") || fixtureMode != "direct"
        let size = CGSize(width: 320, height: 426)
        let format = UIGraphicsImageRendererFormat(); format.scale = 1; format.opaque = true
        let source = UIGraphicsImageRenderer(size: size, format: format).image { context in
            UIColor.darkGray.setFill(); context.fill(CGRect(origin: .zero, size: size))
            for column in 0..<8 {
                UIColor(hue: CGFloat(column) / 10, saturation: 0.4, brightness: 0.6, alpha: 1).setFill()
                context.fill(CGRect(x: column * 40, y: 0, width: 40, height: 426))
            }
            ("SYNTHETIC RGB" as NSString).draw(at: CGPoint(x: 12, y: 20),
                withAttributes: [.font: UIFont.boldSystemFont(ofSize: 23), .foregroundColor: UIColor.white])
        }
        let inverse: [Double] = (0..<4096).map { i -> Double in
            let meters = 0.4 + Double(i / 64) / 63.0 * 5.5
            return (1.0 / meters - 0.08) / 1.7
        }
        let fit = calibrated ? RelativeDepthCalibration(scale: 1.7, offset: 0.08, medianError: 0.04, p90Error: 0.11, samples: 320) : nil
        qnxDepthImage = RGBDepthRenderer.render(rgb: source.cgImage!, inverse: inverse,
            calibration: fit, caption: "DEBUG synthetic depth. Not a live measurement.")
        qnxDepthTimestamp = ProcessInfo.processInfo.systemUptime - (args.contains("--depth-stale") ? 8 : 0.9)
        qnxDepthCalibrated = calibrated; qnxDepthModel = "MiDaS small · synthetic fixture"
        qnxDepthMode = calibrated ? fixtureMode : "relative"
        qnxDepthConfidence = calibrated ? (fixtureMode == "direct" ? 0.7 : 0.3) : 0
        qnxDepthAnchorAge = calibrated ? (fixtureMode == "direct" ? 0 : 2.0) : nil
        qnxDepthInferenceMS = 820
        qnxDepthFitSummary = calibrated ? "\(fixtureMode): 320 pairs · p90 0.11 m · heuristic confidence \(Int(qnxDepthConfidence*100))% (synthetic)"
            : "No source-matched LiDAR; meter scale unavailable (synthetic)"
        depthStatusString = calibrated ? "LiDAR-calibrated depth · synthetic inferred gap points"
            : "Relative depth only · no metric points accepted"
    }
#endif

    private func report(_ message: String) { DispatchQueue.main.async { self.status = message } }
    private func reportState() {
        guard wanted else { return }
        let lidar = ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) ? "LiDAR" : "LiDAR unavailable"
        report("Rear RGB + \(lidar) • \(lastTracking.isEmpty ? "initializing" : lastTracking) • AI on QNX")
    }

    func start() {
        queue.async {
            guard !self.wanted else { return }
            self.wanted = true
            self.advanceGeneration()
            let token = self.epoch
            AVCaptureDevice.requestAccess(for: .video) { granted in
                self.queue.async {
                    guard self.wanted, self.epoch == token else { return }
                    guard granted else { self.wanted = false; self.report("Camera permission denied"); return }
                    self.begin()
                }
            }
        }
    }

    private func begin() {
        guard ARWorldTrackingConfiguration.isSupported else {
            wanted = false; report("AR world tracking unsupported"); return
        }
        do {
            try previewQueue.sync { try video.start(port: 5558) { [weak self] in self?.report($0) } }
            try sceneQueue.sync { try scene.start(port: 5561) { [weak self] in self?.report($0) } }
            try safetyQueue.sync { try safety.start(port: 5562) { [weak self] in self?.report($0) } }
            try rgbQueue.sync { try rgb.start(port: 5563) { [weak self] in self?.report($0) } }
            if loadedMap == nil { sessionID = UUID().uuidString; floorY = nil; explicitFloor = false }
            runSession(map: loadedMap)
            loadedMap = nil
        } catch { end(); report("Camera listener: \(error.localizedDescription)") }
    }

    private func runSession(map: ARWorldMap?) {
        advanceGeneration()
        latestFrame = nil
        nextScene = -.infinity; nextPreview = -.infinity; nextSafety = -.infinity
        lastTracking = ""
        awaitingFirstFrame = true
        relocalizing = map != nil
        clearInspection("Waiting for paired frames")
        let config = ARWorldTrackingConfiguration()
        config.worldAlignment = .gravity
        config.planeDetection = [.horizontal]
        config.initialWorldMap = map
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) { config.frameSemantics.insert(.sceneDepth) }
        // Prefer 30 Hz acquisition; source-aligned inspection images are capped at 10 Hz.
        if let format = ARWorldTrackingConfiguration.supportedVideoFormats
            .filter({ $0.framesPerSecond == 30 })
            .min(by: { $0.imageResolution.width < $1.imageResolution.width }) { config.videoFormat = format }
        runRequestedAt = ProcessInfo.processInfo.systemUptime
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
        reportState()
    }

    func stop() { queue.async { self.end(); self.report("Rear camera stopped") } }
    private func end() {
        wanted = false; advanceGeneration(); latestFrame = nil
        session.pause()
        previewQueue.sync { video.stop() }
        sceneQueue.sync { scene.stop() }
        safetyQueue.sync { safety.stop() }
        rgbQueue.sync { rgb.stop() }
        clearInspection("Camera stopped")
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard wanted, frame.timestamp >= runRequestedAt else { return }
        latestFrame = frame
        // The first post-run callback is never accepted as relocalized data.
        let first = awaitingFirstFrame
        awaitingFirstFrame = false
        let tracking: String
        switch frame.camera.trackingState {
        case .normal:
            if relocalizing && !first { relocalizing = false }
            tracking = (relocalizing || first) ? "limited" : "normal"
        case .limited: tracking = "limited"
        case .notAvailable: tracking = "unavailable"
        }
        if tracking != lastTracking {
            if !lastTracking.isEmpty { advanceGeneration() }
            lastTracking = tracking
            if tracking != "normal" { clearInspection("Tracking \(tracking); target locations paused") }
            reportState()
        }
        let token = epoch, id = sessionID, floor = floorY
        // This delegate only schedules bounded work. The guard is dispatched
        // first and owns its own queue, stream, depth copy and ACK deadline.
        if !safetyBusy, frame.timestamp >= nextSafety {
            nextSafety = nextDeadline(nextSafety, at: frame.timestamp, interval: 0.05)
            safetySequence &+= 1
            publishSafety(frame: frame, tracking: tracking, floor: floor, sequence: safetySequence, token: token)
        }
        if !sceneBusy, frame.timestamp >= nextScene {
            nextScene = nextDeadline(nextScene, at: frame.timestamp, interval: 0.2)
            let objects = tracking == "normal" && frame.timestamp - locatedTimestamp <= 1.5 ? locatedObjects : []
            let estimatesFresh = tracking == "normal" && frame.timestamp - estimatedTimestamp <= 2
            let persistentFresh = tracking == "normal" && frame.timestamp - persistentTimestamp <= 2
            publishScene(frame: frame, tracking: tracking, floor: floor, id: id, objects: objects,
                         estimated: estimatesFresh ? estimatedPoints : [],
                         calibration: estimatesFresh ? depthCalibration : nil,
                         persistent: persistentFresh ? persistentPoints : [],
                         persistentFit: persistentFresh ? persistentCalibration : nil, token: token)
        }
        if !previewBusy, frame.timestamp >= nextPreview {
            nextPreview = nextDeadline(nextPreview, at: frame.timestamp, interval: 0.1)
            publishPreview(frame: frame, tracking: tracking, token: token)
        }
        // No fixed AI-frame timer: submit the newest camera frame whenever the
        // encoder is free. The stream still permits only one unacknowledged
        // packet, and QNX retains only the latest frame for each AI consumer.
        if !rgbBusy {
            publishRGB(frame: frame, tracking: tracking, id: id, token: token)
        }
    }

    private func clearInspection(_ message: String) {
        DispatchQueue.main.async {
            self.detections = []
            self.inspectionImage = nil
            self.inspectionTimestamp = 0
            self.inspectionStatus = message
            self.qnxInspectionImage = nil
            self.qnxInspectionTimestamp = 0
            self.qnxStatusString = "Waiting for QNX detections"
            self.depthStatusString = "Waiting for QNX relative depth"
            self.qnxDepthImage = nil; self.qnxDepthTimestamp = 0
            self.qnxDepthCalibrated = false; self.qnxDepthModel = "Waiting for MiDaS"
            self.qnxDepthInferenceMS = nil
            self.qnxDepthMode = "relative"; self.qnxDepthConfidence = 0; self.qnxDepthAnchorAge = nil
            self.qnxDepthFitSummary = "Scan nearby surfaces at varied distances to seed depth anchors, then look farther."
            self.inspectionPerformance = "Measuring local preview rate"
            self.performanceStarted = ProcessInfo.processInfo.systemUptime
            self.performanceFrames = 0
            self.performanceProcessing = 0
        }
    }

    private func recordInspection(processing: TimeInterval) {
        let now = ProcessInfo.processInfo.systemUptime
        if performanceStarted == 0 { performanceStarted = now }
        performanceFrames += 1
        performanceProcessing += processing
        let elapsed = now - performanceStarted
        guard elapsed >= 1 else { return }
        inspectionPerformance = String(format: "Local %.1f fps • %.0f ms processing",
            Double(performanceFrames) / elapsed, performanceProcessing / Double(performanceFrames) * 1000)
        performanceStarted = now
        performanceFrames = 0
        performanceProcessing = 0
    }

    private static func appendBE<T: FixedWidthInteger>(_ value: T, to data: inout Data) {
        var bytes = value.bigEndian
        withUnsafeBytes(of: &bytes) { data.append(contentsOf: $0) }
    }

    private static func appendFloat(_ value: Float, to data: inout Data) { appendBE(value.bitPattern, to: &data) }

    private func publishSafety(frame: ARFrame, tracking: String, floor: Float?, sequence: UInt32, token: UUID) {
        safetyBusy = true
        safetyQueue.async {
            defer { self.queue.async { self.safetyBusy = false } }
            guard self.current(token), self.safety.available else { return }
            let t = frame.camera.transform
            let horizontal = hypot(t.columns.2.x, t.columns.2.z)
            let normal = tracking == "normal"
            let flags: UInt32 = normal ? (1 | (floor == nil ? 0 : 2) | (horizontal > 0.2 ? 4 : 0)) : 0
            let points = normal ? (SceneDepthGrid(frame)?.points(limit: 1024, allowMedium: true) ?? []) : []
            var data = Data("LDR1".utf8)
            Self.appendBE(sequence, to: &data)
            Self.appendBE(UInt64(max(0, frame.timestamp) * 1_000_000), to: &data)
            Self.appendBE(flags, to: &data)
            Self.appendBE(UInt32(points.count), to: &data)
            for value in [t.columns.3.x, t.columns.3.y, t.columns.3.z,
                          atan2(-t.columns.2.x, -t.columns.2.z), floor ?? 0] { Self.appendFloat(value, to: &data) }
            for point in points { for value in point { Self.appendFloat(value, to: &data) } }
            if self.current(token) { self.safety.send(data) }
        }
    }

    private func publishScene(frame: ARFrame, tracking: String, floor: Float?, id: String,
                              objects: [[String: Any]], estimated: [[Float]],
                              calibration: [String: Any]?, persistent: [[Float]], persistentFit: [String: Any]?, token: UUID) {
        sceneBusy = true
        sceneQueue.async {
            defer { self.queue.async { self.sceneBusy = false } }
            guard self.current(token), self.scene.available else { return }
            let depth = tracking == "normal" ? SceneDepthGrid(frame) : nil
            let t = frame.camera.transform
            var outgoingCalibration = calibration ?? [:]
            if let observed = outgoingCalibration["observation_timestamp"] as? Double {
                outgoingCalibration["source_age_s"] = max(0, frame.timestamp - observed)
            }
            let packet: [String: Any] = ["version": 1, "session": id, "timestamp": frame.timestamp,
                "tracking": tracking,
                "pose": [t.columns.3.x, t.columns.3.y, t.columns.3.z, atan2(-t.columns.2.x, -t.columns.2.z)],
                "floor_y": floor.map { $0 as Any } ?? NSNull(), "points": depth?.points(allowMedium: true) ?? [], "objects": objects,
                "surfaces": depth?.surfaceSamples(frame.capturedImage) ?? [],
                "depth_size": depth.map { [$0.width, $0.height] } ?? [],
                "depth_intrinsics": depth.map { [$0.intrinsics.columns.0.x, $0.intrinsics.columns.1.y,
                                                $0.intrinsics.columns.2.x, $0.intrinsics.columns.2.y] } ?? [],
                "camera_transform_columns": (0..<4).map { [t[$0].x, t[$0].y, t[$0].z, t[$0].w] },
                "image_orientation": "right", "depth_range_m": [0.2, 4.0],
                "estimated_points": estimated, "depth_calibration": outgoingCalibration,
                "persistent_points": persistent, "persistent_calibration": persistentFit ?? [:]]
            guard self.current(token), let data = try? JSONSerialization.data(withJSONObject: packet) else { return }
            self.scene.send(data)
        }
    }

    private func publishPreview(frame: ARFrame, tracking: String, token: UUID) {
        previewBusy = true
        let started = ProcessInfo.processInfo.systemUptime
        previewQueue.async {
            guard self.current(token) else { self.queue.async { self.previewBusy = false }; return }
            let depth = SceneDepthGrid(frame)
            let caption = "Live RGB + measured LiDAR • AI on QNX"
            let rendered = InspectionRenderer.render(frame: frame, depth: depth, detections: [],
                                                       caption: caption, context: self.previewContext)
            let jpeg = self.video.available ? rendered?.jpegData(compressionQuality: 0.7) : nil
            let processing = ProcessInfo.processInfo.systemUptime - started
            if self.current(token), let jpeg { self.video.send(jpeg) }
            self.queue.async {
                guard self.current(token), let rendered,
                      ProcessInfo.processInfo.systemUptime - frame.timestamp < 0.75 else {
                    self.previewBusy = false; return
                }
                DispatchQueue.main.async {
                    self.inspectionImage = rendered
                    self.inspectionTimestamp = frame.timestamp
                    self.inspectionStatus = caption
                    self.recordInspection(processing: processing)
                    self.queue.async { self.previewBusy = false }
                }
            }
        }
    }

    private func publishRGB(frame: ARFrame, tracking: String, id: String, token: UUID) {
        rgbBusy = true
        rgbQueue.async {
            defer { self.queue.async { self.rgbBusy = false } }
            guard self.current(token), self.rgb.available else { return }
            let portrait = CIImage(cvPixelBuffer: frame.capturedImage).oriented(.right)
            let scale = min(1, 480 / portrait.extent.width)
            let reduced = portrait.transformed(by: CGAffineTransform(scaleX: scale, y: scale))
            guard let image = self.rgbContext.createCGImage(reduced, from: reduced.extent),
                  let jpeg = UIImage(cgImage: image).jpegData(compressionQuality: 0.7), jpeg.count <= 2_000_000 else { return }
            let frameID = UInt64(max(0, frame.timestamp) * 1_000_000)
            let source = RGBSource(frameID: frameID, timestamp: frame.timestamp, epoch: token, image: image,
                                   depth: tracking == "normal" ? SceneDepthGrid(frame) : nil, tracking: tracking,
                                   geometry: DepthCameraGeometry(intrinsics: frame.camera.intrinsics,
                                     transform: frame.camera.transform,
                                     resolution: SIMD2(Float(frame.camera.imageResolution.width), Float(frame.camera.imageResolution.height))),
                                   texture: DepthRGBTexture.make(image), session: id)
            var data = Data("RGB1".utf8)
            Self.appendBE(frameID, to: &data); data.append(jpeg)
            guard self.current(token) else { return }
            let outgoing = data
            let newAnchors: [MeasuredAnchor]?
            if frame.timestamp >= self.nextAnchorRGB {
                self.nextAnchorRGB = frame.timestamp + 0.2
                newAnchors = Self.extractMeasuredAnchors(source)
            } else { newAnchors = nil }
            self.queue.async {
                guard self.current(token) else { return }
                self.sources[frameID] = source; self.sourceOrder.append(frameID)
                if let newAnchors { self.rememberMeasuredAnchors(source, additions: newAnchors) }
                // Twelve frames lasted six seconds at the old 2 Hz send rate,
                // but only 0.4 seconds at 30 Hz, losing MiDaS source matches.
                // Keep a bounded three-second window at the preferred 30 Hz.
                while let oldest = self.sourceOrder.first,
                      self.sourceOrder.count > 90 || source.timestamp - (self.sources[oldest]?.timestamp ?? -.infinity) > 3 {
                    self.sources.removeValue(forKey: self.sourceOrder.removeFirst())
                }
                // Register the exact source before a fast QNX result can return.
                self.rgbQueue.async { if self.current(token) { self.rgb.send(outgoing) } }
            }
        }
    }

    func applyQNXResult(_ value: [String: Any]) {
        queue.async {
            let frameID: UInt64?
            if let raw = value["frame_id"] as? String { frameID = UInt64(raw) }
            else if let raw = value["frame_id"] as? NSNumber, CFGetTypeID(raw) != CFBooleanGetTypeID() {
                frameID = UInt64(exactly: raw.doubleValue)
            } else { frameID = nil }
            guard let frameID, frameID > self.lastQNXFrame, !self.resultBusy,
                  let source = self.sources[frameID], self.current(source.epoch),
                  let objects = value["objects"] as? [[String: Any]], objects.count <= 20 else { return }
            var detections = [DetectedObject]()
            for object in objects {
                guard let label = object["label"] as? String, !label.isEmpty, label.count <= 64,
                      let confidence = object["confidence"] as? Double, confidence.isFinite, (0...1).contains(confidence),
                      let box = object["bbox"] as? [Double], box.count == 4,
                      box.allSatisfy({ $0.isFinite && (0...1).contains($0) }), box[2] > 0, box[3] > 0,
                      box[0] + box[2] <= 1.00001, box[1] + box[3] <= 1.00001 else { return }
                // QNX coordinates are top-left normalized portrait pixels.
                // Internal drawing retains Vision's bottom-left convention.
                let bounds = CGRect(x: box[0], y: 1 - box[1] - box[3], width: box[2], height: box[3])
                let position = source.depth.flatMap { depth -> SIMD3<Float>? in
                    let x = Int((1 - bounds.midY) * CGFloat(depth.width))
                    let y = Int((1 - bounds.midX) * CGFloat(depth.height))
                    return depth.centerPosition(x: x, y: y)
                }
                detections.append(DetectedObject(label: label, confidence: Float(confidence),
                                                  boundingBox: bounds, position: position))
            }
            self.lastQNXFrame = frameID
            self.resultBusy = true
            let matched = detections
            self.resultQueue.async {
                let depthImage = source.depth.flatMap { InspectionRenderer.depthImage($0, context: self.rgbContext) }
                let image = InspectionRenderer.render(rgb: source.image, depthView: depthImage, detections: matched,
                                                       caption: "QNX detections • matched source frame")
                self.queue.async {
                    guard self.current(source.epoch) else { self.resultBusy = false; return }
                    let age = ProcessInfo.processInfo.systemUptime - source.timestamp
                    if source.tracking == "normal", self.lastTracking == "normal", age <= 1.5 {
                        self.locatedTimestamp = source.timestamp
                        self.locatedObjects = matched.compactMap { object -> [String: Any]? in
                            guard let p = object.position else { return nil }
                            return ["label": object.label, "confidence": object.confidence,
                                    "position": [p.x, p.y, p.z], "observation_timestamp": source.timestamp]
                        }
                    }
                    DispatchQueue.main.async {
                        self.qnxInspectionImage = image
                        self.qnxInspectionTimestamp = source.timestamp
                        self.qnxStatusString = "\(matched.count) QNX detections • \(String(format: "%.1f", age)) s source age"
                        self.detections = matched
                        self.queue.async { self.resultBusy = false }
                    }
                }
            }
        }
    }

    // Projection and texture checks run on rgbQueue, never the AR delegate queue.
    private static func extractMeasuredAnchors(_ source: RGBSource) -> [MeasuredAnchor] {
        guard source.tracking == "normal", let depth = source.depth, let texture = source.texture else { return [] }
        var anchors = [MeasuredAnchor]()
        for (index, xyz) in depth.points(limit: 768).enumerated() where index % 3 == 0 {
            let point = SIMD3<Float>(xyz[0],xyz[1],xyz[2])
            guard let projected = source.geometry.project(point),
                  let patch = texture.patch(u: projected.u,v: projected.v), DepthRGBTexture.agrees(patch,patch) else { continue }
            anchors.append(MeasuredAnchor(position:point,timestamp:source.timestamp,session:source.session,
                epoch:source.epoch,observer:source.geometry,patch:patch))
        }
        return anchors // At most 256 additions, at most 5 times per second.
    }

    private func rememberMeasuredAnchors(_ source: RGBSource, additions: [MeasuredAnchor]) {
        measuredAnchors = measuredAnchors.filter { _, anchor in
            anchor.epoch == source.epoch && anchor.session == source.session
                && (0...30).contains(source.timestamp-anchor.timestamp)
        }
        for anchor in additions {
            let point = anchor.position
            let cell = AnchorCell(x:Int(floor(point.x/0.15)),y:Int(floor(point.y/0.15)),z:Int(floor(point.z/0.15)))
            measuredAnchors[cell] = anchor
        }
        if measuredAnchors.count > 4096 {
            for (key, _) in measuredAnchors.sorted(by: { $0.value.timestamp < $1.value.timestamp }).prefix(measuredAnchors.count-4096) {
                measuredAnchors.removeValue(forKey:key)
            }
        }
    }

    private static func measuredNear(u: Double,v: Double,depth: SceneDepthGrid?) -> [Float] {
        guard let depth else { return [] }
        let x = min(depth.width-1,max(0,Int(v*Double(depth.width))))
        let y = min(depth.height-1,max(0,Int((1-u)*Double(depth.height))))
        return (-1...1).flatMap { dy in (-1...1).compactMap { dx in depth.depth(x: x+dx,y: y+dy) } }
    }

    func applyQNXDepthResult(_ value: [String: Any]) {
        queue.async {
            let frameID: UInt64?
            if let raw = value["frame_id"] as? String { frameID = UInt64(raw) }
            else if let raw = value["frame_id"] as? NSNumber, CFGetTypeID(raw) != CFBooleanGetTypeID() {
                frameID = UInt64(exactly: raw.doubleValue)
            } else { frameID = nil }
            guard let frameID, frameID > self.lastDepthFrame, !self.fusionBusy,
                  let source = self.sources[frameID], self.current(source.epoch),
                  value["kind"] as? String == "relative_inverse_depth",
                  value["depth_width"] as? Int == 64, value["depth_height"] as? Int == 64,
                  let inverse = value["inverse_depth"] as? [Double], inverse.count == 4096,
                  inverse.allSatisfy({ $0.isFinite && abs($0) < 1e9 }) else { return }
            self.lastDepthFrame = frameID; self.fusionBusy = true
            let anchors = Array(self.measuredAnchors.values), cached = self.cachedDepthFit
            let model = String((value["model"] as? String ?? "QNX relative depth").prefix(80))
            let inferenceMS = (value["inference_ms"] as? Double).flatMap { $0.isFinite && $0 >= 0 ? $0 : nil }
            self.fusionQueue.async {
                var fit: RelativeDepthCalibration?
                var mode = "relative", confidence = 0.0, anchorAge = 0.0
                var originalFitTimestamp = source.timestamp
                var rejection = "Scan nearby textured surfaces at varied distances to seed anchors, then look farther."
                func prediction(_ u: Double,_ v: Double) -> Double { RememberedDepthMath.sample(inverse,u: u,v: v) }
                if source.tracking == "normal" {
                    if let depth = source.depth {
                        var pairs = [(Double,Double)]()
                        for y in stride(from: 2,to: depth.height,by: max(1,depth.height/24)) {
                            for x in stride(from: 2,to: depth.width,by: max(1,depth.width/32)) {
                                guard let measured = depth.depth(x: x,y: y) else { continue }
                                let u = 1-(Double(y)+0.5)/Double(depth.height), v = (Double(x)+0.5)/Double(depth.width)
                                pairs.append((prediction(u,v),Double(measured)))
                            }
                        }
                        do {
                            fit = try RelativeDepthCalibration.fit(pairs); mode = "direct"
                            confidence = max(0.6,0.75-(fit?.p90Error ?? 0)*0.6)
                        } catch { rejection = "Current LiDAR: \(error.localizedDescription)" }
                    } else { rejection = "No current LiDAR pairs; scan nearby textured surfaces at varied distances first." }
                    if fit == nil, let texture = source.texture {
                        // Unique model pixels, current reprojection and a conservative
                        // appearance check. These approximate correspondences are not safety evidence.
                        var candidates: [Int:(relative: Double,meters: Double,age: Double)] = [:]
                        for anchor in anchors {
                            let age = source.timestamp-anchor.timestamp
                            guard anchor.epoch == source.epoch,anchor.session == source.session,(0.1...30).contains(age) else { continue }
                            let motion = source.geometry.motion(from: anchor.observer)
                            guard motion.translation <= 0.5,motion.degrees <= 20,
                                  let projected = source.geometry.project(anchor.position),
                                  let patch = texture.patch(u: projected.u,v: projected.v),
                                  DepthRGBTexture.agrees(anchor.patch,patch) else { continue }
                            let measured = Self.measuredNear(u: projected.u,v: projected.v,depth: source.depth)
                            guard measured.allSatisfy({ RememberedDepthMath.unoccluded(anchorMeters: projected.meters,measuredMeters: Double($0)) }) else { continue }
                            let bin = min(63,Int(projected.v*64))*64+min(63,Int(projected.u*64))
                            if let old = candidates[bin],old.meters <= projected.meters { continue }
                            candidates[bin] = (prediction(projected.u,projected.v),projected.meters,age)
                        }
                        do {
                            let samples = candidates.sorted { $0.key < $1.key }.map { $0.value }
                            fit = try RelativeDepthCalibration.fit(samples.map { ($0.relative,$0.meters) },maximumAnchor: 20)
                            mode = "anchors"; anchorAge = samples.map { $0.age }.max() ?? 0
                            confidence = max(0.2,0.55*exp(-anchorAge/30))
                        } catch { rejection += " · Remembered anchors: \(error.localizedDescription)" }
                    }
                    if fit == nil,let cached, cached.source.epoch == source.epoch,cached.source.session == source.session,
                       let texture = source.texture,let oldTexture = cached.source.texture {
                        let motion = source.geometry.motion(from: cached.source.geometry)
                        let age = source.timestamp-cached.source.timestamp
                        if source.timestamp-cached.anchorTimestamp <= 30,
                           RememberedDepthMath.cacheEligible(age: age,translation: motion.translation,degrees: motion.degrees),
                           texture.similar(to: oldTexture), RememberedDepthMath.predictionsAgree(inverse,cached.inverse) {
                            fit = cached.fit; mode = "cached"; originalFitTimestamp = cached.source.timestamp
                            anchorAge = source.timestamp-cached.anchorTimestamp
                            confidence = max(0.05,min(0.35,cached.confidence)*max(0,1-age/3))
                        } else { rejection += " · Cached fit expired or viewpoint/predictions changed." }
                    }
                } else { rejection = "Tracking is not normal; remembered anchors and cached scale are unavailable." }

                var visualPoints = [[Float]](), safetyPoints = [[Float]]()
                var visualMetadata: [String:Any]?, safetyMetadata: [String:Any]?
                if let fit {
                    for row in 0..<16 { for column in 0..<16 {
                        let u = (Double(column)+0.5)/16, v = (Double(row)+0.5)/16
                        // Fresh measured samples always win. No hole is replaced with
                        // a clamp-to-range endpoint: invalid/negative/far inverses stay absent.
                        guard Self.measuredNear(u: u,v: v,depth: source.depth).isEmpty,
                              let meters = fit.meters(prediction(u,v),maximum: 20),
                              let point = source.geometry.world(u: u,v: v,meters: meters) else { continue }
                        let xyz = [point.x,point.y,point.z]
                        visualPoints.append(xyz)
                        if mode == "direct",meters <= 8 { safetyPoints.append(xyz) }
                    } }
                    let base: [String:Any] = ["scale":fit.scale,"offset":fit.offset,"heldout_median_m":fit.medianError,
                        "heldout_p90_m":fit.p90Error,"samples":fit.samples,"observation_timestamp":source.timestamp,"model":model]
                    visualMetadata = base.merging(["usage":"persistent_map","visualization_only":true,
                        "calibration_timestamp":originalFitTimestamp,"mode":mode,"confidence":confidence,
                        "anchor_age_s":anchorAge,"max_range_m":20.0]) { _,new in new }
                    // This is the only route to the safety field: same-frame measured fit.
                    if mode == "direct" { safetyMetadata = base.merging(["usage":"conservative_obstacle_veto"]) { _,new in new } }
                }
                let fitSummary = fit.map { String(format:"%@: %d pairs · p90 %.2f m · heuristic confidence %.0f%% · anchors %.1f s old",
                    mode,$0.samples,$0.p90Error,confidence*100,anchorAge) } ?? String(rejection.prefix(350))
                let status = fit == nil ? "Relative depth only · no metric geometry accepted"
                    : "\(mode.capitalized) scale · \(visualPoints.count) visual points (≤20 m) · \(safetyPoints.count) direct-fit veto points (≤8 m)"
                let image = RGBDepthRenderer.render(rgb: source.image,inverse: inverse,calibration: fit,
                    caption: fit == nil ? "Relative model prediction. Seed nearby anchors before looking farther."
                        : "\(mode.capitalized) scale · model predictions to 20 m. Far range unvalidated; black = invalid/out of range.")
                let outputFit = fit, outputMode = mode, outputConfidence = confidence, outputAnchorAge = anchorAge
                let outputVisual = visualPoints, outputSafety = safetyPoints
                let outputVisualMetadata = visualMetadata, outputSafetyMetadata = safetyMetadata
                self.queue.async {
                    self.fusionBusy = false
                    guard self.current(source.epoch) else { return }
                    let age = ProcessInfo.processInfo.systemUptime-source.timestamp
                    let fresh = (0...2).contains(age) && source.tracking == "normal" && self.lastTracking == "normal"
                    self.estimatedPoints = fresh ? outputSafety : []
                    self.estimatedTimestamp = fresh && outputSafetyMetadata != nil ? source.timestamp : -.infinity
                    self.depthCalibration = fresh ? outputSafetyMetadata : nil
                    self.persistentPoints = fresh ? outputVisual : []
                    self.persistentTimestamp = fresh && outputVisualMetadata != nil ? source.timestamp : -.infinity
                    self.persistentCalibration = fresh ? outputVisualMetadata : nil
                    if fresh,let outputFit,outputMode == "direct" || outputMode == "anchors" {
                        self.cachedDepthFit = CachedDepthFit(fit:outputFit,source:source,inverse:inverse,mode:outputMode,
                            anchorTimestamp:source.timestamp-outputAnchorAge,confidence:outputConfidence)
                    }
                    let message = fresh || outputFit == nil ? status : status+" · expired/untracked: geometry withheld"
                    DispatchQueue.main.async {
                        guard self.current(source.epoch) else { return }
                        self.qnxDepthImage = image; self.qnxDepthTimestamp = source.timestamp
                        self.qnxDepthCalibrated = outputFit != nil; self.qnxDepthModel = model
                        self.qnxDepthInferenceMS = inferenceMS; self.qnxDepthFitSummary = fitSummary
                        self.qnxDepthMode = outputMode; self.qnxDepthConfidence = outputConfidence
                        self.qnxDepthAnchorAge = outputFit == nil ? nil : outputAnchorAge
                        self.depthStatusString = message
                    }
                }
            }
        }
    }

#if DEBUG
    static func depthCalibrationSelfTest() -> [String: Bool] {
        let clean = (0..<100).map { i -> (Double, Double) in
            let meters = 0.4 + Double(i) / 99 * 3.3
            return ((1 / meters - 0.08) / 1.7, meters)
        }
        var outliers = clean
        for i in stride(from: 7, to: 100, by: 10) { outliers[i].1 = 3.8 }
        let fit = try? RelativeDepthCalibration.fit(clean)
        let robust = try? RelativeDepthCalibration.fit(outliers)
        let flat = try? RelativeDepthCalibration.fit(Array(repeating: (0.5, 2.0), count: 100))
        let wrongSign = try? RelativeDepthCalibration.fit(clean.map { (-$0.0, $0.1) })
        let tooFew = try? RelativeDepthCalibration.fit(Array(clean.prefix(20)))
        let inconsistent = try? RelativeDepthCalibration.fit(clean.enumerated().map {
            ($0.element.0, $0.offset % 2 == 0 ? 0.4 : 3.8)
        })
        return ["known_scale": fit.map { abs($0.scale - 1.7) < 1e-8 && abs($0.offset - 0.08) < 1e-8 } ?? false,
                "training_outliers": robust.map { $0.p90Error < 0.01 } ?? false,
                "flat_rejected": flat == nil, "negative_scale_rejected": wrongSign == nil,
                "too_few_rejected": tooFew == nil, "inconsistent_rejected": inconsistent == nil]
    }
#endif

    func session(_ session: ARSession, didAdd anchors: [ARAnchor]) { updateFloor(anchors) }
    func session(_ session: ARSession, didUpdate anchors: [ARAnchor]) { updateFloor(anchors) }
    private func updateFloor(_ anchors: [ARAnchor]) {
        guard wanted, !explicitFloor, lastTracking == "normal" else { return }
        for case let plane as ARPlaneAnchor in anchors where plane.classification == .floor {
            let center = plane.transform * SIMD4<Float>(plane.center.x, plane.center.y, plane.center.z, 1)
            floorY = center.y
        }
    }

    func setFloor() {
        queue.async {
            guard self.wanted, self.lastTracking == "normal", let frame = self.latestFrame,
                  let depth = SceneDepthGrid(frame),
                  let point = depth.centerPosition(x: depth.width / 2, y: depth.height / 2) else {
                self.report("Aim center at the floor with normal tracking and valid LiDAR depth"); return
            }
            self.floorY = point.y; self.explicitFloor = true
            self.report("Floor set at center depth point; keep phone aimed at the ground when setting")
        }
    }

    private struct SavedMap: Codable { let session: String; let floor: Float?; let explicitFloor: Bool; let worldMap: Data }
    private var mapURL: URL { FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0].appendingPathComponent("surroundings-worldmap.json") }

    func saveMap(completion: (() -> Void)? = nil) {
        queue.async {
            guard self.wanted, self.lastTracking == "normal", let frame = self.latestFrame,
                  frame.worldMappingStatus == .mapped || frame.worldMappingStatus == .extending else {
                self.report("Map not ready: move slowly around the room with normal tracking"); return
            }
            let token = self.epoch, id = self.sessionID, floor = self.floorY, explicit = self.explicitFloor
            self.session.getCurrentWorldMap { map, error in
                self.queue.async {
                    guard self.epoch == token, self.wanted else { return }
                    guard self.lastTracking == "normal" else {
                        self.report("Tracking changed while saving; try Save map again with normal tracking"); return
                    }
                    do {
                        guard let map else { throw error ?? NSError(domain: "Scene", code: 2, userInfo: [NSLocalizedDescriptionKey: "ARKit returned no world map"]) }
                        let data = try NSKeyedArchiver.archivedData(withRootObject: map, requiringSecureCoding: true)
                        let saved = SavedMap(session: id, floor: floor, explicitFloor: explicit, worldMap: data)
                        try JSONEncoder().encode(saved).write(to: self.mapURL, options: .atomic)
                        self.report("World map saved on iPhone; Pi keeps its observation map")
                        if let completion { DispatchQueue.main.async(execute: completion) }
                    } catch { self.report("Save map: \(error.localizedDescription)") }
                }
            }
        }
    }

    func loadMap(completion: (() -> Void)? = nil) {
        queue.async {
            do {
                let saved = try JSONDecoder().decode(SavedMap.self, from: Data(contentsOf: self.mapURL))
                guard let map = try NSKeyedUnarchiver.unarchivedObject(ofClass: ARWorldMap.self, from: saved.worldMap) else {
                    throw NSError(domain: "Scene", code: 3, userInfo: [NSLocalizedDescriptionKey: "Invalid saved map"])
                }
                self.sessionID = saved.session; self.floorY = saved.floor; self.explicitFloor = saved.explicitFloor
                if self.wanted { self.runSession(map: map) } else { self.loadedMap = map }
                self.report(self.wanted ? "Relocalizing: point rear camera at the saved room" : "Saved map loaded; press Start in the saved room")
                if let completion { DispatchQueue.main.async(execute: completion) }
            } catch { self.report("Load map: \(error.localizedDescription)") }
        }
    }

    func resetMap() {
        queue.async {
            self.sessionID = UUID().uuidString; self.floorY = nil; self.explicitFloor = false; self.loadedMap = nil
            if self.wanted { self.runSession(map: nil) } else { self.advanceGeneration() }
            self.report("New map started; saved map remains available to load")
        }
    }

    func session(_ session: ARSession, didFailWithError error: Error) {
        end(); report("AR session failed: \(error.localizedDescription)")
    }
    func sessionWasInterrupted(_ session: ARSession) {
        lastTracking = "unavailable"; latestFrame = nil; advanceGeneration()
        clearInspection("AR session interrupted")
        report("AR session interrupted; mapping paused")
    }
    func sessionInterruptionEnded(_ session: ARSession) {
        relocalizing = true; awaitingFirstFrame = true; report("AR session resumed; waiting for tracking")
    }
}
