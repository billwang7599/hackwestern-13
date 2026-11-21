import SwiftUI

@main
struct TARSDepthApp: App {
    var body: some Scene {
        WindowGroup { ContentView() }
    }
}

struct ContentView: View {
    @StateObject private var s = DepthStreamer()
    @AppStorage("host") private var host = "192.168.0.10"
    @AppStorage("port") private var port = 5559
    @AppStorage("rate") private var rate = 12.0
    @AppStorage("bins") private var bins = 61
    @AppStorage("camHeight") private var camHeight = 0.12

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 12) {
                Text("TARS Depth").font(.title.bold())
                HStack {
                    TextField("Pi IP", text: $host)
                        .keyboardType(.numbersAndPunctuation)
                        .textInputAutocapitalization(.never).autocorrectionDisabled()
                        .textFieldStyle(.roundedBorder)
                    TextField("port", value: $port, format: .number.grouping(.never))
                        .keyboardType(.numberPad).frame(width: 80)
                        .textFieldStyle(.roundedBorder)
                    Button(s.running ? "Stop" : "Start") {
                        if s.running { s.stop() } else {
                            s.rateHz = rate
                            s.params.bins = max(8, min(181, bins))
                            s.params.camHeight = Float(camHeight)
                            s.start(host: host, port: UInt16(clamping: port))
                        }
                    }
                    .buttonStyle(.borderedProminent).tint(s.running ? .red : .green)
                }
                .disabled(false)
                HStack {
                    Stepper("Rate \(Int(rate)) Hz", value: $rate, in: 5...15)
                    Stepper("Lens \(Int(camHeight * 100)) cm", value: $camHeight, in: 0.04...0.6, step: 0.01)
                }
                .disabled(s.running).font(.callout)

                if let e = s.error { Text(e).foregroundStyle(.red) }
                if let w = s.mountWarning { Text(w).foregroundStyle(.orange) }

                HStack(alignment: .firstTextBaseline, spacing: 24) {
                    stat("ahead ±15°", s.minAhead.map { String(format: "%.2f m", $0) } ?? "clear/none",
                         color: (s.minAhead ?? 9) < 0.3 ? .red : .primary, big: true)
                    stat("tracking", s.track, color: s.track == "normal" ? .green : .orange)
                    stat("sending", String(format: "%.1f Hz", s.sendHz))
                    stat("frames", "\(s.sent)" + (s.sendErrors > 0 ? " (\(s.sendErrors) err)" : ""))
                }
                Text(s.poseText).font(.callout.monospaced())
                RangePlot(ranges: s.ranges, fov: s.fov).frame(height: 160)
                Text("Mount the phone landscape, rear camera facing forward, level, about the same height as the webcam. Keep this app in front: if it goes to the background, the Pi sees stale data and treats it as no lidar.")
                    .font(.footnote).foregroundStyle(.secondary)
            }
            .padding()
        }
    }

    private func stat(_ k: String, _ v: String, color: Color = .primary, big: Bool = false) -> some View {
        VStack(alignment: .leading) {
            Text(k).font(.caption).foregroundStyle(.secondary)
            Text(v).font(big ? .title2.monospacedDigit().bold() : .body.monospacedDigit()).foregroundStyle(color)
        }
    }
}

/// Top-down view: robot at the bottom centre, one ray per bin, 3 m full scale.
struct RangePlot: View {
    let ranges: [Float?]
    let fov: (Float, Float)

    var body: some View {
        Canvas { ctx, size in draw(ctx, size) }
        .background(Color.secondary.opacity(0.08))
        .clipShape(RoundedRectangle(cornerRadius: 8))
    }

    private func draw(_ ctx: GraphicsContext, _ size: CGSize) {
        let n = ranges.count
        guard n > 0 else { return }
        let ox: Double = Double(size.width) / 2
        let oy: Double = Double(size.height) - 4
        let scale: Double = (Double(size.height) - 8) / 3.0
        let lo = Double(fov.0)
        let step = Double(fov.1 - fov.0) / Double(n)
        for i in 0..<n {
            let r: Float? = ranges[i]
            let a: Double = (lo + (Double(i) + 0.5) * step) * Double.pi / 180
            let len: Double = Double(min(r ?? 3, 3)) * scale
            var p = Path()
            p.move(to: CGPoint(x: ox, y: oy))
            p.addLine(to: CGPoint(x: ox + sin(a) * len, y: oy - cos(a) * len))
            let col: Color
            if let r = r { col = r < 0.3 ? .red : .blue } else { col = Color.gray.opacity(0.25) }
            ctx.stroke(p, with: .color(col), lineWidth: 2)
        }
    }
}
