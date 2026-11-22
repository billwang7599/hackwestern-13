import SwiftUI
import CoreFoundation

/// All readings in this card come back from the Pi, including in headless mode.
struct PiTelemetryView: View {
    let status: [String: Any]
    let connected: Bool

    private var streaming: Bool { status["streaming"] as? Bool ?? false }
    private var motionLive: Bool { connected && streaming && (status["motion_live"] as? Bool ?? false) }
    private var cameraLive: Bool { connected && streaming && (status["camera_live"] as? Bool ?? false) }
    private var motor: [String: Any] { status["motor"] as? [String: Any] ?? [:] }
    private var scene: [String: Any] { status["surroundings"] as? [String: Any] ?? [:] }

    var body: some View {
        VStack(alignment: .leading, spacing: 14) {
            HStack {
                Text("Pi telemetry").font(.title2.bold())
                Spacer()
                Text(connected ? (streaming ? "RECEIVING" : "PAUSED") : "DISCONNECTED")
                    .font(.caption.bold())
                    .foregroundStyle(connected ? (streaming ? Color.green : Color.orange) : Color.orange)
            }
            Text(modeText).font(.subheadline)
            if !connected {
                Text("Showing the last Pi readings. Values are no longer updating.")
                    .font(.footnote).foregroundStyle(.orange)
            }

            VStack(alignment: .leading, spacing: 14) {
                LazyVGrid(columns: [GridItem(.adaptive(minimum: 110), alignment: .leading)], alignment: .leading, spacing: 12) {
                    metric("Motion received", value: formatted(status["motion_hz"], suffix: " Hz"), live: motionLive)
                    metric("Paired frames received", value: formatted(status["camera_fps"], suffix: " fps"), live: cameraLive)
                    metric("Motion samples", value: count(status["samples"]))
                    metric("Sequence gaps", value: count(status["gaps"]))
                    metric("Frame count", value: count(status["frames"]))
                    metric("Pi receipt age", value: formatted(status["receipt_age_ms"], digits: 0, suffix: " ms"))
                }
                Text("Receipt age is measured on the Pi; it is not end-to-end sensor latency. Frame rate counts receipt, not display.")
                    .font(.caption).foregroundStyle(.secondary)

                ViewThatFits(in: .horizontal) {
                    HStack(alignment: .top, spacing: 14) {
                        motionPanel("Acceleration", unit: "m/s²", key: "accel_m_s2", scale: 12).frame(minWidth: 210)
                        motionPanel("Angular velocity", unit: "rad/s", key: "gyro_rad_s", scale: 3).frame(minWidth: 210)
                    }
                    VStack(spacing: 14) {
                        motionPanel("Acceleration", unit: "m/s²", key: "accel_m_s2", scale: 12)
                        motionPanel("Angular velocity", unit: "rad/s", key: "gyro_rad_s", scale: 3)
                    }
                }

                Divider()
                Text("Pi mapping").font(.headline)
                Text("Tracking: \(shortText(scene["tracking"], fallback: "waiting")) · Floor: \((scene["floor_ready"] as? Bool ?? false) ? "calibrated" : "needed")")
                    .font(.footnote)
                LazyVGrid(columns: [GridItem(.adaptive(minimum: 110), alignment: .leading)], alignment: .leading, spacing: 12) {
                    metric("Scene frames", value: count(scene["frames"]))
                    metric("Stored 3D cells", value: count(scene["voxels"]))
                    metric("Remembered targets", value: scene["objects"] is [Any] ? String((scene["objects"] as? [Any])?.count ?? 0) : "—")
                }
                Text("Motor backend: \(shortText(motor["backend"], fallback: "unknown")) · \((connected && motor["connected"] as? Bool == true) ? "connected" : "unavailable")")
                    .font(.footnote.monospaced())
                Text("Output L \(signed((motor["output"] as? [Any])?.first)) / R \(signed((motor["output"] as? [Any])?.dropFirst().first))")
                    .font(.footnote.monospaced())
                Text((motor["mode"] as? String == "stub") ? "Stub backend: commands are exercised without physical motor output." : "Command values do not confirm wheel motion. Check the motor link and guard status.")
                    .font(.caption).foregroundStyle(.orange)
                Text("\((motor["inhibited"] as? Bool ?? true) ? "STOP / inhibited" : "Gate open") · \(shortText(motor["reason"], fallback: "Awaiting motor status"))")
                    .font(.caption).foregroundStyle((motor["inhibited"] as? Bool ?? true) ? Color.orange : Color.secondary)
                if let error = motor["command_error"] as? String, !error.isEmpty {
                    Text(String(error.prefix(180))).font(.caption).foregroundStyle(.red)
                }
            }
            .opacity(connected ? 1 : 0.55)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(14)
        .background(Color.secondary.opacity(0.08), in: RoundedRectangle(cornerRadius: 14))
    }

    private var modeText: String {
        switch status["mode"] as? String {
        case "headless": return "Headless Pi · no monitor required"
        case "dashboard": return "Pi dashboard is open"
        default: return "Waiting for Pi mode"
        }
    }

    private func metric(_ title: String, value: String, live: Bool? = nil) -> some View {
        VStack(alignment: .leading, spacing: 3) {
            Text(title).font(.caption).foregroundStyle(.secondary)
            Text(value).font(.system(.body, design: .monospaced).weight(.semibold))
            if let live {
                Text(live ? "LIVE" : (connected && !streaming ? "PAUSED" : "STALE"))
                    .font(.caption2.bold()).foregroundStyle(live ? Color.green : Color.orange)
            }
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }

    private func motionPanel(_ title: String, unit: String, key: String, scale: Double) -> some View {
        let values = vector(status[key])
        return VStack(alignment: .leading, spacing: 9) {
            HStack {
                Text(title).font(.headline)
                Spacer()
                Text(unit).font(.caption).foregroundStyle(.secondary)
            }
            ForEach(0..<3) { index in
                axisRow(index: index, value: values?[index], scale: scale)
            }
        }
        .padding(12)
        .background(Color.secondary.opacity(0.07), in: RoundedRectangle(cornerRadius: 10))
        .opacity(motionLive ? 1 : 0.6)
        .accessibilityElement(children: .contain)
        .accessibilityLabel("Pi-received \(title), \(motionLive ? "live" : "stale")")
    }

    private func axisRow(index: Int, value: Double?, scale: Double) -> some View {
        let axis = ["X", "Y", "Z"][index]
        let color: Color = [.blue, .green, .orange][index]
        let fraction = max(-1, min(1, (value ?? 0) / scale))
        return VStack(spacing: 3) {
            HStack {
                Text(axis).foregroundStyle(color)
                Spacer()
                Text(value.map { String(format: "%+.3f", $0) } ?? "—")
                    .monospacedDigit()
            }.font(.system(.body, design: .monospaced))
            GeometryReader { geometry in
                let half = geometry.size.width / 2
                ZStack(alignment: .leading) {
                    Capsule().fill(Color.secondary.opacity(0.16))
                    Rectangle().fill(color)
                        .frame(width: half * abs(fraction))
                        .offset(x: fraction < 0 ? half * (1 + fraction) : half)
                    Rectangle().fill(Color.secondary).frame(width: 1).offset(x: half)
                }
            }.frame(height: 5)
        }
        .accessibilityElement(children: .ignore)
        .accessibilityLabel("\(axis): \(value.map { String(format: "%+.3f", $0) } ?? "unavailable")")
    }

    private func number(_ value: Any?) -> Double? {
        guard let number = value as? NSNumber, CFGetTypeID(number) != CFBooleanGetTypeID() else { return nil }
        let result = number.doubleValue
        return result.isFinite && abs(result) <= 1e12 ? result : nil
    }

    private func vector(_ value: Any?) -> [Double]? {
        guard let items = value as? [Any], items.count == 3 else { return nil }
        let values = items.compactMap { number($0) }
        return values.count == 3 ? values : nil
    }

    private func formatted(_ value: Any?, digits: Int = 1, suffix: String) -> String {
        guard let value = number(value), value >= 0 else { return "—" }
        return String(format: "%.*f", digits, value) + suffix
    }

    private func count(_ value: Any?) -> String {
        guard let value = number(value), value >= 0, value.rounded(.towardZero) == value else { return "—" }
        return String(Int64(value))
    }

    private func signed(_ value: Any?) -> String {
        guard let value = number(value), abs(value) <= 1 else { return "—" }
        return String(format: "%+.2f", value)
    }

    private func shortText(_ value: Any?, fallback: String) -> String {
        guard let text = value as? String, !text.isEmpty else { return fallback }
        return String(text.prefix(64))
    }
}

/// Drives while held: resends every 0.2 s (the Pi expires a drive after 0.5 s)
/// and halts on release. A dropped USB or Wi-Fi link stops the car the same way.
struct HoldDriveButton: View {
    let label: String
    let left: Double, right: Double
    @ObservedObject var pi: PiStatusServer
    @Binding var lastTap: String
    @State private var timer: Timer?

    var body: some View {
        Text(label).font(.body.bold()).foregroundStyle(.white)
            .padding(.horizontal, 14).padding(.vertical, 10)
            .background(timer == nil ? Color.accentColor : Color.green, in: RoundedRectangle(cornerRadius: 8))
            .opacity(pi.connected ? 1 : 0.4)
            .gesture(DragGesture(minimumDistance: 0)
                .onChanged { _ in if timer == nil && pi.connected { begin() } }
                .onEnded { _ in end() })
            .onDisappear { end() }
            .accessibilityAddTraits(.isButton)
    }

    private func begin() {
        lastTap = label
        pi.command(["op": "control_phone"])
        send()
        timer = Timer.scheduledTimer(withTimeInterval: 0.2, repeats: true) { _ in send() }
    }

    private func send() { pi.command(["op": "drive", "left": left, "right": right]) }

    private func end() {
        guard let running = timer else { return }
        running.invalidate()
        timer = nil
        pi.command(["op": "halt"])
    }
}

/// Every diagnostic value here is echoed by the Pi, not assumed from a tap.
struct ManualDrivePanel: View {
    @ObservedObject var pi: PiStatusServer
    @State private var lastTap = "None"
    private var keyboard: [String: Any] { pi.latestStatus["keyboard"] as? [String: Any] ?? [:] }
    private var motor: [String: Any] { pi.latestStatus["motor"] as? [String: Any] ?? [:] }
    private func wheels(_ key: String) -> String {
        let v = motor[key] as? [Double] ?? [0, 0]
        guard v.count == 2 else { return "—" }
        return String(format: "L %+.2f / R %+.2f", v[0], v[1])
    }
    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Manual drive & input check").font(.title2.bold())
            Text("Hold to drive at 25% speed over USB or Wi-Fi; release stops. Each press takes phone control; the LiDAR guard still decides whether movement is allowed.")
                .font(.caption).foregroundStyle(.secondary)
            VStack(spacing: 8) {
                HoldDriveButton(label: "W ↑ Forward", left: 0.25, right: 0.25, pi: pi, lastTap: $lastTap)
                HStack {
                    HoldDriveButton(label: "A ← Left", left: -0.25, right: 0.25, pi: pi, lastTap: $lastTap)
                    Button("STOP") { lastTap = "STOP"; pi.command(["op": "halt"]) }
                        .buttonStyle(.borderedProminent).tint(.red).disabled(!pi.connected)
                    HoldDriveButton(label: "D Right →", left: 0.25, right: -0.25, pi: pi, lastTap: $lastTap)
                }
                HoldDriveButton(label: "S ↓ Reverse", left: -0.25, right: -0.25, pi: pi, lastTap: $lastTap)
            }.frame(maxWidth: .infinity)
            Text("Last phone tap: \(lastTap)").font(.caption)
            Divider()
            let keys = keyboard["keys"] as? [String] ?? []
            let last = keyboard["last_keys"] as? [String] ?? []
            Text("Pi keyboard: \((keyboard["connected"] as? Bool == true && pi.connected) ? "connected" : "unavailable")")
                .font(.headline)
            Text("Keys received: \(keys.isEmpty ? "none" : keys.joined(separator: " + "))")
                .font(.title3.monospaced()).foregroundStyle(keys.isEmpty ? Color.secondary : Color.blue)
            Text("Last event: \(last.isEmpty ? "none" : last.joined(separator: " + ")) · count \(keyboard["event_count"] as? Int ?? 0)")
                .font(.caption.monospaced())
            Text("Last pressed: \((keyboard["last_press"] as? [String] ?? []).joined(separator: " + "))").font(.caption.monospaced())
            Text("Control owner: \(keyboard["owner"] as? String ?? "unknown")")
                .font(.caption)
            Text("Pi requested: \(wheels("requested"))\nNative output: \(wheels("output"))")
                .font(.footnote.monospaced())
            Text("Guard / stop reason: \(motor["reason"] as? String ?? "waiting")")
                .font(.footnote).foregroundStyle(.orange)
            Text("Serial: \(motor["serial_device"] as? String ?? "not configured") · \(motor["serial_device_present"] as? Bool == true ? "device present" : "device missing")\nArduino READY messages: \(motor["arduino_ready_count"] as? Int ?? 0)")
                .font(.caption.monospaced())
            Text("Key received + zero output: check guard / stop reason. Nonzero output without motion: check the Arduino link, power and wiring. Output is a command, not measured wheel motion.")
                .font(.caption).foregroundStyle(.secondary)
            if !pi.connected { Text("Disconnected: readings above are the last received values.").font(.caption).foregroundStyle(.orange) }
        }.frame(maxWidth: .infinity, alignment: .leading).padding(14)
            .background(Color.secondary.opacity(0.08), in: RoundedRectangle(cornerRadius: 14))
    }
}
