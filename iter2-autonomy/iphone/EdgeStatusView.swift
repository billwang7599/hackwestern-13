import SwiftUI
import CoreFoundation

struct EdgeStatusView: View {
    let status: [String: Any]
    let connected: Bool
    private var guardState: [String: Any] { status["safety"] as? [String: Any] ?? [:] }
    private var ai: [String: Any] { status["ai"] as? [String: Any] ?? [:] }
    private var depth: [String: Any] { status["depth"] as? [String: Any] ?? [:] }
    private var guardLive: Bool { connected && guardState["live"] as? Bool == true }
    private var verdict: String { guardLive ? guardState["verdict"] as? String ?? "UNKNOWN" : "STALE" }

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("QNX independent pipelines").font(.title2.bold())
            HStack {
                Label("LiDAR + conservative depth veto", systemImage: "shield.lefthalf.filled").font(.headline)
                Spacer()
                Text(verdict).font(.caption.bold()).foregroundStyle(ObstaclePresentation(status: status, connected: connected).color)
            }
            Text("\(number(guardState, "tick_hz", digits: 0)) Hz watchdog · \(number(guardState, "sample_age_ms", digits: 0)) ms sample age")
                .font(.footnote.monospacedDigit())
            Text("Frames \(number(guardState, "frames", digits: 0)) · late ticks \(number(guardState, "late_ticks", digits: 0)) · missed ticks \(number(guardState, "missed_ticks", digits: 0))")
                .font(.caption.monospacedDigit())
            Text("Maximum tick lateness: \(number(guardState, "max_lateness_ms")) ms")
                .font(.caption.monospacedDigit())
            Text("Priority \(number(guardState, "priority", digits: 0)) · CPU mask \(number(guardState, "runmask", digits: 0))")
                .font(.caption.monospacedDigit())
            if (guardState["schedule_error"] as? Int ?? 0) != 0 || (guardState["affinity_error"] as? Int ?? 0) != 0 {
                Text("Requested scheduling was not fully applied.").font(.caption).foregroundStyle(.orange)
            }
            Text("Uncalibrated, unknown, blocked or stale input denies drive. The native motor gate stops independently of this display.")
                .font(.caption).foregroundStyle(.secondary)
            Text(ObstaclePresentation(status: status, connected: connected).detail).font(.footnote)
            Text("LiDAR: \(guardState["lidar_verdict"] as? String ?? "UNKNOWN") · stop source: \(guardState["stop_source"] as? String ?? "none")")
                .font(.caption)
            Text("Inferred veto: \((guardLive && guardState["inferred_live"] as? Bool == true) ? "fresh" : "unavailable") · nearest \(number(guardState, "inferred_nearest_m", digits: 2)) m")
                .font(.caption.monospacedDigit())
            Divider()
            worker("RGB object detection", value: ai)
            worker("RGB depth estimation", value: depth)
            Text("AI cannot grant clearance. Fresh LiDAR-calibrated depth may add obstacles or stop motion. The independent measured LiDAR guard remains required.")
                .font(.caption).foregroundStyle(.secondary)
        }.frame(maxWidth: .infinity, alignment: .leading).padding(14)
            .background(Color.secondary.opacity(0.08), in: RoundedRectangle(cornerRadius: 14))
    }

    private func worker(_ title: String, value: [String: Any]) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            HStack {
                Text(title).font(.headline)
                Spacer()
                Text(connected && value["live"] as? Bool == true ? "LIVE" : "STALE")
                    .font(.caption.bold()).foregroundStyle(connected && value["live"] as? Bool == true ? Color.green : Color.orange)
            }
            Text("\(value["model"] as? String ?? "Waiting for model") · ONNX Runtime \(value["runtime"] as? String ?? "—") on QNX")
                .font(.caption)
            Text("Inference \(number(value, "inference_ms", digits: 0)) ms · processing \(number(value, "processing_ms", digits: 0)) ms")
                .font(.caption.monospacedDigit())
            if let error = value["error"] as? String { Text(String(error.prefix(200))).font(.caption).foregroundStyle(.orange) }
        }
    }

    private func number(_ value: [String: Any], _ key: String, digits: Int = 1) -> String {
        guard let number = value[key] as? Double, number.isFinite, abs(number) < 1e12 else { return "—" }
        return String(format: "%.*f", digits, number)
    }
}

/// Presentation only. The native Pi guard independently decides motor permission.
struct ObstaclePresentation {
    let color: Color
    let title: String
    let detail: String
    let symbol: String

    init(status: [String: Any], connected: Bool) {
        let guardState = status["safety"] as? [String: Any] ?? [:]
        func number(_ key: String) -> Double? {
            guard let value = guardState[key] as? NSNumber,
                  CFGetTypeID(value) != CFBooleanGetTypeID(), value.doubleValue.isFinite else { return nil }
            return value.doubleValue
        }
        let stop = max(0.01, number("stop_distance_m") ?? 0.6)
        let warning = max(stop, number("warning_distance_m") ?? 1.2)
        let nearest = number("nearest_obstacle_m").flatMap { $0 >= 0 && $0 <= 100 ? $0 : nil }
        let verdict = guardState["verdict"] as? String ?? "UNKNOWN"
        let live = connected && guardState["live"] as? Bool == true
        let stopSource = guardState["stop_source"] as? String ?? "none"
        let inferred = stopSource == "inferred"
        let distance = nearest.map { String(format: inferred ? "Inferred forward obstacle %.2f m" : "Nearest forward obstacle %.2f m", $0) }
        let reason = guardState["reason"] as? String
        if !connected {
            color = .red; title = "STOP · Pi disconnected"; symbol = "exclamationmark.shield"
            detail = "Waiting for current Pi status"
        } else if !live || verdict == "STALE" || verdict == "DISCONNECTED" {
            color = .red; symbol = "exclamationmark.shield"
            switch live ? reason : nil {
            case "frame_late":
                title = "STOP · frame late"; detail = "LiDAR updates are late or queued on the Pi."
            case "lidar_disconnected", "waiting_for_frame":
                title = "STOP · waiting for LiDAR"; detail = "Check the phone connection and start the sensor stream."
            default:
                title = "STOP · guard unavailable"; detail = "Waiting for fresh LiDAR guard status."
            }
        } else if verdict == "BLOCKED" || nearest.map({ $0 <= stop }) == true {
            color = .red
            title = inferred ? "STOP · inferred obstacle" : (stopSource == "lidar" ? "STOP · LiDAR obstacle" : "STOP · obstacle proximity")
            symbol = "hand.raised.fill"
            detail = (distance ?? (inferred ? "Calibrated RGB depth veto" : "Measured obstacle inside the stop zone"))
                + (inferred ? " · conservative estimate" : "")
        } else if guardState["permit"] as? Bool != true {
            color = .red; symbol = "exclamationmark.triangle.fill"
            switch reason {
            case "tracking_limited":
                title = "STOP · tracking limited"; detail = "Keep the camera steady with room features in view."
            case "set_floor":
                title = "STOP · set floor"; detail = "Aim at the floor and tap Set floor, then aim forward."
            case "aim_forward":
                title = "STOP · aim forward"; detail = "Point the rear camera along the robot’s direction of travel."
            case "no_depth_points":
                title = "STOP · no depth points"; detail = "The latest frame has no usable LiDAR returns."
            case "snapshot_busy":
                title = "STOP · guard updating"; detail = "Waiting for the next guard reading."
            default:
                title = "STOP · clearance unknown"; detail = "The Pi has not reported a specific cause."
            }
        } else if verdict == "CAUTION" {
            color = .orange; title = "CAUTION · partial floor coverage"; symbol = "exclamationmark.triangle.fill"
            detail = "Drive permitted. Parts of the forward corridor are unobserved."
        } else if nearest.map({ $0 < warning }) == true {
            color = .orange; title = "CAUTION · approaching obstacle"; symbol = "exclamationmark.triangle.fill"
            detail = (distance ?? "") + String(format: " · stop zone %.2f m", stop)
        } else {
            color = .green; title = "CLEAR · observed forward corridor"; symbol = "checkmark.shield.fill"
            detail = distance ?? "No obstacle measured in the guarded corridor"
        }
    }
}
