import Foundation
import Network
import AVFoundation

// Robot mic: the robot has no microphone, so the phone riding on the car is its
// ear, through the user's AirPods (Bluetooth HFP input). The audio goes to the
// Pi's tars_voice, which runs the same whisper + wake word + intent path as a
// wired mic (iter2-autonomy/voice, `input = phone`).
//
// Transport (TCP 5577, forwarded over USB by iproxy like the 5573 speech link):
// the phone LISTENS and tars_voice on the Pi dials it. The phone sends
//   "MIC1 16000 1 s16le\n"                   once per connection
//   <type:1><len:2 LE><payload>              packets:
//     'A' audio  16 kHz mono s16le, len even 2...8192 (we send 20 ms = 640 bytes)
//     'M' mute   len 0: QNXplorer is talking (half duplex), ignore audio until 'U'
//     'U' unmute len 0
//     'K' keepalive len 0, at least once a second when no audio flows
// The Pi never writes back. Spec and parser: iter2-autonomy/voice/src/audio.h / audio.c.
//
// Half duplex: iOS keeps a Bluetooth headset's mic and output together, and
// forcing the phone speaker (overrideOutputAudioPort(.speaker)) also switches
// the input to the phone mic. So while QNXplorer talks the mic stream is muted and
// the reply comes out of the phone speaker; when it finishes, the AirPods
// route and the stream come back. SpeechLink drives this through
// speechWillStart()/speechDidFinish(). The switch time is shown in the UI.

// BEGIN PHONE_MIC_PURE
enum MicStreamProtocol {
    static let port: UInt16 = 5577
    static let headerLine = "MIC1 16000 1 s16le"
    static var header: Data { Data((headerLine + "\n").utf8) }
    static let sampleRate: Double = 16000
    /// Samples per audio packet: 20 ms, the Pi's VAD frame.
    static let packetSamples = 320
    static let maxAudioBytes = 8192
    static let keepaliveInterval: TimeInterval = 1.0
    /// Audio packets allowed in flight before we drop new ones (~2 s at 20 ms).
    static let maxInFlight = 100

    enum Packet: Equatable {
        case audio([Int16])
        case mute, unmute, keepalive
    }

    /// One packet on the wire. Audio longer than maxAudioBytes is split into several packets.
    static func encode(_ packet: Packet) -> Data {
        func frame(_ type: UInt8, _ payload: Data) -> Data {
            var d = Data([type, UInt8(payload.count & 0xff), UInt8(payload.count >> 8)])
            d.append(payload)
            return d
        }
        switch packet {
        case .mute: return frame(0x4d, Data())
        case .unmute: return frame(0x55, Data())
        case .keepalive: return frame(0x4b, Data())
        case .audio(let samples):
            var out = Data()
            let per = maxAudioBytes / 2
            var i = 0
            while i < samples.count {
                let chunk = samples[i..<min(i + per, samples.count)]
                var payload = Data(capacity: chunk.count * 2)
                for s in chunk {
                    let u = UInt16(bitPattern: s)
                    payload.append(UInt8(u & 0xff)); payload.append(UInt8(u >> 8))
                }
                out.append(frame(0x41, payload))
                i += per
            }
            return out
        }
    }

    /// The Pi's parser, mirrored (for tests and debugging tools). Same rules as audio.c.
    struct Decoder {
        enum Failure: Error, Equatable { case badHeader, badPacket(String) }
        private var headerDone = false
        private var headerBytes = Data()
        private var pending = Data()

        mutating func feed(_ data: Data) throws -> [Packet] {
            var out: [Packet] = []
            var rest = data[...]
            if !headerDone {
                for (k, b) in rest.enumerated() {
                    if b == 10 {
                        var line = String(decoding: headerBytes, as: UTF8.self)
                        if line.hasSuffix("\r") { line.removeLast() }
                        guard line == MicStreamProtocol.headerLine else { throw Failure.badHeader }
                        headerDone = true
                        rest = rest.dropFirst(k + 1)
                        break
                    }
                    guard headerBytes.count < 79, (b >= 0x20 && b <= 0x7e) || b == 13 else { throw Failure.badHeader }
                    headerBytes.append(b)
                }
                if !headerDone { return [] }
            }
            pending.append(contentsOf: rest)
            while pending.count >= 3 {
                let base = pending.startIndex
                let type = pending[base]
                let len = Int(pending[base + 1]) | Int(pending[base + 2]) << 8
                switch type {
                case 0x41:
                    guard len >= 2, len <= MicStreamProtocol.maxAudioBytes, len % 2 == 0 else {
                        throw Failure.badPacket("audio length \(len)")
                    }
                case 0x4d, 0x55, 0x4b:
                    guard len == 0 else { throw Failure.badPacket("control length \(len)") }
                default: throw Failure.badPacket("type \(type)")
                }
                guard pending.count >= 3 + len else { break }
                let body = pending[(base + 3)..<(base + 3 + len)]
                switch type {
                case 0x41:
                    var s: [Int16] = []
                    s.reserveCapacity(len / 2)
                    var i = body.startIndex
                    while i < body.endIndex {
                        s.append(Int16(bitPattern: UInt16(body[i]) | UInt16(body[i + 1]) << 8)); i += 2
                    }
                    out.append(.audio(s))
                case 0x4d: out.append(.mute)
                case 0x55: out.append(.unmute)
                default: out.append(.keepalive)
                }
                pending = Data(pending.dropFirst(3 + len))
            }
            return out
        }
    }
}

enum MicPCM {
    /// Float (-1...1) to s16 with clipping and rounding.
    static func toInt16(_ x: Float) -> Int16 {
        guard x.isFinite else { return 0 }
        let v = (x * 32767).rounded()
        return Int16(max(-32768, min(32767, v)))
    }

    /// Output frames to allocate for converting inputFrames at inputRate to 16 kHz (with slack).
    static func outputCapacity(inputFrames: Int, inputRate: Double) -> Int {
        guard inputRate > 0 else { return 0 }
        return Int((Double(inputFrames) * MicStreamProtocol.sampleRate / inputRate).rounded(.up)) + 32
    }

    /// RMS level in dBFS (silence = -120).
    static func levelDBFS(_ samples: [Int16]) -> Double {
        guard !samples.isEmpty else { return -120 }
        var e = 0.0
        for s in samples { let x = Double(s) / 32768; e += x * x }
        let rms = (e / Double(samples.count)).squareRoot()
        return rms > 1e-6 ? max(-120, 20 * log10(rms)) : -120
    }

    /// Linear resampler to 16 kHz that keeps its phase across buffers. Used only when
    /// AVAudioConverter can't be made for the input format (it filters better).
    struct LinearResampler {
        let inputRate: Double
        private var pos = 0.0          // next output position, in input samples relative to `last`
        private var last: Float?

        init(inputRate: Double) { self.inputRate = inputRate }

        mutating func process(_ input: [Float]) -> [Int16] {
            guard inputRate > 0, !input.isEmpty else { return [] }
            // virtual stream: [last] + input; index 0 = last (if any)
            let src: [Float] = last.map { [$0] + input } ?? input
            let step = inputRate / MicStreamProtocol.sampleRate
            var out: [Int16] = []
            out.reserveCapacity(Int(Double(input.count) / step) + 2)
            while true {
                let i = Int(pos)
                if i + 1 >= src.count { break }
                let f = Float(pos - Double(i))
                out.append(MicPCM.toInt16(src[i] * (1 - f) + src[i + 1] * f))
                pos += step
            }
            // keep the last sample; re-base pos on it
            pos -= Double(src.count - 1)
            last = src.last
            return out
        }
    }

    /// Collects converted samples and hands out fixed 20 ms packets.
    struct Chunker {
        private var buffer: [Int16] = []
        mutating func push(_ samples: [Int16]) -> [[Int16]] {
            buffer.append(contentsOf: samples)
            var out: [[Int16]] = []
            let n = MicStreamProtocol.packetSamples
            while buffer.count >= n {
                out.append(Array(buffer[0..<n]))
                buffer.removeFirst(n)
            }
            return out
        }
        mutating func reset() { buffer.removeAll() }
    }
}

/// Half-duplex policy: listen through the headset, talk through the phone speaker,
/// never both. Pure: the class performs the actions it returns, in order.
struct MicDuplex {
    enum Phase: Equatable { case off, listening, talking }
    enum Event: Equatable {
        case enable, disable
        case speechWillStart, speechDidFinish
        case interruptionBegan, interruptionEnded
    }
    enum Action: Equatable {
        case configureSession        // .playAndRecord + Bluetooth HFP + defaultToSpeaker, active
        case routeToHeadset          // no speaker override, prefer the Bluetooth HFP input
        case routeToSpeaker          // overrideOutputAudioPort(.speaker): QNXplorer talks out loud
        case startCapture, stopCapture
        case sendMute, sendUnmute
        case releaseSession          // mic off and quiet: hand the session back to SpeechLink
    }

    private(set) var phase = Phase.off
    private(set) var interrupted = false
    private(set) var releasePending = false

    /// Audio packets should flow to the Pi.
    var streaming: Bool { phase == .listening && !interrupted }
    /// SpeechLink must route its speech through us (not its own .playback session).
    var ownsSession: Bool { phase != .off || releasePending }

    mutating func handle(_ event: Event) -> [Action] {
        switch (event, phase) {
        case (.enable, .off):
            phase = .listening
            releasePending = false
            return interrupted ? [.configureSession, .sendMute]
                               : [.configureSession, .routeToHeadset, .startCapture, .sendUnmute]
        case (.enable, _): return []

        case (.disable, .off): return []
        case (.disable, .listening):
            phase = .off
            return [.sendMute, .stopCapture, .releaseSession]
        case (.disable, .talking):
            phase = .off
            releasePending = true            // speech still playing: release when it ends
            return [.sendMute, .stopCapture]

        case (.speechWillStart, .listening):
            phase = .talking
            return interrupted ? [.routeToSpeaker] : [.sendMute, .stopCapture, .routeToSpeaker]
        case (.speechWillStart, _): return []

        case (.speechDidFinish, .talking):
            phase = .listening
            return interrupted ? [.routeToHeadset] : [.routeToHeadset, .startCapture, .sendUnmute]
        case (.speechDidFinish, .off):
            guard releasePending else { return [] }
            releasePending = false
            return [.releaseSession]
        case (.speechDidFinish, .listening): return []

        case (.interruptionBegan, _):
            guard !interrupted else { return [] }
            interrupted = true
            return phase == .listening ? [.sendMute, .stopCapture] : []
        case (.interruptionEnded, _):
            guard interrupted else { return [] }
            interrupted = false
            return phase == .listening ? [.configureSession, .routeToHeadset, .startCapture, .sendUnmute] : []
        }
    }
}
// END PHONE_MIC_PURE

final class PhoneMic: ObservableObject {
    @Published private(set) var enabled = false
    @Published private(set) var inputName = "—"
    @Published private(set) var usingHeadset = false
    @Published private(set) var piConnected = false
    @Published private(set) var levelDBFS = -120.0
    @Published private(set) var talking = false
    /// mute -> speaker ready (ms) and speech end -> first AirPods audio again (ms)
    @Published private(set) var talkSwitchMs: Int?
    @Published private(set) var listenSwitchMs: Int?
    @Published private(set) var summary = "Robot mic off"
    @Published private(set) var warning = ""
    @Published private(set) var droppedPackets = 0

    private var duplex = MicDuplex()
    var ownsSession: Bool { duplex.ownsSession }

    // network (all on q)
    private let q = DispatchQueue(label: "tars.phonemic")
    private var listener: NWListener?
    private var connection: NWConnection?
    private var inFlight = 0
    private var lastSent = Date.distantPast
    private var keepalive: DispatchSourceTimer?
    private var streamingOnQ = false
    private var dropped = 0

    // capture (tap thread + main)
    private let engine = AVAudioEngine()
    private var tapInstalled = false
    private var converter: AVAudioConverter?
    private var fallback: MicPCM.LinearResampler?
    private var chunker = MicPCM.Chunker()
    private let outFormat = AVAudioFormat(commonFormat: .pcmFormatInt16, sampleRate: MicStreamProtocol.sampleRate,
                                          channels: 1, interleaved: true)!
    private var lastLevelPublish = Date.distantPast
    private var resumeStartedAt: Date?
    private var observers: [NSObjectProtocol] = []

    init() {
        let nc = NotificationCenter.default
        observers.append(nc.addObserver(forName: AVAudioSession.routeChangeNotification, object: nil, queue: .main) { [weak self] n in
            self?.routeChanged(n)
        })
        observers.append(nc.addObserver(forName: AVAudioSession.interruptionNotification, object: nil, queue: .main) { [weak self] n in
            guard let self, let raw = n.userInfo?[AVAudioSessionInterruptionTypeKey] as? UInt,
                  let type = AVAudioSession.InterruptionType(rawValue: raw) else { return }
            self.perform(self.duplex.handle(type == .began ? .interruptionBegan : .interruptionEnded))
            self.warning = type == .began ? "Mic interrupted (call/Siri)" : ""
        })
        observers.append(nc.addObserver(forName: .AVAudioEngineConfigurationChange, object: engine, queue: .main) { [weak self] _ in
            // route/format changed under the engine (AirPods in/out): rebuild the tap
            guard let self, self.duplex.streaming else { return }
            self.stopCapture()
            self.startCapture()
        })
    }

    deinit { observers.forEach(NotificationCenter.default.removeObserver) }

    // MARK: public (main thread)

    func setEnabled(_ on: Bool) {
        guard on != enabled else { return }
        if on {
            AVAudioApplication.requestRecordPermission { granted in
                DispatchQueue.main.async {
                    guard granted else {
                        self.warning = "Microphone permission denied (Settings > MotionUSB > Microphone)"
                        self.enabled = false
                        return
                    }
                    self.enabled = true
                    self.warning = ""
                    self.startListener()
                    self.perform(self.duplex.handle(.enable))
                }
            }
        } else {
            enabled = false
            perform(duplex.handle(.disable))
            stopListener()
            summary = "Robot mic off"
        }
    }

    /// App foreground again: same as before backgrounding.
    func resume() { if enabled { startListener(); if duplex.phase == .off { perform(duplex.handle(.enable)) } } }

    /// App going to the background: everything stops (like the other listeners).
    func suspend() {
        guard enabled else { return }
        perform(duplex.handle(.disable))
        stopListener()
    }

    /// SpeechLink is about to speak (first utterance of a burst).
    func speechWillStart() {
        let t0 = Date()
        perform(duplex.handle(.speechWillStart))
        talking = true
        talkSwitchMs = Int(Date().timeIntervalSince(t0) * 1000)
    }

    /// SpeechLink's queue is empty and the synthesizer stopped.
    func speechDidFinish() {
        resumeStartedAt = Date()
        perform(duplex.handle(.speechDidFinish))
        talking = false
    }

    // MARK: actions

    private func perform(_ actions: [MicDuplex.Action]) {
        let session = AVAudioSession.sharedInstance()
        for action in actions {
            switch action {
            case .configureSession:
                do {
                    try session.setCategory(.playAndRecord, mode: .default,
                                            options: [.allowBluetoothHFP, .defaultToSpeaker, .duckOthers])
                    try session.setPreferredSampleRate(MicStreamProtocol.sampleRate)
                    try session.setActive(true)
                } catch { warning = "Audio session: \(error.localizedDescription)" }
            case .routeToHeadset:
                try? session.overrideOutputAudioPort(.none)
                if let hfp = session.availableInputs?.first(where: { $0.portType == .bluetoothHFP }) {
                    try? session.setPreferredInput(hfp)
                }
                updateRoute()
            case .routeToSpeaker:
                do { try session.overrideOutputAudioPort(.speaker) }
                catch { warning = "Speaker switch: \(error.localizedDescription)" }
            case .startCapture: startCapture()
            case .stopCapture: stopCapture()
            case .sendMute: setStreaming(false)
            case .sendUnmute: setStreaming(true)
            case .releaseSession:
                try? session.overrideOutputAudioPort(.none)
                try? session.setActive(false, options: [.notifyOthersOnDeactivation])
                levelDBFS = -120
            }
        }
        refreshSummary()
    }

    private func updateRoute() {
        let inputs = AVAudioSession.sharedInstance().currentRoute.inputs
        inputName = inputs.first?.portName ?? "—"
        usingHeadset = inputs.contains { $0.portType == .bluetoothHFP }
        if enabled && !usingHeadset && !inputs.isEmpty {
            warning = "AirPods not connected: using the phone's own mic"
        } else if usingHeadset, warning.hasPrefix("AirPods") {
            warning = ""
        }
    }

    private func routeChanged(_ n: Notification) {
        guard enabled else { return }
        let raw = n.userInfo?[AVAudioSessionRouteChangeReasonKey] as? UInt ?? 0
        let reason = AVAudioSession.RouteChangeReason(rawValue: raw)
        // AirPods came back while listening: prefer them again
        if reason == .newDeviceAvailable, duplex.streaming,
           let hfp = AVAudioSession.sharedInstance().availableInputs?.first(where: { $0.portType == .bluetoothHFP }) {
            try? AVAudioSession.sharedInstance().setPreferredInput(hfp)
        }
        if duplex.phase != .talking { updateRoute() }
        refreshSummary()
    }

    private func refreshSummary() {
        guard enabled else { summary = "Robot mic off"; return }
        let link = piConnected ? "Pi linked" : "waiting for Pi on \(MicStreamProtocol.port)"
        let state = duplex.interrupted ? "interrupted" : duplex.phase == .talking ? "muted (QNXplorer talking)" : "listening"
        summary = "Mic \(state) · \(inputName) · \(link)"
    }

    // MARK: capture

    private func startCapture() {
        guard !tapInstalled else { return }
        let input = engine.inputNode
        let inFormat = input.outputFormat(forBus: 0)
        guard inFormat.sampleRate > 0, inFormat.channelCount > 0 else {
            warning = "No microphone input available"
            return
        }
        converter = AVAudioConverter(from: inFormat, to: outFormat)
        converter?.downmix = true
        fallback = converter == nil ? MicPCM.LinearResampler(inputRate: inFormat.sampleRate) : nil
        chunker.reset()
        let frames = AVAudioFrameCount(max(256, Int(inFormat.sampleRate / 50)))     // ~20 ms
        input.installTap(onBus: 0, bufferSize: frames, format: inFormat) { [weak self] buffer, _ in
            self?.captured(buffer)
        }
        tapInstalled = true
        engine.prepare()
        do { try engine.start() } catch {
            warning = "Mic start: \(error.localizedDescription)"
            input.removeTap(onBus: 0)
            tapInstalled = false
        }
        updateRoute()
    }

    private func stopCapture() {
        guard tapInstalled else { return }
        engine.inputNode.removeTap(onBus: 0)
        engine.stop()
        tapInstalled = false
        DispatchQueue.main.async { self.levelDBFS = -120 }
    }

    /// Tap thread: convert to 16 kHz mono s16 and hand 20 ms packets to the network queue.
    private func captured(_ buffer: AVAudioPCMBuffer) {
        var samples: [Int16] = []
        if let converter {
            let cap = AVAudioFrameCount(MicPCM.outputCapacity(inputFrames: Int(buffer.frameLength),
                                                             inputRate: buffer.format.sampleRate))
            guard let out = AVAudioPCMBuffer(pcmFormat: outFormat, frameCapacity: cap) else { return }
            var given = false
            var error: NSError?
            converter.convert(to: out, error: &error) { _, status in
                if given { status.pointee = .noDataNow; return nil }
                given = true
                status.pointee = .haveData
                return buffer
            }
            if error != nil { return }
            if let p = out.int16ChannelData?[0] { samples = Array(UnsafeBufferPointer(start: p, count: Int(out.frameLength))) }
        } else if var resampler = fallback, let p = buffer.floatChannelData?[0] {
            samples = resampler.process(Array(UnsafeBufferPointer(start: p, count: Int(buffer.frameLength))))
            fallback = resampler
        }
        guard !samples.isEmpty else { return }
        let packets = chunker.push(samples)
        let now = Date()
        if now.timeIntervalSince(lastLevelPublish) > 0.1 {
            lastLevelPublish = now
            let level = MicPCM.levelDBFS(samples)
            let resumed = resumeStartedAt.map { Int(now.timeIntervalSince($0) * 1000) }
            if resumed != nil { resumeStartedAt = nil }
            DispatchQueue.main.async {
                self.levelDBFS = level
                if let resumed { self.listenSwitchMs = resumed }
            }
        }
        q.async { for p in packets { self.sendAudio(p) } }
    }

    // MARK: network (q)

    private func startListener() {
        q.async {
            guard self.listener == nil else { return }
            do {
                let params = piLinkParameters()
                params.allowLocalEndpointReuse = true
                let l = try NWListener(using: params, on: NWEndpoint.Port(rawValue: MicStreamProtocol.port)!)
                l.newConnectionHandler = { [weak self] c in self?.accept(c) }
                l.stateUpdateHandler = { [weak self] state in
                    if case .failed(let e) = state {
                        DispatchQueue.main.async { self?.warning = "Mic port \(MicStreamProtocol.port): \(e.localizedDescription)" }
                        self?.q.async { self?.listener?.cancel(); self?.listener = nil }
                    }
                }
                self.listener = l
                l.start(queue: self.q)
                self.startKeepalive()
            } catch {
                DispatchQueue.main.async { self.warning = "Mic port: \(error.localizedDescription)" }
            }
        }
    }

    private func stopListener() {
        q.async {
            self.keepalive?.cancel(); self.keepalive = nil
            self.listener?.cancel(); self.listener = nil
            self.connection?.cancel(); self.connection = nil
            self.inFlight = 0
            DispatchQueue.main.async { self.piConnected = false; self.refreshSummary() }
        }
    }

    /// One Pi at a time: the newest connection wins (a redial replaces a dead one).
    private func accept(_ c: NWConnection) {
        connection?.cancel()
        connection = c
        inFlight = 0
        c.stateUpdateHandler = { [weak self, weak c] state in
            guard let self, let c else { return }
            switch state {
            case .ready:
                var hello = MicStreamProtocol.header
                hello.append(MicStreamProtocol.encode(self.streamingOnQ ? .unmute : .mute))
                self.write(hello, on: c)
                DispatchQueue.main.async { self.piConnected = true; self.refreshSummary() }
                self.drain(c)
            case .failed, .cancelled:
                if self.connection === c {
                    self.connection = nil
                    DispatchQueue.main.async { self.piConnected = false; self.refreshSummary() }
                }
            default: break
            }
        }
        c.start(queue: q)
    }

    /// The Pi never writes; reading just notices when it hangs up.
    private func drain(_ c: NWConnection) {
        c.receive(minimumIncompleteLength: 1, maximumLength: 256) { [weak self] _, _, complete, error in
            if complete || error != nil { c.cancel(); return }
            self?.drain(c)
        }
    }

    private func write(_ data: Data, on c: NWConnection) {
        lastSent = Date()
        c.send(content: data, completion: .contentProcessed { _ in })
    }

    private func setStreaming(_ on: Bool) {
        q.async {
            self.streamingOnQ = on
            if let c = self.connection { self.write(MicStreamProtocol.encode(on ? .unmute : .mute), on: c) }
        }
    }

    private func sendAudio(_ samples: [Int16]) {
        guard streamingOnQ, let c = connection else { return }
        guard inFlight < MicStreamProtocol.maxInFlight else {
            dropped += 1
            if dropped % 50 == 1 { let d = dropped; DispatchQueue.main.async { self.droppedPackets = d } }
            return
        }
        inFlight += 1
        lastSent = Date()
        c.send(content: MicStreamProtocol.encode(.audio(samples)), completion: .contentProcessed { [weak self] _ in
            self?.q.async { self?.inFlight = max(0, (self?.inFlight ?? 1) - 1) }
        })
    }

    private func startKeepalive() {
        let t = DispatchSource.makeTimerSource(queue: q)
        t.schedule(deadline: .now() + 0.5, repeating: 0.5)
        t.setEventHandler { [weak self] in
            guard let self, let c = self.connection,
                  Date().timeIntervalSince(self.lastSent) >= MicStreamProtocol.keepaliveInterval * 0.8 else { return }
            self.write(MicStreamProtocol.encode(.keepalive), on: c)
        }
        keepalive?.cancel()
        keepalive = t
        t.resume()
    }
}

import SwiftUI

/// Compact robot-mic status: state, input device, Pi link, level meter, switch times.
struct MicStatusView: View {
    @ObservedObject var mic: PhoneMic
    var body: some View {
        VStack(alignment: .leading, spacing: 1) {
            HStack(spacing: 4) {
                Image(systemName: mic.talking ? "speaker.wave.2.fill" : mic.usingHeadset ? "airpodspro" : "mic.fill")
                Text(mic.summary).lineLimit(1).minimumScaleFactor(0.7)
            }
            if mic.enabled {
                HStack(spacing: 4) {
                    // -60 dBFS .. 0 dBFS
                    let fill = max(0, min(1, (mic.levelDBFS + 60) / 60))
                    GeometryReader { g in
                        ZStack(alignment: .leading) {
                            Capsule().fill(.gray.opacity(0.3))
                            Capsule().fill(mic.talking ? Color.gray : Color.green).frame(width: g.size.width * fill)
                        }
                    }.frame(width: 60, height: 5)
                    Text(String(format: "%.0f dB", mic.levelDBFS)).monospacedDigit()
                    if let t = mic.talkSwitchMs { Text("→spk \(t) ms") }
                    if let l = mic.listenSwitchMs { Text("→mic \(l) ms") }
                    if mic.droppedPackets > 0 { Text("drop \(mic.droppedPackets)").foregroundStyle(.orange) }
                }
            }
            if !mic.warning.isEmpty { Text(mic.warning).foregroundStyle(.orange).lineLimit(2) }
        }
        .accessibilityElement(children: .combine)
    }
}
