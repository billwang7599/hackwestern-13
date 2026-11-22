import SwiftUI
import Speech
import AVFoundation

/// Hold-to-talk voice commands. The phone only turns speech into text (on-device when the
/// recognizer supports it) and sends it to the Pi as {"op":"voice","text":...} on the control
/// link. On the Pi, headless.py passes it to tars_voice as `HEAR <text>`, so the same intent
/// classifier as the robot mic decides (MOVE/TURN/GOTO/SCAN/MEMORY/...). No wake word needed.
/// A stop word is sent the moment it shows up in the partial transcript, plus a halt.
final class VoiceCommander: ObservableObject {
    @Published private(set) var listening = false
    @Published private(set) var transcript = ""
    @Published private(set) var lastSent = ""
    @Published private(set) var problem = ""

    /// Set by the view: sends one command object to the Pi.
    var send: (([String: Any]) -> Void)?

    private let recognizer = SFSpeechRecognizer(locale: Locale(identifier: "en-US"))
    private let engine = AVAudioEngine()
    private var request: SFSpeechAudioBufferRecognitionRequest?
    private var task: SFSpeechRecognitionTask?
    private var stopSent = false
    private var finishing: DispatchWorkItem?

    static let stopWords: Set<String> = ["stop", "halt", "freeze", "whoa"]

    static func containsStop(_ text: String) -> Bool {
        text.lowercased().split(whereSeparator: { !$0.isLetter }).contains { stopWords.contains(String($0)) }
    }

    func begin() {
        guard !listening else { return }
        problem = ""
        SFSpeechRecognizer.requestAuthorization { status in
            DispatchQueue.main.async {
                guard status == .authorized else {
                    self.problem = "Speech recognition is off for this app (Settings → Privacy → Speech Recognition)."
                    return
                }
                AVAudioApplication.requestRecordPermission { granted in
                    DispatchQueue.main.async {
                        if granted { self.start() } else { self.problem = "Microphone access is off for this app." }
                    }
                }
            }
        }
    }

    private func start() {
        guard let recognizer, recognizer.isAvailable else {
            problem = "Speech recognizer unavailable right now."
            return
        }
        do {
            let session = AVAudioSession.sharedInstance()
            try session.setCategory(.playAndRecord, mode: .default,
                                    options: [.allowBluetoothHFP, .defaultToSpeaker, .duckOthers])
            try session.setActive(true)
            let req = SFSpeechAudioBufferRecognitionRequest()
            req.shouldReportPartialResults = true
            if recognizer.supportsOnDeviceRecognition { req.requiresOnDeviceRecognition = true }
            req.taskHint = .confirmation
            let input = engine.inputNode
            input.removeTap(onBus: 0)
            input.installTap(onBus: 0, bufferSize: 1024, format: input.outputFormat(forBus: 0)) { buffer, _ in
                req.append(buffer)
            }
            engine.prepare()
            try engine.start()
            request = req
            transcript = ""
            stopSent = false
            listening = true
            task = recognizer.recognitionTask(with: req) { [weak self] result, error in
                DispatchQueue.main.async {
                    guard let self else { return }
                    if let result {
                        self.transcript = result.bestTranscription.formattedString
                        if !self.stopSent && Self.containsStop(self.transcript) {
                            self.stopSent = true
                            self.send?(["op": "halt"])
                            self.send?(["op": "voice", "text": "stop"])
                            self.lastSent = "stop"
                        }
                        if result.isFinal { self.finish() }
                    } else if error != nil {
                        self.finish()
                    }
                }
            }
        } catch {
            problem = "Microphone unavailable: \(error.localizedDescription)"
            teardown()
        }
    }

    /// Button released: stop recording, wait briefly for the final transcript, then send it.
    func end() {
        guard listening else { return }
        request?.endAudio()
        stopEngine()
        let work = DispatchWorkItem { [weak self] in self?.finish() }
        finishing = work
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.2, execute: work)
    }

    private func finish() {
        guard listening else { return }
        let text = transcript.trimmingCharacters(in: .whitespacesAndNewlines)
        teardown()
        // A stop already went out the moment it was heard; don't send it twice.
        if !text.isEmpty && !stopSent { sendText(String(text.prefix(200))) }
        else if text.isEmpty && !stopSent { problem = "Didn't catch that." }
    }

    func sendText(_ text: String) {
        if Self.containsStop(text) { send?(["op": "halt"]) }
        send?(["op": "voice", "text": text])
        lastSent = text
    }

    private func stopEngine() {
        if engine.isRunning { engine.stop() }
        engine.inputNode.removeTap(onBus: 0)
    }

    private func teardown() {
        finishing?.cancel()
        finishing = nil
        stopEngine()
        task?.cancel()
        task = nil
        request = nil
        listening = false
    }
}

/// Voice command card: hold to talk, or tap a sample phrase. What the Pi's classifier made
/// of it comes back in the status (`voice`: text / state / reply).
struct VoiceCommandPanel: View {
    @ObservedObject var pi: PiStatusServer
    @ObservedObject var voice: VoiceCommander
    /// The robot mic (AirPods stream) owns the audio input while it's on.
    let micBusy: Bool

    static let samples = ["stop", "go forward", "back up", "turn left", "turn right ninety",
                          "turn around", "look around", "go to the trash can", "what do you remember",
                          "forget everything"]

    private var piVoice: [String: Any] { pi.latestStatus["voice"] as? [String: Any] ?? [:] }

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Voice commands").font(.title2.bold())
            Text("Hold and speak, no \"TARS\" needed. The Pi's voice classifier decides what it means; \"stop\" stops at once. Moves still need the LiDAR guard to allow them.")
                .font(.caption).foregroundStyle(.secondary)
            Text(voice.listening ? "Listening… release to send" : "Hold to talk")
                .font(.body.bold()).foregroundStyle(.white)
                .frame(maxWidth: .infinity).padding(.vertical, 14)
                .background(voice.listening ? Color.red : Color.accentColor, in: RoundedRectangle(cornerRadius: 10))
                .opacity(pi.connected && !micBusy ? 1 : 0.4)
                .gesture(DragGesture(minimumDistance: 0)
                    .onChanged { _ in if !voice.listening && pi.connected && !micBusy { voice.begin() } }
                    .onEnded { _ in voice.end() })
                .accessibilityAddTraits(.isButton)
            if micBusy {
                Text("Turn off \"Robot mic (AirPods)\" to use hold-to-talk; the robot mic already hears you.")
                    .font(.caption).foregroundStyle(.orange)
            }
            if !voice.transcript.isEmpty {
                Text("Heard: \(voice.transcript)").font(.callout)
            }
            if !voice.problem.isEmpty {
                Text(voice.problem).font(.caption).foregroundStyle(.orange)
            }
            ScrollView(.horizontal, showsIndicators: false) {
                HStack {
                    ForEach(Self.samples, id: \.self) { phrase in
                        Button(phrase) { voice.sendText(phrase) }
                            .buttonStyle(.bordered)
                            .tint(phrase == "stop" ? .red : .accentColor)
                            .disabled(!pi.connected)
                    }
                }
            }
            if let text = piVoice["text"] as? String {
                let state = piVoice["state"] as? String ?? "sent"
                Text("Pi: \"\(text)\" → \(piVoice["reply"] as? String ?? "waiting…")")
                    .font(.caption.monospaced())
                    .foregroundStyle(state == "error" ? Color.orange : Color.secondary)
                    .lineLimit(3)
            } else if !voice.lastSent.isEmpty {
                Text("Sent \"\(voice.lastSent)\", waiting for the Pi").font(.caption).foregroundStyle(.secondary)
            }
        }
        .frame(maxWidth: .infinity, alignment: .leading).padding(14)
        .background(Color.secondary.opacity(0.08), in: RoundedRectangle(cornerRadius: 14))
        .onAppear { voice.send = { pi.command($0) } }
    }
}
