import Foundation
import Network
import AVFoundation

// TARS speech link: the robot has no speaker, so goto/voice on the Pi send
// short sentences here (TCP 5573, forwarded over USB by iproxy) and the phone
// says them with on-device AVSpeechSynthesizer voices. Additive and isolated:
// it shares no state with the sensor/status listeners.
//
// Lines (newline-delimited, UTF-8, any number of clients):
//   SAY <text>
//   {"type":"say","text":"...","priority":"normal|urgent"}
//   HUSH            or {"type":"hush"}   stop speaking and clear the queue
// Each line gets a one-line reply: "OK say", "OK hush" or "ERR <reason>".

// BEGIN SPEECH_LINK_PURE
enum SpeechPriority: String, Equatable { case normal, urgent }

struct SpeechRequest: Equatable {
    let text: String
    let priority: SpeechPriority
}

enum SpeechCommand: Equatable {
    case say(SpeechRequest)
    case hush
}

enum SpeechLinkProtocol {
    static let port: UInt16 = 5573
    /// Longest accepted line in bytes, excluding the newline. Longer lines are dropped whole.
    static let maxLineBytes = 1024
    /// Spoken text is cut to this many characters (on a word boundary when possible).
    static let maxTextCharacters = 300
    static let maxClients = 8

    /// Parse one line (without its newline). Returns nil for garbage.
    static func parse(_ line: Data) -> SpeechCommand? {
        guard line.count <= maxLineBytes, var text = String(data: line, encoding: .utf8) else { return nil }
        if text.hasSuffix("\r") { text.removeLast() }
        text = text.trimmingCharacters(in: .whitespaces)
        if text.hasPrefix("{") {
            guard let object = try? JSONSerialization.jsonObject(with: Data(text.utf8)) as? [String: Any],
                  let type = object["type"] as? String else { return nil }
            switch type.lowercased() {
            case "hush": return .hush
            case "say":
                guard let raw = object["text"] as? String, let spoken = clean(raw) else { return nil }
                var priority = SpeechPriority.normal
                if let value = object["priority"] {
                    guard let name = value as? String, let parsed = SpeechPriority(rawValue: name.lowercased()) else { return nil }
                    priority = parsed
                }
                return .say(SpeechRequest(text: spoken, priority: priority))
            default: return nil
            }
        }
        let upper = text.uppercased()
        if upper == "HUSH" { return .hush }
        guard upper.hasPrefix("SAY "), let spoken = clean(String(text.dropFirst(4))) else { return nil }
        return .say(SpeechRequest(text: spoken, priority: .normal))
    }

    /// Control characters become spaces, whitespace collapses, and the result is
    /// capped at maxTextCharacters. Empty text is nil.
    static func clean(_ raw: String) -> String? {
        var scalars = String.UnicodeScalarView()
        var lastWasSpace = true
        for scalar in raw.unicodeScalars {
            let isSpace = CharacterSet.whitespacesAndNewlines.contains(scalar) || CharacterSet.controlCharacters.contains(scalar)
            if isSpace {
                if !lastWasSpace { scalars.append(" ") }
                lastWasSpace = true
            } else {
                scalars.append(scalar)
                lastWasSpace = false
            }
        }
        var text = String(scalars).trimmingCharacters(in: .whitespaces)
        if text.count > maxTextCharacters {
            text = String(text.prefix(maxTextCharacters))
            if let space = text.lastIndex(of: " "), text.distance(from: text.startIndex, to: space) >= maxTextCharacters / 2 {
                text = String(text[..<space])
            }
        }
        return text.isEmpty ? nil : text
    }
}

/// Splits a TCP byte stream into lines. A line longer than maxLineBytes is
/// dropped whole (reported once as an oversize line) and buffering never grows
/// past that bound.
struct SpeechLineFramer {
    enum Item: Equatable { case line(Data), oversize }
    private var buffer = Data()
    private var discarding = false

    mutating func feed(_ data: Data) -> [Item] {
        var items: [Item] = []
        var rest = data[...]
        while let newline = rest.firstIndex(of: 10) {
            let piece = rest[rest.startIndex..<newline]
            rest = rest[rest.index(after: newline)...]
            if discarding {
                discarding = false
            } else if buffer.count + piece.count > SpeechLinkProtocol.maxLineBytes {
                buffer.removeAll()
                items.append(.oversize)
            } else {
                buffer.append(contentsOf: piece)
                items.append(.line(buffer))
                buffer = Data()
            }
        }
        if !discarding {
            if buffer.count + rest.count > SpeechLinkProtocol.maxLineBytes {
                buffer.removeAll()
                discarding = true
                items.append(.oversize)
            } else {
                buffer.append(contentsOf: rest)
            }
        }
        return items
    }
}

/// What waits to be spoken. Normal requests queue (oldest dropped past
/// capacity). Urgent requests interrupt the current utterance and replace
/// everything still pending, so a stale sentence never follows an urgent one.
struct SpeechQueue {
    static let capacity = 5
    private(set) var pending: [SpeechRequest] = []

    struct Outcome: Equatable {
        let interrupt: Bool
        let dropped: Int
    }

    mutating func push(_ request: SpeechRequest) -> Outcome {
        switch request.priority {
        case .urgent:
            let dropped = pending.count
            pending = [request]
            return Outcome(interrupt: true, dropped: dropped)
        case .normal:
            pending.append(request)
            let dropped = max(0, pending.count - SpeechQueue.capacity)
            if dropped > 0 { pending.removeFirst(dropped) }
            return Outcome(interrupt: false, dropped: dropped)
        }
    }

    mutating func pop() -> SpeechRequest? { pending.isEmpty ? nil : pending.removeFirst() }
    mutating func clear() { pending.removeAll() }
}
// END SPEECH_LINK_PURE

final class SpeechLink: NSObject, ObservableObject, AVSpeechSynthesizerDelegate {
    @Published private(set) var lastSpoken = ""
    @Published private(set) var lastSpokenAt: Date?
    @Published private(set) var clientCount = 0
    @Published private(set) var summary = "Speech link stopped"

    private var listener: NWListener?
    private var clients: [ObjectIdentifier: (connection: NWConnection, framer: SpeechLineFramer)] = [:]
    private let synthesizer = AVSpeechSynthesizer()
    private var queue = SpeechQueue()
    private var current: AVSpeechUtterance?
    private var sessionActive = false

    override init() {
        super.init()
        synthesizer.delegate = self
    }

    func start() {
        guard listener == nil else { return }
        do {
            let parameters = NWParameters.tcp
            parameters.allowLocalEndpointReuse = true
            let listener = try NWListener(using: parameters, on: NWEndpoint.Port(rawValue: SpeechLinkProtocol.port)!)
            self.listener = listener
            listener.stateUpdateHandler = { [weak self] state in
                switch state {
                case .ready: self?.summary = "Speech link listening on \(SpeechLinkProtocol.port)"
                case .failed(let error):
                    self?.stop(); self?.summary = "Speech link: \(error.localizedDescription)"
                default: break
                }
            }
            listener.newConnectionHandler = { [weak self] connection in self?.accept(connection) }
            listener.start(queue: .main)
        } catch { summary = "Speech link: \(error.localizedDescription)" }
    }

    func stop() {
        listener?.cancel(); listener = nil
        for client in clients.values { client.connection.cancel() }
        clients.removeAll(); clientCount = 0
        hush()
        summary = "Speech link stopped"
    }

    private func accept(_ connection: NWConnection) {
        guard clients.count < SpeechLinkProtocol.maxClients else { connection.cancel(); return }
        let id = ObjectIdentifier(connection)
        clients[id] = (connection, SpeechLineFramer())
        clientCount = clients.count
        connection.stateUpdateHandler = { [weak self, weak connection] state in
            guard let self, let connection else { return }
            switch state {
            case .ready: self.receive(connection)
            case .failed, .cancelled: self.drop(connection)
            default: break
            }
        }
        connection.start(queue: .main)
    }

    private func drop(_ connection: NWConnection) {
        let id = ObjectIdentifier(connection)
        guard clients.removeValue(forKey: id) != nil else { return }
        connection.cancel()
        clientCount = clients.count
    }

    private func receive(_ connection: NWConnection) {
        connection.receive(minimumIncompleteLength: 1, maximumLength: 4096) { [weak self] data, _, complete, error in
            guard let self, var client = self.clients[ObjectIdentifier(connection)] else { return }
            if let data, !data.isEmpty {
                let items = client.framer.feed(data)
                self.clients[ObjectIdentifier(connection)] = client
                for item in items {
                    let reply: String
                    switch item {
                    case .oversize: reply = "ERR line too long"
                    case .line(let line):
                        if line.allSatisfy({ $0 == 32 || $0 == 9 || $0 == 13 }) { continue }
                        switch SpeechLinkProtocol.parse(line) {
                        case .say(let request)?: self.enqueue(request); reply = "OK say"
                        case .hush?: self.hush(); reply = "OK hush"
                        case nil: reply = "ERR expected SAY <text> or {\"type\":\"say\",\"text\":...}"
                        }
                    }
                    connection.send(content: Data((reply + "\n").utf8), completion: .contentProcessed { _ in })
                }
            }
            if complete || error != nil { self.drop(connection); return }
            self.receive(connection)
        }
    }

    // MARK: speaking (main queue only)

    private func enqueue(_ request: SpeechRequest) {
        let outcome = queue.push(request)
        if outcome.interrupt, current != nil {
            current = nil
            synthesizer.stopSpeaking(at: .immediate)
        }
        speakNext()
    }

    private func hush() {
        queue.clear()
        current = nil
        if synthesizer.isSpeaking { synthesizer.stopSpeaking(at: .immediate) }
        endAudioSession()
    }

    private func speakNext() {
        guard current == nil else { return }
        guard let request = queue.pop() else { endAudioSession(); return }
        beginAudioSession()
        let utterance = AVSpeechUtterance(string: request.text)
        utterance.voice = AVSpeechSynthesisVoice(language: "en-US")
        current = utterance
        lastSpoken = request.text
        lastSpokenAt = Date()
        synthesizer.speak(utterance)
    }

    /// Playback category so speech is heard with the ring/silent switch on; other
    /// audio is ducked while TARS talks and restored afterwards.
    private func beginAudioSession() {
        guard !sessionActive else { return }
        let session = AVAudioSession.sharedInstance()
        do {
            try session.setCategory(.playback, mode: .spokenAudio, options: [.duckOthers])
            try session.setActive(true)
            sessionActive = true
        } catch { summary = "Speech audio session: \(error.localizedDescription)" }
    }

    private func endAudioSession() {
        guard sessionActive, !synthesizer.isSpeaking else { return }
        try? AVAudioSession.sharedInstance().setActive(false, options: [.notifyOthersOnDeactivation])
        sessionActive = false
    }

    private func finished(_ utterance: AVSpeechUtterance) {
        DispatchQueue.main.async {
            // A cancel for an utterance we already replaced (urgent) is ignored;
            // one arriving after HUSH just lets the audio session end.
            if utterance === self.current { self.current = nil } else if self.current != nil { return }
            self.speakNext()
        }
    }

    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer, didFinish utterance: AVSpeechUtterance) { finished(utterance) }
    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer, didCancel utterance: AVSpeechUtterance) { finished(utterance) }
}
