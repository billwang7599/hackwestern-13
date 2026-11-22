#!/usr/bin/env python3
"""Run the robot-mic (PhoneMic.swift) pure logic on macOS: MIC1 framing, the Pi parser
mirror, PCM conversion/resampling/level math and the half-duplex state machine.

Extracts the code between BEGIN/END PHONE_MIC_PURE in PhoneMic.swift (the same code the
app runs), compiles it with checks, and runs it. No iOS, no audio hardware. The wire bytes
are also compared with the Python encoder that iter2-autonomy/voice/tests/test_phone_mic.py
feeds into the real C parser (tars_voice), so the phone and the Pi agree on the format.
"""
from pathlib import Path
import struct
import subprocess
import tempfile

source = Path(__file__).with_name('PhoneMic.swift').read_text()
pure = source.split('// BEGIN PHONE_MIC_PURE', 1)[1].split('// END PHONE_MIC_PURE', 1)[0]
checks = r'''
var checks = 0
func check(_ condition: @autoclosure () -> Bool, _ name: String) {
    precondition(condition(), "FAILED: " + name); checks += 1
}
func hex(_ d: Data) -> String { d.map { String(format: "%02x", $0) }.joined() }
typealias P = MicStreamProtocol

// ---- wire format
check(P.port == 5577, "port 5577")
check(P.header == Data("MIC1 16000 1 s16le\n".utf8), "header line")
check(hex(P.encode(.mute)) == "4d0000" && hex(P.encode(.unmute)) == "550000" && hex(P.encode(.keepalive)) == "4b0000", "control packets")
check(hex(P.encode(.audio([1, -1, 0x1234, -32768, 32767]))) == "410a00" + "0100" + "ffff" + "3412" + "0080" + "ff7f", "audio packet, s16 little endian")
let twenty = [Int16](repeating: 7, count: P.packetSamples)
check(P.encode(.audio(twenty)).count == 3 + 640, "20 ms packet = 643 bytes")
let big = (0..<5000).map { Int16(truncatingIfNeeded: $0) }
let bigWire = P.encode(.audio(big))
check(bigWire.count == (3 + 8192) + (3 + 1808), "audio over 8192 bytes split in two packets")
// the sample stream the Python side builds too (printed for comparison)
var sample = P.header
sample.append(P.encode(.unmute)); sample.append(P.encode(.audio([0, 100, -100]))); sample.append(P.encode(.mute)); sample.append(P.encode(.keepalive))
print("WIRE " + hex(sample))

// ---- decoder (mirror of audio.c)
var d = P.Decoder()
let stream = P.header + P.encode(.unmute) + P.encode(.audio(twenty)) + P.encode(.mute) + P.encode(.keepalive) + bigWire
var got: [P.Packet] = []
for b in stream { got += try! d.feed(Data([b])) }            // one byte at a time
check(got.count == 6, "byte-by-byte decode: 6 packets (got \(got.count))")
check(got[0] == .unmute && got[1] == .audio(twenty) && got[2] == .mute && got[3] == .keepalive, "decoded in order")
if case .audio(let a) = got[4], case .audio(let b) = got[5] { check(a + b == big, "split audio rejoins") } else { check(false, "split audio decoded") }
var d2 = P.Decoder()
check((try? d2.feed(stream))?.count == 6, "whole stream at once")
func fails(_ data: Data) -> Bool { var x = P.Decoder(); return (try? x.feed(data)) == nil }
check(fails(Data("HELLO 44100 2 f32\n".utf8)), "wrong header rejected")
check(fails(Data([0x00, 0xff, 0x10])), "binary noise rejected")
check(fails(Data(repeating: 0x4d, count: 200)), "endless header rejected")
check(fails(P.header + Data([0x5a, 0, 0])), "unknown type rejected")
check(fails(P.header + Data([0x41, 3, 0, 1, 2, 3])), "odd audio length rejected")
check(fails(P.header + Data([0x41, 0, 0])), "empty audio rejected")
check(fails(P.header + Data([0x41, 0x60, 0xea])), "oversize audio rejected")
check(fails(P.header + Data([0x4d, 2, 0, 1, 2])), "control with payload rejected")
var d3 = P.Decoder()
check((try? d3.feed(Data("MIC1 16000 1 s16le\r\n".utf8))) == [], "CRLF header accepted")

// ---- PCM
check(MicPCM.toInt16(0) == 0 && MicPCM.toInt16(1) == 32767 && MicPCM.toInt16(-1) == -32767, "float -> s16")
check(MicPCM.toInt16(2) == 32767 && MicPCM.toInt16(-2) == -32768, "clipped")
check(MicPCM.toInt16(.nan) == 0 && MicPCM.toInt16(.infinity) == 0, "NaN/inf -> 0")
check(MicPCM.toInt16(0.5) == 16384, "rounded")
check(MicPCM.outputCapacity(inputFrames: 4800, inputRate: 48000) == 1600 + 32, "48k capacity")
check(MicPCM.outputCapacity(inputFrames: 1024, inputRate: 44100) == 372 + 32, "44.1k capacity rounds up")
check(MicPCM.outputCapacity(inputFrames: 100, inputRate: 0) == 0, "bad rate -> 0")
check(MicPCM.levelDBFS([]) == -120 && MicPCM.levelDBFS([0, 0, 0]) == -120, "silence -120 dBFS")
check(abs(MicPCM.levelDBFS([32767, -32767, 32767, -32767])) < 0.01, "full-scale square 0 dBFS")
let sine = (0..<1600).map { MicPCM.toInt16(0.5 * Float(sin(2 * Double.pi * 1000 * Double($0) / 16000))) }
check(abs(MicPCM.levelDBFS(sine) - (-9.03)) < 0.1, "0.5 sine = -9.03 dBFS (\(MicPCM.levelDBFS(sine)))")

// resampler: 48 kHz 1 kHz sine, in odd-sized buffers, equals one-shot and keeps the pitch
let n48 = 48000
let src48 = (0..<n48).map { Float(0.5 * sin(2 * Double.pi * 1000 * Double($0) / 48000)) }
var one = MicPCM.LinearResampler(inputRate: 48000)
let whole = one.process(src48)
var many = MicPCM.LinearResampler(inputRate: 48000)
var pieces: [Int16] = []
var i = 0, k = 0
let sizes = [1, 7, 480, 1023, 4096, 333]
while i < n48 {
    let end = min(i + sizes[k % sizes.count], n48); k += 1
    pieces += many.process(Array(src48[i..<end])); i = end
}
check(abs(whole.count - 16000) <= 1, "48k -> 16k count (\(whole.count))")
check(pieces == whole, "buffer boundaries don't change the output")
let expect = (0..<whole.count).map { MicPCM.toInt16(0.5 * Float(sin(2 * Double.pi * 1000 * Double($0) / 16000))) }
check(zip(whole, expect).allSatisfy { abs(Int($0) - Int($1)) <= 2 }, "decimated sine matches a 16 kHz sine")
var ident = MicPCM.LinearResampler(inputRate: 16000)
let same = ident.process([0.1, 0.2, 0.3]) + ident.process([0.4, 0.5])
check(same == [0.1, 0.2, 0.3, 0.4].map { MicPCM.toInt16(Float($0)) }, "16k passes through (last sample held for the next buffer)")
var r441 = MicPCM.LinearResampler(inputRate: 44100)
var c441 = 0
for _ in 0..<100 { c441 += r441.process([Float](repeating: 0, count: 441)).count }
check(abs(c441 - 16000) <= 1, "44.1 kHz -> 16k count over 1 s (\(c441))")

var ch = MicPCM.Chunker()
check(ch.push([Int16](repeating: 1, count: 500)).map(\.count) == [320], "chunker: one 20 ms packet")
check(ch.push([Int16](repeating: 2, count: 140)) == [[Int16](repeating: 1, count: 180) + [Int16](repeating: 2, count: 140)], "chunker carries the remainder")
check(ch.push([Int16](repeating: 3, count: 960)).count == 3, "chunker: 3 packets")
ch.reset()
check(ch.push([Int16](repeating: 4, count: 319)).isEmpty, "reset drops the remainder")

// ---- half duplex
var m = MicDuplex()
check(!m.streaming && !m.ownsSession, "starts off")
check(m.handle(.speechWillStart) == [] && m.handle(.speechDidFinish) == [], "mic off: SpeechLink keeps its own session")
check(m.handle(.enable) == [.configureSession, .routeToHeadset, .startCapture, .sendUnmute], "enable: AirPods in, stream")
check(m.streaming && m.ownsSession && m.phase == .listening, "listening")
check(m.handle(.enable) == [], "enable twice: nothing")
check(m.handle(.speechWillStart) == [.sendMute, .stopCapture, .routeToSpeaker], "talk: mute first, then the speaker")
check(!m.streaming && m.phase == .talking, "talking: not streaming")
check(m.handle(.speechWillStart) == [], "second utterance: nothing")
check(m.handle(.speechDidFinish) == [.routeToHeadset, .startCapture, .sendUnmute], "done: AirPods back, unmute last")
check(m.handle(.speechDidFinish) == [], "finish twice: nothing")
// interruption (phone call, Siri)
check(m.handle(.interruptionBegan) == [.sendMute, .stopCapture], "interruption mutes")
check(m.handle(.interruptionBegan) == [] && !m.streaming, "interruption twice")
check(m.handle(.speechWillStart) == [.routeToSpeaker], "talking during an interruption: no double mute")
check(m.handle(.speechDidFinish) == [.routeToHeadset], "...and no capture until it ends")
check(m.handle(.interruptionEnded) == [.configureSession, .routeToHeadset, .startCapture, .sendUnmute], "interruption over: resume")
check(m.handle(.interruptionEnded) == [], "ended twice")
// interruption ending while talking: capture comes back when speech ends
_ = m.handle(.speechWillStart)
_ = m.handle(.interruptionBegan)
check(m.handle(.interruptionEnded) == [], "interruption ends mid-speech: wait")
check(m.handle(.speechDidFinish) == [.routeToHeadset, .startCapture, .sendUnmute], "then resume on speech end")
// disable
check(m.handle(.disable) == [.sendMute, .stopCapture, .releaseSession], "disable while listening")
check(!m.ownsSession && m.handle(.disable) == [], "disable twice")
_ = m.handle(.enable)
_ = m.handle(.speechWillStart)
check(m.handle(.disable) == [.sendMute, .stopCapture], "disable while talking: keep the session for the speech")
check(m.ownsSession && m.phase == .off, "release pending")
check(m.handle(.speechDidFinish) == [.releaseSession], "release when the speech ends")
check(!m.ownsSession && m.handle(.speechDidFinish) == [], "released once")
// every path that unmutes starts capture first; every path that talks mutes before the speaker
var fuzz = MicDuplex()
var lcg: UInt64 = 42
let events: [MicDuplex.Event] = [.enable, .disable, .speechWillStart, .speechDidFinish, .interruptionBegan, .interruptionEnded]
var talkingSpeaker = false, okUnmute = true, okMute = true, okSpeaker = true, okTalk = true
for _ in 0..<20000 {
    lcg = lcg &* 6364136223846793005 &+ 1442695040888963407
    let acts = fuzz.handle(events[Int(lcg >> 33) % events.count])
    if let u = acts.firstIndex(of: .sendUnmute), !acts[..<u].contains(.startCapture) { okUnmute = false }
    if let s = acts.firstIndex(of: .routeToSpeaker), let mu = acts.firstIndex(of: .sendMute), mu > s { okMute = false }
    if acts.contains(.routeToSpeaker) { talkingSpeaker = true }
    if acts.contains(.routeToHeadset) { talkingSpeaker = false }
    if fuzz.streaming && talkingSpeaker { okSpeaker = false }
    if fuzz.phase == .talking && fuzz.streaming { okTalk = false }
}
check(okUnmute, "fuzz 20000 events: every unmute comes after starting capture")
check(okMute, "fuzz: mute always before the speaker switch")
check(okSpeaker, "fuzz: never streaming while routed to the speaker")
check(okTalk, "fuzz: talking never streams")
print("Passed \(checks) phone mic framing/PCM/duplex checks")
'''


def py_pkt(kind: bytes, payload: bytes = b"") -> bytes:      # same as voice/tests/test_phone_mic.py
    return kind + struct.pack("<H", len(payload)) + payload


with tempfile.TemporaryDirectory(prefix='hw13-phone-mic-') as directory:
    directory = Path(directory)
    program = directory / 'main.swift'
    program.write_text('import Foundation\n' + pure + checks)
    binary = directory / 'phone_mic_checks'
    subprocess.run(['xcrun', 'swiftc', '-module-cache-path', str(directory / 'module-cache'),
                    str(program), '-o', str(binary)], check=True)
    run = subprocess.run([str(binary)], capture_output=True, text=True)
    out = run.stdout
    print(out, end='')
    if run.returncode != 0:
        print(run.stderr[-2000:])
        raise SystemExit(f"phone mic checks failed (exit {run.returncode})")
    wire = next(l.split()[1] for l in out.splitlines() if l.startswith('WIRE '))
    want = (b"MIC1 16000 1 s16le\n" + py_pkt(b"U") + py_pkt(b"A", struct.pack("<3h", 0, 100, -100))
            + py_pkt(b"M") + py_pkt(b"K")).hex()
    assert wire == want, f"Swift wire bytes differ from the Python/C test encoder:\n {wire}\n {want}"
    print("Passed 1 cross-check: Swift encoder bytes == the encoder the C parser test uses")
