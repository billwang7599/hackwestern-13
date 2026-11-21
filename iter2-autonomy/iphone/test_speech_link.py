#!/usr/bin/env python3
"""Run the real speech-link parser, line framer and queue policy on macOS.

Extracts the code between BEGIN/END SPEECH_LINK_PURE in SpeechLink.swift (the
same code the app runs), compiles it with checks, and runs it. No iOS needed.
"""
from pathlib import Path
import subprocess
import tempfile

source = Path(__file__).with_name('SpeechLink.swift').read_text()
pure = source.split('// BEGIN SPEECH_LINK_PURE', 1)[1].split('// END SPEECH_LINK_PURE', 1)[0]
checks = r'''
var checks = 0
func check(_ condition: @autoclosure () -> Bool, _ name: String) {
    precondition(condition(), "FAILED: " + name); checks += 1
}
func p(_ s: String) -> SpeechCommand? { SpeechLinkProtocol.parse(Data(s.utf8)) }
func say(_ s: String, _ pr: SpeechPriority = .normal) -> SpeechCommand { .say(SpeechRequest(text: s, priority: pr)) }

// SAY lines
check(p("SAY hello there") == say("hello there"), "plain SAY")
check(p("say hello") == say("hello"), "SAY is case-insensitive")
check(p("  SAY   I remember   2 things.  \r") == say("I remember 2 things."), "whitespace collapsed, CR stripped")
check(p("SAY") == nil && p("SAY ") == nil && p("SAY    ") == nil, "empty SAY rejected")
check(p("SAYhello") == nil, "SAY needs a space")
check(p("SAY tab\there\u{7}bell") == say("tab here bell"), "control chars become spaces")
check(p("SAY héllo – café 👋") == say("héllo – café 👋"), "unicode kept")
check(p("HUSH") == .hush && p("hush") == .hush, "HUSH")
// JSON lines
check(p(#"{"type":"say","text":"Stopping."}"#) == say("Stopping."), "JSON normal default")
check(p(#"{"type":"say","text":"Stop!","priority":"urgent"}"#) == say("Stop!", .urgent), "JSON urgent")
check(p(#"{"type":"say","text":"x","priority":"NORMAL"}"#) == say("x"), "JSON priority case-insensitive")
check(p(#"{"type":"say","text":"x","priority":"loud"}"#) == nil, "unknown priority rejected")
check(p(#"{"type":"say","text":"x","priority":3}"#) == nil, "non-string priority rejected")
check(p(#"{"type":"say","text":"  "}"#) == nil, "blank JSON text rejected")
check(p(#"{"type":"say","text":42}"#) == nil, "non-string text rejected")
check(p(#"{"type":"say"}"#) == nil, "missing text rejected")
check(p(#"{"type":"status","text":"x"}"#) == nil, "other JSON types ignored")
check(p(#"{"type":"hush"}"#) == .hush, "JSON hush")
check(p(#"{"type":"say","text":"a\nb"}"#) == say("a b"), "escaped newline in JSON text flattened")
// garbage
check(p("") == nil && p("hello") == nil && p("GOTO trash can") == nil, "garbage lines rejected")
check(p("{not json") == nil && p("[1,2]") == nil && p("{}") == nil, "bad JSON rejected")
check(SpeechLinkProtocol.parse(Data([0x53,0x41,0x59,0x20,0xff,0xfe])) == nil, "invalid UTF-8 rejected")
// oversize
let long = String(repeating: "word ", count: 100)
if case .say(let r)? = p("SAY " + long) {
    check(r.text.count <= SpeechLinkProtocol.maxTextCharacters, "text capped at 300")
    check(r.text.hasSuffix("word"), "cap lands on a word boundary")
} else { check(false, "long SAY still spoken") }
let unbroken = String(repeating: "a", count: 400)
if case .say(let r)? = p("SAY " + unbroken) { check(r.text.count == 300, "unbroken text hard-capped") } else { check(false, "unbroken parsed") }
check(SpeechLinkProtocol.parse(Data(("SAY " + String(repeating: "x", count: 1100)).utf8)) == nil, "line over 1024 bytes rejected")

// framer
var f = SpeechLineFramer()
check(f.feed(Data("SAY a\nSAY b\n".utf8)) == [.line(Data("SAY a".utf8)), .line(Data("SAY b".utf8))], "two lines in one chunk")
check(f.feed(Data("SAY hel".utf8)) == [], "partial line buffered")
check(f.feed(Data("lo\n".utf8)) == [.line(Data("SAY hello".utf8))], "partial line completed")
check(f.feed(Data("\n".utf8)) == [.line(Data())], "empty line surfaces as empty")
var g = SpeechLineFramer()
check(g.feed(Data(repeating: 65, count: 1000)) == [], "1000 bytes buffered")
check(g.feed(Data(repeating: 65, count: 100)) == [.oversize], "overflow reported once")
check(g.feed(Data(repeating: 65, count: 5000)) == [], "rest of oversize line discarded silently")
check(g.feed(Data("tail\nSAY ok\n".utf8)) == [.line(Data("SAY ok".utf8))], "recovers at next newline")
var h = SpeechLineFramer()
check(h.feed(Data(repeating: 66, count: 1024) + Data("\n".utf8)) == [.line(Data(repeating: 66, count: 1024))], "exactly 1024 bytes accepted")
check(h.feed(Data(repeating: 66, count: 1025) + Data("\nSAY x\n".utf8)) == [.oversize, .line(Data("SAY x".utf8))], "1025 bytes rejected in one chunk")
var k = SpeechLineFramer()
var collected: [SpeechLineFramer.Item] = []
for byte in Data("SAY one\nSAY two\n".utf8) { collected += k.feed(Data([byte])) }
check(collected == [.line(Data("SAY one".utf8)), .line(Data("SAY two".utf8))], "byte-at-a-time framing")

// queue policy
var q = SpeechQueue()
let n = { (s: String) in SpeechRequest(text: s, priority: .normal) }
let u = { (s: String) in SpeechRequest(text: s, priority: .urgent) }
check(q.push(n("1")) == .init(interrupt: false, dropped: 0), "normal does not interrupt")
for i in 2...5 { _ = q.push(n(String(i))) }
check(q.pending.map(\.text) == ["1","2","3","4","5"], "queue holds 5")
check(q.push(n("6")) == .init(interrupt: false, dropped: 1), "6th drops one")
check(q.pending.map(\.text) == ["2","3","4","5","6"], "oldest dropped")
check(q.pop()?.text == "2" && q.pending.count == 4, "FIFO pop")
check(q.push(u("STOP")) == .init(interrupt: true, dropped: 4), "urgent interrupts and clears pending")
check(q.pending.map(\.text) == ["STOP"], "urgent is next")
_ = q.push(n("after"))
check(q.pending.map(\.text) == ["STOP","after"], "normal queues behind urgent")
_ = q.push(u("again"))
check(q.pending.map(\.text) == ["again"], "newer urgent replaces older urgent")
q.clear()
check(q.pop() == nil && q.pending.isEmpty, "clear/empty pop")
print("Passed \(checks) speech link parser/framer/queue checks")
'''
with tempfile.TemporaryDirectory(prefix='hw13-speech-link-') as directory:
    directory = Path(directory)
    program = directory / 'main.swift'
    program.write_text('import Foundation\n' + pure + checks)
    binary = directory / 'speech_link_checks'
    subprocess.run(['xcrun', 'swiftc', '-module-cache-path', str(directory / 'module-cache'),
                    str(program), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
