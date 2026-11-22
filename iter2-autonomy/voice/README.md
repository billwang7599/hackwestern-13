# iter2-autonomy/voice/: offline speech commands for QNXplorer (QNX native)

"TARS, go to the trash can." → the robot drives there. "Stop!" → it stops. Everything runs
**on the Pi 5 under QNX 8.0, in C/C++ built with `qcc`**. No laptop, no internet, no Python on the robot.

```
phone mic (AirPods, TCP 5577) or USB mic ─► 16 kHz mono         src/audio.c
        ─► energy VAD, 20 ms frames, segments ≤ 3 s           src/vad.c
        ─► whisper.cpp tiny.en-q5_1, 2 threads                src/stt.c
        ─► text → intent (generated C header, ~2 µs)           intent/intent.c + intent_model.h
             1. normalise   2. wake word "TARS"   3. SAFETY STOP pass
             4. classifier  5. slots: amount ("ninety", "half a metre"), object ("the bin" → trash can)
        ─► TCP 5555 tars_motor:  S · MOVE ±mm · TURN ±deg · STATUS · LED · BEEP
        ─► TCP 5570 go-to:       GOTO · RECALL <object> · SCAN · FORGET · MEMORY SAY · CANCEL   src/main.c
        ─► TCP 5573 phone:       SAY <sentence> (best effort; urgent "Stopping." after S + CANCEL)
```

Python is only used off the robot: training (`intent/train.py`), `intent/try.py` and the tests.

| You say | Robot gets |
|---|---|
| "TARS, go forward" / "…go ahead two steps" / "…forward half a metre" | `MOVE 300` / `MOVE 200` / `MOVE 500` |
| "TARS, back up" / "…reverse 30 centimetres" | `MOVE -300` / `MOVE -300` |
| "TARS, turn left" / "…turn right ninety" / "…hang a left" | `TURN -30` / `TURN 90` / `TURN -30` |
| "TARS, turn around" / "…do a spin" | `TURN 180` / `TURN 360` |
| "TARS, go to the trash can" / "…find the door" / "…head over to that bin" | `GOTO trash can` / `GOTO door` / `GOTO trash can` → 5570 |
| "TARS, go back to where the bin was" / "…return to the chair" | `RECALL trash can` / `RECALL chair` → 5570 (map-guided, uses the stored position) + `BEEP 1` |
| "TARS, look around" / "…what do you see" | `SCAN` → 5570 |
| "TARS, forget everything" / "…forget the bin" | `FORGET` / `FORGET trash can` → 5570, `BEEP 2` |
| "TARS, what do you remember" / "…what's in your memory" | `MEMORY SAY` → 5570; prints `[memory] I remember 2 things: …`, `BEEP n` (one per object, max 4; `LED no` for none), `SAY <sentence>` → phone 5573 |
| "TARS, status" | `STATUS` (logged) + `BEEP 2` |
| **"Stop" / "halt" / "freeze" / "whoa"** (no wake word needed) · "TARS, no / wait / hold on" | **`S` to tars_motor, then `CANCEL` to go-to** |
| anything else after "TARS" / unknown object ("go to the kitchen") | nothing moves; `LED no` + `BEEP 3` |
| speech without "TARS" | ignored |

**Exact-move keepalive:** `tars_motor` on main aborts a `MOVE`/`TURN` after 500 ms without a
command, so a small thread in `tars_voice` PINGs (on its own connection) while a voice move runs.
**Phone mic (the default, `input = phone`):** the robot has no microphone, so the iPhone on the car
streams the user's AirPods to `tars_voice` (iPhone app: controls → "Robot mic (AirPods)"). The phone
listens on 5577 (forwarded to the Pi by iproxy, `service.py`), `tars_voice` dials it and redials with a
0.5–5 s backoff. Protocol `MIC1` (spec in `src/audio.h`): header line `MIC1 16000 1 s16le`, then packets
`A` audio / `M` mute / `U` unmute / `K` keepalive. It's hands-free: the same VAD, whisper, "TARS" wake word
and stop-first path as a wired mic. While QNXplorer speaks a `SAY` from the phone speaker the phone sends `M`
(half duplex): what it hears then is dropped, and the segment in progress is finished at the mute. Link
down / muted: `audio_read` returns within 100 ms with nothing, so the loop (LED off, talk button) keeps
running. Bad data or 3 s of silence (no `K`) drops the link. `phone:PORT` / `phone:HOST:PORT` for other
addresses; `input = qnx` for a USB mic. Fake phone for the Pi or a Mac:
`python3 iter2-autonomy/iphone/tools/mic_stream.py cmd.wav` (or `-` for raw stdin). Test:
`make test-phone` (38 checks, fake phone + real `tars_motor`). **Not yet run on the Pi/QNX or with the
real phone.** Caveat: a spoken "stop" while QNXplorer is talking is not heard; the other stops are unaffected.
**Speech link:** `say_host`/`say_port`/`say_ms` in `voice.conf` (127.0.0.1:5573, 150 ms connect
timeout, `say_port = 0` = off). Failures are ignored.
**Network input (laptop mission control, `iter2-autonomy/mission/`):** `net_host`/`net_port`
(127.0.0.1:5574, `net_port = 0` = off). `HEAR <text>` and `AUDIO <bytes>` + 16 kHz mono PCM/WAV
(≤ 15 s) go through the same `intent_decide()` and stop-first path as the mic, no wake word
(push-to-talk). Clips are transcribed on their own thread, so a typed stop never waits for whisper.
`WATCH` gives JSON events (`heard` with src/text/intent/prob, `sent`, `memory`, `say`). The mic keeps
working at the same time. Tests: `make test-net` (25 checks). `build/stt_cli MODEL FILE.wav` (host only)
prints just the transcript, for the laptop's `--laptop-stt`.

Defaults: 30° and 300 mm. Before a manual move (`MOVE`/`TURN`) voice asks the go-to controller
`STATUS`: if it's not `state=idle`, it sends `CANCEL` and waits up to 1.5 s for the `OK` (goto sends
its `S` to `tars_motor` before replying), so its steering doesn't fight the move. An idle go-to gets no
`CANCEL`, because goto answers every `CANCEL` with an `S` that could land after the move and abort it.
Any go-to reply that misses `goto_reply_ms` drops and reopens the link, so a late reply is never read as
the answer to the next line. A recognised command gets `BEEP 1`. Test: `make test-goto-link` (the real
goto + `tars_motor`, including a goto stalled for 0.8 s).

## Quick start

On the laptop (Mac or Linux):
```sh
iter2-autonomy/voice/fetch_whisper.sh            # pinned whisper.cpp v1.9.5 (d1be6fd) -> iter2-autonomy/voice/third_party/ (not committed)
iter2-autonomy/voice/get_model.sh                # ggml-tiny.en-q5_1.bin, 31 MB, SHA-256 checked  (--base for base.en-q5_1)
make -C iter2-autonomy/voice host                # Mac/Linux build: tars_voice_host, intent_cli
make -C iter2-autonomy/voice test                # classifier tests + end-to-end (needs macOS `say`, numpy)
python3 iter2-autonomy/voice/intent/try.py       # type sentences, see intent / confidence / command
```
For the Pi, follow **docs/PI_TEST.md → Step 7: voice**. Run **`mic_test` first**: if the mic doesn't work under QNX, nothing else here matters.

**On the Pi itself (no SDP, no Docker, no cmake):** the robot builds its own `tars_voice` with its clang.
whisper.cpp/ggml is compiled straight from the pinned sources (CPU backend only, static, Armv8.2-A +
dotprod + fp16):
```sh
iter2-autonomy/voice/fetch_whisper.sh && iter2-autonomy/voice/get_model.sh   # or copy third_party/ + models/ from the laptop
make -C iter2-autonomy/voice native -j4          # -> build/native/tars_voice, build/native/mic_test (a few minutes)
cd iter2-autonomy/voice && ./build/native/tars_voice -c voice.conf --dry     # prints what it hears, sends nothing
TARS_ENABLE_VOICE=1 TARS_ENABLE_GOTO=1 python3 iter2-autonomy/dashboard/service.py start   # the real thing
```
`service.py` picks up `build/native/tars_voice` and runs it from `iter2-autonomy/voice/`, so `voice.conf`
and `models/` are used in place. The default input is the phone (AirPods, or the phone's own mic when none
are connected) on 5577. The laptop (bench `--target pi`) only relays text or a recorded clip to 5574 via
`mission_relay`.

Cross-build from a laptop with the SDP (the older route):
```sh
make -C iter2-autonomy/voice mic_test            # QNX build in docker/run.sh (needs ~/qnx800), small: no whisper
make -C iter2-autonomy/voice                     # QNX: build/qnx/tars_voice (+ mic_test)
make -C iter2-autonomy/voice deploy PI_HOST=qnxuser@<pi-ip>
```
Live mic on a Mac (dev only): `ffmpeg -f avfoundation -i ":0" -ac 1 -ar 16000 -f s16le - | iter2-autonomy/voice/build/tars_voice_host --input raw:- --dry`

## Adding phrases and retraining (for teammates)

1. Open **`iter2-autonomy/voice/intent/phrases.txt`**. Each line is `intent<TAB>phrase`:
   ```
   forward	scoot ahead
   goto	roll over to the {o}
   unknown	is this the line for food
   ```
   Write it how a person says it, without "TARS" (that's added automatically), lowercase, no punctuation.
   `{n}` = a turn amount, `{d}` = a distance, `{o}` = an object. A phrase may only be under one intent.
   Intents: `forward back left right turn_around stop goto spin status unknown`.
2. New object, or a new name for one? Edit **`iter2-autonomy/voice/intent/objects.txt`** (`canonical<TAB>alias,alias`).
   Keep it in sync with `iter2-autonomy/goto/objects.txt`; it uses the same format.
3. Run **`python3 iter2-autonomy/voice/intent/train.py`**. It reads both files, adds simulated whisper mistakes
   ("for word", "turn lift", "stars" for TARS, dropped/merged words, typos), trains, writes
   `intent_model.h`, rebuilds the C checker and runs all classifier tests. It ends with
   `RETRAIN OK` or `TESTS FAIL`. Stop recall must stay 100%. About 15 s.
4. Check your sentence: `python3 iter2-autonomy/voice/intent/try.py "tars scoot ahead"`.
5. Commit `phrases.txt`, `objects.txt` and `intent_model.h`, then rebuild and redeploy `tars_voice`
   (the model is compiled in).

## The intent classifier

- **Safety pass first** (`safety_stop`): any word within edit distance 1 of *stop / halt / freeze / whoa*
  ("stob", "stopped", "hault", "frees", "breeze"…) → `S`, even without the wake word. After the wake word,
  *no / nope / wait / hold / enough / cancel / brake / stay* also stop. Nothing later can override this.
  A small exclusion list keeps *step, top, half, shop…* from stopping. The classifier can add stops
  (prob ≥ 0.30 for `stop` → `S`) but can never remove one.
- **Model:** hashed character 2–4-grams + whole words (4096 bins, FNV-1a, L2-normalised), multinomial
  logistic regression (numpy, Adam, L2 1e-4). Numbers become one placeholder token first (`num`,
  `n180`, `n360`), so "turn right one hundred and eighty degrees" looks like "turn right n180 degrees".
  The amount is the slot parser's job. Weights go into `intent_model.h` (~450 KB of `float`s);
  `intent.py` parses that same header, so Python and C use identical weights.
- **Confidence cutoff:** picked automatically on validation data, never below 0.52 (currently 0.52): +1 per correct
  command, −4 per wrong one. Below it: nothing moves, `LED no` + `BEEP 3`.
- **Slots (C, `intent_parse_amount` / `intent_find_object`):** digits, number words, "one eighty",
  "one hundred and twenty", "half a metre", "one and a half feet", "a bit", "two steps" (100 mm each),
  units deg/m/cm/mm/ft/in. A bare number means: ≤ 10 → steps, 11–200 → cm, > 200 → mm. MOVE is clamped
  to ±1000 mm and TURN to 360°, the same limits as `tars_motor`. Objects: the longest alias wins
  ("garbage can" before "can"), plus plurals, 1-edit typos of 5+ letter words, and merged words
  ("trashcan").

### Accuracy (Mac, `make -C iter2-autonomy/voice test-intent`)

**Held-out phrasings**: test templates hashed out of training, so the model has **never seen these
phrasings**, only the vocabulary. 3090 sentences with simulated whisper errors (about 40% with their
articles dropped, "go to bottle"):

| intent | test sentences | precision | recall |
|---|---|---|---|
| forward | 210 | 0.99 | 0.94 |
| back | 360 | 0.96 | 0.89 |
| left | 120 | 0.92 | 1.00 |
| right | 210 | 0.96 | 0.99 |
| turn_around | 150 | 0.90 | 1.00 |
| stop | 150 | 0.82 | 1.00 |
| goto | 270 | 0.82 | 0.99 |
| spin | 120 | 0.86 | 0.86 |
| status | 210 | 0.78 | 0.81 |
| scan | 180 | 0.94 | 0.64 |
| forget | 150 | 0.96 | 0.77 |
| memory | 240 | 0.84 | 0.96 |
| recall | 90 | 0.72 | 0.87 |
| unknown | 630 | 0.90 | 0.82 |
| **all** | **3090** | | **accuracy 0.887** |

Confusion matrix (classifier only, before thresholds):

| true \ predicted | fwd | back | left | right | around | stop | goto | spin | status | scan | forget | mem | recall | unk |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| **fwd** | 198 | 2 | 1 | · | · | · | · | · | · | · | · | · | · | 9 |
| **back** | · | 319 | · | · | · | 1 | 2 | · | · | · | · | · | 29 | 9 |
| **left** | · | · | 120 | · | · | · | · | · | · | · | · | · | · | · |
| **right** | · | · | 1 | 208 | · | · | · | · | · | · | · | · | · | 1 |
| **around** | · | · | · | · | 150 | · | · | · | · | · | · | · | · | · |
| **stop** | · | · | · | · | · | 150 | · | · | · | · | · | · | · | · |
| **goto** | 1 | 1 | · | · | · | · | 266 | 1 | · | · | · | · | 1 | · |
| **spin** | · | · | · | · | · | · | · | 103 | · | · | · | · | · | 17 |
| **status** | · | · | · | · | · | · | · | · | 171 | · | · | 26 | · | 13 |
| **scan** | · | · | · | · | 3 | · | 21 | · | 28 | 116 | 3 | 3 | · | 6 |
| **forget** | · | · | · | · | · | 30 | · | 1 | · | 2 | 116 | 1 | · | · |
| **mem** | · | · | · | · | · | · | · | · | 3 | 3 | 1 | 230 | · | 3 |
| **recall** | · | 2 | · | · | · | · | 10 | · | · | · | · | · | 78 | · |
| **unk** | 2 | 8 | 9 | 8 | 14 | 1 | 24 | 15 | 16 | 3 | 1 | 14 | · | 515 |

After the threshold, on the same set: **59.1% correct command, 2.2% wrong command, 39% rejected**
(LED no, "say it again"). Most wrong commands are safe ones: "start fresh" (forget) heard as stop, goto/recall swaps (same
object, live search vs map)
or chatter read as `spin`. The model is deliberately cautious on new phrasings: most misses are
status/unknown phrasings it has never seen, and they are rejected, not acted on.

| Test | Result |
|---|---|
| Hand-written realistic whisper transcripts (`intent/data/realistic.tsv`, not used in training) | **112/114** exact, **0 wrong motions** (both misses are safe rejections: "go bag" 0.51, "where is the exit" 0.34) |
| **Stop recall** (every stop sentence, generated + hand-written) | **1131/1131 = 100%** |
| Python ↔ C parity (3951 sentences incl. fuzz + regressions, wake required and not) | **3951/3951 identical**, both modes |
| Decision latency (normalise + safety + classifier + slots) | **C 2.5 µs**, Python 63 µs per sentence (M4 Pro). Pi 5 estimate ~5–10 µs |

### End to end: speech → whisper → intent → real `tars_motor` (Mac, `make -C iter2-autonomy/voice e2e`)

21 commands spoken by macOS `say` (2 voices), joined into one stream with pauses. The stream runs
through `tars_voice_host` (whisper tiny.en-q5_1, 2 threads) into the real `iter1-wasd/pi/build/tars_motor_host
-b stub` and a fake go-to controller.

| Condition | Samantha | Daniel | Wrong motions |
|---|---|---|---|
| clean | **21/21** | **21/21** | 0 |
| white noise, 20 dB SNR | 18/21 | 18/21 | 0 (1 right command, wrong distance: "two steps" heard as "to specs" → 300 mm) |
| crowd chatter + noise, 15 dB SNR | 4/21 | 4/21 | 0 |
| crowd chatter + noise, 10 dB SNR | 3/21 | 3/21 | 0 |

Every stop clip stops in clean and white noise, and every `S` is followed by `CANCEL` on 5570. The
go-to controller received exactly the `GOTO` lines sent. A real-time stdin test also checks that the
**talk button** works: "Turn left." without the wake word is ignored, then after a press it gives `TURN -30`.

**Crowd chatter is the weak spot.** With competing speech at similar loudness there's no quiet gap,
so segments run the full 3 s and whisper transcribes the crowd. It fails safe (nothing moves), but
commands are missed. For the demo floor: **use a close-talk USB mic (headset or lavalier) and/or the
talk button.**

### Speed and CPU (budget: 2 threads, leave cores for YOLO)

| | Mac M4 Pro (measured) | Pi 5 (estimate, not measured) |
|---|---|---|
| whisper tiny.en-q5_1 per ≤3 s segment, `audio_ctx` 512 | mean 153 ms, max 185 ms | ~0.6–1.2 s (A76 has ~4–6× less per-core NEON throughput than M4; plus YOLO contention) |
| same, full 30 s window (`audio_ctx -1`) | 479 ms | ~2–4 s |
| intent decision | 1.8 µs | ~10 µs |
| between commands | VAD only: ~0% CPU | VAD only: < 1% of one core |
| model load / peak RAM | 30 ms / 120 MB RSS (measured) | < 1 s / ~120 MB (estimate) |

`audio_ctx` (the encoder window) is sized to the segment, with a minimum of 512. That's about 3×
faster than whisper's default 30 s window. Below about 384, tiny.en starts repeating the sentence.
tiny.en can still loop ("Tars, turn left, turn left, turn left, …"), which turns a clean command into a
rejection. Two fixes in `src/stt.c`: whisper's **temperature fallback** (`temperature_inc 0.2`, entropy
check 2.4) re-decodes only a decode that looks like a loop, and the **`stt_dedupe` filter** drops repeated
sentences and collapses a 1–6-word run repeated back to back to one copy. The filter only deletes copies,
so a stop word can't vanish (the old filter cut everything after the first repeated sentence, so
"Turn left. Turn left. Stop!" lost the stop). On 1140 `say` clips (`realistic.tsv`, 5 voices, fast/slow
speech, white noise 15 dB, babble 10 dB; Mac, load 6–10): loops **32 → 0**, correct 982 → 986, wrong
motions 4 → 4, stops caught 177 → 179 of 210, whisper time +3%. Test: `make test-stt`.
**Measure on the Pi:** the `[heard]` log line has `stt=…ms` for every segment.

`tars_voice` runs at normal priority, and whisper never touches `tars_motor`'s SCHED_RR loop.
If the Pi is too slow with YOLO running, try `threads = 3`, or `get_model.sh --base` for accuracy
(2–3× slower).

## Robustness features

- **Wake word, fuzzy:** "TARS", "Tars," "Stars", "Cars", "Tires", "Taurus", "tar's", "Tons", "TOS"…
  Whisper misspells it a lot, so the first (or second, after hey/ok) word only has to be close.
- **Wake carry-over:** "TARS" … pause … "turn left" works. A segment that is just the wake word arms the
  next 4 s. This matters because in noise the VAD often splits after "TARS,".
- **Talk button:** a press on the Uno's button (`BTN TEACH` → `btn_teach` in STATUS, polled every
  250 ms) beeps and arms listening for 4 s with no wake word. `button_talk = 1` in `voice.conf`.
- **VAD noise floor:** minimum statistics (the quietest 20 ms frame of the last 1.5 s), so it follows
  fan and crowd noise. A fixed floor deadlocked in steady noise. Segments: ≥150 ms of speech (a short
  "Stop!"), ≤3 s, 300 ms pre-roll.
- **Bare fragments are rejected:** "TARS, turn." / "move it" / "rotate" / "go to" → LED no, never a 180°
  turn. A rule (`intent_is_bare`), not the model: only verbs with no default direction (turn/rotate/move)
  plus fillers. Bare "TARS, go!" still means forward. Tested in `test_intent.py` (regressions section).
- **"one" is a word, not 1°:** "the left one" / "that one" aren't amounts, and a bare turn amount under 10
  ("turn left two") gives the default 30°. "five degrees" still gives 5°.
- **Link handling:** `tars_motor` and the go-to controller each reconnect every 1–2 s. `S` and
  `CANCEL` get one immediate retry. If the go-to controller is down, stops still reach `tars_motor`
  first. If `tars_voice` dies, `start_voice.sh` restarts it (1→10 s backoff), and `tars_motor`'s
  disconnect handling stops only a move that voice started.

## Files

| Path | What |
|---|---|
| `src/main.c` | `tars_voice`: config, loop, routing to 5555/5570, wake carry-over, talk button |
| `src/audio.c` | capture: QNX io-audio (`snd_pcm_*` plugin API), WAV file, raw stdin |
| `src/vad.c`, `src/stt.c` | VAD; whisper.cpp wrapper + repeat-loop filter |
| `intent/intent.c/.h` | C inference: normalise, wake, safety, features, softmax, slots, objects |
| `intent/intent_model.h` | **generated** weights + thresholds + object table |
| `intent/intent.py` | Python reference (spec) for the same pipeline; used by training/tests/try |
| `intent/phrases.txt`, `intent/objects.txt` | **edit these** to change what it understands |
| `intent/gen_data.py`, `intent/train.py`, `intent/try.py` | data generation, one-command retrain, playground |
| `qnx/mic_test.c` | 10-minute mic check on QNX (list devices, record 3 s, level meter, verdict) |
| `qnx/start_voice.sh`, `voice.conf` | supervisor/boot script, config |
| `cmake/qnx-aarch64.cmake` | whisper.cpp cross-compile toolchain (qcc, Cortex-A76 `armv8.2-a+dotprod+fp16`) |
| `tests/test_intent.py`, `tests/test_e2e.py` | classifier tests; end-to-end speech tests |
| `tests/test_stt.py`, `tests/test_goto_link.py` | repeat-loop filter; voice moves vs the real goto + `tars_motor` |

## Not tested yet (be honest at the demo)

- **Nothing has run on the Pi / QNX.** SDP 8.0 isn't installed on the build Mac, so the QNX build
  of whisper.cpp (`cmake/qnx-aarch64.cmake`), `tars_voice` and `mic_test` has **never been compiled**.
  ggml's only Linux-specific code (CPU pinning) is behind `#ifdef __linux__`. Likely first fixes:
  linker flags (`-lsocket`, `-lc++`), and `GGML_CPU_ARM_ARCH` if qcc's gcc rejects `+dotprod+fp16`
  (drop to `armv8.2-a`).
- **io-audio capture** (`src/audio.c`, the `__QNX__` part) was written from the QNX io-audio API docs
  and only syntax-checked against a stub header. Struct field names may need small fixes against the
  real `<sys/asoundlib.h>`. Whether QNX 8.0 on the Pi 5 sees the C270's mic at all is **unknown**, and
  that's what `mic_test` is for. If `snd_pcm` doesn't work, any recorder that writes raw s16le 16 kHz
  to stdout can feed `tars_voice --input raw:-`.
- **Real voices and real rooms:** the end-to-end audio is synthetic (`say`) with synthetic noise. No
  human voice or real mic has been tested.
- **Pi latency and CPU** numbers above are estimates.
- **The go-to controller protocol** (`GOTO <canonical name>` / `CANCEL`, one line each, on 5570, reply
  optional) is this branch's assumption. The `kothar/goto` worker hadn't published its interface yet.
  Canonical names contain spaces (`GOTO trash can`).
- `start_voice.sh` boot integration: which startup file the QNX image uses is unverified.
