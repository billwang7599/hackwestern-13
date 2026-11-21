# Persistent native inference worker

Build on QNX using `sh pi/edge-ai/ai-build.sh`, after installing QNX's
`onnxruntime-dev` and `libjpeg-turbo-dev` packages and their runtime dependencies.
The build uses native `clang++`, not a host binary. `ORT_INCLUDE` can override the
header location. One inference thread is the default; `--threads 2` is supported
for explicit benchmark comparisons. Both ORT inter/intra-op spinning are off.

Smoke test:

```sh
pi/edge-ai/build/ai_detector --model pi/edge-ai/models/yolox_nano.onnx \
  --image pi/edge-ai/models/dog.jpg --repeat 5
```

With no `--image`, stdin contains jobs: big-endian uint32 job ID, big-endian
uint32 JPEG length, then exactly that many JPEG bytes. JPEGs are limited to
2,000,000 bytes and decoded dimensions to 2048×2048. One complete input produces
one newline-delimited JSON result on stdout. Logs go to stderr. The parent must
allow only one inference in flight, replace pending RGB frames instead of
queuing them, and associate job IDs with their original phone timestamp/session.
A truncated or oversized framed input ends the process; an invalid JPEG produces
an error result for that ID and does not desynchronize the next frame.

Successful result:

```json
{"id":7,"width":640,"height":853,"model":"YOLOX-Nano","backend":"ONNX Runtime CPU","runtime":"1.24.4","inference_ms":125.0,"processing_ms":130.0,"objects":[{"label":"chair","confidence":0.81,"bbox":[0.2,0.3,0.25,0.5]}]}
```

Numbers above are an example, not measured performance. `bbox` is normalized
`[left,top,width,height]` with a **top-left** origin in the source JPEG. It is not
Vision's bottom-left coordinate system. All outputs are finite and clipped to
the source image, with at most 20 objects. On recoverable failure the result is
`{"id":7,"error":"...","objects":[]}`. Results must never be applied to a
newer camera frame just because the detector finished late.

The worker deliberately has no camera socket, LiDAR input, safety control or
motor backend. Process scheduling/CPU placement and failure supervision belong
to the separate QNX service. A paused/crashed worker must leave sensor processing
and its stale-data stop condition running.

## Relative-depth worker

`build/ai_depth --model models/midas-small.onnx --threads 1` uses the identical
input framing and isolated process. Its stdout result adds `kind` equal to
`relative_inverse_depth`, `depth_width=64`, `depth_height=64`, and
`inverse_depth` containing4096 finite nonnegative floats. Width and height refer
to the input JPEG; grid coordinates map across that full image without padding
or cropping. The output uses6significant digits and stays below60,000bytes for
the parent's bounded local UDP result channel. Errors return `id`, `error`, and
an empty `inverse_depth` array. A malformed result is not a usable depth map.

These predictions are asynchronous, approximate visual memory only. They are
not measurements, have no units of meters, and never substitute for the LiDAR
safety channel. Source-frame metadata must remain attached by the parent; no
worker processing timestamp can substitute for the original capture time.
