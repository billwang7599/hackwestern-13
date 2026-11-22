"""Checks for service.py's TARS_DEMO=1 preflight + memory clearing, on a throwaway copy of the layout."""
import importlib.util, os, shutil, sys, tempfile
from pathlib import Path

repo = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2]
tmp = Path(tempfile.mkdtemp())
dash = tmp / "iter2-autonomy" / "dashboard"
dash.mkdir(parents=True)
shutil.copy(repo / "iter2-autonomy/dashboard/service.py", dash / "service.py")
nav = tmp / "iter2-autonomy" / "goto"
(nav / "build").mkdir(parents=True)
(nav / "models").mkdir()
(nav / "goto.conf").write_text("memory_path = memory.txt   # note\n")
spec = importlib.util.spec_from_file_location("svc", dash / "service.py")
svc = importlib.util.module_from_spec(spec); spec.loader.exec_module(svc)

ok = fail = 0
def check(c, m):
    global ok, fail
    print(("ok   " if c else "FAIL ") + m); ok += bool(c); fail += not c

for k in ("TARS_ENABLE_GOTO", "TARS_ENABLE_VOICE", "TARS_MOTOR_BACKEND", "TARS_MOTOR_DEVICE", "TARS_BENCH_TOKEN", "TARS_VOICE_DIR"):
    os.environ.pop(k, None)
p = svc.demo_preflight()
check(any("stub" in x for x in p) and any("detect" in x for x in p) and any("tars_voice" in x for x in p)
      and any("TARS_BENCH_TOKEN" in x for x in p), f"nothing set: refuses with {len(p)} problems")
check(os.environ["TARS_ENABLE_GOTO"] == "1" and os.environ["TARS_ENABLE_VOICE"] == "1", "demo turns autonomy + voice on")
os.environ["TARS_ENABLE_GOTO"] = "map"
check(svc.demo_preflight() == ["TARS_DEMO=1 needs TARS_ENABLE_GOTO=1 (it is map)"], "an explicit map-only mode is refused")
os.environ["TARS_ENABLE_GOTO"] = "1"
os.environ.update(TARS_MOTOR_BACKEND="uno", TARS_MOTOR_DEVICE="/dev/serusb1", TARS_BENCH_TOKEN="tars-demo-token-1")
for f in ("build/goto", "build/detect", "models/yolo_world.onnx", "models/labels.txt"):
    (nav / f).write_text("x")
vd = tmp / "voice"; vd.mkdir(); (vd / "tars_voice").write_text("x"); os.environ["TARS_VOICE_DIR"] = str(vd)
check(svc.demo_preflight() == [], "everything real and built: go")
os.environ["TARS_MOTOR_BACKEND"] = "stub"
check(any("stub" in x for x in svc.demo_preflight()), "stub motors refused even when the rest is ready")
(nav / "memory.txt").write_text("trash can#1 ...\n")
svc.demo_clear_memory()
aside = list(nav.glob("memory.txt.before-demo-*"))
check(not (nav / "memory.txt").exists() and len(aside) == 1 and aside[0].read_text() == "trash can#1 ...\n",
      "old goto memory moved aside, not deleted")
svc.demo_clear_memory()
check(len(list(nav.glob("memory.txt.before-demo-*"))) == 1, "no memory file: nothing to do")
print(f"{ok}/{ok + fail}")
sys.exit(1 if fail else 0)
