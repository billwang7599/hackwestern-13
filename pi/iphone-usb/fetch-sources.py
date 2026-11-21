"""Fetch pinned upstream sources locally, before transfer to QNX."""
from pathlib import Path
import subprocess

here = Path(__file__).resolve().parent
root = here.parent.parent
destination = root / 'vendor' / 'iphone-usb'
destination.mkdir(parents=True, exist_ok=True)
for line in (here / 'sources.lock').read_text().splitlines():
    name, commit, url = line.split('\t')
    path = destination / name
    if not path.exists():
        subprocess.run(['git', 'clone', url, str(path)], check=True)
        subprocess.run(['git', '-C', str(path), 'checkout', '--detach', commit], check=True)
    actual = subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()
    if actual != commit:
        raise SystemExit(f'{name}: checkout differs from sources.lock; leaving it untouched')
    if (path / 'git-version-gen').exists():
        version = subprocess.check_output(['sh', 'git-version-gen'], cwd=path, text=True).strip()
        (path / '.tarball-version').write_text(version + '\n')
for name in ('usbmuxd', 'libusb'):
    patch = here / f'{name}-qnx.patch'
    repo = destination / name
    already_applied = subprocess.run(['git', 'apply', '--reverse', '--check', str(patch)], cwd=repo,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if already_applied.returncode:
        subprocess.run(['git', 'apply', '--check', str(patch)], cwd=repo, check=True)
        subprocess.run(['git', 'apply', str(patch)], cwd=repo, check=True)
print('Pinned USB sources ready locally')
