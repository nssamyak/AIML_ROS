#!/usr/bin/env python3
"""Run every module demo quickly and without windows. Works on any OS.

    python run_all.py            # quick check (--fast)
    python run_all.py --full     # full-size runs (several minutes)

Figures and GIFs are written to output/<module>/.
"""
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def main():
    full = '--full' in sys.argv
    demos = sorted(ROOT.glob('0*_*/demo.py'))
    print(f'Python {sys.version.split()[0]} on {sys.platform}: running {len(demos)} demos '
          f'({"full" if full else "fast"} mode)\n')
    failed = []
    for demo in demos:
        cmd = [sys.executable, str(demo), '--no-show'] + ([] if full else ['--fast'])
        t0 = time.perf_counter()
        result = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
        dt = time.perf_counter() - t0
        ok = result.returncode == 0
        print(f'{"PASS" if ok else "FAIL"}  {demo.parent.name:28s} {dt:6.1f} s')
        if not ok:
            failed.append(demo.parent.name)
            print('      ' + '\n      '.join((result.stderr or result.stdout).strip().splitlines()[-15:]))
    print(f'\n{len(demos) - len(failed)}/{len(demos)} passed. Output in {ROOT / "output"}')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
