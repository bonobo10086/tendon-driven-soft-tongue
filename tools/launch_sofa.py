"""Launch the scene using an existing, matching SOFA installation.

No packages are installed and no firmware is uploaded by this script.
"""

import argparse
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def command(sofa_dir, python_dir=None, gui='qt'):
    sofa = Path(sofa_dir).expanduser().resolve()
    executable = sofa / 'bin' / ('runSofa.exe' if os.name == 'nt' else 'runSofa')
    if not executable.is_file():
        raise FileNotFoundError(f'Cannot find runSofa: {executable}')
    env = os.environ.copy()
    paths = [str(sofa / 'bin')]
    for plugin in ('SofaPython3', 'Sofa.Qt'):
        directory = sofa / 'plugins' / plugin
        if directory.is_dir():
            paths.extend(str(p) for p in (directory / 'bin', directory / 'lib') if p.is_dir())
    if python_dir:
        python = Path(python_dir).expanduser().resolve()
        if not python.is_dir():
            raise FileNotFoundError(f'Cannot find matching Python directory: {python}')
        paths.insert(0, str(python))
    env['PATH'] = os.pathsep.join(paths + [env.get('PATH', '')])
    return [str(executable), '-g', gui, '-l', 'SofaPython3', str(ROOT / 'tongue_motion.py')], env


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sofa-dir', default=os.environ.get('SOFA_ROOT'))
    parser.add_argument('--python-dir', default=os.environ.get('SOFA_PYTHON_ROOT'))
    parser.add_argument('--gui', default='qt')
    parser.add_argument('--check', action='store_true', help='Print the launch command without starting SOFA')
    args = parser.parse_args()
    if not args.sofa_dir:
        parser.error('Supply --sofa-dir or set SOFA_ROOT.')
    try:
        cmd, env = command(args.sofa_dir, args.python_dir, args.gui)
    except FileNotFoundError as error:
        parser.exit(2, str(error) + '\n')
    if args.check:
        print(subprocess.list2cmdline(cmd))
        return
    raise SystemExit(subprocess.call(cmd, cwd=ROOT, env=env))


if __name__ == '__main__':
    main()
