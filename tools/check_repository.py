"""Check shipped integrity, Python syntax, Markdown links and basic secrets.

This is not a SOFA runtime test, electrical inspection, or hardware load test.
"""

import ast
import hashlib
import json
from pathlib import Path
import re
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]


def main():
    checked = 0
    for item in json.loads((ROOT / 'docs/SOURCE_FILES.json').read_text(encoding='utf-8')):
        data = (ROOT / item['path']).read_bytes()
        assert hashlib.sha256(data).hexdigest() == item['sha256'], item['path']
        checked += 1
    py_count, link_count = 0, 0
    for path in ROOT.rglob('*'):
        if not path.is_file() or '.git' in path.relative_to(ROOT).parts:
            continue
        if path.suffix == '.py':
            ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
            py_count += 1
        if path.suffix in ('.py', '.ino', '.md', '.cmd', '.json', '.txt'):
            text = path.read_text(encoding='utf-8')
            assert not re.search(r'(?i)(?:gh[pousr]_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|sk-proj-[A-Za-z0-9_-]{20,}|[A-Z]:[\\/]Users[\\/])', text), str(path)
        if path.suffix != '.md':
            continue
        for href in re.findall(r'\]\(([^\s)]+)(?:\s+[^)]*)?\)', path.read_text(encoding='utf-8')):
            url = urlsplit(href)
            if url.scheme or href.startswith('#'):
                continue
            target = (path.parent / unquote(url.path)).resolve()
            assert target.is_relative_to(ROOT), f'Out-of-repo link: {path.relative_to(ROOT)}'
            assert target.exists(), f'Broken link {href} in {path.relative_to(ROOT)}'
            link_count += 1
    print(json.dumps({'original_files_sha256_verified': checked,
                      'python_files_parsed': py_count,
                      'local_markdown_links_verified': link_count,
                      'basic_secret_and_path_scan': 'passed',
                      'scope': 'Repository integrity only; no hardware has been powered.'}, indent=2))


if __name__ == '__main__':
    main()
