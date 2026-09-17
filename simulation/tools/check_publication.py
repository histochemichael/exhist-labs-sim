"""Check intended Git inputs without printing possible credential contents."""
import json,re,subprocess
from pathlib import Path
from urllib.parse import unquote
ROOT=Path(__file__).resolve().parents[1]
paths=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','-z'],cwd=ROOT).decode().split('\0')
patterns=[rb'gh[pousr]_[A-Za-z0-9]{30,}',rb'github_pat_[A-Za-z0-9_]{40,}',rb'sk-(?:proj-)?[A-Za-z0-9_-]{35,}',rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----']
bad=[];large=[];count=0
for name in sorted(set(paths)):
    if not name:continue
    p=ROOT/name
    if not p.is_file():continue
    count+=1
    if p.stat().st_size>=100*1024*1024:large.append(name)
    if p.suffix.lower() in ('.py','.json','.xml','.urdf','.md','.txt','.yml','.yaml','.ps1','.toml','.ini','.cfg') or p.name.startswith('.'):
        data=p.read_bytes()
        if any(re.search(pattern,data) for pattern in patterns):bad.append(name)
missing=[]
for md in [ROOT/'README.md',ROOT/'docs/PROMO.md',ROOT/'docs/QC_FOLDER45.md',ROOT/'docs/CAD_AND_ASSETS.md',ROOT/'LICENSE-NOTICE.md']:
    for url in re.findall(r'\]\(([^)]+)\)',md.read_text()):
        if url.startswith(('https://','http://','#')):continue
        if not (md.parent/unquote(url.split('#')[0])).exists():missing.append([md.name,url])
report=dict(files_checked=count,oversized_files=large,possible_secret_files=bad,missing_document_links=missing,scope='Common secret patterns and publication file/link checks; not a comprehensive security audit.')
print(json.dumps(report,indent=2));assert not bad and not large and not missing
(ROOT/'docs/publication-check.json').write_text(json.dumps(report,indent=2))
