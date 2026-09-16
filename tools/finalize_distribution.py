"""Losslessly compress only known large distribution artifacts and audit tracked inputs."""
import gzip,hashlib,json,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
TARGETS=('cad/step/leica-workstation.stp','rail_loop_lab_trace.json','rail_loop_reference/rail_loop_trace.json')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    rows=[]
    for name in TARGETS:
        p=ROOT/name;target=p.with_name(p.name+'.gz')
        with target.open('wb') as raw:
            with gzip.GzipFile(filename='',fileobj=raw,mode='wb',mtime=0,compresslevel=6) as out:
                out.write(p.read_bytes())
        assert hashlib.sha256(gzip.decompress(target.read_bytes())).hexdigest()==sha(p)
        rows.append(dict(file=name,archive=target.relative_to(ROOT).as_posix(),sha256=sha(p),bytes=p.stat().st_size,archive_bytes=target.stat().st_size))
    (ROOT/'compressed-assets.json').write_text(json.dumps(rows,indent=2))
    inventory=[]
    for folder in ('cad','models','assets','source','rail_loop_reference'):
        for p in sorted((ROOT/folder).rglob('*')):
            if not p.is_file() or '__pycache__' in p.parts:continue
            inventory.append(dict(path=p.relative_to(ROOT).as_posix(),bytes=p.stat().st_size,sha256=sha(p)))
    (ROOT/'docs/asset-file-manifest.json').write_text(json.dumps(inventory,indent=2))
    missing=[]
    for md in [ROOT/'README.md',ROOT/'docs/CAD_AND_ASSETS.md',ROOT/'LICENSE-NOTICE.md']:
        for link in re.findall(r'\]\(([^)]+)\)',md.read_text()):
            if link.startswith(('https://','http://','#')):continue
            from urllib.parse import unquote
            if not (md.parent/unquote(link.split('#')[0])).exists():missing.append([md.name,link])
    if missing:raise RuntimeError(missing)
    print(json.dumps(dict(compressed=rows,asset_files=len(inventory),readme_links='PASS'),indent=2))
if __name__=='__main__':main()
