"""Restore compressed large assets, checking their content hashes; no dependencies."""
import gzip, hashlib, json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
    path=ROOT/'compressed-assets.json'
    if not path.exists(): return
    for row in json.loads(path.read_text()):
        target=ROOT/row['file']
        assert target.resolve().is_relative_to(ROOT)
        if target.exists():
            if hashlib.sha256(target.read_bytes()).hexdigest()!=row['sha256']:
                raise RuntimeError(f'Existing modified file will not be overwritten: {target}')
            continue
        content=gzip.decompress((ROOT/row['archive']).read_bytes())
        assert len(content)==row['bytes'] and hashlib.sha256(content).hexdigest()==row['sha256']
        target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(content)
        print('Restored',row['file'])
    print('Compressed assets verified.')
if __name__=='__main__': main()
