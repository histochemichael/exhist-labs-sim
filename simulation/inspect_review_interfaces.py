"""Read-only coordinate diagnostics for the review correction pass."""
import json
import xml.etree.ElementTree as ET
import numpy as np
from build_scene import ROOT

def main():
    root = ET.parse(ROOT/'exhist_first_pass.xml').getroot()
    for b in root.findall('.//body'):
        n = b.get('name','')
        if n.startswith(('left_', 'quincy_1', 'imaging_a_s60', 'special_staining_jar', 'sorter_1', 'special_lehisto', 'qc_cassette_1')) or n == 'lift_top_link':
            print('BODY',n,b.get('pos'),b.get('quat'),b.get('euler'))
            for e in b:
                if e.tag in ('joint','site'):print(e.tag,e.attrib)
    for e in root.findall('.//geom'):
        n=e.get('name','')
        if ('table' in n and 'top' in n) or 'workbay_1' in n:print('SUPPORT',e.attrib)
    for key in ('rack24','leica_rack_handled','scanner_cassette','output_magazine','quincy','s60'):
        src=json.loads((ROOT/'assets'/f'{key}.json').read_text())
        print('ASSET',key)
        for p in src['parts']:
            if key in ('quincy','s60') and not any(s in p['name'].lower() for s in ('handle','door','shelf')):continue
            v=np.array(p['vertices']).reshape(-1,3)
            if key in ('leica_rack_handled','scanner_cassette') and not any(s in p['name'].lower() for s in ('handle','base','rack_top')):continue
            print(p['name'],v.min(0).round(6).tolist(),v.max(0).round(6).tolist())
        if key=='rack24':
            v=np.array(src['parts'][0]['vertices']).reshape(-1,3)
            for h in (.07,.08,.085,.09):
                t=v[v[:,2]>h];print('HANDLE SECTION',h,t.min(0),t.max(0))

if __name__=='__main__':main()
