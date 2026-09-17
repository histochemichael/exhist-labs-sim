"""Reproducible candidate pull-profile trials; no changes to production CAD."""
import argparse,json
import equipment_access as e

def main():
    p=argparse.ArgumentParser();p.add_argument('--radius',type=float,default=.01);p.add_argument('--depth',type=float,default=.104);p.add_argument('--kind',default='st_load');a=p.parse_args()
    build=e.build
    def candidate(*args,**kwargs):
        root,meta=build(*args,**kwargs)
        g=root.find(".//geom[@name='access_handle']")
        g.set('type','cylinder');g.set('size',e.vec([a.radius,.041]));g.set('quat','.707106781 0 .707106781 0');g.set('rgba','.25 .58 .62 1');g.set('group','1')
        meta['candidate_round_bar_radius_m']=a.radius
        return root,meta
    e.build=candidate
    sim=e.Access(a.kind,grasp_depth=a.depth)
    for _ in range(100000):
        if sim.step():break
    result=sim.result();result['candidate_round_bar_radius_m']=a.radius
    path=e.ROOT/f'candidate_{a.kind}_round_{a.radius:.3f}_depth_{a.depth:.3f}.json';path.write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='trace'},indent=2))

if __name__=='__main__':main()
