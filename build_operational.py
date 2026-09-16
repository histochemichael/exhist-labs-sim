"""Create animated, typed carrier instances on the existing CAD scene."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
from build_scene import convert,el
ROOT=Path(__file__).resolve().parent

def build():
    source=json.loads((ROOT/"assets/s60.json").read_text())
    source["parts"]=[p for p in source["parts"] if p["name"].startswith("cassette_0 |")]
    assert source["parts"],"No scanner cassette found"
    source["document"]="S60 bottom cassette from branded CAD"
    (ROOT/"assets/scanner_cassette.json").write_text(json.dumps(source))
    models={k:convert(k) for k in ["leica_rack","rack24","output_magazine","scanner_cassette","slide_folder"]}
    root=ET.parse(ROOT/"exhist.xml").getroot();root.set("model","ExHist workflow simulation v02")
    world=root.find("worldbody");assets=root.find("asset")
    existing={m.get("name") for m in assets.findall("mesh")}
    for model in models.values():
        for m in model["meshes"]:
            if m["name"] not in existing:
                el(assets,"mesh",name=m["name"],file=m["file"],inertia="shell")
    for child in list(world):
        name=child.get("name","")
        if name.startswith(("filled_leica_","protocol_rack24_","special_dropoff","sendout_folder_","bucket_","reagent_","special_staining_jar_","special_bath_target_")) or name.endswith("_staging"):
            world.remove(child)
    # Genuine matching jars were found in the source Slide Rack v13 document.
    layout=json.loads((ROOT/"layout.json").read_text())
    sx=next(s["x"] for s in layout["stations"] if s["name"]=="special")
    for k in range(6):
        from jar_riser import RACK_SEAT_Z
        el(world,"site",name=f"special_bath_target_{k+1}",pos=f"{sx-0.375+k*.15} 0.30 {RACK_SEAT_Z}",
           type="cylinder",size="0.025 0.002",rgba="0.6 0.4 0.9 0.5")
    from cad_jar import add_special_jars
    add_special_jars(root)
    for j in range(4):
        for key,model in models.items():
            body=el(world,"body",name=f"B{j+1:02d}_{key}",mocap="true",pos="0 0 -10")
            for i,m in enumerate(model["meshes"]):
                el(body,"geom",name=f"B{j+1:02d}_{key}_{i}",type="mesh",mesh=m["name"],rgba=" ".join(map(str,m["rgba"])),
                   contype="0",conaffinity="0",group="1",density="0")
    from articulate import add_robots
    add_robots(root)
    from articulate_machines import add_machines
    add_machines(root)
    from passive_doors import add_passive_doors
    add_passive_doors(root)
    from gripper_physics import apply_nori
    apply_nori(root)
    from front_docking import add_head_cameras,write_station_docks
    add_head_cameras(root);write_station_docks()
    from canopy_hoods import add_special_canopy
    add_special_canopy(root)
    # Persist the accepted layout on rebuild; old film choreography is separate.
    from special_layout import SNAPSHOT,apply as apply_special_layout
    if SNAPSHOT.exists():apply_special_layout(root)
    from passive_props import apply as apply_passive_props
    apply_passive_props(root)
    ET.indent(root);ET.ElementTree(root).write(ROOT/"exhist_operational.xml",encoding="utf-8",xml_declaration=True)
    print("Operational scene built: four tracked batches, five carrier types.")
if __name__=="__main__":build()
