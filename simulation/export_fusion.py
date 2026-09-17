import adsk.core,adsk.fusion,json,pathlib

def run(_context: str):
 app=adsk.core.Application.get();d=adsk.fusion.Design.cast(app.activeProduct);r=d.rootComponent
 out=pathlib.Path(r'C:/Users/Owner/.codex/visualizations/2026/09/13/01a09cad-d87f-7ad2-82ec-ff5b9e1d79fd/exhist-mujoco/assets/ASSET.json')
 bodies=[('root',b) for b in r.bRepBodies]+[(o.fullPathName,b) for o in r.allOccurrences for b in o.bRepBodies]
 data={'document':app.activeDocument.name,'units':'m','parts':[]}
 for path,b in bodies:
  if b.faces.count==0:continue
  calc=b.meshManager.createMeshCalculator();calc.setQuality(adsk.fusion.TriangleMeshQualityOptions.LowQualityTriangleMesh);m=calc.calculate()
  if not m or m.triangleCount==0:continue
  col=[0.7,0.72,0.75,1]
  ap=b.appearance
  if ap:
   for pid in ['opaque_albedo','generic_diffuse','metal_f0','surface_albedo']:
    p=adsk.core.ColorProperty.cast(ap.appearanceProperties.itemById(pid))
    if p:
     c=p.value;col=[c.red/255,c.green/255,c.blue/255,1];break
  data['parts'].append({'name':path+'/'+b.name,'rgba':col,'vertices':[v*0.01 for v in m.nodeCoordinatesAsDouble],'triangles':list(m.nodeIndices)})
 out.write_text(json.dumps(data,separators=(',',':')),encoding='utf-8')
 print('EXPORTED',out.name,'parts',len(data['parts']),'triangles',sum(len(p['triangles'])//3 for p in data['parts']),'bytes',out.stat().st_size,'source',data['document'])

