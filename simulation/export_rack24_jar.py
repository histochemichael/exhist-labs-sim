"""Run read-only through Fusion's script API; exports the existing jar, no CAD edits."""
import adsk.core,adsk.fusion,json,pathlib

def run(_context: str):
    app=adsk.core.Application.get();doc=next(d for d in app.documents if d.name=='Slide Rack v13')
    design=adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    body=next(b for b in design.rootComponent.bRepBodies if b.name=='Staining Jar')
    calc=body.meshManager.createMeshCalculator();calc.setQuality(adsk.fusion.TriangleMeshQualityOptions.HighQualityTriangleMesh);mesh=calc.calculate()
    data={'document':doc.name,'units':'m','parts':[{'name':'root/Staining Jar','rgba':[.55,.68,.73,1],'vertices':[v*.01 for v in mesh.nodeCoordinatesAsDouble],'triangles':list(mesh.nodeIndices)}]}
    path=pathlib.Path('C:/Users/Owner/.codex/visualizations/2026/09/13/01a09cad-d87f-7ad2-82ec-ff5b9e1d79fd/exhist-mujoco/assets/rack24_jar.json')
    path.write_text(json.dumps(data,separators=(',',':')))
    faces=[]
    for face in body.faces:
        plane=adsk.core.Plane.cast(face.geometry)
        if plane:faces.append({'normal':[plane.normal.x,plane.normal.y,plane.normal.z],'origin_m':[plane.origin.x*.01,plane.origin.y*.01,plane.origin.z*.01],'area_m2':face.area*.0001})
    path.with_name('rack24_jar_faces.json').write_text(json.dumps(faces,indent=2))
    print('Exported existing Staining Jar:',mesh.triangleCount,'triangles; source CAD unchanged.')
