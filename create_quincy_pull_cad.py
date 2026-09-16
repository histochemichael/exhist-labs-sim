"""Separate Fusion concept matching the tested simulation pull; not fabrication approval."""
import adsk.core,adsk.fusion,pathlib,json

def run(_context: str):
    folder=pathlib.Path('C:/Users/Owner/.codex/visualizations/2026/09/13/01a09cad-d87f-7ad2-82ec-ff5b9e1d79fd/exhist-mujoco')
    app=adsk.core.Application.get();doc=app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    doc.name='ExHist Quincy stand-off pull - CONCEPT v01'
    design=adsk.fusion.Design.cast(app.activeProduct);design.designType=adsk.fusion.DesignTypes.DirectDesignType
    root=design.rootComponent;temp=adsk.fusion.TemporaryBRepManager.get()
    p=lambda x,y,z:adsk.core.Point3D.create(x*100,y*100,z*100)
    body=temp.createCylinderOrCone(p(0,-.035,-.05),.5,p(0,-.035,.05),.5)
    for z in (-.045,.045):
        box=adsk.core.OrientedBoundingBox3D.create(p(0,-.007,z),adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),1.,5.6,1.)
        assert temp.booleanOperation(body,temp.createBox(box),adsk.fusion.BooleanTypes.UnionBooleanType)
    saved=root.bRepBodies.add(body);saved.name='10 mm round pull - 35 mm stand-off - 80 mm clear grip span'
    root.attributes.add('ExHist','Status','PROPOSED interface only. 80 g modeled mass. Fastening, thermal compatibility, actual OEM mounting dimensions and load capacity remain unvalidated. Original incubator CAD untouched.')
    assert design.exportManager.execute(design.exportManager.createSTEPExportOptions(str(folder/'ExHist-Quincy-Pull-Concept-v01.step')))
    assert design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(str(folder/'ExHist-Quincy-Pull-Concept-v01.f3d')))
    box=saved.boundingBox
    report=dict(document=doc.name,solids=root.bRepBodies.count,volume_cm3=saved.volume,bounds_cm=[[box.minPoint.x,box.minPoint.y,box.minPoint.z],[box.maxPoint.x,box.maxPoint.y,box.maxPoint.z]],status='CONCEPT; not fabrication-ready')
    (folder/'quincy_pull_cad_validation.json').write_text(json.dumps(report,indent=2));app.activeViewport.fit();print(json.dumps(report))
