"""Run through Fusion's script API. Creates a separate concept, not source edits."""
import adsk.core,adsk.fusion,json,pathlib

def run(_context: str):
    rootpath=pathlib.Path('C:/Users/Owner/.codex/visualizations/2026/09/13/01a09cad-d87f-7ad2-82ec-ff5b9e1d79fd/exhist-mujoco')
    data=json.loads((rootpath/'drawer_pull_design.json').read_text())
    app=adsk.core.Application.get();doc=app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    doc.name='ExHist robot drawer pulls - CONCEPT v02 - original claw clearance'
    design=adsk.fusion.Design.cast(app.activeProduct);design.designType=adsk.fusion.DesignTypes.DirectDesignType
    root=design.rootComponent;temp=adsk.fusion.TemporaryBRepManager.get()
    point=lambda v:adsk.core.Point3D.create(*(x*100 for x in v))
    for row in data['pulls']:
        c=row['front_center_m'];occ=root.occurrences.addNewComponent(adsk.core.Matrix3D.create());occ.component.name=row['cad_joint']+' - robot pull CONCEPT'
        bodies=[]
        for delta,size in [([-.037,-.0165,0],[.008,.033,.010]),([.037,-.0165,0],[.008,.033,.010]),([0,-.030,0],[.082,.006,.010])]:
            center=[a+b for a,b in zip(c,delta)]
            ob=adsk.core.OrientedBoundingBox3D.create(point(center),adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),*[x*100 for x in size])
            bodies.append(temp.createBox(ob))
        body=bodies[0]
        for b in bodies[1:]:assert temp.booleanOperation(body,b,adsk.fusion.BooleanTypes.UnionBooleanType)
        for dx in [-.037,.037]:
            a=[c[0]+dx,c[1]+.001,c[2]];b=[c[0]+dx,c[1]-.034,c[2]]
            hole=temp.createCylinderOrCone(point(a),.165,point(b),.165)
            assert temp.booleanOperation(body,hole,adsk.fusion.BooleanTypes.DifferenceBooleanType)
        saved=occ.component.bRepBodies.add(body);saved.name='U-pull 82 x 33 x 10 mm - two 3.3 mm mounting bores'
        occ.component.attributes.add('ExHist','Interface',json.dumps(row))
    root.attributes.add('ExHist','Status','CONCEPT ONLY. Verify actual drawer panel, fastening, latch force and left-hand reach before fabrication.')
    assert design.exportManager.execute(design.exportManager.createSTEPExportOptions(str(rootpath/'ExHist-Robot-Drawer-Pulls-v02.step')))
    assert design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(str(rootpath/'ExHist-Robot-Drawer-Pulls-v02.f3d')))
    app.activeViewport.fit();print('Exported four separate pull concepts; original machine designs untouched.')
