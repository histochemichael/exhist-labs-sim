import adsk.core,adsk.fusion,math

def run(_context: str):
 app=adsk.core.Application.get();d=adsk.fusion.Design.cast(app.activeProduct);r=d.rootComponent
 print('ROOT',r.name)
 occ=r.occurrences.addNewComponent(adsk.core.Matrix3D.create());c=occ.component;c.name='S60 Front Branding'
 def color(name,rgb):
  a=d.appearances.itemByName(name) or d.appearances.addByCopy(d.appearances.itemByName('S60 white'),name)
  for pid in ['opaque_albedo','generic_diffuse','surface_albedo']:
   p=adsk.core.ColorProperty.cast(a.appearanceProperties.itemById(pid))
   if p:p.value=adsk.core.Color.create(*rgb,255)
  return a
 cream=color('S60 Logo Cream',(224,214,182));purple=color('S60 Logo Lavender',(162,148,206));red=color('S60 HAMAMATSU Red',(222,47,65))
 pi=c.constructionPlanes.createInput();pi.setByOffset(c.xYConstructionPlane,adsk.core.ValueInput.createByReal(35.401));plane=c.constructionPlanes.add(pi)
 def extrude(sk,profile,name,appearance):
  e=c.features.extrudeFeatures.addSimple(profile,adsk.core.ValueInput.createByReal(0.015),adsk.fusion.FeatureOperations.NewBodyFeatureOperation);e.name=name
  for i,b in enumerate(e.bodies):b.name=name+' '+str(i+1);b.appearance=appearance
  sk.isVisible=False
 def text(label,h,box,name,appearance):
  sk=c.sketches.add(plane);sk.name=name;inp=sk.sketchTexts.createInput3("'"+label+"'",adsk.core.ValueInput.createByReal(h));inp.fontName='Arial'
  inp.setAsMultiLine(adsk.core.Point3D.create(box[0],box[1],0),adsk.core.Point3D.create(box[2],box[3],0),adsk.core.HorizontalAlignments.LeftHorizontalAlignment,adsk.core.VerticalAlignments.BottomVerticalAlignment,0)
  extrude(sk,sk.sketchTexts.add(inp),name,appearance)
 text('Nano',1.85,[-25,60,-18.5,62.8],'Nano wordmark',cream)
 text('Z',5.0,[-18.5,58,-14,64],'Central Z',cream)
 text('oomer',1.65,[-14.5,60,-6.5,62.6],'oomer wordmark',cream)
 text('S60',1.45,[-11,57.8,-6.5,60],'S60 model',cream)
 text('HAMAMATSU',1.15,[-22.1,53.8,-8.5,56],'HAMAMATSU manufacturer',red)
 sk=c.sketches.add(plane);sk.name='Lavender magnifier symbol';cx=-16.3;cy=60.1
 angles=[math.radians(35+i*290/80) for i in range(81)]
 points=[(cx+4.6*math.cos(a),cy+4.6*math.sin(a)) for a in angles]+[(cx+4.13*math.cos(a),cy+4.13*math.sin(a)) for a in reversed(angles)]
 for p,q in zip(points,points[1:]+points[:1]):sk.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(*p,0),adsk.core.Point3D.create(*q,0))
 extrude(sk,sk.profiles.item(0),'Magnifier ring',purple)
 sk=c.sketches.add(plane);pts=[(-13.5,56.75),(-12.98,57.15),(-10.18,54.75),(-10.7,54.35)]
 for p,q in zip(pts,pts[1:]+pts[:1]):sk.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(*p,0),adsk.core.Point3D.create(*q,0))
 extrude(sk,sk.profiles.item(0),'Magnifier handle',purple)
 plane.isLightBulbOn=False
 print('CREATED',c.bRepBodies.count)

