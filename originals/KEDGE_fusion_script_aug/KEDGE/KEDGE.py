# KEDGE v5.2 — self-calibrating axes, stepped camera bore. Dims mm. Wipes first.
import adsk.core, adsk.fusion, traceback, math

HULL_OD=30.0; HULL_ID=26.0
TUBE_X0=-110.0; TUBE_X1=106.0
AXLE_X=70.0; AXLE_Z=-8.0; AXLE_BORE=4.5; BOSS_D=12.0; BOSS_T=4.0
SPIG_X0=86.0; CONE_X0=106.0; TIP_X=134.0; TIP_R=1.5
CAMBAY_D=14.0; CAMBAY_X1=118.0; APERT_D=6.5
RAIL_X0=110.0; RAIL_X1=154.0; RAIL_Z0=8.0; RAIL_Z1=16.0; RAIL_Y=6.7; RAIL_T=4.0
ROOT_X=112.0; BRIDGE_X=150.0
CLV_X=150.0; CH_T=5.0; CH_GAP=9.6; CH_Z0=18.0; CH_Z1=34.0; PIN_Z=28.0; PIN_BORE=4.5
MBAY_X0=73.0; MBAY_X1=103.0; MBAY_W=13.0; MBAY_Z0=-7.5; MBAY_Z1=3.5
PLUG_X1=-92.0; PLUG_X0=-110.0; CAP_X0=-118.0
EYE_X=-134.0; EYE_MAJ=9.0; EYE_MIN=3.5
CRD_X0=25.0; CRD_X1=95.0; SAD_X=56.0; GEAR_X=70.0

def run(context):
    ui=None
    try:
        app=adsk.core.Application.get(); ui=app.userInterface
        design=adsk.fusion.Design.cast(app.activeProduct)
        root=design.rootComponent
        for coll in (root.occurrences, root.bRepBodies, root.sketches):
            while coll.count: coll.item(0).deleteMe()
        mm=0.1; P=adsk.core.Point3D.create; V=adsk.core.ValueInput
        F=adsk.fusion.FeatureOperations

        # ---- calibrate: which world direction is sketch +Y on the xZ plane? ----
        cal=root.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component
        skT=cal.sketches.add(cal.xZConstructionPlane)
        skT.sketchCurves.sketchCircles.addByCenterRadius(P(0,10*mm,0),2*mm)
        eT=cal.features.extrudeFeatures.createInput(skT.profiles.item(0),
             F.NewBodyFeatureOperation)
        eT.setSymmetricExtent(V.createByReal(2*mm),True)
        tb=cal.features.extrudeFeatures.add(eT).bodies.item(0)
        zc=(tb.boundingBox.minPoint.z+tb.boundingBox.maxPoint.z)/2.0
        S = 1.0 if zc>0 else -1.0          # world Z of a feature drawn at sketchY=+10
        cal.deleteMe() if hasattr(cal,'deleteMe') else None
        for coll in (root.occurrences, root.bRepBodies, root.sketches):
            while coll.count: coll.item(0).deleteMe()

        def comp(name):
            o=root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            o.component.name=name; return o.component
        def tools(c):
            ext=c.features.extrudeFeatures; rev=c.features.revolveFeatures
            pls=c.constructionPlanes
            def plane(off):
                pi=pls.createInput(); pi.setByOffset(c.xZConstructionPlane,V.createByReal(off*mm))
                return pls.add(pi)
            def sy(z): return S*z            # world Z -> sketchY, measured
            def slab(x0,z0,x1,z1,yc,w,op):
                for sgn in ((1,-1) if yc!=0 else (0,)):
                    sk=c.sketches.add(plane(sgn*yc)); L=sk.sketchCurves.sketchLines
                    q=[(x0,sy(z0)),(x1,sy(z0)),(x1,sy(z1)),(x0,sy(z1)),(x0,sy(z0))]
                    for i in range(4):
                        L.addByTwoPoints(P(q[i][0]*mm,q[i][1]*mm,0),P(q[i+1][0]*mm,q[i+1][1]*mm,0))
                    ei=ext.createInput(sk.profiles.item(0),op)
                    ei.setSymmetricExtent(V.createByReal(w*mm),True); ext.add(ei)
            def boreY(x,z,r,ln):
                sk=c.sketches.add(c.xZConstructionPlane)
                sk.sketchCurves.sketchCircles.addByCenterRadius(P(x*mm,sy(z)*mm,0),r*mm)
                ei=ext.createInput(sk.profiles.item(0),F.CutFeatureOperation)
                ei.setSymmetricExtent(V.createByReal(ln*mm),True); ext.add(ei)
            def bossY(x,z,r,off,t):
                sk=c.sketches.add(plane(off))
                sk.sketchCurves.sketchCircles.addByCenterRadius(P(x*mm,sy(z)*mm,0),r*mm)
                ei=ext.createInput(sk.profiles.item(0),F.JoinFeatureOperation)
                ei.setSymmetricExtent(V.createByReal(t*mm),True); ext.add(ei)
            def revP(pts,op):
                sk=c.sketches.add(c.xZConstructionPlane); L=sk.sketchCurves.sketchLines
                for i in range(len(pts)-1):
                    L.addByTwoPoints(P(pts[i][0]*mm,pts[i][1]*mm,0),P(pts[i+1][0]*mm,pts[i+1][1]*mm,0))
                ri=rev.createInput(sk.profiles.item(0),c.xConstructionAxis,op)
                ri.setAngleExtent(False,V.createByString('360 deg')); rev.add(ri)
            return slab,boreY,bossY,revP

        # ---------- P1 hull tube ----------
        c=comp('P1_hull_tube'); slab,boreY,bossY,revP=tools(c)
        revP([(TUBE_X0,HULL_ID/2),(TUBE_X0,HULL_OD/2),(TUBE_X1,HULL_OD/2),
              (TUBE_X1,HULL_ID/2),(TUBE_X0,HULL_ID/2)],F.NewBodyFeatureOperation)
        ey=math.sqrt((HULL_OD/2)**2-AXLE_Z**2)
        for sgn in (1,-1):
            bossY(AXLE_X,AXLE_Z,BOSS_D/2,sgn*(ey+BOSS_T/2-0.5),BOSS_T)
        boreY(AXLE_X,AXLE_Z,AXLE_BORE/2,2*(ey+BOSS_T)+6)

        # ---------- P2 nose module ----------
        c=comp('P2_nose_module'); slab,boreY,bossY,revP=tools(c)
        revP([(SPIG_X0,0),(SPIG_X0,HULL_ID/2-0.2),(CONE_X0,HULL_ID/2-0.2),
              (CONE_X0,HULL_OD/2),(TIP_X,TIP_R),(TIP_X,0),(SPIG_X0,0)],F.NewBodyFeatureOperation)
        slab(RAIL_X0,RAIL_Z0,RAIL_X1,RAIL_Z1,RAIL_Y,RAIL_T,F.JoinFeatureOperation)
        slab(ROOT_X-5,6.0,ROOT_X+5,16.0,0.0,17.4,F.JoinFeatureOperation)
        slab(BRIDGE_X-6,12.0,BRIDGE_X+6,18.0,0.0,17.4,F.JoinFeatureOperation)
        slab(CLV_X-7,CH_Z0,CLV_X+7,CH_Z1,CH_GAP/2+CH_T/2,CH_T,F.JoinFeatureOperation)
        # stepped camera bore: fat module cavity, narrow lens aperture
        revP([(CONE_X0+2,0),(CONE_X0+2,CAMBAY_D/2),(CAMBAY_X1,CAMBAY_D/2),
              (CAMBAY_X1,APERT_D/2),(TIP_X+2,APERT_D/2),(TIP_X+2,0),
              (CONE_X0+2,0)],F.CutFeatureOperation)
        boreY(CLV_X,PIN_Z,PIN_BORE/2,40.0)
        slab(MBAY_X0,MBAY_Z0,MBAY_X1,MBAY_Z1,0.0,MBAY_W,F.CutFeatureOperation)

        # ---------- P3 tail cap ----------
        c=comp('P3_tail_cap'); slab,boreY,bossY,revP=tools(c)
        revP([(PLUG_X1,0),(PLUG_X1,HULL_ID/2-0.2),(PLUG_X0,HULL_ID/2-0.2),
              (PLUG_X0,HULL_OD/2),(CAP_X0,HULL_OD/2),
              (-122.0,4.8),(-131.0,4.8),(-131.0,0),(PLUG_X1,0)],F.NewBodyFeatureOperation)
        skE=c.sketches.add(c.xYConstructionPlane)
        ax=skE.sketchCurves.sketchLines.addByTwoPoints(P(EYE_X*mm,-25*mm,0),P(EYE_X*mm,25*mm,0))
        ax.isConstruction=True
        skE.sketchCurves.sketchCircles.addByCenterRadius(P((EYE_X-EYE_MAJ)*mm,0,0),EYE_MIN*mm)
        ri=c.features.revolveFeatures.createInput(skE.profiles.item(0),ax,F.JoinFeatureOperation)
        ri.setAngleExtent(False,V.createByString('360 deg')); c.features.revolveFeatures.add(ri)

        # ---------- P4 motor cradle ----------
        c=comp('P4_motor_cradle'); slab,boreY,bossY,revP=tools(c)
        slab(CRD_X0,-13.0,CRD_X1,-9.0,0.0,20.0,F.NewBodyFeatureOperation)
        slab(SAD_X-17,-10.0,SAD_X+17,-2.0,0.0,14.0,F.JoinFeatureOperation)
        slab(SAD_X-16.75,-9.9,SAD_X+16.75,2.2,0.0,10.2,F.CutFeatureOperation)
        boreY(GEAR_X,AXLE_Z,8.0,12.0)

        ui.messageBox('KEDGE v5.2 — axis self-calibrated (S={:+.0f}), camera bore stepped.\n'
            'Clevis/rails must be UP, cradle DOWN, cone tip intact.'.format(S))
    except:
        if ui: ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))