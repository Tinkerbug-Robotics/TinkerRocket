# ReflowMold.py  --  Fusion 360 script
# Builds the 3D-printable mold for the Mold Max 60 open-frame reflow carrier of
# rocket-computer-mini v1.01 (terminal-block side down, back side open to the
# oven) as a native, parametric Fusion design.  Run from  UTILITIES > ADD-INS > Scripts and Add-Ins (Shift+S).
#
# Generated from the KiCad STEP export; geometry data is embedded below.
# Reference geometry (verified against the STEP solids): rocket-computer-mini-reflow-mold.step
#
# Units: the Fusion API works in centimetres internally; all data below is in
# millimetres and converted by mm().

import adsk.core, adsk.fusion, traceback, json, math, os, time

LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ReflowMold.log')

def log(msg):
    try:
        with open(LOG, 'a') as f:
            f.write(time.strftime('%H:%M:%S ') + msg + '\n')
    except Exception:
        pass

GEOM = json.loads(r'''{
 "version": "rocket-computer-mini v1.01 open-frame reflow carrier mold (v2)",
 "params": {
  "board_w": 22.5,
  "board_l": 69.57,
  "board_corner_r": 1.0,
  "board_t": 1.7,
  "board_clr": 0.2,
  "wall": 5.0,
  "sil_h": 14.0,
  "outer_r": 2.0,
  "lip_max": 1.5,
  "lip_min": 0.6,
  "lip_min_len": 2.0,
  "clr_xy": 0.5,
  "catch_clr": 0.3,
  "catch_margin": 1.0,
  "draft_deg": 1.5,
  "mold_wall": 2.5,
  "floor_t": 2.0,
  "freeboard": 3.0,
  "groove_w": 0.5,
  "groove_d": 0.4,
  "label": "RCM v1.01"
 },
 "box": {
  "w": 37.5,
  "l": 84.57,
  "r": 4.5,
  "z0": -2.0,
  "z1": 17.0
 },
 "cavity": {
  "w": 32.5,
  "l": 79.57,
  "r": 2.0,
  "z0": 0.0,
  "z1": 18.0
 },
 "groove": {
  "w_out": 33.3,
  "l_out": 80.36999999999999,
  "w_in": 32.5,
  "l_in": 79.57,
  "z0": 13.75,
  "h": 0.5
 },
 "plateau": {
  "w": 22.9,
  "l": 69.97,
  "r": 1.2,
  "z0": 0.0,
  "z1": 1.7
 },
 "core": {
  "z0": 0.0,
  "z1": 17.0,
  "taper_deg": 1.5,
  "polys": [
   {
    "ext": [
     [
      -3.5708,
      -41.785
     ],
     [
      3.9908,
      -41.785
     ],
     [
      3.9908,
      -33.285
     ],
     [
      9.75,
      -33.285
     ],
     [
      9.75,
      -27.8318
     ],
     [
      11.45,
      -27.8318
     ],
     [
      11.45,
      -16.6982
     ],
     [
      9.75,
      -16.6982
     ],
     [
      9.75,
      -14.4081
     ],
     [
      10.2231,
      -14.4081
     ],
     [
      10.2231,
      -12.5631
     ],
     [
      11.45,
      -12.5542
     ],
     [
      11.45,
      -10.5358
     ],
     [
      10.0031,
      -10.5358
     ],
     [
      10.0031,
      -8.4219
     ],
     [
      9.9318,
      -8.4219
     ],
     [
      9.9318,
      -6.5427
     ],
     [
      11.45,
      -6.5427
     ],
     [
      11.45,
      6.1618
     ],
     [
      9.75,
      6.1618
     ],
     [
      9.75,
      18.1637
     ],
     [
      11.45,
      18.1637
     ],
     [
      11.45,
      20.0463
     ],
     [
      10.7092,
      20.0463
     ],
     [
      10.6892,
      22.1842
     ],
     [
      9.75,
      22.1842
     ],
     [
      9.75,
      26.4637
     ],
     [
      11.45,
      26.4637
     ],
     [
      11.45,
      28.3463
     ],
     [
      10.7592,
      28.3463
     ],
     [
      10.7592,
      30.4242
     ],
     [
      9.75,
      30.4242
     ],
     [
      9.75,
      32.1619
     ],
     [
      10.8131,
      32.1619
     ],
     [
      10.8012,
      34.8445
     ],
     [
      10.4841,
      34.9619
     ],
     [
      10.25,
      34.985
     ],
     [
      8.8172,
      34.985
     ],
     [
      8.8172,
      37.1722
     ],
     [
      -10.549,
      37.1722
     ],
     [
      -10.549,
      20.756
     ],
     [
      -9.75,
      20.756
     ],
     [
      -9.75,
      13.0377
     ],
     [
      -11.45,
      13.0377
     ],
     [
      -11.45,
      2.0184
     ],
     [
      -9.75,
      2.0184
     ],
     [
      -9.75,
      1.0969
     ],
     [
      -10.1657,
      1.0969
     ],
     [
      -10.1657,
      -0.7458
     ],
     [
      -11.45,
      -0.7458
     ],
     [
      -11.45,
      -2.7642
     ],
     [
      -10.6662,
      -2.7642
     ],
     [
      -10.6662,
      -6.9319
     ],
     [
      -11.45,
      -6.9319
     ],
     [
      -11.45,
      -8.9581
     ],
     [
      -10.3119,
      -8.9581
     ],
     [
      -10.3119,
      -10.8107
     ],
     [
      -9.75,
      -10.8107
     ],
     [
      -9.75,
      -11.9123
     ],
     [
      -11.45,
      -11.9123
     ],
     [
      -11.45,
      -14.9777
     ],
     [
      -10.8131,
      -14.9777
     ],
     [
      -10.8131,
      -17.5581
     ],
     [
      -9.75,
      -17.5581
     ],
     [
      -9.75,
      -27.9869
     ],
     [
      -11.45,
      -27.9908
     ],
     [
      -11.45,
      -30.0092
     ],
     [
      -9.75,
      -30.0131
     ],
     [
      -9.75,
      -33.285
     ],
     [
      -3.5708,
      -33.285
     ]
    ],
    "holes": []
   }
  ]
 },
 "catch": {
  "z0": 11.850130037155227,
  "z1": 19.0,
  "polys": [
   {
    "ext": [
     [
      9.8172,
      19.756
     ],
     [
      9.8172,
      22.1842
     ],
     [
      9.75,
      22.1842
     ],
     [
      9.75,
      26.4637
     ],
     [
      9.8172,
      26.4637
     ],
     [
      9.8172,
      30.4242
     ],
     [
      9.75,
      30.4242
     ],
     [
      9.75,
      32.1619
     ],
     [
      9.8172,
      32.1619
     ],
     [
      9.8172,
      34.985
     ],
     [
      8.8172,
      34.985
     ],
     [
      8.8172,
      37.1722
     ],
     [
      -10.549,
      37.1722
     ],
     [
      -10.549,
      20.756
     ],
     [
      -9.75,
      20.756
     ],
     [
      -9.75,
      19.756
     ]
    ],
    "holes": []
   }
  ]
 },
 "label": {
  "text": "RCM v1.01",
  "height": 3.0,
  "depth": 0.4,
  "x": -13.95,
  "y": -14.0
 },
 "silicone": {
  "w": 32.5,
  "l": 79.57,
  "r": 2.0,
  "z0": 0.0,
  "z1": 14.0
 }
}''')

MAKE_SILICONE_BODY = True   # also create the cast silicone carrier as a reference body


def mm(v):
    return v / 10.0


def P3(x, y, z=0.0):
    return adsk.core.Point3D.create(mm(x), mm(y), mm(z))


def draw_polygon(sketch, pts):
    lines = sketch.sketchCurves.sketchLines
    n = len(pts)
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        lines.addByTwoPoints(P3(a[0], a[1]), P3(b[0], b[1]))


def draw_rounded_rect(sketch, w, l, r, cx=0.0, cy=0.0):
    """Centred axis-aligned rounded rectangle (4 lines + 4 CCW quarter arcs)."""
    lines = sketch.sketchCurves.sketchLines
    arcs = sketch.sketchCurves.sketchArcs
    hw, hl = w / 2.0, l / 2.0
    if r <= 1e-6:
        lines.addTwoPointRectangle(P3(cx - hw, cy - hl), P3(cx + hw, cy + hl))
        return
    lines.addByTwoPoints(P3(cx - hw + r, cy - hl), P3(cx + hw - r, cy - hl))   # bottom
    lines.addByTwoPoints(P3(cx + hw, cy - hl + r), P3(cx + hw, cy + hl - r))   # right
    lines.addByTwoPoints(P3(cx + hw - r, cy + hl), P3(cx - hw + r, cy + hl))   # top
    lines.addByTwoPoints(P3(cx - hw, cy + hl - r), P3(cx - hw, cy - hl + r))   # left
    q = math.pi / 2.0
    arcs.addByCenterStartSweep(P3(cx + hw - r, cy - hl + r), P3(cx + hw - r, cy - hl), q)  # bottom-right
    arcs.addByCenterStartSweep(P3(cx + hw - r, cy + hl - r), P3(cx + hw, cy + hl - r), q)  # top-right
    arcs.addByCenterStartSweep(P3(cx - hw + r, cy + hl - r), P3(cx - hw + r, cy + hl), q)  # top-left
    arcs.addByCenterStartSweep(P3(cx - hw + r, cy - hl + r), P3(cx - hw, cy - hl + r), q)  # bottom-left


def outer_profiles(sketch, n_loops):
    """Profiles of the sketch having exactly n_loops loops (outer ring + holes)."""
    coll = adsk.core.ObjectCollection.create()
    for i in range(sketch.profiles.count):
        p = sketch.profiles.item(i)
        if p.profileLoops.count == n_loops:
            coll.add(p)
    if coll.count == 0:
        raise RuntimeError('sketch %s: no profile with %d loops (has %d profiles)' % (sketch.name, n_loops, sketch.profiles.count))
    return coll


def extrude(comp, profiles, z0, dist, op, name, taper_deg=0.0, participants=None, z0_expr=None, dist_expr=None):
    """Extrude profiles from Z=z0 by dist (mm, +Z).  Expressions may reference user parameters."""
    ext_feats = comp.features.extrudeFeatures
    inp = ext_feats.createInput(profiles, op)
    start = adsk.core.ValueInput.createByString(z0_expr) if z0_expr else adsk.core.ValueInput.createByReal(mm(z0))
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(start)
    d = adsk.core.ValueInput.createByString(dist_expr) if dist_expr else adsk.core.ValueInput.createByReal(mm(dist))
    if abs(taper_deg) > 1e-9:
        inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(d),
                             adsk.fusion.ExtentDirections.PositiveExtentDirection,
                             adsk.core.ValueInput.createByString('%g deg' % taper_deg))
    else:
        inp.setDistanceExtent(False, d)
    if participants:
        inp.participantBodies = participants
    ext = ext_feats.add(inp)
    ext.name = name
    return ext


def polygon_feature(comp, name, polys, z0, z1, op, taper_deg=0.0, dist_expr=None):
    """One sketch + one extrude per polygon (polygons may have holes)."""
    feats = []
    for k, poly in enumerate(polys):
        sk = comp.sketches.add(comp.xYConstructionPlane)
        sk.name = name if len(polys) == 1 else '%s %d' % (name, k + 1)
        sk.isComputeDeferred = True
        draw_polygon(sk, poly['ext'])
        for h in poly['holes']:
            draw_polygon(sk, h)
        sk.isComputeDeferred = False
        profs = outer_profiles(sk, 1 + len(poly['holes']))
        feats.append(extrude(comp, profs, z0, z1 - z0, op, sk.name, taper_deg=taper_deg, dist_expr=dist_expr))
    return feats


def profile_area(profiles):
    a = 0.0
    for i in range(profiles.count):
        a += profiles.item(i).areaProperties(adsk.fusion.CalculationAccuracy.LowCalculationAccuracy).area
    return a


def build(design, comp, ui):
    G = GEOM
    P = G['params']
    JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
    CUT = adsk.fusion.FeatureOperations.CutFeatureOperation
    NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    notes = []

    # ---- user parameters (drive the box, fill line, recess depth and core height)
    up = design.userParameters
    def param(name, val_mm, comment):
        if up.itemByName(name) is None:
            up.add(name, adsk.core.ValueInput.createByString('%g mm' % val_mm), 'mm', comment)
    param('sil_h', P['sil_h'], 'Silicone ring height = fill line above the mold floor')
    param('freeboard', P['freeboard'], 'Wall height above the fill line')
    param('floor_t', P['floor_t'], 'Mold floor thickness')
    param('groove_w', P['groove_w'], 'Fill-line groove width')
    param('board_t', P['board_t'], 'PCB thickness incl. copper (board recess depth)')

    # ---- 1. mold box (new body) and cavity
    log('stage: 1. mold box (new body) and cavity')
    sk = comp.sketches.add(comp.xYConstructionPlane); sk.name = 'Box outline'
    draw_rounded_rect(sk, G['box']['w'], G['box']['l'], G['box']['r'])
    box = extrude(comp, outer_profiles(sk, 1), G['box']['z0'], G['box']['z1'] - G['box']['z0'], NEW, 'Mold box',
                  z0_expr='-floor_t', dist_expr='floor_t + sil_h + freeboard')
    mold_body = box.bodies.item(0)
    mold_body.name = 'Mold'

    sk = comp.sketches.add(comp.xYConstructionPlane); sk.name = 'Cavity outline'
    draw_rounded_rect(sk, G['cavity']['w'], G['cavity']['l'], G['cavity']['r'])
    extrude(comp, outer_profiles(sk, 1), 0.0, G['cavity']['z1'], CUT, 'Cavity',
            participants=[mold_body], dist_expr='sil_h + freeboard + 1 mm')

    # ---- 2. fill-line groove on the inner walls
    log('stage: 2. fill-line groove on the inner walls')
    sk = comp.sketches.add(comp.xYConstructionPlane); sk.name = 'Fill-line groove'
    draw_rounded_rect(sk, G['groove']['w_out'], G['groove']['l_out'], G['cavity']['r'] + P['groove_d'])
    draw_rounded_rect(sk, G['groove']['w_in'], G['groove']['l_in'], G['cavity']['r'])
    extrude(comp, outer_profiles(sk, 2), G['groove']['z0'], G['groove']['h'], CUT, 'Fill line (sil_h)',
            participants=[mold_body], z0_expr='sil_h - groove_w/2', dist_expr='groove_w')

    # ---- 3. board plateau (the recess the PCB drops into)
    log('stage: 3. board plateau (the recess the PCB drops into)')
    sk = comp.sketches.add(comp.xYConstructionPlane); sk.name = 'Board plateau'
    draw_rounded_rect(sk, G['plateau']['w'], G['plateau']['l'], G['plateau']['r'])
    extrude(comp, outer_profiles(sk, 1), 0.0, G['plateau']['z1'], JOIN, 'Board plateau (recess)', dist_expr='board_t')

    # ---- 4. window core with draft.  Fusion's taper-angle sign convention is
    log('stage: 4. window core with draft.  Fusions taper-angle sign convention is')
    # checked on a throw-away test extrusion so the draft is guaranteed inward.
    taper_sign = None
    for sign in (-1.0, +1.0):
        sk = comp.sketches.add(comp.xYConstructionPlane); sk.name = 'taper test'
        sk.sketchCurves.sketchLines.addTwoPointRectangle(P3(100, 100), P3(105, 105))
        profs = outer_profiles(sk, 1)
        a0 = profile_area(profs)
        f = extrude(comp, profs, 0.0, 5.0, NEW, 'taper test', taper_deg=sign * 5.0)
        a1 = sum(f.endFaces.item(i).area for i in range(f.endFaces.count))
        f.deleteMe(); sk.deleteMe()
        if a1 < a0 * 0.99:            # top smaller than base -> this sign drafts inward
            taper_sign = sign
            break
    if taper_sign is None:
        raise RuntimeError('could not establish the taper sign convention')
    notes.append('core draft: %+g deg (Fusion sign convention checked, inward)' % (taper_sign * G['core']['taper_deg']))
    polygon_feature(comp, 'Window core', G['core']['polys'], G['core']['z0'], G['core']['z1'], JOIN,
                    taper_deg=taper_sign * G['core']['taper_deg'], dist_expr='sil_h + freeboard')

    # ---- 5. lower the core over J2 so the cast ring gets its catch floor
    log('stage: 5. lower the core over J2 so the cast ring gets its catch floor')
    for k, poly in enumerate(G['catch']['polys']):
        sk = comp.sketches.add(comp.xYConstructionPlane); sk.name = 'J2 catch floor'
        draw_polygon(sk, poly['ext'])
        extrude(comp, outer_profiles(sk, 1), G['catch']['z0'], G['catch']['z1'] - G['catch']['z0'], CUT,
                'J2 catch floor (core top at %.2f mm)' % G['catch']['z0'], participants=[mold_body])

    # ---- 6. label (raised on the floor, mirrored so it reads correctly on the cast)
    log('stage: 6. label (raised on the floor, mirrored so it reads correctly on the cast)')
    try:
        L = G['label']
        sk = comp.sketches.add(comp.xYConstructionPlane); sk.name = 'Label'
        texts = sk.sketchTexts
        tin = texts.createInput2(L['text'], mm(L['height']))
        half_len = L['height'] * 0.75 * len(L['text']) / 2.0 + 2.0
        tin.setAsMultiLine(P3(L['x'] - half_len, L['y'] - L['height']),
                           P3(L['x'] + half_len, L['y'] + L['height']),
                           adsk.core.HorizontalAlignments.CenterHorizontalAlignment,
                           adsk.core.VerticalAlignments.MiddleVerticalAlignment, 0)
        for attr, val in (('angle', math.pi / 2.0), ('isHorizontalFlip', True), ('fontName', 'Arial')):
            try:
                setattr(tin, attr, val)
            except Exception:
                notes.append('label: could not set %s' % attr)
        txt = texts.add(tin)
        inp = comp.features.extrudeFeatures.createInput(txt, JOIN)
        inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(L['depth'])))
        f = comp.features.extrudeFeatures.add(inp); f.name = 'Label'
    except Exception:
        notes.append('label skipped: ' + traceback.format_exc().splitlines()[-1])

    # ---- 7. reference: the cast silicone ring
    log('stage: 7. reference: the cast silicone ring')
    if MAKE_SILICONE_BODY:
        try:
            S = G['silicone']
            sk = comp.sketches.add(comp.xYConstructionPlane); sk.name = 'Silicone block'
            draw_rounded_rect(sk, S['w'], S['l'], S['r'])
            blk = extrude(comp, outer_profiles(sk, 1), 0.0, S['z1'], NEW, 'Silicone block', dist_expr='sil_h')
            sil_body = blk.bodies.item(0)
            tools = adsk.core.ObjectCollection.create(); tools.add(mold_body)
            cin = comp.features.combineFeatures.createInput(sil_body, tools)
            cin.operation = CUT
            cin.isKeepToolBodies = True
            cf = comp.features.combineFeatures.add(cin); cf.name = 'Cast silicone = block - mold'
            sil_body.name = 'Silicone carrier (cast result, reference)'
            sil_body.isVisible = False
        except Exception:
            notes.append('silicone reference body skipped: ' + traceback.format_exc().splitlines()[-1])

    return notes


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        try:
            open(LOG, 'w').close()
        except Exception:
            pass
        log('ReflowMold start; Fusion %s' % app.version)
        design = adsk.fusion.Design.cast(app.activeProduct)
        reuse = False
        try:
            r = design.rootComponent
            reuse = (r.bRepBodies.count == 0 and r.occurrences.count == 0 and r.sketches.count == 0)
        except Exception:
            reuse = False
        if not reuse:
            doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
            design = adsk.fusion.Design.cast(app.activeProduct)
        log('using %s design' % ('the empty active' if reuse else 'a new'))
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        um = design.unitsManager
        try:
            design.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
        except Exception:
            pass
        root = design.rootComponent
        # the root component cannot be renamed, so build inside a named sub-component
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        comp = occ.component
        comp.name = 'RCM v1.01 reflow carrier mold'
        notes = build(design, comp, ui)
        log('build done; notes: %s' % notes)
        app.activeViewport.fit()
        G = GEOM
        ui.messageBox('Open-frame reflow carrier mold built.\n\nMold box %.1f x %.1f x %.1f mm, silicone ring %.1f x %.1f x %.1f mm.\n'
                      'User parameters: sil_h, freeboard, floor_t, groove_w, board_t.\n%s'
                      % (G['box']['w'], G['box']['l'], G['box']['z1'] - G['box']['z0'],
                         G['silicone']['w'], G['silicone']['l'], G['silicone']['z1'],
                         '\n'.join(notes)))
    except Exception:
        log('FAILED:\n' + traceback.format_exc())
        if ui:
            ui.messageBox('ReflowMold failed (details in ReflowMold.log next to the script):\n{}'.format(traceback.format_exc()))
