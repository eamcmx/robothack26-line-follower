# The check of prompts/LineFollower_basic.txt, measured in Autodesk Fusion on what a script built.
# Use it: set PATH below to the script the model wrote, then run this file in Fusion (Utilities > Scripts and
# Add-Ins > the green + > pick this file > Run). It calls that script's build() and prints ok or FAIL for each
# acceptance line of the prompt; open the Text Commands window (View > Show Text Commands) to read it.
import math, traceback
import adsk.core, adsk.fusion

PATH = r"C:/path/to/your/line_follower.py"   # set this to the script you want to check
fails = []
NAMES = (['Chassis', 'Motor L', 'Motor R', 'Wheel L', 'Wheel R', 'Tyre L', 'Tyre R', 'Ball housing', 'Ball',
          'Sensor board', 'Controller', 'Battery holder', 'Cell 1', 'Cell 2', 'Test board', 'Line'] +
         ['Sensor %d' % i for i in range(1, 6)] + ['Standoff %d' % i for i in range(1, 5)])


def ok(cond, msg):
    print(('  ok   ' if cond else '  FAIL ') + msg)
    if not cond:
        fails.append(msg)


def low_z(occ):
    """Lowest point of an occurrence's bodies, in mm, from their bounding boxes in world coordinates."""
    zs = [b.boundingBox.minPoint.z * 10 for b in occ.bRepBodies]
    return min(zs) if zs else None


def run(_context):
    app = adsk.core.Application.get()
    try:
        src = open(PATH, encoding='utf-8').read().replace('ui.messageBox(', '_quiet(')
        ns = {'__name__': 'checked', '__file__': PATH, '_quiet': lambda *a, **k: None}
        exec(compile(src, PATH, 'exec'), ns)
        ok('build' in ns, 'the script has build()')
        msg = ns['build']()
        print('  build() said: %s' % (str(msg).splitlines()[0] if msg else ''))
    except Exception:
        print(traceback.format_exc().strip().splitlines()[-1])
        print('  FAIL the script did not build: ' + traceback.format_exc().strip().splitlines()[-1])
        print('%d FAILED' % (len(fails) + 1))
        return
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    occs = {o.component.name: o for o in root.allOccurrences}
    missing = [n for n in NAMES if n not in occs]
    ok(not missing, 'every named component exists%s' % ((' (missing: %s)' % ', '.join(missing)) if missing else ''))
    many = [n for n in NAMES if n in occs and occs[n].component.bRepBodies.count != 1]
    ok(not many, 'one body each%s' % ((' (not one: %s)' % ', '.join(many)) if many else ''))
    for n in ('Chassis', 'Tyre L', 'Wheel L', 'Ball', 'Sensor board', 'Cell 1'):
        if n in occs and occs[n].component.bRepBodies.count:
            b = occs[n].component.bRepBodies.item(0)
            print('  material of %s: %s' % (n, b.material.name if b.material else 'none'))
    col = adsk.core.ObjectCollection.create()
    for o in root.allOccurrences:
        if o.component.bRepBodies.count:
            col.add(o)
    res = []
    if col.count >= 2:
        inp = des.createInterferenceInput(col)
        inp.areCoincidentFacesIncluded = False
        res = des.analyzeInterference(inp)
    pairs = []
    for r in (res or []):
        a, b = r.entityOne, r.entityTwo
        na = a.assemblyContext.component.name if a.assemblyContext else a.name
        nb = b.assemblyContext.component.name if b.assemblyContext else b.name
        pairs.append((na, nb, r.interferenceBody.volume * 1000))
    allowed = lambda x, y: ({x[:5], y[:5]} == {'Motor', 'Wheel'} and x[-1] == y[-1]) or \
        ({x, y} & {'Cell 1', 'Cell 2'} and {x, y} & {'Battery holder'})
    bad = [(a, b, round(v, 1)) for a, b, v in pairs if not allowed(a, b)]
    ok(not bad, 'no part inside another, but shafts in bores and cells in the holder%s' % ((': ' + str(bad[:6])) if bad else ''))
    for n in ('Tyre L', 'Tyre R', 'Ball'):
        z = low_z(occs[n]) if n in occs else None
        ok(z is not None and abs(z) < 0.1, '%s touches the ground: lowest point %s mm' % (n, 'none' if z is None else '%.3f' % z))
    sz = [low_z(occs['Sensor %d' % i]) for i in range(1, 6) if 'Sensor %d' % i in occs]
    ok(len(sz) == 5 and all(abs(z - 4) < 0.2 for z in sz), 'sensors 4 mm above the ground: %s' % ['%.2f' % z for z in sz])
    js = {j.name: j for j in list(root.allAsBuiltJoints) + list(root.allJoints)}
    j = js.get('Wheel L turns')
    ok(j is not None and hasattr(j.jointMotion, 'rotationValue'), 'a revolute joint "Wheel L turns"')
    if j is not None and hasattr(j.jointMotion, 'rotationValue') and 'Chassis' in occs and 'Wheel L' in occs:
        c0 = occs['Chassis'].transform2.translation.asArray()
        w0 = [occs['Wheel L'].transform2.getCell(0, k) for k in range(3)]
        for k in range(1, 13):
            j.jointMotion.rotationValue = math.radians(30 * k - 1e-3 if k == 12 else 30 * k)
            adsk.doEvents()
            if k == 3:
                w3 = [occs['Wheel L'].transform2.getCell(0, i) for i in range(3)]
        c1 = occs['Chassis'].transform2.translation.asArray()
        moved = max(abs(a - b) for a, b in zip(c0, c1)) * 10
        turned = max(abs(a - b) for a, b in zip(w0, w3))
        ok(turned > 0.5 and moved < 0.01, 'driving "Wheel L turns" turns the wheel (at 90 deg its axis moved %.2f) and the chassis stays (moved %.3f mm)' % (turned, moved))
        j.jointMotion.rotationValue = 0
        adsk.doEvents()
    total = 0.0
    for o in root.allOccurrences:
        if o.component.name in ('Test board', 'Line'):
            continue
        for b in o.component.bRepBodies:
            total += b.physicalProperties.mass * 1000
    ok(250 <= total <= 600, 'robot mass %.1f g, between 250 and 600 g' % total)
    print('ALL PASS' if not fails else '%d FAILED' % len(fails))
