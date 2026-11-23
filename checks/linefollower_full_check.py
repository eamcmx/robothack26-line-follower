# The check of prompts/LineFollower_full.txt, measured in Autodesk Fusion on what a script built.
# Use it: set PATH below to the script the model wrote, then run this file in Fusion (Utilities > Scripts and
# Add-Ins > the green + > pick this file > Run). It calls that script's build() and prints ok or FAIL for each
# acceptance line of the prompt; open the Text Commands window (View > Show Text Commands) to read it.
import math, traceback
import adsk.core, adsk.fusion

PATH = r"C:/path/to/your/line_follower.py"   # set this to the script you want to check
fails = []
NAMES = (['Chassis', 'Motor L', 'Motor R', 'Motor bracket L', 'Motor bracket R', 'Wheel L', 'Wheel R', 'Tyre L',
          'Tyre R', 'Ball', 'Caster housing', 'Caster spacer 1', 'Caster spacer 2', 'Sensor board', 'Sensor bracket L',
          'Sensor bracket R', 'Carrier', 'Arduino Nano', 'Battery holder', 'Cell 1', 'Cell 2', 'Test board', 'Line',
          'Set screw L', 'Set screw R'] + ['Sensor %d' % i for i in range(1, 6)] + ['Standoff %d' % i for i in range(1, 5)])


def ok(cond, msg):
    print(('  ok   ' if cond else '  FAIL ') + msg)
    if not cond:
        fails.append(msg)


def low_z(occ):
    zs = [b.boundingBox.minPoint.z * 10 for b in occ.bRepBodies]
    return min(zs) if zs else None


def allowed(a, b):
    """The pairs the prompt lets touch inside each other."""
    s = {a, b}
    side = lambda n: n[-1] if n[-2:] in (' L', ' R') else None
    if {a[:5], b[:5]} == {'Motor', 'Wheel'} and side(a) == side(b) and 'bracket' not in a + b:
        return True                                      # the shaft in its wheel's bore
    if any(n.startswith('Set screw') for n in s) and (any(n.startswith('Motor ') and 'bracket' not in n for n in s) or any(n.startswith('Wheel') for n in s)):
        return True                                      # a set screw in its boss, on the shaft's flat
    if s & {'Cell 1', 'Cell 2'} and 'Battery holder' in s:
        return True
    if any(n.startswith('Screw ') for n in s) and any(n.startswith('Standoff') or n.startswith('Nut ') for n in s):
        return True                                      # a screw in a standoff or its nut
    return False


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
        print('  FAIL the script did not build: ' + traceback.format_exc().strip().splitlines()[-1])
        print(traceback.format_exc().strip().splitlines()[-3] if len(traceback.format_exc().strip().splitlines()) > 2 else '')
        print('%d FAILED' % (len(fails) + 1))
        return
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    occs = {o.component.name: o for o in root.allOccurrences}
    missing = [n for n in NAMES if n not in occs]
    ok(not missing, 'every named component exists%s' % ((' (missing: %s)' % ', '.join(missing)) if missing else ''))
    many = [n for n in NAMES if n in occs and occs[n].component.bRepBodies.count != 1 and n != 'Battery holder']
    ok(not many, 'one body each%s' % ((' (not one: %s)' % ', '.join(many)) if many else ''))
    screws = [n for n in occs if n.startswith('Screw ')]
    nuts = [n for n in occs if n.startswith('Nut ')]
    ok(len(screws) >= 20 and len(nuts) >= 14, 'at least 20 screws and 14 nuts: %d screws, %d nuts' % (len(screws), len(nuts)))
    for n in ('Chassis', 'Motor bracket L', 'Tyre L', 'Wheel L', 'Ball', 'Sensor board', 'Standoff 1', 'Cell 1', 'Screw 1'):
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
    bad = [(a, b, round(v, 2)) for a, b, v in pairs if not allowed(a, b)]
    ok(not bad, 'no part inside another, but the pairs the prompt allows%s' % ((': %d pairs, e.g. %s' % (len(bad), bad[:5])) if bad else ''))
    for n in ('Tyre L', 'Tyre R', 'Ball'):
        z = low_z(occs[n]) if n in occs else None
        ok(z is not None and abs(z) < 0.1, '%s touches the ground: lowest point %s mm' % (n, 'none' if z is None else '%.3f' % z))
    sz = [low_z(occs['Sensor %d' % i]) for i in range(1, 6) if 'Sensor %d' % i in occs]
    ok(len(sz) == 5 and all(abs(z - 4) < 0.2 for z in sz), 'sensors 4 mm above the ground: %s' % ['%.2f' % z for z in sz])
    js = {j.name: j for j in list(root.allAsBuiltJoints) + list(root.allJoints)}
    j = js.get('Wheel L turns')
    ok(j is not None and hasattr(j.jointMotion, 'rotationValue'), 'a revolute joint "Wheel L turns"')
    if j is not None and hasattr(j.jointMotion, 'rotationValue') and 'Chassis' in occs and 'Wheel L' in occs and 'Tyre L' in occs:
        c0 = occs['Chassis'].transform2.translation.asArray()
        w0 = [occs['Wheel L'].transform2.getCell(0, k) for k in range(3)]
        t0 = [occs['Tyre L'].transform2.getCell(0, k) for k in range(3)]
        j.jointMotion.rotationValue = math.radians(90)
        adsk.doEvents()
        w3 = [occs['Wheel L'].transform2.getCell(0, i) for i in range(3)]
        t3 = [occs['Tyre L'].transform2.getCell(0, i) for i in range(3)]
        for k in range(4, 13):
            j.jointMotion.rotationValue = math.radians(30 * k - (1e-3 if k == 12 else 0))
            adsk.doEvents()
        c1 = occs['Chassis'].transform2.translation.asArray()
        moved = max(abs(a - b) for a, b in zip(c0, c1)) * 10
        turned = max(abs(a - b) for a, b in zip(w0, w3))
        tyre = max(abs(a - b) for a, b in zip(t0, t3))
        ok(turned > 0.5 and tyre > 0.5 and moved < 0.01,
           'driving "Wheel L turns" turns the wheel and its tyre (at 90 deg: %.2f, %.2f) and the chassis stays (moved %.3f mm)' % (turned, tyre, moved))
        j.jointMotion.rotationValue = 0
        adsk.doEvents()
    total = 0.0
    for o in root.allOccurrences:
        if o.component.name in ('Test board', 'Line'):
            continue
        for b in o.component.bRepBodies:
            total += b.physicalProperties.mass * 1000
    ok(300 <= total <= 700, 'robot mass %.1f g, between 300 and 700 g' % total)
    print('ALL PASS' if not fails else '%d FAILED' % len(fails))
