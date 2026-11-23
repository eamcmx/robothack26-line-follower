import math
import traceback
import adsk.core
import adsk.fusion

app = None
ui = None

# ---------- given inputs (mm) ----------
CHASSIS_L, CHASSIS_W, CHASSIS_T = 150.0, 110.0, 3.0
REAR_R, FRONT_BULGE = 12.0, 18.0
SLOT_L, SLOT_W = 22.0, 8.0

GEARBOX_X, GEARBOX_Y, GEARBOX_Z = 10.0, 24.0, 12.0  # long side (24) along y, height (12) matches the can's diameter
CAN_DIAM, CAN_LEN = 12.0, 15.0
SHAFT_DIAM, SHAFT_LEN = 3.0, 10.0

WHEEL_DIAM, WHEEL_W = 43.0, 19.0
HUB_DIAM, BORE_DIAM = 36.0, 3.2
HOLE_DIAM, HOLE_CIRCLE_DIAM, HOLE_COUNT = 7.0, 22.0, 6

BALL_DIAM = 12.0
HOUSING_OD, HOUSING_WALL = 16.0, 1.0
HOUSING_SETBACK = 15.0  # behind front edge

SENSOR_W, SENSOR_D, SENSOR_H = 10.0, 6.0, 7.0
SENSOR_SPACING, SENSOR_COUNT = 14.0, 5
SENSOR_BOARD_L, SENSOR_BOARD_W, SENSOR_BOARD_T = 72.0, 14.0, 1.6
SENSOR_BOTTOM_Z = 4.0

CTRL_L, CTRL_W, CTRL_T = 45.0, 18.0, 1.6
STANDOFF_DIAM, STANDOFF_H = 5.0, 10.0

BATT_L, BATT_W, BATT_H = 77.0, 41.0, 20.0
CELL_DIAM, CELL_LEN = 18.0, 65.0

TESTBOARD_L, TESTBOARD_W, TESTBOARD_T = 600.0, 400.0, 5.0
LINE_W = 19.0
LINE_STRAIGHT = 200.0
LINE_ARC_R = 120.0

WHEEL_Y_CLEAR = 3.0

# ---------- design choices (documented to the user, not fixed by the prompt) ----------
MOTOR_X = -10.0
CTRL_X = 35.0
BATT_X = -30.0

# ---------- derived (computed, never typed) ----------
WHEEL_R = WHEEL_DIAM / 2.0
HUB_R = HUB_DIAM / 2.0
BORE_R = BORE_DIAM / 2.0
HOLE_R = HOLE_DIAM / 2.0
HOLE_CIRCLE_R = HOLE_CIRCLE_DIAM / 2.0
CAN_R = CAN_DIAM / 2.0
SHAFT_R = SHAFT_DIAM / 2.0
BALL_R = BALL_DIAM / 2.0
HOUSING_OR = HOUSING_OD / 2.0
HOUSING_IR = HOUSING_OR - HOUSING_WALL
STANDOFF_R = STANDOFF_DIAM / 2.0
CELL_R = CELL_DIAM / 2.0

AXLE_H = WHEEL_R
CHASSIS_UNDERSIDE_Z = AXLE_H + GEARBOX_Z / 2.0
CHASSIS_TOP_Z = CHASSIS_UNDERSIDE_Z + CHASSIS_T

HALF_L = CHASSIS_L / 2.0
HALF_W = CHASSIS_W / 2.0
REAR_X = -HALF_L
FRONT_X = HALF_L

BALL_X = (FRONT_X + FRONT_BULGE) - HOUSING_SETBACK  # measured behind the bulge tip, the actual front edge
BALL_TOP_Z = 2.0 * BALL_R
HOUSING_LEN = CHASSIS_UNDERSIDE_Z - BALL_TOP_Z

SENSOR_BOARD_BOTTOM_Z = SENSOR_BOTTOM_Z + SENSOR_H
SENSOR_BOARD_TOP_Z = SENSOR_BOARD_BOTTOM_Z + SENSOR_BOARD_T
SENSOR_X = 30.0  # choice: set back from the ball/housing so the two don't occupy the same footprint

CTRL_BOTTOM_Z = CHASSIS_TOP_Z + STANDOFF_H
CTRL_TOP_Z = CTRL_BOTTOM_Z + CTRL_T

BATT_BOTTOM_Z = CHASSIS_TOP_Z
BATT_TOP_Z = BATT_BOTTOM_Z + BATT_H

ARC_CX = SENSOR_X + LINE_STRAIGHT
ARC_CY = LINE_ARC_R
LINE_R_IN = LINE_ARC_R - LINE_W / 2.0
LINE_R_OUT = LINE_ARC_R + LINE_W / 2.0

BOARD_CX = (REAR_X + (ARC_CX + LINE_R_OUT)) / 2.0
BOARD_CY = (-80.0 + ARC_CY) / 2.0


# ---------- small geometry helpers ----------
def mm(v):
    return v / 10.0


def pt(x, y, z):
    return adsk.core.Point3D.create(mm(x), mm(y), mm(z))


def vec(x, y, z):
    return adsk.core.Vector3D.create(x, y, z)


def tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def make_box(cx, cy, cz, dx, dy, dz):
    obb = adsk.core.OrientedBoundingBox3D.create(pt(cx, cy, cz), vec(1, 0, 0), vec(0, 1, 0), mm(dx), mm(dy), mm(dz))
    return tbm().createBox(obb)


def make_cyl(cx, cy, cz, axis, radius, height):
    half = height / 2.0
    if axis == 'x':
        p0, p1 = pt(cx - half, cy, cz), pt(cx + half, cy, cz)
    elif axis == 'y':
        p0, p1 = pt(cx, cy - half, cz), pt(cx, cy + half, cz)
    else:
        p0, p1 = pt(cx, cy, cz - half), pt(cx, cy, cz + half)
    return tbm().createCylinderOrCone(p0, mm(radius), p1, mm(radius))


def make_sphere(cx, cy, cz, radius):
    return tbm().createSphere(pt(cx, cy, cz), mm(radius))


def union(a, b):
    tbm().booleanOperation(a, b, adsk.fusion.BooleanTypes.UnionBooleanType)
    return a


def subtract(a, b):
    tbm().booleanOperation(a, b, adsk.fusion.BooleanTypes.DifferenceBooleanType)
    return a


def intersect(a, b):
    tbm().booleanOperation(a, b, adsk.fusion.BooleanTypes.IntersectionBooleanType)
    return a


def make_quarter_ring(cx, cy, cz, r_in, r_out, height, qx_sign, qy_sign):
    outer = make_cyl(cx, cy, cz, 'z', r_out, height)
    inner = make_cyl(cx, cy, cz, 'z', r_in, height)
    ring = subtract(outer, inner)
    big = 2000.0
    quad = make_box(cx + qx_sign * big / 2.0, cy + qy_sign * big / 2.0, cz, big, big, height + 2.0)
    return intersect(ring, quad)


def make_slot(cx, cy, cz, length_total, width, height):
    r = width / 2.0
    half_straight = (length_total - width) / 2.0
    body = make_box(cx, cy, cz, length_total - width, width, height)
    c1 = make_cyl(cx - half_straight, cy, cz, 'z', r, height)
    c2 = make_cyl(cx + half_straight, cy, cz, 'z', r, height)
    return union(union(body, c1), c2)


def new_component(root, name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ


def finish_body(comp, body, name):
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(body, bf)
    bf.finishEdit()
    b = comp.bRepBodies.item(comp.bRepBodies.count - 1)
    b.name = name
    return b


def find_material(names):
    for n in names:
        for i in range(app.materialLibraries.count):
            lib = app.materialLibraries.item(i)
            try:
                m = lib.materials.itemByName(n)
                if m:
                    return m
            except Exception:
                pass
    return None


def find_appearance(names):
    for n in names:
        for i in range(app.materialLibraries.count):
            lib = app.materialLibraries.item(i)
            try:
                a = lib.appearances.itemByName(n)
                if a:
                    return a
            except Exception:
                pass
    return None


def apply_material(body, names):
    m = find_material(names)
    if m:
        body.material = m


def apply_appearance(body, names):
    a = find_appearance(names)
    if a:
        body.appearance = a


BOARD_MATERIALS = ['FR-4', 'FR4', 'ABS Plastic']


# ---------- parts ----------
def build_chassis(root):
    occ = new_component(root, 'Chassis')
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sp = sk.sketchPoints
    A = sp.add(pt(REAR_X, -HALF_W, 0))
    B = sp.add(pt(FRONT_X, -HALF_W, 0))
    C = sp.add(pt(FRONT_X, HALF_W, 0))
    D = sp.add(pt(REAR_X, HALF_W, 0))
    lines = sk.sketchCurves.sketchLines
    lines.addByTwoPoints(A, B)
    sk.sketchCurves.sketchArcs.addByThreePoints(B, pt(FRONT_X + FRONT_BULGE, 0, 0), C)
    lines.addByTwoPoints(C, D)
    lines.addByTwoPoints(D, A)
    prof = sk.profiles.item(0)
    extrudes = comp.features.extrudeFeatures
    extInput = extrudes.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(CHASSIS_T)))
    extInput.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(mm(CHASSIS_UNDERSIDE_Z)))
    ext = extrudes.add(extInput)
    body = ext.bodies.item(0)

    fillet_edges = adsk.core.ObjectCollection.create()
    for e in body.edges:
        s, en = e.startVertex.geometry, e.endVertex.geometry
        if abs(s.x - en.x) < 1e-6 and abs(s.y - en.y) < 1e-6 and abs(s.x - mm(REAR_X)) < 1e-4:
            fillet_edges.add(e)
    filletInput = comp.features.filletFeatures.createInput()
    filletInput.addConstantRadiusEdgeSet(fillet_edges, adsk.core.ValueInput.createByReal(mm(REAR_R)), True)
    comp.features.filletFeatures.add(filletInput)

    slot_h = CHASSIS_T + 1.0
    slot_cz = CHASSIS_UNDERSIDE_Z + CHASSIS_T / 2.0
    slot_cy = HALF_W - GEARBOX_Y / 2.0
    slot1 = finish_body(comp, make_slot(MOTOR_X, slot_cy, slot_cz, SLOT_L, SLOT_W, slot_h), 'slot1')
    slot2 = finish_body(comp, make_slot(MOTOR_X, -slot_cy, slot_cz, SLOT_L, SLOT_W, slot_h), 'slot2')
    tools = adsk.core.ObjectCollection.create()
    tools.add(slot1)
    tools.add(slot2)
    combInput = comp.features.combineFeatures.createInput(body, tools)
    combInput.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
    combInput.isKeepToolBodies = False
    comp.features.combineFeatures.add(combInput)

    body.name = 'Chassis'
    apply_material(body, ['Aluminum'])
    apply_appearance(body, ['Aluminum - Satin', 'Aluminum - Brushed'])
    return occ, body


def build_motor(root, side, s):
    name = 'Motor ' + side
    occ = new_component(root, name)
    comp = occ.component
    chassis_side_y = s * HALF_W
    gy = chassis_side_y - s * GEARBOX_Y / 2.0
    cy = chassis_side_y - s * (GEARBOX_Y + CAN_LEN / 2.0)
    sy = chassis_side_y + s * SHAFT_LEN / 2.0
    gearbox = make_box(MOTOR_X, gy, AXLE_H, GEARBOX_X, GEARBOX_Y, GEARBOX_Z)
    can = make_cyl(MOTOR_X, cy, AXLE_H, 'y', CAN_R, CAN_LEN)
    shaft = make_cyl(MOTOR_X, sy, AXLE_H, 'y', SHAFT_R, SHAFT_LEN)
    body = union(union(gearbox, can), shaft)
    b = finish_body(comp, body, name)
    apply_material(b, ['Steel'])
    return occ, b


def build_wheel(root, side, s):
    name = 'Wheel ' + side
    occ = new_component(root, name)
    comp = occ.component
    wy = s * (HALF_W + WHEEL_Y_CLEAR + WHEEL_W / 2.0)
    hub = make_cyl(MOTOR_X, wy, AXLE_H, 'y', HUB_R, WHEEL_W)
    bore = make_cyl(MOTOR_X, wy, AXLE_H, 'y', BORE_R, WHEEL_W + 2.0)
    hub = subtract(hub, bore)
    for i in range(HOLE_COUNT):
        ang = math.radians(360.0 * i / HOLE_COUNT)
        ox = MOTOR_X + HOLE_CIRCLE_R * math.cos(ang)
        oz = AXLE_H + HOLE_CIRCLE_R * math.sin(ang)
        hole = make_cyl(ox, wy, oz, 'y', HOLE_R, WHEEL_W + 2.0)
        hub = subtract(hub, hole)
    body = finish_body(comp, hub, name)
    apply_material(body, ['ABS Plastic'])
    apply_appearance(body, ['Paint - Enamel Glossy (White)'])
    return occ, body, wy


def build_tyre(root, side, s, wy):
    name = 'Tyre ' + side
    occ = new_component(root, name)
    comp = occ.component
    outer = make_cyl(MOTOR_X, wy, AXLE_H, 'y', WHEEL_R, WHEEL_W)
    inner = make_cyl(MOTOR_X, wy, AXLE_H, 'y', HUB_R, WHEEL_W + 2.0)
    outer = subtract(outer, inner)
    body = finish_body(comp, outer, name)
    apply_material(body, ['Rubber'])
    apply_appearance(body, ['Rubber - Soft', 'Paint - Enamel Glossy (Black)'])
    return occ, body


def build_ball(root):
    occ = new_component(root, 'Ball')
    comp = occ.component
    s = make_sphere(BALL_X, 0, BALL_R, BALL_R)
    b = finish_body(comp, s, 'Ball')
    apply_material(b, ['Steel'])
    return occ, b


def build_ball_housing(root):
    occ = new_component(root, 'Ball housing')
    comp = occ.component
    cz = (BALL_TOP_Z + CHASSIS_UNDERSIDE_Z) / 2.0
    outer = make_cyl(BALL_X, 0, cz, 'z', HOUSING_OR, HOUSING_LEN)
    inner = make_cyl(BALL_X, 0, cz, 'z', HOUSING_IR, HOUSING_LEN + 2.0)
    outer = subtract(outer, inner)
    b = finish_body(comp, outer, 'Ball housing')
    apply_material(b, ['ABS Plastic'])
    return occ, b


def build_sensor_board(root):
    occ = new_component(root, 'Sensor board')
    comp = occ.component
    cz = (SENSOR_BOARD_BOTTOM_Z + SENSOR_BOARD_TOP_Z) / 2.0
    b = make_box(SENSOR_X, 0, cz, SENSOR_BOARD_L, SENSOR_BOARD_W, SENSOR_BOARD_T)
    body = finish_body(comp, b, 'Sensor board')
    apply_material(body, BOARD_MATERIALS)
    apply_appearance(body, ['Paint - Enamel Glossy (Green)'])
    return occ, body


def build_sensor(root, i, y):
    name = 'Sensor %d' % i
    occ = new_component(root, name)
    comp = occ.component
    cz = (SENSOR_BOTTOM_Z + SENSOR_BOARD_BOTTOM_Z) / 2.0
    b = make_box(SENSOR_X, y, cz, SENSOR_W, SENSOR_D, SENSOR_H)
    body = finish_body(comp, b, name)
    apply_material(body, ['ABS Plastic'])
    apply_appearance(body, ['Paint - Enamel Glossy (Black)'])
    return occ, body


def build_controller(root):
    occ = new_component(root, 'Controller')
    comp = occ.component
    cz = (CTRL_BOTTOM_Z + CTRL_TOP_Z) / 2.0
    b = make_box(CTRL_X, 0, cz, CTRL_L, CTRL_W, CTRL_T)
    body = finish_body(comp, b, 'Controller')
    apply_material(body, BOARD_MATERIALS)
    apply_appearance(body, ['Paint - Enamel Glossy (Blue)'])
    return occ, body


def build_standoff(root, i, dx, dy):
    name = 'Standoff %d' % i
    occ = new_component(root, name)
    comp = occ.component
    cz = (CHASSIS_TOP_Z + CTRL_BOTTOM_Z) / 2.0
    b = make_cyl(CTRL_X + dx, dy, cz, 'z', STANDOFF_R, STANDOFF_H)
    body = finish_body(comp, b, name)
    apply_material(body, ['Brass'])
    return occ, body


def build_battery_holder(root):
    occ = new_component(root, 'Battery holder')
    comp = occ.component
    cz = (BATT_BOTTOM_Z + BATT_TOP_Z) / 2.0
    holder = make_box(BATT_X, 0, cz, BATT_L, BATT_W, BATT_H)
    cav1 = make_cyl(BATT_X, -10.0, cz, 'x', CELL_R, CELL_LEN)
    cav2 = make_cyl(BATT_X, 10.0, cz, 'x', CELL_R, CELL_LEN)
    holder = subtract(subtract(holder, cav1), cav2)
    body = finish_body(comp, holder, 'Battery holder')
    apply_material(body, ['ABS Plastic'])
    apply_appearance(body, ['Paint - Enamel Glossy (Black)'])
    return occ, body


def build_cell(root, i, y):
    name = 'Cell %d' % i
    occ = new_component(root, name)
    comp = occ.component
    cz = (BATT_BOTTOM_Z + BATT_TOP_Z) / 2.0
    c = make_cyl(BATT_X, y, cz, 'x', CELL_R, CELL_LEN)
    body = finish_body(comp, c, name)
    apply_material(body, ['Aluminum'])
    return occ, body


def groove_profile(cz, height):
    mid_x = (SENSOR_X + ARC_CX) / 2.0
    straight = make_box(mid_x, 0, cz, LINE_STRAIGHT, LINE_W, height)
    arc = make_quarter_ring(ARC_CX, ARC_CY, cz, LINE_R_IN, LINE_R_OUT, height, 1, -1)
    return union(straight, arc)


def build_test_board(root):
    occ = new_component(root, 'Test board')
    comp = occ.component
    board = make_box(BOARD_CX, BOARD_CY, -TESTBOARD_T / 2.0, TESTBOARD_L, TESTBOARD_W, TESTBOARD_T)
    board_body = finish_body(comp, board, 'Test board')
    groove = finish_body(comp, groove_profile(-0.25, 0.7), 'groove_tool')
    tools = adsk.core.ObjectCollection.create()
    tools.add(groove)
    combInput = comp.features.combineFeatures.createInput(board_body, tools)
    combInput.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
    combInput.isKeepToolBodies = False
    comp.features.combineFeatures.add(combInput)
    board_body.name = 'Test board'
    apply_material(board_body, ['ABS Plastic'])
    apply_appearance(board_body, ['Paint - Enamel Glossy (White)'])
    return occ, board_body


def build_line(root):
    occ = new_component(root, 'Line')
    comp = occ.component
    body = finish_body(comp, groove_profile(-0.25, 0.5), 'Line')
    apply_material(body, ['ABS Plastic'])
    apply_appearance(body, ['Paint - Enamel Glossy (Black)'])
    return occ, body


def add_rigid(root, moving_occ, fixed_occ, jname):
    geom = adsk.fusion.JointGeometry.createByPoint(moving_occ.component.originConstructionPoint)
    inp = root.asBuiltJoints.createInput(moving_occ, fixed_occ, geom)
    inp.setAsRigidJointMotion()
    j = root.asBuiltJoints.add(inp)
    j.name = jname


def add_revolute(root, wheel_occ, wy, chassis_occ, jname):
    comp = wheel_occ.component
    planeInput = comp.constructionPlanes.createInput()
    planeInput.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(AXLE_H)))
    plane = comp.constructionPlanes.add(planeInput)
    sk = comp.sketches.add(plane)
    axis_point = sk.sketchPoints.add(adsk.core.Point3D.create(mm(MOTOR_X), mm(wy), 0))
    geom = adsk.fusion.JointGeometry.createByPoint(axis_point)
    inp = root.asBuiltJoints.createInput(wheel_occ, chassis_occ, geom)
    inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.YAxisJointDirection)
    j = root.asBuiltJoints.add(inp)
    j.name = jname


def build():
    global app, ui
    app = adsk.core.Application.get()
    ui = app.userInterface
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent

    chassis_occ, chassis_body = build_chassis(root)

    motor_occ, motor_body = {}, {}
    wheel_occ, wheel_body, wheel_y = {}, {}, {}
    tyre_occ, tyre_body = {}, {}
    for side, s in (('L', 1.0), ('R', -1.0)):
        motor_occ[side], motor_body[side] = build_motor(root, side, s)
        wheel_occ[side], wheel_body[side], wheel_y[side] = build_wheel(root, side, s)
        tyre_occ[side], tyre_body[side] = build_tyre(root, side, s, wheel_y[side])

    ball_occ, _ = build_ball(root)
    housing_occ, _ = build_ball_housing(root)
    board_occ, _ = build_sensor_board(root)
    sensor_occ = {}
    for i in range(1, SENSOR_COUNT + 1):
        y = (i - (SENSOR_COUNT + 1) / 2.0) * SENSOR_SPACING
        sensor_occ[i], _ = build_sensor(root, i, y)
    ctrl_occ, _ = build_controller(root)
    standoff_occ = {}
    for i, (dx, dy) in enumerate(((CTRL_L / 2.0 - 4.0, CTRL_W / 2.0 - 4.0),
                                   (CTRL_L / 2.0 - 4.0, -(CTRL_W / 2.0 - 4.0)),
                                   (-(CTRL_L / 2.0 - 4.0), CTRL_W / 2.0 - 4.0),
                                   (-(CTRL_L / 2.0 - 4.0), -(CTRL_W / 2.0 - 4.0))), 1):
        standoff_occ[i], _ = build_standoff(root, i, dx, dy)
    batt_occ, _ = build_battery_holder(root)
    cell_occ = {}
    for i, y in ((1, -10.0), (2, 10.0)):
        cell_occ[i], _ = build_cell(root, i, y)
    board_test_occ, _ = build_test_board(root)
    line_occ, _ = build_line(root)

    board_test_occ.isGrounded = True

    for side in ('L', 'R'):
        add_rigid(root, motor_occ[side], chassis_occ, 'Motor %s fixed' % side)
        add_revolute(root, wheel_occ[side], wheel_y[side], chassis_occ, 'Wheel %s turns' % side)
        add_rigid(root, tyre_occ[side], wheel_occ[side], 'Tyre %s fixed' % side)
    add_rigid(root, ball_occ, chassis_occ, 'Ball fixed')
    add_rigid(root, housing_occ, chassis_occ, 'Ball housing fixed')
    add_rigid(root, board_occ, chassis_occ, 'Sensor board fixed')
    for i in range(1, SENSOR_COUNT + 1):
        add_rigid(root, sensor_occ[i], chassis_occ, 'Sensor %d fixed' % i)
    add_rigid(root, ctrl_occ, chassis_occ, 'Controller fixed')
    for i in range(1, 5):
        add_rigid(root, standoff_occ[i], chassis_occ, 'Standoff %d fixed' % i)
    add_rigid(root, batt_occ, chassis_occ, 'Battery holder fixed')
    for i in (1, 2):
        add_rigid(root, cell_occ[i], chassis_occ, 'Cell %d fixed' % i)
    add_rigid(root, line_occ, board_test_occ, 'Line fixed')

    return ('Line-follower robot built: chassis, two drive wheels with tyres, ball caster, '
            '5-sensor bar, controller, battery pack, and a test board with a straight-then-curved '
            'black line; Wheel L/R turns are revolute, everything else rigid.')


def run(context):
    try:
        msg = build()
        ui.messageBox(msg)
    except Exception:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
