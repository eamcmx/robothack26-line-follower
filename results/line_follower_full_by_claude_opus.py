import adsk.core, adsk.fusion, math, traceback

# ---------------------------------------------------------------- INPUTS (mm; x forwards, y left, z up)
# chassis
CH_T, CH_L, CH_W = 3.0, 160.0, 120.0
CH_REAR_R = 10.0
CH_BULGE = 20.0
CH_CHAMFER = 0.5
# gear motors (N20)
GB_W, GB_D, GB_H = 12.0, 10.0, 10.0          # gearbox: along y, along x, z
CAN_D, CAN_FLATS, CAN_L = 12.0, 10.0, 15.0
SHAFT_D, SHAFT_L, SHAFT_FLAT = 3.0, 10.0, 0.5
SHAFT_GAP = 1.0                              # gearbox face to the boss's inner face (decision)
AXLE_X = 20.0                                # wheel axle position along x (decision)
# motor brackets
BR_T, BR_W = 1.5, 10.0
BR_FOOT = 10.0                               # length of each foot (decision)
# wheels
HUB_D, HUB_W = 34.0, 16.0
LH_N, LH_D, LH_PCD = 6, 7.0, 21.0
BOSS_D, BOSS_L = 10.0, 6.0
SET_L = 4.0
TYRE_OD, TYRE_ID, TYRE_W = 43.0, 34.0, 19.0
TREAD_N, TREAD_W, TREAD_DEPTH = 24, 1.5, 1.0
TYRE_CLEAR = 3.0
# ball caster
BALL_D = 9.5
HOUS_D = 14.0
HOUS_BELOW_CENTRE = 1.75                     # housing reaches this far below the ball centre (decision)
HOUS_H = 15.0                                # housing height, bottom to flange top (decision)
HOUS_CLEAR = 0.1                             # ball cavity clearance on the radius (decision)
FL_L, FL_W, FL_T = 30.0, 10.0, 2.0           # flange: along y, along x, thick
FL_PITCH = 22.0
SP_OD, SP_ID = 6.0, 3.2
CASTER_X = -65.0                             # decision
# sensor bar
SB_L, SB_D, SB_T = 72.0, 14.0, 1.6           # along y, along x, thick
SEN_X, SEN_Y, SEN_Z = 10.2, 5.8, 7.0
SEN_N, SEN_PITCH = 5, 14.0
SEN_GROUND = 4.0
SENSOR_X = 90.0                              # sensor bar centre along x (decision)
SBR_T, SBR_W = 1.5, 10.0
SBR_SCREW_Y = 34.0                           # board screws and brackets at y = +-34 (decision)
SBR_TOP_LEN = 12.0                           # top flange length behind the web (decision)
SBR_BOTTOM_PAST = 5.0                        # bottom flange reaches 5 past the board screw (decision)
# controller
CAR_L, CAR_W, CAR_T = 60.0, 40.0, 1.6        # along x, along y
CAR_CX = 22.0                                # decision
NANO_L, NANO_W, NANO_T = 45.0, 18.0, 1.6
USB_L, USB_W, USB_H = 8.0, 7.5, 4.0
PIN_N, PIN_SQ, PIN_PITCH, PIN_H, PIN_ROWS = 15, 0.64, 2.54, 8.0, 15.24
SO_L, SO_AF = 10.0, 5.5
SO_INSET = 4.0                               # standoff centres 4 in from the carrier edges (decision)
# battery
BH_L, BH_W, BH_H = 77.0, 41.0, 20.0          # along y, along x, z
BH_X0 = -55.0                                # rear edge of the holder (decision)
BH_FLOOR = 1.5                               # decision
CELL_D, CELL_L = 18.0, 65.0
BAT_SCREW_Y = 20.0                           # decision
# fasteners
M3 = 3.0
CAP_HD, CAP_HH, CAP_SOCK, CAP_SOCK_D = 5.5, 3.0, 2.5, 1.5
CSK_HD = 6.0
CSK_SOCK, CSK_SOCK_D = 2.0, 1.2              # ISO 10642 M3 socket (decision)
SET_SOCK, SET_SOCK_D = 1.5, 1.5              # grub screw socket (decision)
NUT_AF, NUT_T = 5.5, 2.4
PITCH = 0.5
STD_LENGTHS = (6, 8, 10, 12, 16, 20)
HOLE_D = 3.4
# test board and line
TB_L, TB_W, TB_T = 600.0, 400.0, 5.0
LINE_W, LINE_DEPTH, LINE_STRAIGHT, LINE_R = 19.0, 0.5, 200.0, 120.0
LINE_LEAD = 10.0                             # the line starts 10 behind the sensor bar's centre (decision)

# ---------------------------------------------------------------- DERIVED
TYRE_R = TYRE_OD / 2
AXLE_Z = TYRE_R
CH_UNDER = AXLE_Z + GB_H / 2 + BR_T
CH_TOP = CH_UNDER + CH_T
CH_X0, CH_X1 = -CH_L / 2, CH_L / 2
FRONT_R = ((CH_W / 2) ** 2 + CH_BULGE ** 2) / (2 * CH_BULGE)        # arc through the corners
FRONT_CX = CH_X1 + CH_BULGE - FRONT_R
TYRE_Y0 = CH_W / 2 + TYRE_CLEAR
TYRE_CY = TYRE_Y0 + TYRE_W / 2
HUB_Y0 = TYRE_CY - HUB_W / 2
BOSS_Y0 = HUB_Y0 - BOSS_L
GB_FACE = BOSS_Y0 - SHAFT_GAP
GB_Y0 = GB_FACE - GB_W
CAN_Y0 = GB_Y0 - CAN_L
SHAFT_TIP = GB_FACE + SHAFT_L
FLAT_Z = AXLE_Z + SHAFT_D / 2 - SHAFT_FLAT
assert SHAFT_TIP >= HUB_Y0, 'the shaft must reach through the boss'
BALL_R = BALL_D / 2
HOUS_Z0 = BALL_R - HOUS_BELOW_CENTRE
HOUS_Z1 = HOUS_Z0 + HOUS_H
SP_LEN = CH_UNDER - HOUS_Z1
assert SP_LEN > 0
SEN_Z0 = SEN_GROUND
SB_Z0 = SEN_Z0 + SEN_Z
SB_Z1 = SB_Z0 + SB_T
SBR_DROP = CH_UNDER - SB_Z1                   # bracket height, chassis underside to board top
LINE_X0 = SENSOR_X - LINE_LEAD


def screw_length(stack):
    for L in STD_LENGTHS:
        if L >= stack + PITCH + 1e-6:
            return L
    raise ValueError('no standard length for a stack of %.2f' % stack)


# ---------------------------------------------------------------- geometry helpers (mm in, cm to Fusion)
tbm = None


def P(x, y, z):
    return adsk.core.Point3D.create(x / 10.0, y / 10.0, z / 10.0)


def V(x, y, z):
    return adsk.core.Vector3D.create(x, y, z)


def box(x0, x1, y0, y1, z0, z1):
    c = P((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
    obb = adsk.core.OrientedBoundingBox3D.create(c, V(1, 0, 0), V(0, 1, 0),
                                                 abs(x1 - x0) / 10, abs(y1 - y0) / 10, abs(z1 - z0) / 10)
    return tbm.createBox(obb)


def obox(c, ldir, wdir, l, w, h):
    obb = adsk.core.OrientedBoundingBox3D.create(P(*c), V(*ldir), V(*wdir), l / 10, w / 10, h / 10)
    return tbm.createBox(obb)


def cyl(p1, p2, r):
    return tbm.createCylinderOrCone(P(*p1), r / 10.0, P(*p2), r / 10.0)


def cone(p1, r1, p2, r2):
    return tbm.createCylinderOrCone(P(*p1), r1 / 10.0, P(*p2), r2 / 10.0)


def sphere(c, r):
    return tbm.createSphere(P(*c), r / 10.0)


def union(a, b):
    tbm.booleanOperation(a, b, adsk.fusion.BooleanTypes.UnionBooleanType)
    return a


def cut(a, b):
    tbm.booleanOperation(a, b, adsk.fusion.BooleanTypes.DifferenceBooleanType)
    return a


def inter(a, b):
    tbm.booleanOperation(a, b, adsk.fusion.BooleanTypes.IntersectionBooleanType)
    return a


def hex_z(cx, cy, z0, z1, af, rot_deg=0.0):
    """Hexagonal prism along z, across flats af, one pair of flats normal to rot_deg."""
    h = None
    for k in range(3):
        a = math.radians(rot_deg + 60 * k)
        s = obox((cx, cy, (z0 + z1) / 2), (math.cos(a), math.sin(a), 0), (-math.sin(a), math.cos(a), 0),
                 af, 2 * af, abs(z1 - z0))
        h = s if h is None else inter(h, s)
    return h


def cap_screw(x, y, zs, down, L):
    """ISO 4762 M3. zs: the face the head bears on; down=True: head above, shank going down."""
    d = -1 if down else 1
    zh = zs - d * CAP_HH
    head = cyl((x, y, zh), (x, y, zs), CAP_HD / 2)
    cut(head, hex_z(x, y, zh - d * 0.01, zh + d * CAP_SOCK_D, CAP_SOCK, 90))
    union(head, cyl((x, y, zs), (x, y, zs + d * L), M3 / 2))
    return head


def csk_screw_up(x, y, zs, L):
    """ISO 10642 M3 from below; zs: the underside the head is flush with; L overall."""
    hh = (CSK_HD - M3) / 2                      # 90 degree head
    s = cone((x, y, zs), CSK_HD / 2, (x, y, zs + hh), M3 / 2)
    union(s, cyl((x, y, zs + hh), (x, y, zs + L), M3 / 2))
    cut(s, hex_z(x, y, zs - 0.01, zs + CSK_SOCK_D, CSK_SOCK, 90))
    return s


def nut(x, y, z0, rot=0.0):
    n = hex_z(x, y, z0, z0 + NUT_T, NUT_AF, rot)
    cut(n, cyl((x, y, z0 - 0.1), (x, y, z0 + NUT_T + 0.1), M3 / 2))
    return n


def hole(x, y, z0, z1, d=HOLE_D):
    return cyl((x, y, z0 - 0.2), (x, y, z1 + 0.2), d / 2)


# ---------------------------------------------------------------- materials and appearances
def lib_by_name(name):
    libs = adsk.core.Application.get().materialLibraries
    for i in range(libs.count):
        if libs.item(i).name == name:
            return libs.item(i)
    return None


class Looks:
    def __init__(self, design):
        self.design = design
        mlib = lib_by_name('Fusion Material Library')
        self.mats = {}
        for i in range(mlib.materials.count):
            m = mlib.materials.item(i)
            self.mats[m.name] = m
        alib = lib_by_name('Fusion Appearance Library')
        self.apps = {}
        for i in range(alib.appearances.count):
            a = alib.appearances.item(i)
            self.apps[a.name] = a
        self.made = {}
        self.notes = []

    def material(self, name):
        if name in self.mats:
            return self.mats[name]
        if name == 'FR4':
            for n, m in self.mats.items():
                if n.replace('-', '').replace(' ', '').upper().startswith('FR4'):
                    return m
            self.notes.append('no FR4 in the library, boards in ABS Plastic')
            return self.mats['ABS Plastic']
        for n in sorted(self.mats, key=len):
            if name.lower() in n.lower():
                self.mats[name] = self.mats[n]
                return self.mats[n]
        raise ValueError('material %s not in the library' % name)

    def _base(self, candidates, contains):
        for c in candidates:
            if c in self.apps:
                return self.apps[c]
        for n, a in self.apps.items():
            if contains in n:
                return a
        return None

    def metal(self, key, candidates):
        if key in self.made:
            return self.made[key]
        base = self._base(candidates, key)
        a = self.design.appearances.addByCopy(base, 'LF ' + key) if base else None
        self.made[key] = a
        return a

    def colour(self, key, rgb):
        if key in self.made:
            return self.made[key]
        base = self._base(['Plastic - Matte (White)', 'Plastic - Glossy (White)', 'Plastic - Matte (Yellow)'],
                          'Plastic - Matte')
        a = self.design.appearances.addByCopy(base, 'LF ' + key)
        prop = a.appearanceProperties.itemById('opaque_albedo')
        if prop is None:
            for i in range(a.appearanceProperties.count):
                p = a.appearanceProperties.item(i)
                if p.objectType == adsk.core.ColorProperty.classType():
                    prop = p
                    break
        try:
            prop.value = adsk.core.Color.create(rgb[0], rgb[1], rgb[2], 0)
        except Exception:
            self.notes.append('colour %s not set' % key)
        self.made[key] = a
        return a


# ---------------------------------------------------------------- the build
def build():
    global tbm
    app = adsk.core.Application.get()
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    tbm = adsk.fusion.TemporaryBRepManager.get()
    looks = Looks(design)

    AL = ('Aluminum', lambda: looks.metal('Aluminum', ['Aluminum - Satin', 'Aluminum - Brushed']))
    ST = ('Steel', lambda: looks.metal('Steel', ['Steel - Satin', 'Steel - Polished']))
    BR = ('Brass', lambda: looks.metal('Brass', ['Brass - Polished', 'Brass - Satin']))
    WHITE = lambda: looks.colour('white', (245, 245, 240))
    BLACK = lambda: looks.colour('black', (20, 20, 20))
    GREEN = lambda: looks.colour('green', (20, 110, 40))
    BLUE = lambda: looks.colour('blue', (20, 60, 170))
    NANO_BLUE = lambda: looks.colour('nano blue', (0, 90, 140))

    occs = {}
    used_mats = []

    def add_part(name, body, mat, look, chamfer=None):
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        comp = occ.component
        comp.name = name
        bf = comp.features.baseFeatures.add()
        bf.startEdit()
        comp.bRepBodies.add(body, bf)
        bf.finishEdit()
        if chamfer:
            chamfer(comp)
        b = comp.bRepBodies.item(0)
        b.name = name
        b.material = looks.material(mat)
        a = look()
        if a is not None:
            b.appearance = a
        if b.material.name not in used_mats:
            used_mats.append(b.material.name)
        occs[name] = occ
        return occ

    screws, nuts = [], []

    def add_screw(body):
        n = 'Screw %d' % (len(screws) + 1)
        screws.append(add_part(n, body, ST[0], ST[1]))

    def add_nut(body):
        n = 'Nut %d' % (len(nuts) + 1)
        nuts.append(add_part(n, body, ST[0], ST[1]))

    # ---- fastener positions in the chassis
    feet_x = (AXLE_X - GB_D / 2 - BR_T - BR_FOOT / 2, AXLE_X + GB_D / 2 + BR_T + BR_FOOT / 2)
    br_cy = GB_Y0 + GB_W / 2                                         # bracket centred on the gearbox
    so_pts = [(CAR_CX + sx * (CAR_L / 2 - SO_INSET), sy * (CAR_W / 2 - SO_INSET)) for sx in (-1, 1) for sy in (-1, 1)]
    bh_cx = BH_X0 + BH_W / 2
    sbr_web_x0 = SENSOR_X - SB_D / 2 - SBR_T
    sbr_top_x0 = sbr_web_x0 - SBR_TOP_LEN
    sbr_top_screw_x = (sbr_top_x0 + sbr_web_x0) / 2
    plain_holes = [(fx, s * br_cy) for fx in feet_x for s in (1, -1)]
    plain_holes += [(CASTER_X, s * FL_PITCH / 2) for s in (1, -1)]
    plain_holes += [(sbr_top_screw_x, s * SBR_SCREW_Y) for s in (1, -1)]
    plain_holes += so_pts
    csk_holes = [(bh_cx, s * BAT_SCREW_Y) for s in (1, -1)]

    # ---- chassis
    z0, z1 = CH_UNDER, CH_TOP
    ch = box(CH_X0, CH_X1 + CH_BULGE + 1, -CH_W / 2, CH_W / 2, z0, z1)
    front = box(CH_X1 - 1, CH_X1 + CH_BULGE + 2, -CH_W / 2 - 1, CH_W / 2 + 1, z0 - 1, z1 + 1)
    cut(front, cyl((FRONT_CX, 0, z0 - 2), (FRONT_CX, 0, z1 + 2), FRONT_R))
    cut(ch, front)
    for s in (1, -1):
        ylo, yhi = (CH_W / 2 - CH_REAR_R, CH_W / 2 + 1) if s > 0 else (-CH_W / 2 - 1, -CH_W / 2 + CH_REAR_R)
        corner = box(CH_X0 - 1, CH_X0 + CH_REAR_R, ylo, yhi, z0 - 1, z1 + 1)
        cut(corner, cyl((CH_X0 + CH_REAR_R, s * (CH_W / 2 - CH_REAR_R), z0 - 2),
                        (CH_X0 + CH_REAR_R, s * (CH_W / 2 - CH_REAR_R), z1 + 2), CH_REAR_R))
        cut(ch, corner)
    for (x, y) in plain_holes:
        cut(ch, hole(x, y, z0, z1))
    for (x, y) in csk_holes:
        cut(ch, hole(x, y, z0, z1))
        cut(ch, cone((x, y, z0 - 0.1), CSK_HD / 2 + 0.2, (x, y, z0 + 1.4), HOLE_D / 2))

    def chamfer_outline(comp):
        body = comp.bRepBodies.item(0)
        edges = adsk.core.ObjectCollection.create()
        for i in range(body.faces.count):
            f = body.faces.item(i)
            g = f.geometry
            if g.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType or abs(g.normal.z) < 0.99:
                continue
            for j in range(f.loops.count):
                lp = f.loops.item(j)
                if lp.isOuter:
                    for k in range(lp.edges.count):
                        edges.add(lp.edges.item(k))
        cf = comp.features.chamferFeatures
        d = adsk.core.ValueInput.createByReal(CH_CHAMFER / 10)
        try:
            inp = cf.createInput2()
            inp.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, d, False)
        except Exception:
            inp = cf.createInput(edges, False)
            inp.setToEqualDistance(d)
        cf.add(inp)

    add_part('Chassis', ch, AL[0], AL[1], chamfer=chamfer_outline)

    # ---- motors, brackets, wheels, tyres, set screws
    for side, s in (('L', 1), ('R', -1)):
        def Y(a, b):
            return (min(s * a, s * b), max(s * a, s * b))
        gz0, gz1 = AXLE_Z - GB_H / 2, AXLE_Z + GB_H / 2
        y0, y1 = Y(GB_Y0, GB_FACE)
        m = box(AXLE_X - GB_D / 2, AXLE_X + GB_D / 2, y0, y1, gz0, gz1)
        can = cyl((AXLE_X, s * CAN_Y0, AXLE_Z), (AXLE_X, s * GB_Y0, AXLE_Z), CAN_D / 2)
        y0, y1 = Y(CAN_Y0 - 1, GB_Y0 + 1)
        inter(can, box(AXLE_X - CAN_D, AXLE_X + CAN_D, y0, y1, AXLE_Z - CAN_FLATS / 2, AXLE_Z + CAN_FLATS / 2))
        union(m, can)
        sh = cyl((AXLE_X, s * GB_FACE, AXLE_Z), (AXLE_X, s * SHAFT_TIP, AXLE_Z), SHAFT_D / 2)
        y0, y1 = Y(GB_FACE - 1, SHAFT_TIP + 1)
        inter(sh, box(AXLE_X - 3, AXLE_X + 3, y0, y1, AXLE_Z - 3, FLAT_Z))
        union(m, sh)
        add_part('Motor ' + side, m, ST[0], ST[1])

        # bracket: strap over the gearbox top, legs down its sides, feet flat under the chassis
        by0, by1 = Y(br_cy - BR_W / 2, br_cy + BR_W / 2)
        xa, xb = AXLE_X - GB_D / 2, AXLE_X + GB_D / 2
        b = box(xa - BR_T - BR_FOOT, xb + BR_T + BR_FOOT, by0, by1, gz1, CH_UNDER)
        union(b, box(xa - BR_T, xa, by0, by1, gz0, gz1))
        union(b, box(xb, xb + BR_T, by0, by1, gz0, gz1))
        for fx in feet_x:
            cut(b, hole(fx, s * br_cy, gz1, CH_UNDER))
        add_part('Motor bracket ' + side, b, ST[0], ST[1])
        L = screw_length(CH_T + BR_T + NUT_T)
        for fx in feet_x:
            add_screw(cap_screw(fx, s * br_cy, CH_TOP, True, L))
            add_nut(nut(fx, s * br_cy, gz1 - NUT_T))

        # hub
        hub = cyl((AXLE_X, s * HUB_Y0, AXLE_Z), (AXLE_X, s * (HUB_Y0 + HUB_W), AXLE_Z), HUB_D / 2)
        union(hub, cyl((AXLE_X, s * BOSS_Y0, AXLE_Z), (AXLE_X, s * HUB_Y0, AXLE_Z), BOSS_D / 2))
        for k in range(LH_N):
            a = math.radians(30 + 360.0 * k / LH_N)
            hx, hz = AXLE_X + LH_PCD / 2 * math.cos(a), AXLE_Z + LH_PCD / 2 * math.sin(a)
            cut(hub, cyl((hx, s * (HUB_Y0 - 0.5), hz), (hx, s * (HUB_Y0 + HUB_W + 0.5), hz), LH_D / 2))
        bore = cyl((AXLE_X, s * (BOSS_Y0 - 0.5), AXLE_Z), (AXLE_X, s * (HUB_Y0 + HUB_W + 0.5), AXLE_Z), SHAFT_D / 2)
        y0, y1 = Y(BOSS_Y0 - 1, HUB_Y0 + HUB_W + 1)
        inter(bore, box(AXLE_X - 3, AXLE_X + 3, y0, y1, AXLE_Z - 3, FLAT_Z))
        cut(hub, bore)
        set_y = s * (BOSS_Y0 + BOSS_L / 2)
        boss_top = AXLE_Z + BOSS_D / 2
        cut(hub, cyl((AXLE_X, set_y, FLAT_Z), (AXLE_X, set_y, boss_top + 1), M3 / 2))
        add_part('Wheel ' + side, hub, 'ABS Plastic', WHITE)

        # tyre
        ty = cyl((AXLE_X, s * TYRE_Y0, AXLE_Z), (AXLE_X, s * (TYRE_Y0 + TYRE_W), AXLE_Z), TYRE_R)
        cut(ty, cyl((AXLE_X, s * (TYRE_Y0 - 1), AXLE_Z), (AXLE_X, s * (TYRE_Y0 + TYRE_W + 1), AXLE_Z), TYRE_ID / 2))
        for k in range(TREAD_N):
            a = math.radians(360.0 / TREAD_N * (k + 0.5))        # half a pitch off: the tread, not a groove, meets the ground
            c = (AXLE_X + TYRE_R * math.cos(a), s * TYRE_CY, AXLE_Z + TYRE_R * math.sin(a))
            cut(ty, obox(c, (math.cos(a), 0, math.sin(a)), (-math.sin(a), 0, math.cos(a)),
                         2 * TREAD_DEPTH, TREAD_W, TYRE_W + 2))
        add_part('Tyre ' + side, ty, 'Rubber', BLACK)

        # set screw on the shaft's flat
        ss = cyl((AXLE_X, set_y, FLAT_Z), (AXLE_X, set_y, FLAT_Z + SET_L), M3 / 2)
        cut(ss, hex_z(AXLE_X, set_y, FLAT_Z + SET_L - SET_SOCK_D, FLAT_Z + SET_L + 0.01, SET_SOCK))
        add_part('Set screw ' + side, ss, ST[0], ST[1])

    # ---- ball caster
    bc = (CASTER_X, 0, BALL_R)
    add_part('Ball', sphere(bc, BALL_R), ST[0], ST[1])
    hs = cyl((CASTER_X, 0, HOUS_Z0), (CASTER_X, 0, HOUS_Z1), HOUS_D / 2)
    union(hs, box(CASTER_X - FL_W / 2, CASTER_X + FL_W / 2, -FL_L / 2, FL_L / 2, HOUS_Z1 - FL_T, HOUS_Z1))
    cut(hs, sphere(bc, BALL_R + HOUS_CLEAR))
    for s in (1, -1):
        cut(hs, hole(CASTER_X, s * FL_PITCH / 2, HOUS_Z1 - FL_T, HOUS_Z1))
    add_part('Caster housing', hs, 'ABS Plastic', BLACK)
    L = screw_length(CH_T + SP_LEN + FL_T + NUT_T)
    for i, s in enumerate((1, -1)):
        y = s * FL_PITCH / 2
        sp = cyl((CASTER_X, y, HOUS_Z1), (CASTER_X, y, CH_UNDER), SP_OD / 2)
        cut(sp, cyl((CASTER_X, y, HOUS_Z1 - 1), (CASTER_X, y, CH_UNDER + 1), SP_ID / 2))
        add_part('Caster spacer %d' % (i + 1), sp, AL[0], AL[1])
    for s in (1, -1):
        y = s * FL_PITCH / 2
        add_screw(cap_screw(CASTER_X, y, CH_TOP, True, L))
        add_nut(nut(CASTER_X, y, HOUS_Z1 - FL_T - NUT_T, 90))

    # ---- sensor bar
    sb = box(SENSOR_X - SB_D / 2, SENSOR_X + SB_D / 2, -SB_L / 2, SB_L / 2, SB_Z0, SB_Z1)
    for s in (1, -1):
        cut(sb, hole(SENSOR_X, s * SBR_SCREW_Y, SB_Z0, SB_Z1))
    add_part('Sensor board', sb, 'FR4', GREEN)
    for k in range(SEN_N):
        y = (k - (SEN_N - 1) / 2) * SEN_PITCH
        add_part('Sensor %d' % (k + 1), box(SENSOR_X - SEN_X / 2, SENSOR_X + SEN_X / 2, y - SEN_Y / 2, y + SEN_Y / 2,
                                            SEN_Z0, SB_Z0), 'ABS Plastic', BLACK)
    fb0, fb1 = SB_Z1, SB_Z1 + SBR_T                                  # bottom flange on the board's top
    ft0, ft1 = CH_UNDER - SBR_T, CH_UNDER                            # top flange under the chassis
    L_top = screw_length(CH_T + SBR_T + NUT_T)
    L_board = screw_length(SBR_T + SB_T + 2 * NUT_T)                 # nut and lock nut
    for side, s in (('L', 1), ('R', -1)):
        y = s * SBR_SCREW_Y
        y0, y1 = y - SBR_W / 2, y + SBR_W / 2
        b = box(sbr_web_x0, SENSOR_X + SBR_BOTTOM_PAST, y0, y1, fb0, fb1)
        union(b, box(sbr_web_x0, sbr_web_x0 + SBR_T, y0, y1, fb0, ft1))
        union(b, box(sbr_top_x0, sbr_web_x0 + SBR_T, y0, y1, ft0, ft1))
        cut(b, hole(SENSOR_X, y, fb0, fb1))
        cut(b, hole(sbr_top_screw_x, y, ft0, ft1))
        add_part('Sensor bracket ' + side, b, AL[0], AL[1])
    for s in (1, -1):
        y = s * SBR_SCREW_Y
        add_screw(cap_screw(sbr_top_screw_x, y, CH_TOP, True, L_top))
        add_nut(nut(sbr_top_screw_x, y, ft0 - NUT_T))
    for s in (1, -1):
        y = s * SBR_SCREW_Y
        add_screw(cap_screw(SENSOR_X, y, fb1, True, L_board))
        add_nut(nut(SENSOR_X, y, SB_Z0 - NUT_T, 90))
        add_nut(nut(SENSOR_X, y, SB_Z0 - 2 * NUT_T, 90))

    # ---- controller
    so0, so1 = CH_TOP, CH_TOP + SO_L
    car0, car1 = so1, so1 + CAR_T
    car = box(CAR_CX - CAR_L / 2, CAR_CX + CAR_L / 2, -CAR_W / 2, CAR_W / 2, car0, car1)
    for (x, y) in so_pts:
        cut(car, hole(x, y, car0, car1))
    add_part('Carrier', car, 'FR4', BLUE)
    n0 = car1 + PIN_H
    nano = box(CAR_CX - NANO_L / 2, CAR_CX + NANO_L / 2, -NANO_W / 2, NANO_W / 2, n0, n0 + NANO_T)
    ux0 = CAR_CX - NANO_L / 2
    union(nano, box(ux0, ux0 + USB_L, -USB_W / 2, USB_W / 2, n0 + NANO_T, n0 + NANO_T + USB_H))
    for r in (1, -1):
        for i in range(PIN_N):
            px = CAR_CX + (i - (PIN_N - 1) / 2) * PIN_PITCH
            py = r * PIN_ROWS / 2
            union(nano, box(px - PIN_SQ / 2, px + PIN_SQ / 2, py - PIN_SQ / 2, py + PIN_SQ / 2, car1, n0))
    add_part('Arduino Nano', nano, 'FR4', NANO_BLUE)
    for i, (x, y) in enumerate(so_pts):
        so = hex_z(x, y, so0, so1, SO_AF)
        cut(so, cyl((x, y, so0 - 0.1), (x, y, so1 + 0.1), M3 / 2))
        add_part('Standoff %d' % (i + 1), so, 'Brass', BR[1])
    L_up = 6
    for (x, y) in so_pts:
        add_screw(cap_screw(x, y, car1, True, L_up))
    for (x, y) in so_pts:
        add_screw(cap_screw(x, y, CH_UNDER, False, L_up))

    # ---- battery
    bz0, bz1 = CH_TOP, CH_TOP + BH_H
    hold = box(BH_X0, BH_X0 + BH_W, -BH_L / 2, BH_L / 2, bz0, bz1)
    cell_z = bz0 + BH_FLOOR + CELL_D / 2
    cell_x = (bh_cx - (CELL_D + 1) / 2, bh_cx + (CELL_D + 1) / 2)
    for cx in cell_x:
        cut(hold, cyl((cx, -CELL_L / 2, cell_z), (cx, CELL_L / 2, cell_z), CELL_D / 2))
        cut(hold, box(cx - CELL_D / 2, cx + CELL_D / 2, -CELL_L / 2, CELL_L / 2, cell_z, bz1 + 1))
    for (x, y) in csk_holes:
        cut(hold, hole(x, y, bz0, bz0 + BH_FLOOR))
        cut(hold, hex_z(x, y, bz0 + BH_FLOOR, bz1 + 1, NUT_AF, 0))
    add_part('Battery holder', hold, 'ABS Plastic', BLACK)
    for i, cx in enumerate(cell_x):
        add_part('Cell %d' % (i + 1), cyl((cx, -CELL_L / 2, cell_z), (cx, CELL_L / 2, cell_z), CELL_D / 2),
                 AL[0], lambda: looks.colour('cell wrap', (30, 90, 200)))
    L_csk = screw_length(CH_T + BH_FLOOR + NUT_T)
    for (x, y) in csk_holes:
        add_screw(csk_screw_up(x, y, CH_UNDER, L_csk))
        add_nut(nut(x, y, bz0 + BH_FLOOR, 0))

    # ---- test board and line
    tbx0 = LINE_X0 - 260
    tby0 = -150.0
    board = box(tbx0, tbx0 + TB_L, tby0, tby0 + TB_W, -TB_T, 0)
    ln = box(LINE_X0, LINE_X0 + LINE_STRAIGHT, -LINE_W / 2, LINE_W / 2, -LINE_DEPTH, 0)
    acx, acy = LINE_X0 + LINE_STRAIGHT, LINE_R
    ring = cyl((acx, acy, -LINE_DEPTH), (acx, acy, 0), LINE_R + LINE_W / 2)
    cut(ring, cyl((acx, acy, -LINE_DEPTH - 1), (acx, acy, 1), LINE_R - LINE_W / 2))
    inter(ring, box(acx, acx + LINE_R + LINE_W, -LINE_W, acy, -LINE_DEPTH - 1, 1))
    union(ln, ring)
    cut(board, tbm.copy(ln))
    add_part('Test board', board, 'ABS Plastic', WHITE)
    add_part('Line', ln, 'Silicone', BLACK)

    # ---- joints
    chassis = occs['Chassis']
    chassis.isGrounded = True
    occs['Test board'].isGrounded = True

    def rigid(a, b):
        ji = root.asBuiltJoints.createInput(a, b, None)
        ji.setAsRigidJointMotion()
        return root.asBuiltJoints.add(ji)

    for side, s in (('L', 1), ('R', -1)):
        w = occs['Wheel ' + side]
        comp = w.component
        pi = comp.constructionPlanes.createInput()
        pi.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(AXLE_Z / 10))
        pl = comp.constructionPlanes.add(pi)
        sk = comp.sketches.add(pl)
        sp = sk.sketchPoints.add(sk.modelToSketchSpace(P(AXLE_X, s * (HUB_Y0 + HUB_W / 2), AXLE_Z)))
        geo = adsk.fusion.JointGeometry.createByPoint(sp.createForAssemblyContext(w))
        ji = root.asBuiltJoints.createInput(w, occs['Motor ' + side], geo)
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.YAxisJointDirection)
        j = root.asBuiltJoints.add(ji)
        j.name = 'Wheel %s turns' % side
        rigid(occs['Tyre ' + side], w).name = 'Tyre %s on hub' % side
        rigid(occs['Set screw ' + side], w).name = 'Set screw %s in hub' % side
    rigid(occs['Line'], occs['Test board']).name = 'Line in board'
    skip = {'Chassis', 'Test board', 'Line', 'Wheel L', 'Wheel R', 'Tyre L', 'Tyre R', 'Set screw L', 'Set screw R'}
    for name, occ in occs.items():
        if name not in skip:
            rigid(occ, chassis)

    # ---- report
    mass = 0.0
    for name, occ in occs.items():
        if name in ('Test board', 'Line'):
            continue
        mass += occ.component.bRepBodies.item(0).physicalProperties.mass
    app.activeViewport.fit()
    msg = ('Line follower v1: %d parts, %d screws, %d nuts, robot %.0f g; chassis underside z=%.1f, spacers %.1f, '
           'bracket drop %.1f; materials as Fusion gave them: %s%s'
           % (len(occs), len(screws), len(nuts), mass * 1000, CH_UNDER, SP_LEN, SBR_DROP, ', '.join(used_mats),
              ('; ' + '; '.join(looks.notes)) if looks.notes else ''))
    return msg


def run(context):
    app = adsk.core.Application.get()
    try:
        msg = build()
    except Exception:
        msg = 'Failed: ' + traceback.format_exc()
    print(msg)
    try:
        app.log(msg)
    except Exception:
        pass
