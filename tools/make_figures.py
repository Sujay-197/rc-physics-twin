"""The campus shuttle and a student, modelled in code (run by Blender, imported by
tools/prepare_assets.py, which writes them with the rest of the look).

At the twin's 1/10 scale, in metres, Blender's axes (x right, y forward, z up);
the export turns them into Unity's. Plain-colour materials, named
shuttle_* and student_*: Editor/CampusLook.cs makes them from MATERIALS here.

The shuttle is an autonomous pod sized for the twin's car (CoG to each axle
0.16 m, track 0.24 m, wheels 0.05 m): 0.42 m long and 0.25 m wide, the
wheels at its corners under arches, a glass band all round, a door on the
left (the kerb side; Blender +x, as it faces -y), LED light bars front and rear, a lidar on the roof.
Its origin is the car body's: 0.03 m above the CoG, wheel centres at
z = -0.04.

The student is about 1.7 m at full scale, low-poly; legs and arms are their
own objects pivoting at the hip and shoulder, so the game can swing them
as the student walks.
"""

from __future__ import annotations

import math

import bmesh
import bpy

# name -> (colour, smoothness, metallic, emission or None)
MATERIALS = {
    "shuttle_paint": ((0.93, 0.94, 0.95), 0.72, 0.0, None),
    "shuttle_glass": ((0.04, 0.06, 0.08), 0.96, 0.0, None),
    "shuttle_trim": ((0.09, 0.1, 0.11), 0.35, 0.0, None),
    "shuttle_accent": ((0.72, 0.1, 0.12), 0.6, 0.0, None),
    "shuttle_head": ((1.0, 0.97, 0.9), 0.8, 0.0, (1.0, 0.95, 0.85, 3.0)),
    "shuttle_tail": ((0.8, 0.05, 0.04), 0.8, 0.0, (1.0, 0.04, 0.02, 1.4)),   # bright red goes white under ACES
    "shuttle_sign": ((0.1, 0.08, 0.02), 0.8, 0.0, (1.0, 0.62, 0.12, 2.5)),
    "shuttle_sensor": ((0.05, 0.05, 0.06), 0.85, 0.3, None),
    "student_skin": ((0.62, 0.43, 0.32), 0.35, 0.0, None),
    "student_hair": ((0.06, 0.05, 0.04), 0.4, 0.0, None),
    "student_shirt": ((0.17, 0.29, 0.55), 0.25, 0.0, None),   # tinted per student in the game
    "student_pants": ((0.16, 0.18, 0.24), 0.2, 0.0, None),
    "student_shoes": ((0.92, 0.92, 0.9), 0.3, 0.0, None),
    "student_bag": ((0.2, 0.22, 0.25), 0.3, 0.0, None),
}

L, W = 0.42, 0.25                  # shuttle length, width
Z0, Z1, Z2, Z3 = -0.03, 0.062, 0.152, 0.18   # bottom, beltline, roof edge, roof top
AXLE_Y, WHEEL_Z, ARCH_R = 0.16, -0.04, 0.06


def material(name):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    c = MATERIALS[name][0]
    m.diffuse_color = (*c, 1.0)
    return m


def box(name, size, at, mat, bevel=0.0, segments=3):
    """A box (size = full extents), its edges rounded by `bevel`, applied."""
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts:
        v.co.x *= size[0]
        v.co.y *= size[1]
        v.co.z *= size[2]
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    o = bpy.data.objects.new(name, me)
    o.location = at
    bpy.context.scene.collection.objects.link(o)
    me.materials.append(material(mat))
    if bevel > 0:
        mod = o.modifiers.new("bevel", "BEVEL")
        mod.width = bevel
        mod.segments = segments
        mod.limit_method = "NONE"
        apply(o)
    return o


def cylinder(name, r, depth, at, mat, axis="z", verts=24, r2=None):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=verts, radius1=r, radius2=r if r2 is None else r2, depth=depth)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    o = bpy.data.objects.new(name, me)
    o.location = at
    if axis == "x":
        o.rotation_euler = (0.0, math.pi / 2, 0.0)
    elif axis == "y":
        o.rotation_euler = (math.pi / 2, 0.0, 0.0)
    bpy.context.scene.collection.objects.link(o)
    me.materials.append(material(mat))
    return o


def sphere(name, r, at, mat, scale=(1, 1, 1), segments=16, rings=10):
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=segments, v_segments=rings, radius=r)
    for v in bm.verts:
        v.co.x *= scale[0]
        v.co.y *= scale[1]
        v.co.z *= scale[2]
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    o = bpy.data.objects.new(name, me)
    o.location = at
    bpy.context.scene.collection.objects.link(o)
    me.materials.append(material(mat))
    return o


def apply(o):
    dg = bpy.context.evaluated_depsgraph_get()
    new = bpy.data.meshes.new_from_object(o.evaluated_get(dg))
    o.modifiers.clear()
    old = o.data
    o.data = new
    bpy.data.meshes.remove(old)


def cut(o, cutter):
    """Boolean difference; the cut faces take the cutter's material."""
    mod = o.modifiers.new("cut", "BOOLEAN")
    mod.operation = "DIFFERENCE"
    mod.solver = "EXACT"
    mod.object = cutter
    mod.material_mode = "TRANSFER"
    apply(o)


def join(objs, name):
    bpy.context.view_layer.update()   # world matrices of parts not yet evaluated
    for o in objs:
        o.data.transform(o.matrix_world)
        o.matrix_world.identity()
    with bpy.context.temp_override(active_object=objs[0], selected_editable_objects=objs, selected_objects=objs):
        bpy.ops.object.join()
    o = objs[0]
    o.name = name
    o.data.name = name
    o.data.shade_smooth()
    o.data.set_sharp_from_angle(angle=math.radians(40))   # rounded where bevelled, crisp at panel edges
    return o


def shuttle():
    """The pod, one mesh named "shuttle", front toward -y (Blender's front view)."""
    parts = []
    # Lower body, the bumper band round its foot, the accent stripe under the beltline.
    lower = box("lower", (W, L, Z1 - Z0), (0, 0, (Z0 + Z1) / 2), "shuttle_paint", bevel=0.022, segments=5)
    band = box("band", (W + 0.004, L + 0.004, 0.022), (0, 0, Z0 + 0.011), "shuttle_trim", bevel=0.012, segments=4)
    stripe = box("stripe", (W + 0.003, L + 0.003, 0.007), (0, 0, Z1 - 0.012), "shuttle_accent", bevel=0.02, segments=5)
    # Wheel arches, lined dark.
    for o in (lower, band):
        for y in (AXLE_Y, -AXLE_Y):
            c = cylinder("arch", ARCH_R, W + 0.1, (0, y, WHEEL_Z), "shuttle_trim", axis="x", verts=40)
            cut(o, c)
            bpy.data.objects.remove(c)
    parts += [lower, band, stripe]
    # The cabin: pillars in paint, a glass band set into them all round, the roof.
    cabin = box("cabin", (W - 0.012, L - 0.024, Z2 - Z1 + 0.004), (0, 0, (Z1 + Z2) / 2), "shuttle_paint", bevel=0.026, segments=5)
    glass_z = (Z1 + 0.012, Z2 - 0.01)
    gh = glass_z[1] - glass_z[0]
    gz = sum(glass_z) / 2
    # Windscreens, front and rear, wide.
    for s in (-1, 1):
        parts.append(box("screen", (W - 0.05, 0.006, gh), (0, s * (L / 2 - 0.013), gz), "shuttle_glass", bevel=0.008))
    # Side glass: three panes a side; on the left (+x, facing -y), the front one is the door, to the floor.
    for side in (-1, 1):
        x = side * (W / 2 - 0.007)
        for y0, y1 in ((-0.165, -0.06), (-0.045, 0.045), (0.06, 0.165)):
            door = side > 0 and y0 < -0.1
            z0 = Z0 + 0.03 if door else glass_z[0]
            parts.append(box("pane", (0.006, y1 - y0, glass_z[1] - z0), (x, (y0 + y1) / 2, (z0 + glass_z[1]) / 2),
                             "shuttle_glass", bevel=0.006))
        if side > 0:   # the door's frame, a dark seam round the tall pane
            parts.append(box("door frame", (0.004, 0.115, glass_z[1] - Z0 - 0.02), (x + 0.0015, -0.1125, (Z0 + 0.02 + glass_z[1]) / 2),
                             "shuttle_trim", bevel=0.003))
    parts.append(cabin)
    roof = box("roof", (W - 0.006, L - 0.014, Z3 - Z2), (0, 0, (Z2 + Z3) / 2), "shuttle_paint", bevel=0.02, segments=5)
    solar = box("solar", (W - 0.06, L - 0.12, 0.004), (0, 0.02, Z3 + 0.001), "shuttle_trim", bevel=0.004)
    lidar = cylinder("lidar", 0.014, 0.016, (0, -0.15, Z3 + 0.008), "shuttle_sensor", verts=24)
    parts += [roof, solar, lidar]
    # Lights: a bar and two corner lamps at the front (-y), red at the back; the sign over the screen.
    for s, mat in ((-1, "shuttle_head"), (1, "shuttle_tail")):
        y = s * (L / 2 + 0.002)
        parts.append(box("light bar", (W - 0.09, 0.004, 0.006), (0, y, 0.04), mat, bevel=0.002))
        for x in (-1, 1):
            parts.append(box("lamp", (0.03, 0.005, 0.014), (x * (W / 2 - 0.03), y, 0.024), mat, bevel=0.004))
    parts.append(box("sign", (0.09, 0.004, 0.012), (0, -(L / 2 - 0.009), Z2 - 0.004), "shuttle_sign", bevel=0.002))
    return join(parts, "shuttle")


def student():
    """The student, facing -y; returns its parts (body, legs, arms) with their pivots."""
    out = []
    hip_z, shoulder_z = 0.082, 0.128
    body = [
        box("torso", (0.046, 0.026, shoulder_z - hip_z + 0.008), (0, 0, (hip_z + shoulder_z) / 2), "student_shirt", bevel=0.009),
        box("hips", (0.044, 0.024, 0.016), (0, 0, hip_z + 0.002), "student_pants", bevel=0.006),
        cylinder("neck", 0.0065, 0.014, (0, 0, shoulder_z + 0.008), "student_skin", verts=12),
        sphere("head", 0.0175, (0, 0, 0.152), "student_skin", scale=(1, 1.05, 1.12)),
        sphere("hair", 0.0186, (0, 0.002, 0.156), "student_hair", scale=(1.02, 1.08, 1.0)),
        box("bag", (0.036, 0.016, 0.04), (0, 0.021, 0.106), "student_bag", bevel=0.006),
    ]
    # The hair as a cap: keep the top and back of its sphere.
    hair = body[4]
    bm = bmesh.new()
    bm.from_mesh(hair.data)
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if v.co.z < -0.004 and v.co.y < 0.004], context="VERTS")
    bm.to_mesh(hair.data)
    bm.free()
    out.append(join(body, "student_body"))
    for side, name in ((-1, "l"), (1, "r")):
        leg = [cylinder("leg", 0.0105, hip_z - 0.008, (0, 0, -(hip_z - 0.008) / 2), "student_pants", verts=12, r2=0.0095),
               box("shoe", (0.016, 0.028, 0.01), (0, -0.004, -hip_z + 0.005), "student_shoes", bevel=0.004)]
        o = join(leg, f"student_leg_{name}")
        o.location = (side * 0.012, 0, hip_z)
        out.append(o)
        arm = [cylinder("sleeve", 0.0075, 0.05, (0, 0, -0.025), "student_shirt", verts=10, r2=0.0068),
               sphere("hand", 0.0068, (0, 0, -0.054), "student_skin", segments=10, rings=6)]
        o = join(arm, f"student_arm_{name}")
        o.location = (side * 0.031, 0, shoulder_z - 0.002)
        out.append(o)
    return out
