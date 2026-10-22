"""Prepare the campus look's models and textures for Unity (run by Blender).

    D:\\blender\\blender.exe -b --factory-startup --python tools/prepare_assets.py
    ... --python tools/prepare_assets.py -- --only shrub_02_a    # one prop

Reads assets_src/ (tools/fetch_assets.py) and writes
unity/RcTwin/Assets/RcTwin/Generated/Look/ (gitignored), which the scene
builder's look pass reads (Editor/CampusLook.cs):

  Models/<prop>.fbx   meshes <prop>_LOD0, _LOD1, _LOD2 (Unity builds the
                      LODGroup from the suffixes), at the twin's 1/10 scale,
                      origin at the base, Y up, one submesh per material
  Textures/<set>_albedo.png   colour; alpha is the opacity where it has one
  Textures/<set>_normal.png   OpenGL convention (Unity's)
  Textures/<set>_mask.png     URP Lit's metallic map: R metallic,
                              A smoothness = 1 - roughness
  Models/shuttle.fbx, Resources/RcTwinLook/student.fbx: the shuttle and a
                      student, modelled in code (tools/make_figures.py)
  Sky/<hdri>.hdr
  Resources/RcTwinLook/Inter-*.ttf (+ LICENSE.txt): the HUD loads them by name
  look.json           what was prepared: props (size, LOD triangles,
                      materials and the texture set each uses), texture sets

LODs: wood and solid parts are collapse-decimated (a tree's twigs, small
islands hidden in the crown, dropped first at the far LODs). Leaves are
curved, alpha-clipped islands of ~25-35 faces; each becomes one flat card
(two triangles) fitted to it, the leaf image whole, so the nearest LOD keeps
every leaf. The far LODs keep a random share of the cards, scaled up about
their centres so the crown keeps its volume.
"""

from __future__ import annotations

import json
import pathlib
import random
import shutil
import sys
import time

import bmesh
import bpy
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC = ROOT / "assets_src"
MODELS = SRC / "polyhaven" / "models"
OUT = ROOT / "unity" / "RcTwin" / "Assets" / "RcTwin" / "Generated" / "Look"
TWIN_SCALE = 0.1   # the campus and the car are 1/10 scale
SKY = "kloppenheim_06_puresky"   # tools/fetch_assets.py's HDRI

# Texture sets: name -> (folder under assets_src/polyhaven, file prefix, options).
#   alpha: "alpha" / "opacity" (a separate map) or "diff" (the colour map's own alpha)
#   metal: a metallic map exists;  wet: smoothness raised for a wet-look road
TEXTURE_SETS = {
    "asphalt_02": ("textures/asphalt_02", "asphalt_02", {"wet": True}),
    "concrete_pavement": ("textures/concrete_pavement", "concrete_pavement", {}),
    "leafy_grass": ("textures/leafy_grass", "leafy_grass", {}),
    "boulder_01": ("models/boulder_01/textures", "boulder_01", {}),
    "jacaranda_tree_branches": ("models/jacaranda_tree/textures", "jacaranda_tree_branches", {}),
    "jacaranda_tree_trunk": ("models/jacaranda_tree/textures", "jacaranda_tree_trunk", {}),
    "jacaranda_tree_leaves": ("models/jacaranda_tree/textures", "jacaranda_tree_leaves", {"alpha": "alpha"}),
    "tree_small_02_branches": ("models/tree_small_02/textures", "tree_small_02_branch", {"alpha": "diff"}),
    "tree_small_02_trunk": ("models/tree_small_02/textures", "tree_small_02", {}),
    "tree_small_02_leaves": ("models/tree_small_02/textures", "tree_small_02_leaves", {"alpha": "alpha"}),
    "shrub_02": ("models/shrub_02/textures", "shrub_02", {"alpha": "alpha"}),
    "shrub_03": ("models/shrub_03/textures", "shrub_03", {"alpha": "alpha"}),
    "street_lamp_01": ("models/street_lamp_01/textures", "street_lamp_01", {"metal": True}),
    "street_lamp_01_glass": ("models/street_lamp_01/textures", "street_lamp_01", {"metal": True, "alpha": "opacity"}),
    "modular_street_seating_armrests": ("models/modular_street_seating/textures", "modular_street_seating_armrests", {"metal": True}),
    "modular_street_seating_supports": ("models/modular_street_seating/textures", "modular_street_seating_supports", {"metal": True}),
    "modular_street_seating_timber": ("models/modular_street_seating/textures", "modular_street_seating_timber", {}),
}

# Material name in the source model -> (texture set, how Unity draws it).
#   opaque | cutout (alpha-clipped, two-sided) | glass (transparent) | emissive
MATERIALS = {
    "boulder_01": ("boulder_01", "opaque"),
    "jacaranda_tree_branches": ("jacaranda_tree_branches", "opaque"),
    "jacaranda_tree_trunk": ("jacaranda_tree_trunk", "opaque"),
    "jacaranda_tree_leaves": ("jacaranda_tree_leaves", "cutout"),
    "tree_small_02_branches": ("tree_small_02_branches", "cutout"),
    "tree_small_02_trunk": ("tree_small_02_trunk", "opaque"),
    "tree_small_02_leaves": ("tree_small_02_leaves", "cutout"),
    "shrub_02": ("shrub_02", "cutout"),
    "shrub_03": ("shrub_03", "cutout"),
    "street_lamp_01": ("street_lamp_01", "opaque"),
    "street_lamp_01_glass": ("street_lamp_01_glass", "glass"),
    "street_lamp_01_bulb": ("street_lamp_01", "emissive"),
    "modular_street_seating_armrests": ("modular_street_seating_armrests", "opaque"),
    "modular_street_seating_supports": ("modular_street_seating_supports", "opaque"),
    "modular_street_seating_connectors": ("modular_street_seating_supports", "opaque"),
    "modular_street_seating_timber": ("modular_street_seating_timber", "opaque"),
}

# A LOD: (ratio for the wood, share of leaf cards kept,
# wood islands smaller than this share of the tree's height dropped: twigs inside the crown,
# wood vertices closer than this share welded: thin branch tubes the collapse cannot reduce).
TREE_LODS = {
    "tree_small_02": [(0.35, 1.0, 0.0, 0.0), (0.10, 0.6, 0.03, 0.001), (0.03, 0.3, 0.08, 0.003)],
    "jacaranda_tree": [(0.08, 1.0, 0.01, 0.0005), (0.025, 0.6, 0.04, 0.002), (0.008, 0.3, 0.10, 0.005)],
}
LEAF_GROWTH = 1.25   # each card a little larger than its leaf: a fuller crown at every distance

# Bushes, made here: tree_small_02's leaf cards packed into a dome (Poly Haven's shrubs are
# sprigs, not bushes). name -> (half width, half depth, height [m, full scale], cards, seed).
BUSHES = {
    "bush_round": (0.65, 0.65, 1.1, 900, 1),
    "bush_hedge": (0.9, 0.45, 0.95, 1000, 2),
    "bush_tall": (0.55, 0.55, 1.7, 1000, 3),
}
BUSH_LODS = [1.0, 0.45, 0.18]   # share of the cards kept
SOLID_LODS = [1.0, 0.35, 0.1]

# Props: name -> (source fbx folder, objects in it, kind, extra scale on top of 1/10,
# LODs: "tree" | "solid" | a list of source objects, one per LOD, recentre in plan).
PROPS = {
    "tree_small_02": ("tree_small_02", ["tree_small_02_LOD0"], "tree", 1.0, "tree", False),
    "jacaranda_tree": ("jacaranda_tree", ["jacaranda_tree_LOD0"], "tree", 0.5, "tree", False),
    "shrub_02_a": ("shrub_02", ["shrub_02_a_LOD0"], "shrub", 1.0, "solid", True),
    "shrub_02_b": ("shrub_02", ["shrub_02_b_LOD0"], "shrub", 1.0, "solid", True),
    "shrub_02_c": ("shrub_02", ["shrub_02_c_LOD0"], "shrub", 1.0, "solid", True),
    "shrub_02_d": ("shrub_02", ["shrub_02_d_LOD0"], "shrub", 1.0, "solid", True),
    "shrub_03_a": ("shrub_03", ["shrub_03_a_LOD0"], "plant", 1.0, "solid", True),
    "shrub_03_b": ("shrub_03", ["shrub_03_b_LOD0"], "plant", 1.0, "solid", True),
    "shrub_03_c": ("shrub_03", ["shrub_03_c_LOD0"], "plant", 1.0, "solid", True),
    "boulder_01": ("boulder_01", ["boulder_01_LOD3"], "rock", 1.0, "solid", True),   # the source's own lightest LOD
    "street_lamp_01": ("street_lamp_01", ["street_lamp_01"], "lamp", 1.0, "solid", False),
    # The kit's complete bench with a back, armrests at both ends.
    "bench": ("modular_street_seating",
              ["arm_rest_01", "arm_rest_02", "back_support_l", "back_support_r", "crossbar",
               "legs_double", "legs_single", "seat", "seat_back", "suspended_support_01"],
              "bench", 1.0, "solid", True),
}


def log(*a):
    print("[prepare_assets]", *a, flush=True)


# ------------------------------------------------------------------ textures

def find_map(folder: pathlib.Path, prefix: str, kind: str) -> pathlib.Path | None:
    for ext in ("png", "jpg", "exr"):
        p = folder / f"{prefix}_{kind}_2k.{ext}"
        if p.exists():
            return p
    return None


def load_pixels(path: pathlib.Path, data: bool) -> np.ndarray:
    img = bpy.data.images.load(str(path), check_existing=False)
    img.colorspace_settings.name = "Non-Color" if data else "sRGB"
    w, h = img.size
    px = np.empty(w * h * 4, np.float32)
    img.pixels.foreach_get(px)
    bpy.data.images.remove(img)
    return px.reshape(h, w, 4)


def save_pixels(px: np.ndarray, path: pathlib.Path, data: bool) -> None:
    h, w, _ = px.shape
    img = bpy.data.images.new(path.stem, w, h, alpha=True, float_buffer=False)
    img.colorspace_settings.name = "Non-Color" if data else "sRGB"
    img.pixels.foreach_set(np.clip(px, 0.0, 1.0).astype(np.float32).ravel())
    img.filepath_raw = str(path)
    img.file_format = "PNG"
    img.save()
    bpy.data.images.remove(img)


def prepare_texture_set(name: str) -> dict:
    sub, prefix, opt = TEXTURE_SETS[name]
    folder = SRC / "polyhaven" / sub
    out = OUT / "Textures"
    diff = load_pixels(find_map(folder, prefix, "diff"), data=False)
    alpha = opt.get("alpha")
    if alpha in ("alpha", "opacity"):
        a = load_pixels(find_map(folder, prefix, alpha), data=True)
        diff[..., 3] = a[..., 0]
    elif alpha != "diff":
        diff[..., 3] = 1.0
    save_pixels(diff, out / f"{name}_albedo.png", data=False)

    nor = load_pixels(find_map(folder, prefix, "nor_gl"), data=True)
    nor[..., 3] = 1.0
    save_pixels(nor, out / f"{name}_normal.png", data=True)

    rough = load_pixels(find_map(folder, prefix, "rough"), data=True)[..., 0]
    smooth = 1.0 - rough
    if opt.get("wet"):
        smooth = np.minimum(1.0, 0.35 + 0.75 * smooth)   # a road after rain: a soft sheen
    mask = np.zeros(rough.shape + (4,), np.float32)
    if opt.get("metal"):
        mask[..., 0] = load_pixels(find_map(folder, prefix, "metal"), data=True)[..., 0]
    mask[..., 1] = 1.0   # occlusion channel unused by Lit's metallic map; keep white
    mask[..., 3] = smooth
    save_pixels(mask, out / f"{name}_mask.png", data=True)
    return {"alpha": alpha is not None, "metal": bool(opt.get("metal")), "wet": bool(opt.get("wet")),
            "smoothness_mean": round(float(smooth.mean()), 3)}


# -------------------------------------------------------------------- meshes

def reset():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def import_fbx(folder: str):
    path = MODELS / folder / f"{folder}_2k.fbx"
    bpy.ops.import_scene.fbx(filepath=str(path))


def tri_count(me) -> int:
    n = np.empty(len(me.polygons), np.int32)
    me.polygons.foreach_get("loop_total", n)
    return int((n - 2).sum())


def join(objs, name):
    """One mesh object from objs, transforms applied, a single UV layer named UVMap."""
    for o in objs:
        me = o.data
        render = next((u for u in me.uv_layers if u.active_render), me.uv_layers[0])
        for u in [u for u in me.uv_layers if u != render]:
            me.uv_layers.remove(u)
        me.uv_layers[0].name = "UVMap"
        o.data.transform(o.matrix_world)
        o.matrix_world.identity()
    if len(objs) > 1:
        with bpy.context.temp_override(active_object=objs[0], selected_editable_objects=objs,
                                       selected_objects=objs):
            bpy.ops.object.join()
    o = objs[0]
    o.name = name
    o.data.name = name
    return o


def copy_object(o, name):
    c = o.copy()
    c.data = o.data.copy()
    c.name = name
    c.data.name = name
    bpy.context.scene.collection.objects.link(c)
    return c


def islands(me, faces: np.ndarray) -> np.ndarray:
    """Connected component label (a vertex index) of each face in `faces`."""
    lst = np.empty(len(me.polygons), np.int32)
    tot = np.empty(len(me.polygons), np.int32)
    me.polygons.foreach_get("loop_start", lst)
    me.polygons.foreach_get("loop_total", tot)
    lv = np.empty(len(me.loops), np.int32)
    me.loops.foreach_get("vertex_index", lv)
    corners = [lv[lst[faces] + np.minimum(j, tot[faces] - 1)] for j in range(tot[faces].max())]
    lab = np.arange(len(me.vertices))
    for _ in range(200):
        m = np.minimum.reduce([lab[c] for c in corners])
        before = lab.copy()
        for c in corners:
            np.minimum.at(lab, c, m)
        lab = lab[lab]
        if np.array_equal(before, lab):
            break
    return lab[corners[0]]


def leaf_cards(o, leaf_slots: set):
    """Replace each leaf island with one flat quad: the affine map from the island's UVs
    to its positions (least squares), evaluated at the corners of its UV box."""
    me = o.data
    n = len(me.polygons)
    mi = np.empty(n, np.int32)
    me.polygons.foreach_get("material_index", mi)
    faces = np.nonzero(np.isin(mi, list(leaf_slots)))[0]
    if len(faces) == 0:
        return
    lab = islands(me, faces)
    ids, inv = np.unique(lab, return_inverse=True)
    lst = np.empty(n, np.int32)
    tot = np.empty(n, np.int32)
    me.polygons.foreach_get("loop_start", lst)
    me.polygons.foreach_get("loop_total", tot)
    lv = np.empty(len(me.loops), np.int32)
    me.loops.foreach_get("vertex_index", lv)
    co = np.empty(len(me.vertices) * 3, np.float32)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3).astype(np.float64)
    uv = np.empty(len(me.loops) * 2, np.float32)
    me.uv_layers[0].data.foreach_get("uv", uv)
    uv = uv.reshape(-1, 2).astype(np.float64)
    loops = np.concatenate([lst[faces] + j for j in range(tot[faces].max())])
    owner = np.concatenate([inv] * tot[faces].max())
    real = np.concatenate([tot[faces] > j for j in range(tot[faces].max())])
    loops, owner = loops[real], owner[real]
    x = np.column_stack([uv[loops], np.ones(len(loops))])          # [u, v, 1]
    P = co[lv[loops]]
    k = len(ids)
    M = np.zeros((k, 3, 3))
    B = np.zeros((k, 3, 3))
    np.add.at(M, owner, x[:, :, None] * x[:, None, :])
    np.add.at(B, owner, x[:, :, None] * P[:, None, :])
    lo = np.full((k, 2), np.inf)
    hi = np.full((k, 2), -np.inf)
    np.minimum.at(lo, owner, uv[loops])
    np.maximum.at(hi, owner, uv[loops])
    ok = (np.abs(np.linalg.det(M)) > 1e-14) & np.all(hi - lo > 1e-5, axis=1)
    A = np.zeros((k, 3, 3))
    A[ok] = np.linalg.solve(M[ok], B[ok])                          # P = [u v 1] @ A
    slot = np.zeros(k, np.int32)
    slot[inv] = mi[faces]
    corners_uv = np.stack([np.column_stack([lo[:, 0], lo[:, 1]]), np.column_stack([hi[:, 0], lo[:, 1]]),
                           np.column_stack([hi[:, 0], hi[:, 1]]), np.column_stack([lo[:, 0], hi[:, 1]])], axis=1)
    corners = np.einsum("kci,kij->kcj", np.concatenate([corners_uv, np.ones((k, 4, 1))], axis=2), A)
    centre = corners.mean(axis=1, keepdims=True)
    corners = centre + LEAF_GROWTH * (corners - centre)

    bm = bmesh.new()
    bm.from_mesh(me)
    bm.faces.ensure_lookup_table()
    bmesh.ops.delete(bm, geom=[bm.faces[i] for i in faces], context="FACES")
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context="VERTS")
    layer = bm.loops.layers.uv[0]
    for i in np.nonzero(ok)[0]:
        vs = [bm.verts.new(c) for c in corners[i]]
        f = bm.faces.new(vs)
        f.material_index = int(slot[i])
        for loop, t in zip(f.loops, corners_uv[i]):
            loop[layer].uv = t
    bm.to_mesh(me)
    bm.free()


def thin_leaves(o, leaf_slots: set, keep: float, rng: random.Random):
    """Delete all but `keep` of the leaf islands; scale the survivors up about their centres."""
    me = o.data
    mi = np.empty(len(me.polygons), np.int32)
    me.polygons.foreach_get("material_index", mi)
    faces = np.nonzero(np.isin(mi, list(leaf_slots)))[0]
    if len(faces) == 0 or keep >= 1.0:
        return
    lab = islands(me, faces)
    ids = np.unique(lab)
    kept = np.array(sorted(rng.sample(list(ids), max(1, int(round(keep * len(ids)))))))
    keep_face = np.isin(lab, kept)

    # Scale kept islands about their centres (by vertex: a vertex belongs to its face's island).
    co = np.empty(len(me.vertices) * 3, np.float32)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    lst = np.empty(len(me.polygons), np.int32)
    me.polygons.foreach_get("loop_start", lst)
    lv = np.empty(len(me.loops), np.int32)
    me.loops.foreach_get("vertex_index", lv)
    tot = np.empty(len(me.polygons), np.int32)
    me.polygons.foreach_get("loop_total", tot)
    kf, kl = faces[keep_face], lab[keep_face]
    isl_of_v = np.full(len(me.vertices), -1, np.int64)   # each vertex of a kept face: its island
    for j in range(tot[kf].max()):
        has = tot[kf] > j
        isl_of_v[lv[lst[kf[has]] + j]] = kl[has]
    vs = np.nonzero(isl_of_v >= 0)[0]
    labels, inv = np.unique(isl_of_v[vs], return_inverse=True)
    sums = np.zeros((len(labels), 3))
    np.add.at(sums, inv, co[vs])
    cnt = np.bincount(inv, minlength=len(labels))[:, None]
    centre = sums / cnt
    scale = min(2.5, keep ** -0.5)   # fewer leaves, each bigger by area: the crown stays as full
    co[vs] = centre[inv] + scale * (co[vs] - centre[inv])
    me.vertices.foreach_set("co", co.ravel())

    bm = bmesh.new()
    bm.from_mesh(me)
    bm.faces.ensure_lookup_table()
    gone = [bm.faces[i] for i in faces[~keep_face]]
    bmesh.ops.delete(bm, geom=gone, context="FACES")
    loose = [v for v in bm.verts if not v.link_faces]
    bmesh.ops.delete(bm, geom=loose, context="VERTS")
    bm.to_mesh(me)
    bm.free()


def drop_small_islands(o, slots: set, min_size: float):
    """Delete the islands of `slots`' faces whose bounding box is smaller than min_size across."""
    me = o.data
    mi = np.empty(len(me.polygons), np.int32)
    me.polygons.foreach_get("material_index", mi)
    faces = np.nonzero(np.isin(mi, list(slots)))[0]
    if len(faces) == 0 or min_size <= 0.0:
        return
    lab = islands(me, faces)
    co = np.empty(len(me.vertices) * 3, np.float32)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    lst = np.empty(len(me.polygons), np.int32)
    me.polygons.foreach_get("loop_start", lst)
    lv = np.empty(len(me.loops), np.int32)
    me.loops.foreach_get("vertex_index", lv)
    p = co[lv[lst[faces]]]   # one corner per face is enough for an island's extent
    ids, inv = np.unique(lab, return_inverse=True)
    lo = np.full((len(ids), 3), np.inf)
    hi = np.full((len(ids), 3), -np.inf)
    np.minimum.at(lo, inv, p)
    np.maximum.at(hi, inv, p)
    small = np.linalg.norm(hi - lo, axis=1) < min_size
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.faces.ensure_lookup_table()
    bmesh.ops.delete(bm, geom=[bm.faces[i] for i in faces[small[inv]]], context="FACES")
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context="VERTS")
    bm.to_mesh(me)
    bm.free()


def weld(o, slots: set, dist: float):
    """Merge the vertices of `slots`' faces closer than dist."""
    if dist <= 0.0:
        return
    bm = bmesh.new()
    bm.from_mesh(o.data)
    verts = list({v for f in bm.faces if f.material_index in slots for v in f.verts})
    bmesh.ops.remove_doubles(bm, verts=verts, dist=dist)
    bm.to_mesh(o.data)
    bm.free()


def foliage_normals(o, slots: set):
    """Point the normals of `slots`' faces out from the foliage's centre, a little up:
    two-sided leaf cards then light as one soft volume, the same from either side."""
    me = o.data
    mi = np.empty(len(me.polygons), np.int32)
    me.polygons.foreach_get("material_index", mi)
    leaf_faces = np.isin(mi, list(slots))
    if not leaf_faces.any():
        return
    tot = np.empty(len(me.polygons), np.int32)
    me.polygons.foreach_get("loop_total", tot)
    leaf_loops = np.repeat(leaf_faces, tot)
    lv = np.empty(len(me.loops), np.int32)
    me.loops.foreach_get("vertex_index", lv)
    co = np.empty(len(me.vertices) * 3, np.float32)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    p = co[lv[leaf_loops]]
    lo, hi = p.min(0), p.max(0)
    centre = (lo + hi) / 2
    half = np.maximum((hi - lo) / 2, 1e-6)
    d = (p - centre) / half                       # an ellipsoid's shape, not the crown's raw extent
    d[:, 2] += 0.35                                # and a little up (Blender z), toward the sky
    d /= np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-6)
    normals = np.empty(len(me.loops) * 3, np.float32)
    me.corner_normals.foreach_get("vector", normals)
    normals = normals.reshape(-1, 3)
    normals[leaf_loops] = d
    me.normals_split_custom_set(normals.tolist())


def decimate(o, ratio_of_slot):
    """Collapse-decimate each material's faces by its own ratio."""
    if all(r >= 0.999 for r in ratio_of_slot.values()):
        return o
    parts = []
    for slot, ratio in ratio_of_slot.items():
        c = copy_object(o, f"{o.name}_part{slot}")
        bm = bmesh.new()
        bm.from_mesh(c.data)
        bmesh.ops.delete(bm, geom=[f for f in bm.faces if f.material_index != slot], context="FACES")
        bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context="VERTS")
        bm.to_mesh(c.data)
        bm.free()
        if len(c.data.polygons) == 0:
            bpy.data.objects.remove(c)
            continue
        if ratio < 0.999:
            mod = c.modifiers.new("decimate", "DECIMATE")
            mod.decimate_type = "COLLAPSE"
            mod.ratio = ratio
            mod.use_collapse_triangulate = True
            dg = bpy.context.evaluated_depsgraph_get()
            new = bpy.data.meshes.new_from_object(c.evaluated_get(dg))
            c.modifiers.clear()
            old = c.data
            c.data = new
            bpy.data.meshes.remove(old)
        parts.append(c)
    name = o.name
    bpy.data.objects.remove(o)
    with bpy.context.temp_override(active_object=parts[0], selected_editable_objects=parts, selected_objects=parts):
        bpy.ops.object.join()
    parts[0].name = name
    parts[0].data.name = name
    return parts[0]


def prepare_prop(name: str, seed: int) -> dict:
    folder, objects, kind, extra, lods, recentre = PROPS[name]
    t0 = time.time()
    reset()
    import_fbx(folder)
    for o in list(bpy.data.objects):   # only this prop's pieces: the source's own LOD names would clash with ours
        if o.name not in objects:
            bpy.data.objects.remove(o)
    scale = TWIN_SCALE * extra

    if isinstance(lods, list):   # the source ships its own LODs
        lod_objs = [join([bpy.data.objects[n]], f"{name}_LOD{i}") for i, n in enumerate(lods)]
    else:
        base = join([bpy.data.objects[n] for n in objects], f"{name}_base")
        lod_objs = []
        mats = [m.name for m in base.data.materials]
        leaf = {i for i, m in enumerate(mats) if MATERIALS[m][1] == "cutout" and kind == "tree" and "leaves" in m}
        wood = set(range(len(mats))) - leaf
        height = base.dimensions.z
        if lods == "tree":
            leaf_cards(base, leaf)
        for i in range(3):
            o = base if i == 2 else copy_object(base, f"{name}_LOD{i}")
            o.name = f"{name}_LOD{i}"
            o.data.name = o.name
            if lods == "tree":
                solid, keep, twig, merge = TREE_LODS[name][i]
                drop_small_islands(o, wood, twig * height)
                weld(o, wood, merge * height)
                thin_leaves(o, leaf, keep, random.Random(seed + i))
                o = decimate(o, {k: (1.0 if k in leaf else solid) for k in range(len(mats))})
            else:
                o = decimate(o, {k: SOLID_LODS[i] for k in range(len(mats))})
            if kind in ("tree", "shrub", "plant"):
                o.data.shade_smooth()
                foliage_normals(o, {k for k, m in enumerate(mats) if MATERIALS[m][1] == "cutout" and "branch" not in m})
            lod_objs.append(o)

    # Scale to the twin, origin at the base (in plan: the bounds' centre, for kit pieces).
    me0 = lod_objs[0].data
    co0 = np.empty(len(me0.vertices) * 3, np.float32)
    me0.vertices.foreach_get("co", co0)
    lo, hi = co0.reshape(-1, 3).min(0), co0.reshape(-1, 3).max(0)
    shift = np.array([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, 0.0]) if recentre else np.zeros(3)
    for o in lod_objs:
        me = o.data
        co = np.empty(len(me.vertices) * 3, np.float32)
        me.vertices.foreach_get("co", co)
        co = (co.reshape(-1, 3) + shift) * scale
        me.vertices.foreach_set("co", co.ravel().astype(np.float32))
        me.update()
        if kind == "rock":
            me.shade_smooth()

    for o in list(bpy.data.objects):
        if o not in lod_objs:
            bpy.data.objects.remove(o)
    out = OUT / "Models" / f"{name}.fbx"
    with bpy.context.temp_override(selected_objects=lod_objs, selected_editable_objects=lod_objs):
        bpy.ops.export_scene.fbx(filepath=str(out), use_selection=True, object_types={"MESH"},
                                 apply_scale_options="FBX_SCALE_ALL", axis_forward="-Z", axis_up="Y",
                                 bake_space_transform=True, mesh_smooth_type="OFF", use_mesh_modifiers=False,
                                 add_leaf_bones=False, path_mode="STRIP", embed_textures=False)
    me0 = lod_objs[0].data
    co = np.empty(len(me0.vertices) * 3, np.float32)
    me0.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    # Blender (x, y, z) is Unity (-x, z, -y) after the export's axis change; sizes are what matter here.
    size = co.max(0) - co.min(0)
    radius = float(np.sqrt((co[:, 0] ** 2 + co[:, 1] ** 2).max()))
    info = {
        "kind": kind,
        "fbx": f"Models/{name}.fbx",
        "height": round(float(co[:, 2].max()), 4),
        "size": [round(float(size[0]), 4), round(float(size[2]), 4), round(float(size[1]), 4)],
        "radius": round(radius, 4),
        "lod_triangles": [tri_count(o.data) for o in lod_objs],
        "materials": {m.name: {"set": MATERIALS[m.name][0], "mode": MATERIALS[m.name][1]}
                      for m in me0.materials},
    }
    log(f"{name}: LOD tris {info['lod_triangles']} height {info['height']} radius {info['radius']} "
        f"({time.time() - t0:.0f} s)")
    return info


def sun_in(hdri: pathlib.Path) -> dict:
    """Where the sun is in the equirectangular sky: the centroid of its brightest pixels,
    u across from the left, v up from the bottom, both 0..1 (Unity's texture coordinates)."""
    px = load_pixels(hdri, data=True)
    lum = px[..., 0] * 0.2126 + px[..., 1] * 0.7152 + px[..., 2] * 0.0722
    top = lum >= np.percentile(lum, 99.995)
    v, u = np.nonzero(top)
    w = lum[top]
    h, wd = lum.shape
    return {"u": round(float((u + 0.5) @ w / w.sum() / wd), 5), "v": round(float((v + 0.5) @ w / w.sum() / h), 5),
            "peak": round(float(lum.max()), 1)}


def prepare_bush(name: str) -> dict:
    """A bush of leaf cards: each card copied from the small tree's crown, turned at random
    and set in a dome, more of them toward its skin; three LODs keeping fewer, larger cards."""
    hx, hy, h, n, seed = BUSHES[name]
    t0 = time.time()
    reset()
    import_fbx("tree_small_02")
    tree = join([bpy.data.objects["tree_small_02_LOD0"]], "tree")
    mats = [m.name for m in tree.data.materials]
    leaf = mats.index("tree_small_02_leaves")
    leaf_cards(tree, {leaf})
    me = tree.data
    cards = []
    uvl = me.uv_layers[0].data
    for f in me.polygons:
        if f.material_index == leaf and f.loop_total == 4:
            co = np.array([me.vertices[me.loops[i].vertex_index].co[:] for i in f.loop_indices])
            uv = np.array([uvl[i].uv[:] for i in f.loop_indices])
            cards.append((co - co.mean(0), uv))
    rng = np.random.default_rng(seed)
    pick = rng.choice(len(cards), n)
    verts, uvs = [], []
    for k in range(n):
        co, uv = cards[pick[k]]
        a, b, c = rng.uniform(0, 2 * np.pi, 3)
        rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
        rx = np.array([[1, 0, 0], [0, np.cos(b), -np.sin(b)], [0, np.sin(b), np.cos(b)]])
        ry = np.array([[np.cos(c), 0, np.sin(c)], [0, 1, 0], [-np.sin(c), 0, np.cos(c)]])
        d = rng.normal(size=3)
        d /= np.linalg.norm(d)
        d[2] = abs(d[2]) * 0.9 + 0.1                       # the upper half, a little below the middle
        r = 0.55 + 0.45 * rng.random() ** 0.5               # most leaves near the skin
        at = np.array([d[0] * hx, d[1] * hy, d[2] * h]) * r
        verts.append(co @ (rz @ rx @ ry).T + at)
        uvs.append(uv)
    mesh = bpy.data.meshes.new(name)
    v = np.concatenate(verts)
    faces = [(4 * k, 4 * k + 1, 4 * k + 2, 4 * k + 3) for k in range(n)]
    mesh.from_pydata(v.tolist(), [], faces)
    layer = mesh.uv_layers.new(name="UVMap")
    layer.data.foreach_set("uv", np.concatenate(uvs).ravel())
    mesh.materials.append(bpy.data.materials["tree_small_02_leaves"])
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o)
    base = bpy.data.objects.new(f"{name}_base", mesh)
    bpy.context.scene.collection.objects.link(base)
    lod_objs = []
    for i, keep in enumerate(BUSH_LODS):
        o = copy_object(base, f"{name}_LOD{i}")
        thin_leaves(o, {0}, keep, random.Random(seed * 10 + i))
        o.data.shade_smooth()
        foliage_normals(o, {0})
        me = o.data
        co = np.empty(len(me.vertices) * 3, np.float32)
        me.vertices.foreach_get("co", co)
        me.vertices.foreach_set("co", (co * TWIN_SCALE).astype(np.float32))
        me.update()
        lod_objs.append(o)
    bpy.data.objects.remove(base)
    out = OUT / "Models" / f"{name}.fbx"
    with bpy.context.temp_override(selected_objects=lod_objs, selected_editable_objects=lod_objs):
        bpy.ops.export_scene.fbx(filepath=str(out), use_selection=True, object_types={"MESH"},
                                 apply_scale_options="FBX_SCALE_ALL", axis_forward="-Z", axis_up="Y",
                                 bake_space_transform=True, mesh_smooth_type="OFF", use_mesh_modifiers=False,
                                 add_leaf_bones=False, path_mode="STRIP", embed_textures=False)
    me0 = lod_objs[0].data
    co = np.empty(len(me0.vertices) * 3, np.float32)
    me0.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    size = co.max(0) - co.min(0)
    info = {
        "kind": "shrub", "fbx": f"Models/{name}.fbx",
        "height": round(float(co[:, 2].max()), 4),
        "size": [round(float(size[0]), 4), round(float(size[2]), 4), round(float(size[1]), 4)],
        "radius": round(float(np.sqrt((co[:, 0] ** 2 + co[:, 1] ** 2).max())), 4),
        "lod_triangles": [tri_count(o.data) for o in lod_objs],
        "materials": {"tree_small_02_leaves": {"set": "tree_small_02_leaves", "mode": "cutout"}},
    }
    log(f"{name}: LOD tris {info['lod_triangles']} height {info['height']} radius {info['radius']} ({time.time() - t0:.0f} s)")
    return info


def export(objs, out: pathlib.Path):
    with bpy.context.temp_override(selected_objects=objs, selected_editable_objects=objs):
        bpy.ops.export_scene.fbx(filepath=str(out), use_selection=True, object_types={"MESH"},
                                 apply_scale_options="FBX_SCALE_ALL", axis_forward="-Z", axis_up="Y",
                                 bake_space_transform=True, mesh_smooth_type="OFF", use_mesh_modifiers=False,
                                 add_leaf_bones=False, path_mode="STRIP", embed_textures=False)


def prepare_figures() -> dict:
    """The shuttle and the student (tools/make_figures.py), with their plain materials."""
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    import make_figures as mf
    out = {}
    for name, build, path in (("shuttle", lambda: [mf.shuttle()], OUT / "Models" / "shuttle.fbx"),
                              ("student", mf.student, OUT / "Resources" / "RcTwinLook" / "student.fbx")):
        reset()
        objs = build()
        export(objs, path)
        mats = {m.name for o in objs for m in o.data.materials}
        out[name] = {
            "kind": name, "fbx": str(path.relative_to(OUT)).replace("\\", "/"),
            "triangles": sum(tri_count(o.data) for o in objs),
            "materials": {m: {"color": list(mf.MATERIALS[m][0]), "smoothness": mf.MATERIALS[m][1],
                              "metallic": mf.MATERIALS[m][2], "emission": list(mf.MATERIALS[m][3] or ())}
                          for m in sorted(mats)},
        }
        log(f"{name}: {out[name]['triangles']} triangles, {len(mats)} materials")
    return out


# ---------------------------------------------------------------------- main

def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    only = argv[argv.index("--only") + 1].split(",") if "--only" in argv else None
    if not SRC.exists():
        sys.exit(f"no {SRC}: run tools/fetch_assets.py first")
    fonts = OUT / "Resources" / "RcTwinLook"
    for d in ("Models", "Textures", "Sky", fonts):
        (OUT / d).mkdir(parents=True, exist_ok=True)
    manifest_path = OUT / "look.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() and only else \
        {"format": "rc-physics-twin campus look 0.1.0", "scale": TWIN_SCALE, "props": {}, "texture_sets": {}}

    for i, name in enumerate(PROPS):
        if only and name not in only:
            continue
        manifest["props"][name] = prepare_prop(name, seed=100 * i)
    for name in BUSHES:
        if only and name not in only:
            continue
        manifest["props"][name] = prepare_bush(name)
    if not only or "figures" in only:
        manifest["figures"] = prepare_figures()
    for name in TEXTURE_SETS:
        if only and name not in only:
            continue
        t0 = time.time()
        manifest["texture_sets"][name] = prepare_texture_set(name)
        log(f"texture set {name} ({time.time() - t0:.0f} s)")

    hdri = next((SRC / "polyhaven" / "hdri").glob(f"{SKY}_*.hdr"))
    for old in (OUT / "Sky").glob("*.hdr*"):
        if not old.name.startswith(SKY):
            old.unlink()
    shutil.copy2(hdri, OUT / "Sky" / hdri.name)
    manifest["sky"] = f"Sky/{hdri.name}"
    manifest["sun"] = sun_in(hdri)
    for f in (SRC / "fonts").iterdir():
        shutil.copy2(f, fonts / f.name)
    manifest["fonts"] = sorted(f"Resources/RcTwinLook/{f.name}" for f in (SRC / "fonts").glob("*.ttf"))
    manifest_path.write_text(json.dumps(manifest, indent=1))
    log(f"wrote {manifest_path}")


main()
