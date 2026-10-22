#!/usr/bin/env python3
"""Fetch the campus look's third-party assets (THIRD_PARTY.md, "Campus look").

Downloads exactly the files listed in ASSETS: a sunset sky, PBR textures and
models from Poly Haven (CC0), and the Inter font (SIL OFL 1.1), into
assets_src/ (gitignored: two models are over GitHub's 100 MB file limit).
Every Poly Haven file is checked against the MD5 its API publishes; files
already present with the right checksum are kept.

    python tools/fetch_assets.py            # everything
    python tools/fetch_assets.py --list     # what it would fetch, and how much

tools/prepare_assets.py (Blender) then turns the models into the scene's
reduced, 1/10-scale versions.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import sys
import urllib.request
import zipfile

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "assets_src"
API = "https://api.polyhaven.com/files/"
UA = {"User-Agent": "rc-physics-twin fetch_assets (github.com/Sujay-197/rc-physics-twin)"}

# (Poly Haven id, kind, resolution)
ASSETS = [
    ("belfast_sunset_puresky", "hdri", "4k"),
    ("asphalt_02", "texture", "2k"),
    ("concrete_pavement", "texture", "2k"),
    ("concrete_wall_008", "texture", "2k"),
    ("leafy_grass", "texture", "2k"),
    ("jacaranda_tree", "model", "2k"),
    ("tree_small_02", "model", "2k"),
    ("shrub_02", "model", "2k"),
    ("shrub_03", "model", "2k"),
    ("street_lamp_01", "model", "2k"),
    ("boulder_01", "model", "2k"),
    ("modular_street_seating", "model", "2k"),
]
TEXTURE_MAPS = ("Diffuse", "nor_gl", "Rough")
INTER = ("https://github.com/rsms/inter/releases/download/v4.1/Inter-4.1.zip",
         ("Inter-Regular.ttf", "Inter-Medium.ttf", "Inter-SemiBold.ttf", "Inter-Bold.ttf"))


def files_of(asset: str, kind: str, res: str) -> list[tuple[str, str, str, int]]:
    """(relative path, url, md5, size) of every file this asset needs."""
    with urllib.request.urlopen(urllib.request.Request(API + asset, headers=UA)) as r:
        f = json.load(r)
    out = []
    if kind == "hdri":
        e = f["hdri"][res]["hdr"]
        out.append((f"hdri/{pathlib.PurePosixPath(e['url']).name}", e["url"], e["md5"], e["size"]))
    elif kind == "texture":
        for m in TEXTURE_MAPS:
            e = f[m][res]["jpg"]
            out.append((f"textures/{asset}/{pathlib.PurePosixPath(e['url']).name}", e["url"], e["md5"], e["size"]))
    else:
        e = f["fbx"][res]["fbx"]
        out.append((f"models/{asset}/{pathlib.PurePosixPath(e['url']).name}", e["url"], e["md5"], e["size"]))
        for rel, inc in e.get("include", {}).items():
            out.append((f"models/{asset}/{rel}", inc["url"], inc["md5"], inc["size"]))
    return out


def md5(path: pathlib.Path) -> str:
    h = hashlib.md5()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def download(url: str, dest: pathlib.Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(dest.suffix + ".part")
    with urllib.request.urlopen(urllib.request.Request(url, headers=UA)) as r, open(tmp, "wb") as fh:
        while chunk := r.read(1 << 20):
            fh.write(chunk)
    tmp.replace(dest)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--list", action="store_true", help="list the files and sizes, download nothing")
    args = ap.parse_args(argv)
    plan = [x for a, k, r in ASSETS for x in files_of(a, k, r)]
    total = sum(x[3] for x in plan)
    print(f"{len(plan)} Poly Haven files, {total / 1e6:.0f} MB, plus Inter 4.1 (34 MB zip)")
    if args.list:
        for rel, _, _, size in plan:
            print(f"  {size / 1e6:7.1f} MB  {rel}")
        return 0
    for rel, url, digest, size in plan:
        dest = OUT / "polyhaven" / rel
        if dest.exists() and md5(dest) == digest:
            continue
        print(f"fetch {rel} ({size / 1e6:.1f} MB)", flush=True)
        download(url, dest)
        if md5(dest) != digest:
            print(f"MD5 mismatch: {rel}", file=sys.stderr)
            return 1
    fonts = OUT / "fonts"
    if not all((fonts / n).exists() for n in INTER[1]):
        z = OUT / "downloads" / "Inter-4.1.zip"
        if not z.exists():
            print("fetch Inter-4.1.zip", flush=True)
            download(INTER[0], z)
        fonts.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(z) as zf:
            names = zf.namelist()
            for want in INTER[1] + ("LICENSE.txt",):
                hit = next((n for n in names if n.endswith("/" + want) or n == want), None)
                if hit is None:
                    print(f"Inter zip has no {want}", file=sys.stderr)
                    return 1
                (fonts / want).write_bytes(zf.read(hit))
    print(f"done: {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
