#!/usr/bin/env python3
"""Check that the vendored Core still produces this repository's recorded
conformance hashes (conformance/reference_hashes.tsv).

SlipX's own checker (core/tools/check_conformance.py) runs the scenario and
keys each result by build; it is reused here unchanged, pointed at our
reference file instead of SlipX's, and without its wheel-pin check (that
guards SlipX's own release workflow, which this repository does not have).

Usage (from the repo root, after building core/ with the simulator):
    python tools/check_determinism.py --build-dir build-core [--require-row]

Exit 0 when every case matches its row (or has none, unless --require-row);
1 on any mismatch; 2 if the conformance binary is missing.
"""

from __future__ import annotations

import argparse
import importlib.util
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
REFERENCE = ROOT / "conformance" / "reference_hashes.tsv"

_spec = importlib.util.spec_from_file_location("check_conformance", ROOT / "core" / "tools" / "check_conformance.py")
cc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cc)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--build-dir", default="build-core",
                    help="a build of core/ with the simulator (relative to the repo root)")
    ap.add_argument("--require-row", action="store_true",
                    help="fail when this build has no reference row (CI on the reference toolchain)")
    args = ap.parse_args(argv)

    cc.REFERENCE_FILE = REFERENCE
    reference = cc.read_reference()
    suffix = ".exe" if sys.platform == "win32" else ""
    binary = ROOT / args.build_dir / "src" / "orchestration" / "slipx_sim" / f"slipx_conformance{suffix}"
    if not binary.exists():
        print(f"conformance binary not found at {binary}", file=sys.stderr)
        return 2

    failures, missing = [], []
    for tier, integrator in cc.CASES:
        manifest = cc.run_case(binary, tier, integrator)
        digest = manifest["trajectory_hash"]
        key = cc.key_of(manifest, tier, integrator)
        row = next((r for r in reference if all(r[c] == key[c] for c in cc.COLUMNS[:-1])), None)
        label = f"{tier}/{integrator}"
        if row is None:
            missing.append(cc.format_row(key, digest))
            print(f"  {label:28s} {digest}  (no reference row for this build)")
        elif row["hash"] != digest:
            failures.append(f"{label}: expected {row['hash']}, got {digest} "
                            f"({manifest['build']['cxx_flags']})")
            print(f"  {label:28s} {digest}  MISMATCH (expected {row['hash']})")
        else:
            print(f"  {label:28s} {digest}  matches")

    if failures:
        print("\nDETERMINISM CHECK FAILED: the Core's arithmetic changed for a recorded build.",
              file=sys.stderr)
        for f in failures:
            print(f"  - {f}", file=sys.stderr)
        return 1
    if missing:
        print(f"\n{len(missing)} case(s) have no row for this build in {REFERENCE.relative_to(ROOT)}:")
        for m in missing:
            print(f"  {m}")
        if args.require_row:
            print("--require-row: a missing row is a failure here.", file=sys.stderr)
            return 1
    print(f"\n{len(cc.CASES) - len(missing)} of {len(cc.CASES)} case(s) match the reference.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
