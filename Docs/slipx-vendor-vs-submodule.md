# SlipX in this repo: vendor or submodule?

**Status:** Decided 2026-10-19: **vendor**. The project owner chose to copy SlipX into `core/` (see ADR 0001, update). The comparison below is kept as the record of the trade-off.  **Date:** 2026-10-19

The README places the Core at `core/` (forked SlipX). There are two ways to put it there. Nothing is in `core/` yet.

## Facts that bear on it

- Pinned commit: `ebdbf3f` on upstream `main` (see `THIRD_PARTY.md`). Our fork is [Sujay-197/slipx](https://github.com/Sujay-197/slipx).
- Upstream size at the pin: 366 tracked files, 106 commits, about 9 MB of history. `src/core/` is 53 files, about 14k lines.
- `slipx_core` builds alone (`-DSLIPX_CORE_ONLY=ON`). The conformance runner `slipx_conformance` is **not** in the core: it lives in `src/orchestration/slipx_sim`. Bringing in only `src/core/` would leave the determinism check behind.
- Upstream's checks (`tools/check_conformance.py`, `licence_scan.py`, `dep_lint.py`, CI workflow) assume the repository root is SlipX's root.
- Upstream is alpha (v0.2.x), one maintainer, active: 53 commits landed between `v0.2.0` (2026-08-13) and the pin (2026-09-13).
- Our planned changes go deep into the Core: Ackermann steering, per-wheel spin state, a suspension tier and the ground-query callback (ADR 0001, spec). Some should go back upstream.

## Comparison

| | Submodule of our fork at `core/` | Vendor (copy) into `core/` |
|---|---|---|
| Where our Core changes live | Branches in `Sujay-197/slipx`; this repo records a commit SHA | Commits in this repo |
| Upstream PRs | Natural: branch in the fork, PR to `ibrahimsel/slipx` | Changes must be extracted from this repo and replayed onto a fork |
| Taking upstream fixes | `git fetch upstream` in the fork, merge, bump the SHA here | Manual diff and copy, or `git subtree pull` if it was added with `git subtree` |
| Upstream checks and conformance | Work unchanged inside `core/` | Work only if the whole tree is copied, not just `src/core/` |
| Atomic change across Core and Unity plugin | Two commits in two repos, pushed in order (fork first) | One commit |
| Cloning and CI | Needs `--recurse-submodules`; CI and Unity checkout must fetch the fork | Plain clone |
| Apache-2.0 notices | `LICENSE`/`NOTICE` stay where upstream put them | Copy `LICENSE`/`NOTICE` into `core/` and keep them current; mark modified files |
| Repo size | Unaffected | Grows by the copied tree |

A middle option is `git subtree add --prefix core <fork> <sha> --squash`: a plain clone like vendoring, with a scripted path for pulling upstream and splitting out patches. It costs some tooling knowledge, and squashed history makes upstream PRs harder than with a submodule.

## Lean

A submodule of the fork at `core/`, pinned to `ebdbf3f`, fits the next phases better: the Core changes are large and partly upstreamable, the upstream is moving, and upstream's own conformance and licence checks keep working. Revisit vendoring if upstream goes quiet or if the two-repo commit order starts to hurt. This is a lean for the user to accept or reject, not a decision.
