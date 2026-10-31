# Contributing

Thanks for looking. Bug reports, questions and pull requests are all welcome. For a larger change, open an issue first so we can agree on the approach. [`Docs/NEXT_STEPS.md`](Docs/NEXT_STEPS.md) lists open work.

## Before you open a pull request

Build and run the tests as in the [README](README.md#getting-started):

```bash
.toolchain/venv/Scripts/ctest --test-dir build --output-on-failure
```

If you touched `core/`, also run the core's tests and the determinism check:

```bash
.toolchain/venv/Scripts/cmake -S core -B build-core -G Ninja -DCMAKE_TOOLCHAIN_FILE="$PWD/.toolchain/zig-toolchain.cmake" -DSLIPX_BUILD_TESTS=ON -DSLIPX_BUILD_PYTHON=OFF
```

```bash
.toolchain/venv/Scripts/cmake --build build-core
```

```bash
.toolchain/venv/Scripts/ctest --test-dir build-core --output-on-failure
```

```bash
python tools/check_determinism.py --build-dir build-core --require-row
```

CI runs all of this on every push. It can't run the Unity tests, which need a licensed editor, so if you touched `unity/`, run the Edit Mode and Play Mode suites in the Test Runner and say in your pull request that you did.

## Rules of the project

- **Parameters are honest.** Label every parameter set `measured`, `identified` or `provisional`, and don't claim validation without a report. Say where a number comes from, and don't make one up.
- **Results stay reproducible.** Report seeds, conditions and the command that produced a number. [`Docs/policy-study.md`](Docs/policy-study.md) is the model to follow, including when the result is negative.
- **The core stays deterministic.** A default-off change to `core/` must leave the reference hashes in `conformance/reference_hashes.tsv` unchanged. Changing them needs a stated reason.
- **Changes to `core/` carry a notice.** Apache-2.0 section 4(b) requires it. Add a one-line "Modified by the rc-physics-twin project" comment under the licence header, list the change in [`THIRD_PARTY.md`](THIRD_PARTY.md), and keep each core change in its own commit so re-vendoring stays mechanical ([ADR 0001](Docs/adr/0001-fork-slipx-as-dynamics-core.md)).
- **Binaries must run elsewhere.** The toolchain pins the `x86_64_v2` CPU target. Don't add `-march=native` or anything else that ties a build to the machine it was built on.
- **Significant decisions get an ADR.** Add one to [`Docs/adr/`](Docs/adr/).

## Third-party material

- Check the licence of anything you bring in, and add it to [`THIRD_PARTY.md`](THIRD_PARTY.md) with its licence file.
- **Don't commit or distribute map screenshots.** `tools/campus_scenery.py --images` can use local screenshots for tree cover and ground colour, for your own reference. Map imagery is usually someone else's copyright, so it and everything made from it (`campus/reference/`, `Generated/Campus/Local/`) stay gitignored.
- **Building a player to share?** Build from a checkout that has no `campus/reference/` or `Generated/Campus/Local/`. Otherwise the player contains that imagery.
- Anything derived from the OpenStreetMap extract and shared publicly stays under the ODbL, with attribution.

## Licensing of contributions

Unless you say otherwise, contributions are under the Apache License 2.0, as section 5 of the licence provides.
