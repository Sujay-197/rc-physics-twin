# ADR 0002: Core is engine-agnostic; Unity is a ground provider and renderer

**Status:** Proposed  **Date:** 2026-10-19

## Context

Unity's PhysX is a general rigid-body engine. Its wheel colliders are a poor fit for RC-scale tire behavior, and tying vehicle dynamics to a game engine makes headless batch runs and cross-validation harder.

## Decision

The Core integrates vehicle dynamics itself. The host supplies a **ground query** (height, normal, friction scale, surface id) per contact patch per step. Unity is responsible for road geometry, rendering, input and UI, and loads the Core as a native plugin through a C ABI.

PhysX may still be used for non-vehicle collisions and for answering ground raycasts.

## Consequences

- Headless runs (edge profiler, RL, CI) need no Unity: a test harness implements the same ground query.
- Cross-validation against gymkhana uses the same Core binary.
- Speed breakers and slope live in road data, not only in mesh colliders, so headless and Unity runs agree.
- Cost: a thin native-plugin layer to build and maintain; ground-query call overhead at 1 kHz.
