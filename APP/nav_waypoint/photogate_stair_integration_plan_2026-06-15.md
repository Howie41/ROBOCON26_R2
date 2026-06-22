# Photogate Stair Integration Plan

## Scope

This file records the planned application-layer integration for rear photogate.

Current rule:

- do not change app logic yet without explicit approval
- integrate in small reviewable phases

---

## Phase 1

Already completed:

- bottom-layer photogate module
- Ozone-visible globals
- periodic update from `liftTask()`

No stair action logic has been changed yet.

---

## Phase 2

Next planned application integration:

- use rear photogate only
- do not touch front photogate logic yet

### Climb-up

- once climb-up action has already entered high mode
- ignore rear `1 -> 0`
- wait for rear `0 -> 1`
- then allow `liftRequestLow()`

### Descend

- once descend preparation action is active
- wait for rear `1 -> 0`
- then allow `liftRequestHigh()`

---

## Required Guards

Rear photogate must not act alone.

At minimum, app layer must gate it with:

1. Merlin stage enabled
2. current action is stair-related
3. current stair phase is correct
4. one-shot latches prevent repeated requests

---

## Why This Plan

This preserves the agreed control philosophy:

- laser gives coarse stage readiness
- photogate gives precise final switching permission
- flat-ground navigation outside Merlin does not get accidental lift switching
