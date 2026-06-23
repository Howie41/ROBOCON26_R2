# Stair Assist Photogate Update

## 1. Purpose

This note records the latest agreed stair-assist decision logic as of 2026-06-15.

The key change is:

- laser distance sensors provide coarse stair-stage judgment
- photogate provides fine final switching permission
- both are only allowed to affect lift switching during Merlin stair actions

This note is intended to supplement the existing `stair_assist.md`.

---

## 2. Vehicle Structure Summary

R2 is a combined stair robot with:

- 4 omni wheels for normal low-mode motion
- 2 lift 3508 motors for low/high body switching
- 2 front 2006 motors for high-mode forward/backward stair drive
- passive rear support structure

Current new sensor layout:

- `photogate1`: front
- `photogate2`: rear

Current priority is to use `photogate2` first.

---

## 3. Sensor Roles

### 3.1 Laser Sensors

Laser sensors are used for coarse judgment:

- close enough to stair
- enter stair preparation region
- allowed to start a stair phase

They should not be the final authority for every low/high switch.

### 3.2 Rear Photogate

Rear photogate is used for fine judgment:

- whether the chassis rear underside is blocked by ground/platform
- whether the robot has truly reached the switching moment

Current confirmed semantics:

- `blocked = 1`: rear underside is blocked
- `blocked = 0`: rear underside is not blocked

Therefore:

- `blockedEdge` means `0 -> 1`
- `unblockedEdge` means `1 -> 0`

---

## 4. Global Gating Principle

Photogate and laser are both stair-assist sensors.

They must not trigger lift switching in non-stair contexts.

Recommended gating:

1. sensors may keep sampling all the time
2. but decision logic is only enabled during Merlin stair actions

That means:

- non-Merlin mode:
  - stair assist decision disabled
  - no photogate-triggered lift switching

- Merlin mode but not currently climbing/descending:
  - sampling allowed
  - no lift switching from photogate

- Merlin mode and current stair action active:
  - laser + photogate may participate in switching decisions

---

## 5. Climb Up Flow: Ground To Upper Platform Center

### Stage A: Ground low mode

- robot is on ground
- low mode active
- rear photogate is usually blocked

This is only a background state and must not trigger low/high switching.

### Stage B: Coarse high-entry permission

When laser condition indicates:

- stair is close enough
- climb-up preparation is ready

then:

- request high mode

### Stage C: High mode after lift-up

After entering high mode:

- the chassis is lifted
- rear photogate often changes from blocked to unblocked

This transition is expected and must not cause a wrong action.

So:

- after climb-up high entry, rear `1 -> 0` must be ignored

### Stage D: 2006 forward drive

After high mode is active:

- 2006 drives forward
- rear photogate remains unblocked for a period

During this phase:

- do not allow return to low mode just because laser roughly thinks it is time
- keep advancing
- rear photogate has the final authority

### Stage E: Final low-entry permission

When 2006 has advanced enough:

- chassis rear underside becomes blocked again
- rear photogate changes from unblocked to blocked

This is the true precise low-entry condition.

Therefore climb-up switching rule becomes:

- enter high mode:
  - by laser coarse condition

- enter low mode:
  - by rear photogate `blockedEdge` (`0 -> 1`)

---

## 6. Descend Flow: Upper Platform Center To Ground

### Stage A: Upper platform center low mode

- robot is on stair/platform center
- low mode active
- rear photogate is usually blocked

### Stage B: Backward approach to edge

Robot moves backward toward the stair edge.

Laser is only used here as coarse preparation information.

### Stage C: Final high-entry permission

When the rear underside leaves support:

- rear photogate changes from blocked to unblocked

This is the true precise high-entry condition.

Therefore descend switching rule becomes:

- enter high mode:
  - by rear photogate `unblockedEdge` (`1 -> 0`)

After that:

- 3508 lifts the body
- 2006 can take over high-mode backward motion

### Stage D: Later descend low-entry

This note does not finalize descend return-to-low logic yet.

That part may later use:

- front photogate
- laser
- or both

Current confirmed point is only:

- rear `1 -> 0` is the precise descend high-entry trigger

---

## 7. Current Agreed Decision Principle

### Climb Up

- laser = coarse permission to enter high mode
- rear photogate = precise permission to return low mode

Specifically:

- if rear photogate is still unblocked, do not enter low mode
- keep current high-mode motion going
- only rear `0 -> 1` may allow low-mode return

### Descend

- laser = coarse preparation information
- rear photogate = precise permission to enter high mode

Specifically:

- only rear `1 -> 0` may allow high-mode entry

---

## 8. Recommended Code Change Plan

To keep the system safe and easy to verify, the next code step should be:

1. keep `Module/photogate` as pure GPIO state + edge provider
2. add stair-action gating in app layer
3. only consume photogate output when:
   - Merlin stage enabled
   - current action is climb-up or descend
4. first integrate rear photogate only
5. leave front photogate for the next phase

---

## 9. Current Verification Status

Current completed verification:

- `PD14` / `PD15` input configured
- photogate module implemented
- photogate module connected to `liftTask()` update loop
- Ozone global mirror variables added
- rear photogate semantics confirmed:
  - blocked = 1
  - unblocked = 0

This means rear photogate logic polarity is already correct for further app-layer integration.
