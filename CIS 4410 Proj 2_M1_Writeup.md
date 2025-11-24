# Milestone 1 Design Write-up

Our Uppaal submission (`Proj_2_M1_V2.xml`) implements the closed-loop system requested in the Smart Pacemaker Milestone 1 handout: a VVI-mode pacemaker controller paired with a non-deterministic RandomHeart environment. The following sections summarize the final model and demonstrate how it satisfies the items listed in the project and milestone documents (especially Table 6 of the McMaster pacemaker spec).

## 1. Heart Automaton (RandomHeart)

* **Locations & clocks** – The heart uses a single clock `xh` with three control locations: `Resting` (time elapses with `xh <= maxIBI`), `SenseReady` (reachable once `xh >= minIBI` and still bounded by `maxIBI`), and two committed locations (`PacedEvent`, `SenseEvent`) that model instantaneous reactions.
* **Intrinsic beats** – When `xh >= minIBI` the automaton transitions into `SenseReady`, and the urgent `sense!` happens immediately from a committed `SenseEvent`, leaving `last_sense = true` until the pacemaker clears it. Guards appear on the entry transition so no clock constraints reside on the urgent edge, satisfying Uppaal’s urgent-channel rule.
* **Paced beats** – Whenever `xh < minIBI`, the heart can accept `pace?` and move to `PacedEvent`, which resets `xh`. This models the requirement that the environment resets its timer when paced.
* **Bounds** – Invariants and guards enforce `minIBI <= xh <= maxIBI` whenever a sensed event fires, meeting the milestone’s demand for a “random heart” with controllable minimum/maximum inter-beat intervals.

## 2. Pacemaker Automaton (VVI Controller)

* **Locations** – The controller has four main control modes: `WaitRI` (primary sensing/pacing state), `SenseReady` (an intermediate state reached once `xv >= URL`), `WaitVRP` (ventricular refractory), and committed `PaceEvent` / `SenseEvent` locations where the actual channel synchronizations occur.
* **Timing constants** – The model uses the constants specified in the project write-up: `LRL`, `URL`, `VRP`, and the hysteresis interval `HRI`. The variable `RI` stores the currently active lower-rate interval (LRL vs. HRI) and is updated during the committed events.
* **Behavior**
  * From `WaitRI`, if `xv >= LRL` we pace (`pace!`) and immediately enter `WaitVRP` after resetting `xv`.
  * When `xv >= URL`, we move into `SenseReady`; the actual `sense?` handshake occurs from this state via the urgent channel. This ensures any state where `last_sense` becomes true had `xv` above the URL threshold.
  * `WaitVRP` enforces `xv <= VRP` and blocks all sensing/pacing until the refractory timer expires, fulfilling the “post-ventricular timing” entry in Table 6.
* **Flags/channels** – `pace` and `sense` channels couple the automata, and boolean flags (`last_pace`, `last_sense`) allow the queries to observe the most recent event, matching the milestone hint about using observers.

## 3. Simulation Notes

Running Uppaal simulations confirms the closed-loop interaction: the heart non-deterministically produces intrinsic beats, the controller inhibits pacing when late sensed beats arrive, and pacing occurs when `xv` hits the active `RI`. The urgent `sense` ensures zero detection latency as assumed by the spec.

## 4. Verified Properties (Table 6 coverage)

All properties below are present in the `<queries>` block of `Proj_2_M1_V2.xml` and have been re-checked after the final edits. Each formula either enforces a design constraint or directly encodes the corresponding Table‑6 requirement.

| Spec item | Uppaal Query | Explanation |
| --- | --- | --- |
| Deadlock freedom | `A[] not deadlock` | Ensures the closed-loop system continues operating, as required by the milestone instructions to simulate the model. |
| VRP hold-off | `A[] Pacemaker.WaitVRP imply Pacemaker.xv <= VRP` | Guarantees sensing/pacing is inhibited during the ventricular refractory period. |
| VRP exit | `A[] (Pacemaker.xv >= VRP) imply !Pacemaker.WaitVRP` | Confirms the controller leaves VRP once the timer expires. |
| Lower Rate Limit | `A[] Pacemaker.xv <= LRL` | The controller must pace before exceeding the lower rate interval when no sense occurs. |
| Upper Rate / sensing | `A[] last_sense imply Pacemaker.xv >= URL` | A sensed event can only occur after exceeding the URL threshold (consistent with hysteresis entry). |
| Post-pace timing | `A[] last_pace imply Pacemaker.xv >= VRP` | Prevents pacing more frequently than permitted by VRP. |
| Hysteresis updates | `A[] last_sense imply RI == HRI` and `A[] last_pace imply RI == LRL` | Matches the spec requirement to switch the active lower-rate interval depending on the last event. |
| Heart safety | `A[] Heart.xh <= maxIBI` and `A[] last_sense imply (Heart.xh >= minIBI && Heart.xh <= maxIBI)` | Keeps the RandomHeart within its physiological range and ensures sensed beats truly originate inside the min/max window. |
| Sensor liveness | `A<> last_sense` | Confirms intrinsic sensing remains reachable, addressing the “Maximum Sensor Rate” constraint. |

These queries (plus the encoded invariants/guards) cover every VVI-mode requirement outlined for Milestone 1. They provide the “brief explanations” requested in part (c) of the milestone sheet by tying each temporal logic formula back to a specific Table‑6 item.
