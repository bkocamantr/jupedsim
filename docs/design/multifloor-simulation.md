# Design: Multi-floor simulation in JuPedSim

Status: **Proposal / plan, revised after syncing with upstream** (upstream
`PedestrianDynamics/jupedsim` master as of 2026-09-23, commit `098099e2`).

The first version of this plan was written against a fork that was still on
v1.3.0 (June 2025). Upstream has since landed
"First version of multi-floor building support" (`4d0fe0f8`, 2026-09-04),
which already implements most of that plan's core: floors, stairs and ramps,
3D positions and routing. This revision describes what upstream now provides,
what is still missing, and how to add the missing parts on top of upstream's
design instead of beside it.

## 1. What upstream already provides

| Capability | Where | Notes |
|---|---|---|
| Floors at different heights | `Geometry/WalkableSurface.{hpp,cpp}`: `AddRegion(polygon, height)` | Overlapping floors at the same height are rejected (`ValidateFloorOverlap`). |
| Stairs and ramps as walkable, inclined connectors | `WalkableSurface::ConnectRegions(fromRegion, fromEdge, toRegion, toEdge)` | A connector is its own inclined region spanning the two edges. Inclines above 50° are rejected (`Geometry/Validation.cpp`). |
| Arbitrary 3D mesh input | `Geometry(SurfaceMesh)`, OBJ files (`examples/geometry/*.obj`) | Upstream says the OBJ input "will change". |
| Automatic split into 2D regions | `Geometry/RegionSplit.{hpp,cpp}` | Each region projects to the x/y plane without overlapping itself, so the models can keep working in 2D. |
| Agent positions with floor information | `Geometry/Location.hpp`; `GenericAgent::location` | 2D `xy` plus region ID plus cached `z`. `move_on_surface()` carries agents across region seams. |
| Wall queries across regions | `Geometry::line_segments_in_range`, `no_geometry_between` | Ray-cast and exact: no walls behind walls, and sight lines cross seams. |
| Neighbours on other floors ignored | `AgentView.hpp` | Candidates more than `InteractionHeight` (2 m) apart in z are skipped. |
| 3D routing | `SurfaceMeshShortestPathRoutingEngine` | Exact shortest paths on the surface mesh (CGAL), held off wall corners by a clearance. It replaced the old navmesh router. |
| Python API | `jps.WalkableSurface`, `add_region`, `connect_regions`; `z_hint=` on agents and stages; `jps.Location` | Upstream: "The Python API is not finalized", and `z_hint` "is likely to change to regions". |
| Examples | `examples/example_3d.py` (U-shaped stair), `examples/example_office_5floors.py` | |
| 3D viewer | `python_modules/trame_viewer`, `examples/simulation_viewer.py` | Upstream calls it temporary, for debugging. |

Other upstream changes that affect this work:

* **The C API (`libjupedsim`) was removed** (`c5bc7467`, 2025-11-16). Python
  (pybind11) is the only public interface. The C API section of the old plan
  is dropped.
* The operational models moved to `libsimulator/src/OperationalModels/<Model>/`.
  Each model implements
  `Point ComputeNextState(current, next, const AgentStep&)`, which returns the
  horizontal movement for one step. `OperationalDecisionSystem` then applies it
  with `location.move_on_surface(movement)`.
* There are now 8 model types: CFSM, CFSM V2, CFSM V3, AVM, GCFM, SFM,
  WarpDriver, and custom models written in Python.
* `EnvironmentQuery` / `AgentView` / `AgentStep` are the single interface
  through which models see walls and neighbours.

## 2. What is still missing

| Gap | Current behaviour | Why it matters |
|---|---|---|
| **Walking speed on stairs and ramps** | Upstream: "Models still only see 2D." Agents cross a stair at their flat-ground horizontal speed. | Stair capacity and travel time are usually what a multi-floor study is about. Real walking speeds on stairs are much lower than on flat ground, and differ between going up and down. |
| **Escalators and moving walkways** | None. | They need one-way travel and a belt velocity. |
| **Lifts** | None. | Not a walkable surface. Agents wait, board, disappear and reappear on another floor after a travel time. |
| **Routing through one-way or discrete connections** | The surface shortest-path router treats every walkable face as usable in both directions. | Escalators (one-way) and lifts (not walkable) can't be expressed as surface paths. |
| **Floor information in trajectory output** | SQLite stores only `pos_x`, `pos_y`. The HDF5 writer has a `z` column but always writes `0.0`. Neither stores the region. | Results can't be analysed per floor, and the 3D geometry isn't saved. |
| **Stable, region-based Python API** | `z_hint` float on agents and stages. | Upstream plans to switch to regions. |
| **Visualiser** | `jupedsim_visualizer` is 2D only; the trame viewer is a temporary debugging tool. | |

## 3. Strategy

1. **Build on upstream's design, not beside it.** Stairs and ramps stay
   upstream's inclined connector regions; positions stay `Location`; routing
   stays surface-based within connected walkable parts.
2. **Coordinate with the upstream maintainers before starting.** The multi-floor code
   is three weeks old and explicitly unfinished (input format, Python API,
   viewer). Open an issue or discussion on `PedestrianDynamics/jupedsim` with
   this plan. Agree which parts upstream is already doing, so work isn't done
   twice, and send the rest as upstream PRs.
3. **Keep the fork in sync.** Merge upstream master at least before each phase.
   The multi-floor code is changing fast, and a stale fork is what caused the
   first version of this plan to miss it.

## 4. Changes, by gap

### 4.1 Stair and ramp walking speed

* Add the slope at the agent's position to what models can query: e.g.
  `AgentView::surface_gradient()`, returning the x/y gradient of `z` on the face
  the agent stands on. `Location` already has the face cached.
* Add a speed factor per region, defaulting to 1.0 for floors. Connectors get
  `speed_factor_up` and `speed_factor_down`, set in
  `WalkableSurface::ConnectRegions`/`connect_regions(...)`. Both have defaults
  (to be calibrated) and can be overridden per connector.
* **Where to apply it:** scale the agent's *desired speed* through `AgentStep`
  (e.g. `step.desired_speed_factor()`), which each model reads when it computes
  its target velocity. Do **not** scale the movement returned by
  `ComputeNextState`. For second-order models (GCFM, SFM, WarpDriver) the
  velocity is stored in the model state, so rescaling only the movement would
  make position and velocity disagree. This touches all 7 built-in models,
  with one line each at the point where `desired_speed` is used. Custom Python
  models get the factor through their `AgentStep` binding.
* Up or down is decided from the sign of `dot(desired direction, gradient)`.
* Plan-view speed is additionally scaled by `cos(slope)`, so the configured
  speed means speed *along* the stair, as measured in experiments.

### 4.2 Directed connectivity and a two-level router

Needed before escalators and lifts.

* Keep `SurfaceMeshShortestPathRoutingEngine` for movement **within** a
  connected walkable part.
* Add a **connection graph** on top of it. Its nodes are one-way seams (escalator
  entries and exits) and lift stops. Its edges are:
  * surface travel between two nodes, weighted by the surface shortest-path length
    divided by speed (computed once, then cached);
  * escalator traversal: one direction only, weighted by length / (belt speed + walking speed);
  * lift trips, weighted by expected waiting time + door time + travel time.
* A route query picks the best sequence of connections (Dijkstra or A*), then
  hands the surface router the next leg's target. `WalkableSurface` already
  builds a boost region graph (`RegionGraph2D`) that the connection graph can
  start from.
* `TacticalDecisionSystem` asks this two-level router for `nextTarget`.

### 4.3 Escalators and moving walkways

* A new connector kind: `WalkableSurface::ConnectRegions(..., ConnectorKind::Escalator,
  direction, belt_speed)`. Geometrically it is an inclined region like a stair.
  A moving walkway is the same thing with no height change.
* **One-way travel:** in `Location::move_on_surface`, crossing the entry seam
  against the direction of travel is blocked as if it were a wall. The
  router (4.2) never plans a route against the direction.
* **Belt velocity:** added to the agent's movement **after** `ComputeNextState`,
  in `OperationalDecisionSystem`. Unlike the stair factor, the belt moves the
  ground under the agent, not the agent's own velocity. Optionally, agents can
  stand still on the escalator (desired speed factor 0).

### 4.4 Lifts

* A new `Lift` entity and `LiftSystem`, run once per iteration in
  `Simulation::Iterate()` after the operational step.
  * One stop per served floor. A stop has a boarding area, reusing the
    `NotifiableWaitingSet` slots, and exit positions.
  * Settings: capacity, door time, speed, and a dispatch strategy (simple
    collective control first).
  * Agents waiting at a stop board when the lift is at that floor with its doors
    open. Boarded agents are taken out of the neighbour grid and the
    operational step. They are re-inserted at the destination's exit
    positions, using `Geometry::get_location` with the stop's height.
* Agent state gains `std::optional<LiftId> inLift`. Agents inside a lift are
  skipped by the neighbour search and the models, and output reports them
  with the lift ID.
* The router (4.2) treats lift trips as edges. When the route's next
  connection is a lift, the strategic system gives the agent the lift's
  boarding stage as its target.

### 4.5 Output

* **SQLite:** bump the format version. Add `pos_z` and `region` columns
  to `trajectory_data`, plus an `in_lift` column once 4.4 exists. Store the
  mesh (`Geometry::vertices()`, `triangles()`, `region_id_per_face()`) in a new
  `geometry_mesh` table. Keep the WKT geometry for 2D-only readers such as
  PedPy.
* **HDF5:** write `agent.location.z` instead of the hard-coded `0.0`, and add
  `region`.
* Helper to export per-floor 2D trajectories for PedPy.

### 4.6 Python API, docs and viewer

* Follow upstream's move from `z_hint` to regions:
  `add_agent(..., region=upper)`, `add_exit_stage(..., region=upper)`.
  Keep `z_hint` as an alternative.
* Keyword arguments for the stair speed factors and escalator settings on
  `connect_regions`, plus `jps.Lift(...)`.
* 2D visualiser: add a floor selector, showing one region stack at a time. Leave 3D
  viewing to upstream's viewer work.
* A docs concept page and a notebook: two floors, a stair, an escalator, a lift.

## 5. Testing

* **C++ unit tests (`libsimulator/test`):**
  * slope and gradient queries;
  * the speed factor applied in each model;
  * one-way seam blocking;
  * connection-graph shortest paths;
  * the lift state machine and capacity.
* **Python tests / system tests:**
  * all existing tests still pass (flat geometries behave the same);
  * travel time on a stair of known length matches the configured factors, up and down;
  * no agent ever moves against an escalator's direction;
  * lift throughput never exceeds its capacity, and travel time is accounted for;
  * the 5-floor office example evacuates completely.
* **Output round-trip:** z and region are written and read back correctly, in
  both SQLite and HDF5.

## 6. Phases

Each phase is one upstream-ready PR, preceded by a merge of upstream master.

| Phase | Content | Depends on |
|---|---|---|
| 0 | Sync fork with upstream (**done**); open an upstream issue with this plan | – |
| 1 | Output: z and region in SQLite and HDF5, mesh stored in SQLite | – |
| 2 | Stair and ramp speed factors (4.1) | – |
| 3 | Connection graph and two-level router (4.2) | – |
| 4 | Escalators and moving walkways (4.3) | 3 |
| 5 | Lifts (4.4) | 3 |
| 6 | Region-based Python API, visualiser floor selector, docs, notebook (4.6) | 1–5, coordinated with upstream |

Phases 1, 2 and 3 are independent and can run in parallel.

## 7. Open questions (for upstream)

* Is upstream already working on any of stair speed, escalators, lifts, output
  or the region-based API? If so, which?
* Where do stair speed factors belong: in the geometry (per connector) or in the
  agent (per-agent stair ability)? Probably both: a per-connector default
  multiplied by a per-agent factor.
* Is upstream planning a replacement for OBJ input (e.g. IFC or DXF), which
  would need to carry connector kinds (stair, escalator, lift)?
* Should the connection graph (4.2) live inside the routing engine or sit
  above it as a separate strategic layer?
* Lift dispatch: which strategies are needed first?

## 8. Estimated Claude Code usage cost

This assumes Claude Code (Opus 5.5) writes the code, at Anthropic API rates:
$4 per million input tokens, $20 per million output tokens, $0.20 per million
for cache reads, $8 per million for 1-hour cache writes. On a Claude
subscription plan, usage counts against plan limits instead of being billed.
These are planning numbers; recalibrate them after the first phase.

A working session (one focused chunk of work, most input re-read from cache)
comes to about **$8** when light (~80 calls, ~80K context) and **$30** when
debugging-heavy (~250 calls, ~180K context).

| Phase | Sessions | Estimated cost |
|---|---|---|
| 1. Output (z, region, mesh) | 2–4 | $15–120 |
| 2. Stair and ramp speed factors | 3–5 | $25–150 |
| 3. Connection graph and two-level router | 6–10 | $50–300 |
| 4. Escalators and moving walkways | 3–6 | $25–180 |
| 5. Lifts | 8–12 | $65–360 |
| 6. Python API, viewer, docs, notebook | 4–8 | $30–240 |
| **Total (all phases)** | **26–45** | **~$210–1,350, most likely ~$550** |

**Stairs only** (phases 1, 2 and the stair part of 6): about 7–13 sessions,
**~$55–390**. Under the first version of this plan, stairs as walkable areas
were estimated at $250–1,500, because floors, connectors, 3D positions
and routing all had to be built. Upstream has now built them.

What moves the number:

* **Upstream churn.** The multi-floor API is explicitly unfinished. Work
  that lands on code upstream then rewrites has to be redone. Allow
  about 20% extra, and agree the plan with upstream first (section 3).
* **Debugging loops.** Phases 3 and 5 have the most new logic and the widest
  cost range.
* **Build and test output.** CGAL compile errors and long test logs sent back
  to Claude add input tokens. Filtering them helps.
* **Model choice.** Sonnet 5 costs roughly half as much per token, but may need
  more attempts on phases 3 and 5.

Not included: your own review and testing time, CI compute, and calibrating
stair speed factors against measured data.
