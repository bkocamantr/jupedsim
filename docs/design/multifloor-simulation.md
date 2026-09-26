# Design: Multi-floor (2.5D) simulation in JuPedSim

Status: **Proposal / plan** — no code changes yet.

## 1. Where we are today

Everything in the simulator assumes exactly one planar walkable area:

| Component | File | Single-geometry assumption |
|---|---|---|
| `Simulation` | `libsimulator/src/Simulation.{hpp,cpp}` | Holds one active `_geometry` + `_routingEngine` pair. The `geometries` map only exists to cache geometries for `SwitchGeometry`; only one is active at a time. |
| `CollisionGeometry` | `libsimulator/src/CollisionGeometry.hpp` | One `PolyWithHoles`, one wall-segment grid. |
| `GeometryBuilder` | `libsimulator/src/GeometryBuilder.{hpp,cpp}` | Unions every added polygon into *one* polygon; `Build()` fails when the result is not one connected polygon. |
| `RoutingEngine` | `libsimulator/src/RoutingEngine.{hpp,cpp}` | Navmesh (CGAL CDT) over one polygon; `ComputeWaypoint(Point, Point)`. |
| `NeighborhoodSearch` | `libsimulator/src/NeighborhoodSearch.hpp` | 2D grid over `agent.pos`; agents on different floors at the same (x,y) would interact. |
| `GenericAgent` | `libsimulator/src/GenericAgent.hpp` | Only `Point pos` (x, y). No notion of floor/area. |
| Stages | `StageDescription.hpp`, `Stage.hpp` | Positions are bare `Point`s; waiting sets/queues query the single geometry. |
| Systems | `TacticalDecisionSystem`, `OperationalDecisionSystem`, `StageSystem` | Receive one `RoutingEngine&` / `CollisionGeometry&`. |
| Operational models | `*Model.cpp` | `ComputeNewPosition(dT, agent, geometry, neighborhoodSearch)` — pure 2D, one geometry. |
| C API | `libjupedsim/include/jupedsim/{geometry,simulation,agent,stage}.h` | `JPS_Simulation_Create(model, geometry, dt)`; `JPS_Point` is 2D. |
| Python | `python_bindings_jupedsim/`, `python_modules/jupedsim/` | `Simulation(model, geometry, ...)`, `build_geometry`, `Geometry.boundary()`. |
| Output | `sqlite_serialization.py` | `trajectory_data(frame,id,pos_x,pos_y,ori_x,ori_y)`, one WKT geometry per frame. |
| Visualizer | `python_modules/jupedsim_visualizer` | 2D only. |

## 2. Modelling approach (the key decision)

**Recommended: a "2.5D" model — a graph of planar walkable areas.**

* The building is described as a set of **walkable areas** (`Area`). Each area is a
  planar polygon-with-holes in its own local 2D plane plus an **elevation
  function** `z(x, y)` (constant for floors, linear for stairs/ramps/escalators).
* Areas are connected by **connectors**. A connector either
  * is a shared **portal** edge between two walkable areas (floor ↔ stair, stair ↔ landing,
    floor ↔ escalator), which agents walk across, or
  * is a **discrete transport** (lift) that removes an agent from one area and
    re-inserts it into another after a travel time.
* Agent state gains an `areaId`; `pos` stays 2D (x, y in plan view). The 3D
  position is derived: `(x, y, area.Elevation(x, y))`.

Why not full 3D (`Point3`, 3D navmesh, 3D collision)?

* All operational models (CFSM, CFSM v2, AVM, GCFM, SFM) are formulated in the
  plane. Keeping them 2D per area means **no model code has to be rewritten**.
* Stairs are physically 2D walking surfaces; their slope only matters as a
  speed modifier and for output. The plan-view projection is sufficient.
* CGAL CDT-based routing, the wall grid and the neighbour grid can be reused
  per area unchanged.
* It is backwards compatible: a single-area simulation is exactly today's
  behaviour.

Connector taxonomy:

| Connector | Representation | Behaviour |
|---|---|---|
| Door / opening between areas on the same level | Portal edge | Agents cross freely. |
| Stair | Walkable `Area` with linear `z(x,y)`, `speedFactorUp`, `speedFactorDown` + 2 portals | Desired speed scaled by direction of travel along slope. |
| Ramp | Same as stair, different default factors | as above |
| Escalator | Walkable `Area` + portals, **direction** (one-way), **conveyor velocity** | Only traversable in one direction; conveyor velocity added to agent's motion; optional "stand right / walk left" later. |
| Moving walkway | Escalator with constant `z` | as above |
| Lift | Discrete connector: per served area a **boarding zone** (waiting slots) + capacity, door time, travel time per floor, schedule/call logic | Agents queue, board up to capacity, are removed from source area, re-inserted at the destination's exit slots after travel time. |

## 3. Architecture changes (C++ core, `libsimulator`)

### 3.1 New types

```cpp
// Area.hpp
class Area {
public:
    using ID = jps::UniqueID<Area>;
    ID id;
    std::string name;                                  // "Floor 1", "Stair A" ...
    AreaKind kind;                                     // Floor, Stair, Ramp, Escalator, Walkway
    std::unique_ptr<CollisionGeometry> geometry;
    std::unique_ptr<RoutingEngine> routing;
    NeighborhoodSearch<GenericAgent> neighbors{2.2};
    ElevationFunction elevation;                       // constant or plane z = a*x + b*y + c
    AreaMotionParams motion;                           // speed factors, conveyor vel, direction
    std::vector<Portal::ID> portals;
};

// Portal.hpp : shared edge between two areas
struct Portal {
    using ID = jps::UniqueID<Portal>;
    LineSegment segmentInA, segmentInB;   // identical in plan view for stairs/floors
    Area::ID a, b;
    bool aToB = true, bToA = true;        // escalators are one-way
};

// Lift.hpp : discrete connector
class Lift {
    using ID = jps::UniqueID<Lift>;
    std::map<Area::ID, LiftStop> stops;   // boarding slots + exit slots per area
    size_t capacity; double doorTime; double travelTimePerMeter; ...
    // state machine: Idle -> DoorsOpen -> Boarding -> Moving -> Alighting ...
};

// Building.hpp : owns all of the above, validates connectivity
class Building {
    std::unordered_map<Area::ID, Area> areas;
    std::unordered_map<Portal::ID, Portal> portals;
    std::unordered_map<Lift::ID, Lift> lifts;
    MultiLevelRoutingGraph routingGraph;   // see 3.4
};
```

`BuildingBuilder` replaces the role of `GeometryBuilder` for multi-area input:
`AddArea(polygon, holes, elevation, kind, params)`, `AddPortal(areaA, areaB,
segment)` (or auto-detect portals from coincident boundary edges, with a
tolerance), `AddLift(...)`. It validates: every portal segment lies on the
boundary of both areas, areas on the same elevation don't overlap unless
separated, graph is connected (warn otherwise).

`GeometryBuilder` stays and is used internally per area; a single-area building
is produced implicitly from a plain `CollisionGeometry` (backwards compat).

### 3.2 Agent

```cpp
struct GenericAgent {
    ...
    Area::ID areaId;                 // NEW – area the agent currently stands in
    std::optional<Lift::ID> inLift;  // NEW – agent is being transported, not simulated in any area
    std::vector<RouteLeg> route;     // NEW – cached multi-area route (see 3.4)
};
```

### 3.3 Simulation loop

`Simulation` holds a `Building` instead of `_geometry/_routingEngine`.
`Iterate()` becomes:

1. `AgentRemovalSystem` — unchanged.
2. **Neighbourhood update per area** (each area has its own grid, agents are
   bucketed by `areaId`). Agents inside a lift are in no grid.
3. `StageSystem` — stages now carry an `areaId`; they are updated with their
   own area's geometry/grid.
4. `StrategicalDecisionSystem` — unchanged (journeys/stages are area-agnostic
   IDs; stage positions carry the area).
5. `TacticalDecisionSystem` — uses the multi-level router: if the target is in
   another area, the next waypoint is the best portal/lift on the route
   (see 3.4); within an area it is today's funnel-algorithm waypoint.
6. `OperationalDecisionSystem` — runs the model **per area**, passing the
   area's `CollisionGeometry` and `NeighborhoodSearch`. Then applies area
   motion params (speed factor, conveyor velocity).
7. **NEW `AreaTransitionSystem`** — after positions are applied, detects
   agents whose step crossed a portal segment (segment/segment intersection of
   `oldPos→newPos` with the portal) and moves them to the other area
   (`areaId` update, neighbour grid update). Rejects crossing of one-way
   portals in the wrong direction (treated as a wall).
8. **NEW `LiftSystem`** — advances each lift's state machine, boards agents
   waiting at the boarding slots, removes/re-inserts agents on arrival.
9. Clock advance.

Parallelism opportunity: steps 2–6 are independent per area.

**Interaction across portals.** At a portal, agents on both sides must see
each other and the walls around the opening. Two options:

* (a) *Overlap band*: each area's collision geometry and neighbour query
  includes a strip (≈ neighbour radius) of the adjacent area beyond each
  portal. Neighbour queries near a portal also query the adjacent area's grid.
* (b) Treat portal edges as non-walls and query adjacent area grids when the
  query circle intersects a portal.

Recommend (b): `NeighborhoodSearch` query wrapper `BuildingNeighbors::Query(areaId,
pos, r)` that also looks into areas whose portal segment is within `r` of
`pos`. Plan-view coordinates of stair and floor coincide at the portal, so no
coordinate transform is needed. Wall segments of an area are only its
boundary *minus* portal segments (portals are open edges).

### 3.4 Routing

Two levels, both reusing existing code:

* **Intra-area**: existing `RoutingEngine` per area (navmesh + funnel).
* **Inter-area**: `MultiLevelRoutingGraph` — nodes are portals (midpoint /
  segment) and lift stops; edges:
  * portal ↔ portal within the same area, weight = navmesh path length /
    (area speed factor) — precomputed with `RoutingEngine::ComputeAllWaypoints`;
  * lift stop ↔ lift stop, weight = expected waiting + travel time;
  * escalator edges only in their direction.

  Query `Route(areaFrom, pos, areaTo, target)`: temporarily attach start and
  goal nodes, run Dijkstra/A* (z-aware heuristic), return a list of
  `RouteLeg{areaId, exitPortal | liftId}`. Cache per (agent, target) and
  recompute on stage change or area change.

  The waypoint for the operational model is the navmesh waypoint towards the
  *closest point on the chosen exit portal segment* (not its midpoint, to
  avoid congestion at the centre of wide openings).

Later extensions (not in first milestone): congestion-aware edge weights,
agent preferences (avoid stairs, prefer lifts for mobility-impaired agents),
per-agent routing profiles.

### 3.5 Stages

Each `StageDescription` gets an `Area::ID`:

```cpp
struct WaypointDescription { Area::ID area; Point position; double distance; };
struct ExitDescription     { Area::ID area; Polygon polygon; };
...
```

Validation (`AddStage`, `ValidateGeometry`) checks against that area's
geometry. Stage "reached" checks also compare `agent.areaId`.

### 3.6 Operational models

No change to the model formulas. Changes around them:

* `OperationalModel::ComputeNewPosition` signature is kept; it receives
  the agent's area geometry and a neighbour view that already merges
  cross-portal neighbours.
* New `AreaMotionModifier` applied in `OperationalDecisionSystem`:
  * stairs/ramps: scale `v0` by `speedFactorUp`/`speedFactorDown`, determined
    from `sign(dot(desiredDirection, gradient(z)))`; plan-view speed
    additionally scaled by `cos(slope)` so that walking speed *along* the
    stair surface is correct.
  * escalators/walkways: add conveyor velocity vector to the displacement
    after the model step; agents may be set to "standing" (v0 = 0).
* Model-specific `v0` access is centralised in a small helper (each `*Data`
  struct has its own `v0` field) — needed so modifiers work for all 5 models.

### 3.7 SwitchGeometry

Generalise to `SwitchAreaGeometry(areaId, geometry)` (e.g. closing a fire door
on one floor). The existing single-geometry `SwitchGeometry` maps to the
default area.

## 4. C API (`libjupedsim`)

Additive, ABI-compatible:

```c
typedef uint64_t JPS_AreaId;
typedef uint64_t JPS_PortalId;
typedef uint64_t JPS_LiftId;
typedef struct JPS_Point3 { double x, y, z; } JPS_Point3;   // output only

typedef struct JPS_BuildingBuilder_t* JPS_BuildingBuilder;
JPS_BuildingBuilder JPS_BuildingBuilder_Create();
JPS_AreaId JPS_BuildingBuilder_AddArea(JPS_BuildingBuilder, JPS_Geometry geometry,
                                       JPS_AreaDescription desc /* kind, elevation, params */);
JPS_PortalId JPS_BuildingBuilder_AddPortal(JPS_BuildingBuilder, JPS_AreaId a, JPS_AreaId b,
                                           JPS_Point p1, JPS_Point p2, bool aToB, bool bToA);
void JPS_BuildingBuilder_AutoDetectPortals(JPS_BuildingBuilder, double tolerance);
JPS_LiftId JPS_BuildingBuilder_AddLift(JPS_BuildingBuilder, JPS_LiftDescription desc);
JPS_Building JPS_BuildingBuilder_Build(JPS_BuildingBuilder, JPS_ErrorMessage*);

JPS_Simulation JPS_Simulation_CreateMultiArea(JPS_OperationalModel, JPS_Building, double dt,
                                              JPS_ErrorMessage*);

// stages: *_InArea variants, e.g.
JPS_StageId JPS_Simulation_AddStageWaypointInArea(JPS_Simulation, JPS_AreaId, JPS_Point, double,
                                                  JPS_ErrorMessage*);
// agents: JPS_<Model>AgentParameters gain `JPS_AreaId areaId` (0 = default area)
JPS_AreaId JPS_Agent_GetAreaId(JPS_Agent);
JPS_Point3 JPS_Agent_GetPosition3D(JPS_Agent);
bool JPS_Agent_IsInLift(JPS_Agent);
```

Existing `JPS_Simulation_Create(model, geometry, dt)` keeps working by creating
a one-area building. Parameter structs get a new trailing field defaulting to
the default area; C++/Python callers are updated accordingly.

## 5. Python (`python_bindings_jupedsim`, `python_modules/jupedsim`)

* pybind11: expose `Building`, `BuildingBuilder`, `AreaDescription`,
  `LiftDescription`, area/portal/lift IDs, new agent accessors.
* High-level API:

```python
floor0 = jps.Floor(polygon0, elevation=0.0, name="GF")
floor1 = jps.Floor(polygon1, elevation=3.5, name="1F")
stair  = jps.Stair(stair_polygon, bottom_edge=((10,0),(10,2)), top_edge=((15,0),(15,2)),
                   bottom=floor0, top=floor1)       # derives z(x,y) and the two portals
esc    = jps.Escalator(poly, bottom_edge=..., top_edge=..., bottom=floor0, top=floor1,
                       direction="up", speed=0.5)
lift   = jps.Lift(stops={floor0: boarding_poly0, floor1: boarding_poly1}, capacity=13,
                  speed=1.0, door_time=3.0)
building = jps.Building(areas=[floor0, floor1, stair, esc], lifts=[lift])

sim = jps.Simulation(model=..., building=building, ...)   # `geometry=` still accepted
exit_id = sim.add_exit_stage(exit_poly, area=floor1)
sim.add_agent(jps.CollisionFreeSpeedModelAgentParameters(position=(1, 1), area=floor0, ...))
agent.area_id, agent.position_3d, agent.in_lift
```

* Input helpers: accept a dict `{name: shapely.Polygon}` per floor; optional
  loader for a simple JSON/GeoJSON building description (floors, stairs,
  lifts); later DXF/IFC import as separate tooling.
* `distributions.py`: per-area agent distribution (unchanged math, area
  argument passed through).

## 6. Output & visualisation

* SQLite trajectory format version bump (v2 → v3):
  * `trajectory_data(frame, id, pos_x, pos_y, pos_z, ori_x, ori_y, area_id)`
  * `areas(id, name, kind, elevation_expr, wkt)` table; `geometry` table keyed by
    area; `portals` and `lifts` tables for the visualiser.
  * Reader keeps v1/v2 support. `pedpy` compatibility: allow exporting a
    per-floor 2D trajectory file.
* `jupedsim_visualizer`: floor selector (show one area stack at a time,
  stairs visible on both adjacent floors), optional isometric/3D view later.
* `notebook_utils.py` plotting helpers: `area=` filter.

## 7. Testing

* **Unit (C++, `libsimulator/test`)**: `Area`/`ElevationFunction`,
  `BuildingBuilder` validation (bad portals, disconnected graph, overlapping
  areas), portal crossing detection incl. one-way portals, cross-portal
  neighbour queries, `MultiLevelRoutingGraph` shortest paths, lift state machine.
* **Library tests (`librarytest`)**: C API round-trips, backwards compatibility of
  `JPS_Simulation_Create` with a single geometry.
* **Python tests / systemtests**:
  * Regression: all existing tests unchanged → single-area path is identical.
  * Two floors + one stair: all agents reach an exit on the upper floor;
    no agent ever leaves its area except through a portal.
  * Escalator one-way enforcement; lift capacity and travel-time accounting.
  * RiMEA-style stair test: flow/speed on stairs matches configured factors.
* Performance test: many-area building vs. single large geometry.

## 8. Implementation phases

Each phase is a mergeable PR that keeps all existing tests green.

1. **Refactor to `Area` (no behaviour change).** Introduce `Area` and
   `Building` holding exactly one area; route `Simulation` through it; add
   `areaId` to agents/stages with a default value. Replace
   `_geometry/_routingEngine` members.
2. **Multiple disconnected areas.** Per-area neighbour search, per-area model
   execution, per-area stage validation; C/Python API to create areas and place
   agents/stages in them. (Useful by itself: independent floors in one run.)
3. **Portals + area transitions.** `AreaTransitionSystem`, cross-portal
   neighbour/wall handling, `BuildingBuilder` portal validation / auto-detection.
4. **Multi-level routing.** `MultiLevelRoutingGraph`, route caching, tactical
   system integration.
5. **Stairs and ramps.** Elevation functions, speed modifiers, `Point3` output.
6. **Escalators / moving walkways.** One-way portals, conveyor velocity.
7. **Lifts.** Lift system, state machine, stages for boarding, API.
8. **Output v3 + visualiser + docs/notebooks.** Multi-floor example notebook
   (two floors, stair, escalator, lift), concepts page in `docs/source/concepts`.

## 9. Open questions

* Should stairs be first-class areas (proposed) or a special portal with
  length/time? First-class areas give realistic congestion on stairs; a
  "portal with delay" is cheaper and could be offered as a simplified option.
* Portal auto-detection tolerance and handling of partially overlapping edges.
* Lift dispatch strategy (simple collective control first; pluggable later).
* Whether `JPS_*AgentParameters` structs may be extended in-place (ABI break)
  or need versioned `_V2` variants.
* Agent-level preferences (stairs vs lift, escalator walking) — needs a small
  per-agent "routing profile" struct; defer to after phase 7.
