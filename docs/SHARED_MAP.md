# One navigation map and observed obstacle recovery

The **native C `goto` process** now owns occupancy, robot-clearance planning, and the active route. Its LiDAR comes from the existing native guard's validated LDR1 forwarding on 5572. A* reads that grid directly. The phone dashboard, browser floor map, and mission control display the same authority's data.

`NAV SNAPSHOT` on loopback TCP 5570 returns a bounded JSON line containing the raw 200×200 occupancy grid, 0.1 m resolution, origin, pose, floor, active route and waypoint index, process instance, map epoch/version and sensor freshness. All fields are serialized in the native event loop, so a UI cannot combine a route from one moment with a grid from another. The grid is raw occupancy; A* also applies the existing 0.35 m robot radius and treats unknown space as blocked.

The dashboard's `native_map.py` only validates/transports snapshots. `SceneFeed` always enables it. It does not classify occupancy or run a second planner. If native navigation is disconnected or its response is malformed, the floor view becomes unknown and has no route. If LiDAR/tracking becomes stale, the last authoritative grid is labeled stale and its route is hidden. There is no fallback to the old Python grid. The Python mapper/planner remains available to offline parity tests; it is not used for runtime floor classification or route previews. Object labels, colored 3D surfaces and inferred-depth visualization remain separate visual information and cannot overwrite native occupancy.

## Recovery rule

- An observed obstacle blocks its cell immediately.
- A blocked cell becomes free after **five distinct floor-only frames over at least 600 ms of receiver time**, with no more than 500 ms between confirming observations.
- Each complete frame contributes at most one vote per cell. An obstacle and floor point in the same cell count as an obstacle, independent of point order.
- A new obstacle, tracking/floor validity loss, or a long observation gap cancels pending recovery. Source timestamps must increase; duplicate/out-of-order packets cannot update pose, clearance or freshness. A burst of buffered frames cannot satisfy the minimum receiver-time span.
- Silence, unknown depth, points outside the floor band and simply waiting do not turn blocked space into free space. The robot must actually reobserve the floor where the person/obstacle was.

This repairs transient occupancy; it does not correct a persistently wrong floor calibration. The height bands remain ±6 cm for floor and 8 cm–1.2 m for obstacles. It is still a 2D projection, not proof that all volume above a floor point is clear. The independent real-time LiDAR motor gate is unchanged and remains the final stop authority.

## Lifecycle and integration

`NAV SESSION <64 lowercase hex characters>` identifies an AR world by its SHA-256 session token. Changing it cancels active navigation and clears the native occupancy and object-memory coordinates. Repeating the same token does nothing. `NAV RESET` performs the same reset for an explicit dashboard map-clear request, retaining the session token. The dashboard will not replay an old snapshot after a session change/reset. Restoring a saved map restores visual memory, but does not inject old saved occupancy into the live planner; the native grid needs fresh observations.

The manual-drive supervisor starts the native map with validated LiDAR forwarding and a disconnected navigation motor endpoint by default. Explicit TARS_ENABLE_GOTO=1 connects autonomous navigation and requires CLEAR. The supervised QNX listener uses the process-local loopback_bind.so so the tested executable serves only loopback clients. `TARS_ENABLE_GOTO=0` is still an explicit opt-out, with the floor map then unavailable. The unified startup profile already enables GOTO and strict LiDAR clearance. Deploy/rebuild `goto`, and deploy `mapping.py`, `native_map.py`, and `service.py` together; this is a Pi-side change. The existing iPhone app consumes the same `grid`/`path` fields and needs no rebuild for this change. The optional browser room view additionally labels the floor panel “A* map” or “A* map stale.”

Work is isolated on `codex/shared-nav-grid`, based on the previous unified build. No changes have been made to the main checkout or its live robot services.

## Validation

Run from the repo root:

```sh
make -C iter2-autonomy/goto host test-nav-asan test-nav-parity
python3 -m unittest discover -s iter2-autonomy/dashboard -p 'test_*.py'
make -C iter2-autonomy/goto test-fullstack
```

Native recovery tests cover a blocked corridor reopening for A*, repeated packets, buffered bursts, intermittent observations, tracking loss and a persistent obstacle mixed with floor points. Native process/dashboard integration tests compare all 40,000 displayed cells and the version/pose/route against `NAV SNAPSHOT`, exercise live recovery, and verify unavailable/reset/session behavior. These are synthetic tests, not a physical walking-person experiment.

### Completed validation (2026-11-21)

- Mac: native nav 117/117 and one million fuzz packets under ASan/UBSan; offline parity 212/212; native core 265/265; YOLO postprocessing 25/25; dashboard 86/86; navigation/socket simulations 60/60; full-stack simulations 33/33.
- Pi/QNX: native build passed; native nav 117/117 plus 200,000 fuzz packets; all six native/dashboard integration tests passed. Tests used synthetic sensor packets and an isolated motor endpoint, never the live motor service.
- The integration suite includes a nonreading viewer that floods snapshot requests while a separate client sends CANCEL. Explicit nonblocking snapshot writes keep control responsive; incomplete snapshots are discarded by the reader.

Validated implementation commit: `0442018`. QNX staging directory: `/data/home/qnxuser/projects/shared-map-c2b388e` (updated with the final implementation). No physical moving-person test or autonomous driving was performed.
