# Campus

The campus fleet sim's road network ([`Sujay-197/campus_fleet_man_unity`](https://github.com/Sujay-197/campus_fleet_man_unity), scene `Assets/Scenes/SampleScene.unity`, commit `b745220`), recreated for the twin at **1/10 scale**, so the 1/10 car trains on the area the fleet sim's buses drive.

## Files

| File | What it is |
|---|---|
| `campus_export.json` | Raw export from the campus scene, Unity metres and frame: the fleet sim's `RoadGraph` (nodes and edges with centreline polylines), its 10 `BusStop`s with demand weights, every road tile with its surfaces' measured extents, the building footprints. Numbers only: no meshes, textures or other assets. |
| `campus_network.json` | Made from the export by `tools/campus_import.py`: the same network at 1/10 in the ISO 8855 frame (x = Unity z, y = -Unity x, z = Unity y), with the measured road dimensions. |
| `../roads/campus/edge_NN.json` | One `roads/road.schema.json` file per edge, for `tools/drive_road.py` and `tools/edge_profiler.py`. |
| `../profiles/campus_edges.json` | The edge profile cache for the fleet sim (`profiles/edge_profile.schema.json`): every edge at 1, 2 and 3 m/s, 0 and 1 kg payload, 80 % and 40 % charge, driven from node `a` to node `b` by the reference driver on the provisional car. Times and energies are for the 1/10 car on the 1/10 campus. |

## The numbers

- 36 nodes, 37 edges, 132.7 m of road (1,327 m in the fleet sim), 10 stops (S3 is the hub with weight 6, AB4 has 4, the rest 1), 4 apartment blocks.
- **Drivable width 1.409 m**: the asphalt of the road tiles (14.09 m), measured from their meshes. A 0.235 m sidewalk runs along each side, raised 0.6 m in the fleet sim and **6 cm here**: more than the car's 5 cm wheel radius, so they are kerbs, not road. The kerb lines and the buildings' footprints are walls (ADR 0009) that the car bounces off, in Unity and in the RL environment alike. `campus_network.json` carries them (`walls`) and the sidewalk runs they edge (`sidewalks`): roads are joined into chains between junctions so kerbs run on round bends, and a kerb stops where it would reach another road's asphalt, so no wall cuts into a road (tested).
- **Why 1/10:** the campus is modelled at full scale (its city pack is sized for real cars), and the twin is a 1/10 car. A 3.5 m lane becomes 0.35 m against the car's 0.24 m track.

## Using it

```python
import importlib.util
spec = importlib.util.spec_from_file_location("campus", "sim_py/campus.py")
campus = importlib.util.module_from_spec(spec); spec.loader.exec_module(campus)

net = campus.CampusNetwork.load()
road = net.route_road("S3", "AB4")         # a road file dict: drive_road.py, edge_profiler.py
env = campus.CampusEnv(seed=0)             # RL: a stop-to-stop trip per episode
obs, info = env.reset(seed=0)              # info["trip"] == (origin, destination)
```

`CampusEnv` draws each episode's trip by the stops' demand weights (as the fleet sim's `DemandAgent` does) and routes it with Dijkstra by edge length (as its `GraphRouter` does), then runs the `RcTwinEnv` task on it: same actions, observations, domain randomisation and reward, plus the campus's walls. A kerb strike bounces the car and costs the reward's `wall` term; leaving the route's road (into a side street at a junction) ends the episode. `CampusEnv(walls=False)` drops the walls. The observation-only pure-pursuit baseline drives every trip tried, including the longest (69 m), 90-degree intersection corners included.

## Re-exporting

When the fleet sim's map changes:

1. Copy `tools/campus_exporter/TwinCampusExporter.cs` into the campus project's `Assets/Editor/`. It lives outside any Unity project here, so nothing compiles it.
2. `Unity -batchmode -quit -projectPath <campus> -executeMethod TwinCampusExporter.Export -exportPath campus/campus_export.json`
3. Delete the exporter again and revert any settings Unity rewrote (`git checkout -- ProjectSettings`), so the campus repo is unchanged.
4. Update `SOURCE["commit"]` in `tools/campus_import.py`, then `python tools/campus_import.py`.

The tests (`sim_py/test_campus.py`, ctest `sim_py.campus`) check that the committed data is exactly what the importer makes from the export. They also rerun the profiler and require every cached entry to be reused; when the car, the driver or the roads change, regenerate the cache:

```bash
python tools/edge_profiler.py roads/campus/edge_*.json --out profiles/campus_edges.json --speeds 1 2 3 --payloads 0 1 --soc 0.8 0.4
```

## The real campus around the roads

The fleet sim's map is a simplified version of the real campus, Amrita Vishwa Vidyapeetham, Coimbatore. It is drawn at about 0.57 of real size, its roads are mostly within 16 m of the real ones, and a few of its roads do not exist (S1 to S2 and S5 to S4 run across the playground). Those roads stay as they are: the fleet sim, the RL environment and the policy study all drive them. `tools/campus_scenery.py` fits the real campus around them, drawn only:

1. **Fit.** A similarity takes OpenStreetMap to the fleet sim frame: it starts from the three academic blocks both maps name, then is refined on the roads. It finds the fleet sim at 1/1.75 of real size, turned 112 degrees.
2. **Warp.** A thin-plate-spline warp then carries each real road onto the fleet sim road it stands for: about 110 of 205 road points are matched, near and parallel. Every building, lawn and path moves with the warp.
3. **Cull.** Anything that lands on a road or its kerbs is handled:
   - a building moves clear, up to 2.5 m, shrinking toward its centre (down to 0.55) when moving alone is not enough;
   - real roads that duplicate a fleet sim road are dropped;
   - side roads are clipped at the kerb.

   Academic Blocks 1 and 2 stand where the fleet sim's AB1 and AB2 boxes are. The boxes stop being drawn, but their walls stay.

`campus/scenery.json` holds the result in the twin's frame: buildings, land use, and side roads and paths. It comes from the OpenStreetMap extract `campus/osm/amrita.osm.json`, under the ODbL (`THIRD_PARTY.md`).

**Trees and ground cover** come from map screenshots in `campus/reference/`, kept out of git. Each screenshot is pinned to the real map: a similarity from tie points on named buildings (`campus/reference/images.json`), refined by fitting the screenshot's road overlay to the real roads (to within 4 to 7 m). Each pixel is then sorted into grass, bare earth, tree cover or built. The result is about 5,000 trees and the ground texture in `unity/RcTwin/Assets/RcTwin/Generated/Campus/Local/`, also kept out of git. Without the screenshots, the campus keeps its plain grass and has no trees.

**RcTwin > Build Campus Scene** adds it all: extruded buildings, the ground texture laid over the grass, and low-poly trees. Nothing new has a collider, so physics and training are unchanged. The replay scene is built from the campus scene, so it shows the same scenery.

```bash
python tools/campus_scenery.py fetch
```

```bash
python tools/campus_scenery.py build --images campus/reference/images.json
```

The tool needs numpy and Pillow. `tools/test_campus_scenery.py` (ctest `tools.campus_scenery`) checks that no building, and no side road, comes within the kerb clearance of a fleet sim road, and that no two buildings overlap.

