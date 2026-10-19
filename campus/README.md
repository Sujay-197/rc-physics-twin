# Campus

The campus fleet sim's road network ([`Sujay-197/campus_fleet_man_unity`](https://github.com/Sujay-197/campus_fleet_man_unity), scene `Assets/Scenes/SampleScene.unity`, commit `b745220`), recreated for the twin at **1/10 scale**, so the 1/10 car trains on the area the fleet sim's buses drive.

## Files

| File | What it is |
|---|---|
| `campus_export.json` | Raw export from the campus scene, Unity metres and frame: the fleet sim's `RoadGraph` (nodes and edges with centreline polylines), its 10 `BusStop`s with demand weights, every road tile with its surfaces' measured extents, the building footprints. Numbers only: no meshes, textures or other assets. |
| `campus_network.json` | Made from the export by `tools/campus_import.py`: the same network at 1/10 in the ISO 8855 frame (x = Unity z, y = -Unity x, z = Unity y), with the measured road dimensions. |
| `../roads/campus/edge_NN.json` | One `roads/road.schema.json` file per edge, for `tools/drive_road.py` and `tools/edge_profiler.py`. |

## The numbers

- 36 nodes, 37 edges, 132.7 m of road (1,327 m in the fleet sim), 10 stops (S3 is the hub with weight 6, AB4 has 4, the rest 1), 4 apartment blocks.
- **Drivable width 1.409 m**: the asphalt of the road tiles (14.09 m), measured from their meshes. A 0.235 m sidewalk runs along each side, raised 0.6 m in the fleet sim and **6 cm here**: more than the car's 5 cm wheel radius, so they are kerbs, not road. Leaving the asphalt ends an episode.
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

`CampusEnv` draws each episode's trip by the stops' demand weights (as the fleet sim's `DemandAgent` does) and routes it with Dijkstra by edge length (as its `GraphRouter` does), then runs the `RcTwinEnv` task on it: same actions, observations, domain randomisation and reward. The observation-only pure-pursuit baseline drives every trip tried, including the longest (69 m), 90-degree intersection corners included.

## Re-exporting

When the fleet sim's map changes:

1. Copy `tools/campus_exporter/TwinCampusExporter.cs` into the campus project's `Assets/Editor/`. It lives outside any Unity project here, so nothing compiles it.
2. `Unity -batchmode -quit -projectPath <campus> -executeMethod TwinCampusExporter.Export -exportPath campus/campus_export.json`
3. Delete the exporter again and revert any settings Unity rewrote (`git checkout -- ProjectSettings`), so the campus repo is unchanged.
4. Update `SOURCE["commit"]` in `tools/campus_import.py`, then `python tools/campus_import.py`.

The tests (`sim_py/test_campus.py`, ctest `sim_py.campus`) check that the committed data is exactly what the importer makes from the export.
