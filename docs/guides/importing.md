# Importing Robots

Drag a MuJoCo XML (MJCF) file into the Content Browser and URLab rebuilds the full model as an Unreal Blueprint, ready to simulate.

Grab an `.xml` from the [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) or use your own, then drop it straight into Unreal.

![Dragging an MJCF file into the Unreal Content Browser and the resulting articulation Blueprint](../images/placeholder.svg)

## Import a model

1. Drag your `.xml` file into the Unreal **Content Browser**.
2. The importer reads the original MJCF and imports referenced meshes directly
   through [`UE_Assimp`](https://github.com/PiggyGaGa/UE_Assimp.git). Install the
   plugin from its `feature/robot` branch as described in
   [Installation](../installation.md). No Python environment, Python package,
   `_ue.xml`, or intermediate GLB is required.
3. The factory builds an `AMjArticulation` Blueprint in four passes: assets (meshes, textures, materials), defaults (the class hierarchy), the worldbody (bodies, geoms, joints, sites), and the rest (actuators, sensors, tendons, equalities, keyframes, contact pairs and excludes).
4. Place the Blueprint in your level. It is ready to simulate.

## What the importer reads

The importer honours the MJCF `<compiler>` element, so `angle="degree"` vs `"radian"`, `autolimits`, and `eulerseq` are all interpreted the way MuJoCo would interpret them.

Geoms written with `fromto` are resolved to explicit position, orientation, and size at import time. The component stores the resolved transform; the original `fromto` is not kept.

Cameras are imported with their intrinsics resolved. Where the MJCF specifies `focal`, `focalpixel`, or `sensorsize` instead of a vertical field of view, the importer derives `fovy` from those values so the Unreal camera matches the MuJoCo camera. See [Sensors & Cameras](sensors_cameras.md) for what you can do with imported cameras.

For the full list of MJCF elements URLab understands, see [MJCF Support](../concepts/mjcf_support.md).

## How defaults are preserved

MJCF `<default>` blocks define properties that child elements inherit by class name. URLab imports these as live components, not static notes. When the Blueprint is compiled back into a MuJoCo spec, each component resolves its class against the default chain exactly as raw XML would.

This means:

- Editing a default component changes every component that references that class.
- Nested defaults keep their inheritance chain.
- Removing a default makes its dependents fall back to their own explicit values.

See [Articulations](articulations.md) for editing defaults in the Blueprint editor.

## Materials, textures, and meshes

The importer creates one shared Unreal material instance per MJCF `<material>` name, so every geom that references the same material shares a single instance. Textures from `<texture>` elements are applied to the matching material instance.

Meshes are located relative to the XML file and saved into a `/Meshes/` subfolder under the import target. DAE, STL, OBJ, FBX, GLTF/GLB and the other formats supported by Assimp are read directly. Each Assimp scene is flattened into one persistent Unreal `UStaticMesh`, while preserving node transforms and UVs. If a mesh cannot be loaded, the geom is still created as a collision primitive (without a visual mesh), a warning is logged, and the rest of the import succeeds.

## Debugging an import

If something does not look right after import, enable `bSaveDebugXml` on the Manager actor. After compilation URLab writes `scene_compiled.xml` and `scene_compiled.mjb` to `Saved/URLab/`. Diff the original MJCF against the compiled XML to spot missing elements, wrong values, or broken default inheritance. The compiled XML can also be opened in native MuJoCo for cross-checking.

!!! note
    The XML is only read at import time. After that, the Unreal components are the source of truth. You can rearrange components, add sensors or cameras, override properties, and save the result as a reusable Blueprint.

## Next steps

- [Articulations](articulations.md) covers editing the imported Blueprint and building one from scratch.
- [Geometry & Collision](geometry.md) covers collision shapes and contact filtering.
- [Controllers](controllers.md) covers how control targets become joint torques.
