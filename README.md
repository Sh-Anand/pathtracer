This repository contains a GPU-accelerated path tracer used for rendering glTF and Collada scenes. The host-side application loads geometry and materials with Assimp, builds a CUDA BVH, and launches a custom renderer that supports temporal and spatial resampling via ReSTIR-GI. Output images are saved as OpenEXR files with color, normal, and depth layers to make post-processing and denoising straightforward.

The `master` branch is the reference implementation that targets a desktop NVIDIA GPU. A separate `vortex` branch contains an alternative backend for the Muon/Vortex architecture. See [docs/README_vortex.md](docs/README_vortex.md) for the workflow on that branch.

---

## Quick Start

```bash
# Configure (Release build by default)
cmake -S . -B build

# or enable debug logging / CUDA line info
cmake -S . -B build -DBUILD_DEBUG=ON

# Build the renderer (ninja or make depending on your generator)
cmake --build build -j

# Render a still image to build/output.exr using a bundled scene
./build/pathtracer -l 4 -m 8 -R 0 -o 1 -f build/output.exr gltf/Chess.glb
```

Generated images are written to the path you pass with `-f`. The OpenEXR file stores RGB in `R/G/B`, surface normals in `NX/NY/NZ`, and the linear depth in `Z`.

After rendering a sequence (`-g`), consider running the optional [`batch_denoise.py`](batch_denoise.py) tool together with [Intel Open Image Denoise](https://github.com/OpenImageDenoise/oidn) and `ffmpeg` to denoise and assemble a video.

---

## Repository Layout

| Path | Purpose |
| --- | --- |
| `src/host/` | Command-line application, renderer orchestration, asset loading and EXR output. |
| `src/target/` | CUDA kernels, sampling utilities, and the GPU path tracer implementation. |
| `src/scene/` | Geometry, BVH, light, and material utilities shared by host and device. |
| `src/util/` | Third-party helper libraries (stb, tinygltf, json, RNG, ReSTIR helpers, etc.). |
| `dae/` | Sample Collada scenes kept from the original CS184 starter kit. |
| `gltf/` | Ready-to-render glTF assets used when launching the renderer from the CLI. |
| `bakes/` | (Optional) Pre-baked textures or lighting data; not tracked in Git. |
| `build/` | CMake build tree. Use `cmake -S . -B build` to populate. |

---

## Dependencies

Install the following before building:

- **CUDA Toolkit** (11.8 or newer recommended) with a compatible NVIDIA GPU.
- **CMake** 3.8+ and either Ninja or Make.
- **A C++17 compiler** (GCC 9+, Clang 10+, or MSVC 2019+).
- **Assimp** development package (provides `assimp::assimp`).
- **OpenEXR** development package for EXR output.

Optional but useful:

- **ffmpeg** for assembling PNG/EXR sequences into videos.
- **Intel Open Image Denoise (oidnDenoise)** when running `batch_denoise.py`.
- **Python 3.9+** with a standard library only (for `batch_denoise.py`).

Install the CUDA Toolkit via the NVIDIA packages that match your driver version.

---

## Building

1. **Configure** a build directory:
   ```bash
   cmake -S . -B build [-DBUILD_DEBUG=ON]
   ```
   `BUILD_DEBUG=ON` switches to a Debug configuration, enables CUDA line info, and turns on verbose renderer logging.

2. **Compile**:
   ```bash
   cmake --build build -j
   ```
   The resulting executable is `build/pathtracer` (or `build/Debug/pathtracer` when using the multi-config generators on Windows).

---

## Running the Renderer

Command-line usage:

```text
Usage: pathtracer [options] <scene-file>

  -l  <INT>        Number of samples per area light
  -g  <INT>        Number of frames to generate (1 for a still)
  -R  <INT>        Enable ReSTIR-GI (0 = off, 1 = on)
  -d  <INT>        Enable debug logging (0 = off, 1 = on)
  -m  <INT>        Maximum ray depth
  -o  <INT>        Accumulate indirect bounces (1) or only use the last bounce (0)
  -f  <PATH>       Output EXR filename
  -r  <W> <H>      Resolution when running headless mode
  -h               Show this help message
```

Typical workflows:

- **One-off render**
  ```bash
  ./build/pathtracer -l 8 -m 12 -o 1 -R 1 -f build/chess_restir.exr -r 1920 1080 gltf/ChessCubeSkybox.glb
  ```
  This writes an EXR with color, normals, and depth. Open it in an EXR viewer

- **Turntable animation**
  ```bash
  ./build/pathtracer -l 4 -m 6 -g 120 -f build/chess_turntable.exr -r 1280 720 gltf/Chess.glb
  ```
  When `-g` > 1 the camera performs a full 360° rotation; frames are stored as `<prefix>0000.exr`, `<prefix>0001.exr`, …

- **Debugging**
  ```bash
  ./build/pathtracer -d 1 -o 0 -m 2 -f build/debug_bunny.exr gltf/bunny.glb
  ```
  Debug output prints kernel configuration, BVH build time, and ray statistics.

### Outputs

Each EXR pixel stores:

- `R/G/B`: ACES-tonemapped radiance.
- `NX/NY/NZ`: Shading normal in world space.
- `Z`: Camera-space depth.
---

## Assets & Scenes

- **glTF scenes**: curated test assets under `gltf/`. Use these to validate the CUDA backend quickly.
- **Collada scenes**: legacy CS184 scenes under `dae/`. Helpful for CPU reference renders or regression testing.

```bash
./build/pathtracer -f build/custom.exr ~/assets/my_scene.glb
```
---

# TODO
- PBRT support
- Better rendering algorithms (Area-ReSTIR, etc)
- More complex scenes
- Tight integration with open image denoise