#!/usr/bin/env python3
import argparse
import subprocess
import os
import time
import glob


def main():
    parser = argparse.ArgumentParser(
        description="Run pathtracer in build/ and denoise each frame as it becomes available"
    )
    parser.add_argument("gltf", help="Path to the glTF scene file")
    parser.add_argument("oidn", help="Path to the oidnDenoise executable")
    parser.add_argument(
        "-s", "--samples", type=int, default=4, help="-s samples per pixel"
    )
    parser.add_argument(
        "-l", "--lights", type=int, default=2, help="-l number of light samples"
    )
    parser.add_argument(
        "-m", "--max-depth", type=int, default=10, help="-m max bounce depth"
    )
    parser.add_argument(
        "-g", "--gather", type=int, default=400, help="-g number of frames to generate"
    )
    parser.add_argument(
        "-r",
        "--resolution",
        nargs=2,
        type=int,
        default=[1200, 1200],
        metavar=("W", "H"),
        help="-r image width and height",
    )
    parser.add_argument(
        "-f", "--prefix", default="spn.png", help="-f output filename prefix"
    )
    args = parser.parse_args()

    # Derive scene base name from glTF filename (without extension)
    scene_base = os.path.splitext(os.path.basename(args.gltf))[0]

    # Directories under build/
    build_dir = "build"
    denoised_dir = os.path.join(build_dir, "images_denoised")
    os.makedirs(denoised_dir, exist_ok=True)

    # Executable and prefix paths
    pt_exec = os.path.join(build_dir, "pathtracer")
    prefix_path = os.path.join(build_dir, args.prefix)
    prefix_name = os.path.splitext(args.prefix)[0]
    prefix_base = os.path.splitext(prefix_path)[0]

    # Prepare pathtracer command
    cmd = [
        pt_exec,
        "-s",
        str(args.samples),
        "-l",
        str(args.lights),
        "-m",
        str(args.max_depth),
        "-g",
        str(args.gather),
        "-r",
        str(args.resolution[0]),
        str(args.resolution[1]),
        "-d 1",
        "-f",
        prefix_path,
        args.gltf,
    ]

    # Launch pathtracer
    proc = subprocess.Popen(cmd)
    print(
        f"Started pathtracer (PID={proc.pid}), generating up to {args.gather} frames in {build_dir}/..."
    )

    processed = set()

    # Poll and denoise
    while len(processed) < args.gather:
        pattern = f"{prefix_base}*.png"
        for filepath in glob.glob(pattern):
            filename = os.path.basename(filepath)
            name, _ = os.path.splitext(filename)
            # extract frame index: filename is prefix_name + idx
            idx = name.replace(prefix_name, "")
            if idx in processed:
                continue

            # Denoise
            out_name = os.path.join(denoised_dir, f"{prefix_name}_denoised{idx}.png")
            den_cmd = [args.oidn, "--hdr", filepath, "-o", out_name]
            print(f"Denoising {filename} -> {os.path.basename(out_name)}")
            subprocess.run(den_cmd, check=True)
            processed.add(idx)

        # break if pathtracer done and all existing frames processed
        if proc.poll() is not None and len(processed) >= len(glob.glob(pattern)):
            break

        time.sleep(0.1)

    print(f"Denoised {len(processed)} frames.")

    # Generate video from raw frames named after the glTF base
    regular_video = os.path.join(build_dir, f"{scene_base}.mp4")
    ff_cmd_raw = [
        "ffmpeg",
        "-y",
        "-framerate",
        "24",
        "-i",
        os.path.join(build_dir, f"{prefix_name}%04d.png"),
        "-c:v",
        "libx264",
        regular_video,
    ]
    print(f"Generating raw video: {regular_video}")
    subprocess.run(ff_cmd_raw, check=True)

    # Generate video from denoised frames
    denoised_video = os.path.join(build_dir, f"{scene_base}_denoised.mp4")
    ff_cmd_den = [
        "ffmpeg",
        "-y",
        "-framerate",
        "24",
        "-i",
        os.path.join(denoised_dir, f"{prefix_name}_denoised%04d.png"),
        "-c:v",
        "libx264",
        denoised_video,
    ]
    print(f"Generating denoised video: {denoised_video}")
    subprocess.run(ff_cmd_den, check=True)

    print("All videos generated.")


if __name__ == "__main__":
    main()
