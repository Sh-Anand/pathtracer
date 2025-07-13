#!/usr/bin/env python3
"""
Stream sim output locally or over SSH, save to temporary file, then run post-processor.
"""
import argparse, os, shlex, subprocess, sys, signal, tempfile

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--ssh", type=int, default=0, help="Run simulation over SSH")
    p.add_argument("--user", help="SSH username (required when --ssh is used)")
    p.add_argument("--host", help="SSH hostname (required when --ssh is used)")
    p.add_argument("--sim",  required=True)
    p.add_argument("--bin",  required=True)
    p.add_argument("--build_folder", required=True, help="Folder to save temporary simulation output file")
    p.add_argument("--post", required=True)
    p.add_argument("--sentinel", required=True, help="Sentinel string to detect end of simulation output")
    p.add_argument("--skip", default="", help="Skip substring for post process")
    p.add_argument("--debug" , type=int, default=0)
    p.add_argument("--outfile", default="render.pfm")
    args = p.parse_args()

    # Validate SSH-related arguments
    if args.ssh:
        if not args.user or not args.host:
            p.error("--user and --host are required when --ssh is specified")

    # Validate build folder exists
    if not os.path.exists(args.build_folder):
        p.error(f"Build folder does not exist: {args.build_folder}")

    # Build simulation command
    if args.ssh:
        sim_cmd = [
            "ssh", f"{args.user}@{args.host}",
            f'{shlex.quote(args.sim)} {shlex.quote(args.bin)}'
        ]
    else:
        sim_cmd = [args.sim, args.bin]

    # Create temporary file in build folder for simulation output
    temp_output_file = os.path.join(args.build_folder, "sim_stdout.log")

    if args.debug:
        print(f"DEBUG: SIMCMD: {' '.join(sim_cmd)}")
        print(f"DEBUG: Saving simulation output to: {temp_output_file}")

    # Run simulation and save output to temporary file
    try:
        with open(temp_output_file, 'w') as temp_file:
            with subprocess.Popen(
                    sim_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1
                 ) as sim:

                for line in sim.stdout:
                    if args.debug:
                        print(f"DEBUG: SIM: {line}")
                    temp_file.write(line)
                    if args.sentinel and args.sentinel in line:
                        if args.debug:
                            print("DEBUG: Complete, terminating sim")
                        break

                if args.sentinel:
                    sim.terminate()
                try:
                    sim.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    if args.debug:
                        print("DEBUG: Simulation didn't terminate, killing forcefully")
                    sim.kill()
                    sim.wait()

        # Now run post-processor on the temporary file
        post_cmd = [sys.executable, args.post, temp_output_file, args.outfile]
        if args.skip:
            post_cmd.append(args.skip)

        if args.debug:
            print(f"DEBUG: {' '.join(post_cmd)}")

        result = subprocess.run(post_cmd)
        sys.exit(result.returncode)

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
