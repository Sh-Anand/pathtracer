#!/usr/bin/env python3
"""
Stream sim output locally or over SSH, save to temporary file, then run post-processor.
"""
import argparse, os, shlex, subprocess, sys, signal, tempfile

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--ssh", type=int, default=0, help="Run simulation over SSH")
    p.add_argument("--no-scp", action="store_true", help="Skip SCP transfer when using SSH")
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

    # Handle SCP transfer for SSH mode
    remote_bin_path = args.bin
    if args.ssh:
        if not args.no_scp:
            # Place binary in same directory as simulator on remote host
            remote_sim_dir = os.path.dirname(args.sim)
            local_filename = os.path.basename(args.bin)
            remote_bin_path = f"{remote_sim_dir}/{local_filename}"
            
            if args.debug:
                print(f"DEBUG: Copying {args.bin} to {args.user}@{args.host}:{remote_bin_path}")
            
            # SCP the binary to remote host
            scp_cmd = ["scp", args.bin, f"{args.user}@{args.host}:{remote_bin_path}"]
            try:
                result = subprocess.run(scp_cmd, check=True, capture_output=True, text=True)
                if args.debug:
                    print(f"DEBUG: SCP completed successfully")
            except subprocess.CalledProcessError as e:
                print(f"Error during SCP: {e}", file=sys.stderr)
                if e.stderr:
                    print(f"SCP stderr: {e.stderr}", file=sys.stderr)
                sys.exit(1)
        else:
            # When using --no-scp, assume binary is already in same dir as simulator
            remote_sim_dir = os.path.dirname(args.sim)
            local_filename = os.path.basename(args.bin)
            remote_bin_path = f"{remote_sim_dir}/{local_filename}"
            if args.debug:
                print(f"DEBUG: Using existing remote binary: {remote_bin_path}")

    # Build simulation command
    if args.ssh:
        sim_cmd = [
            "ssh", f"{args.user}@{args.host}",
            f'{shlex.quote(args.sim)} {shlex.quote(remote_bin_path)}'
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

                line_count = 0
                for line in sim.stdout:
                    line_count += 1
                    if args.debug:
                        print(f"DEBUG: SIM: {line.rstrip()}")
                    temp_file.write(line)
                    temp_file.flush()  # Ensure line is written immediately
                    if args.sentinel and args.sentinel in line:
                        if args.debug:
                            print("DEBUG: Complete, terminating sim")
                        break

                if args.debug:
                    print(f"DEBUG: Captured {line_count} lines of simulation output")

                if args.sentinel:
                    sim.terminate()
                try:
                    sim.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    if args.debug:
                        print("DEBUG: Simulation didn't terminate, killing forcefully")
                    sim.kill()
                    sim.wait()

        # Check if temp file has content
        if os.path.getsize(temp_output_file) == 0:
            print("Error: No simulation output captured", file=sys.stderr)
            sys.exit(1)

        if args.debug:
            print(f"DEBUG: Simulation output file size: {os.path.getsize(temp_output_file)} bytes")

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
