#!/bin/bash
# Cyclotron simulator wrapper script for run_sim.py
# Usage: cyclotron_wrapper.sh "<cyclotron_dir> <config_file>"

ARGS="$1"
if [ -z "$ARGS" ]; then
    echo "Usage: $0 \"<cyclotron_dir> <config_file>\"" >&2
    exit 1
fi

# Parse the arguments (cyclotron_dir and config_file are space-separated in the single argument)
CYCLOTRON_DIR=$(echo "$ARGS" | cut -d' ' -f1)
CONFIG_FILE=$(echo "$ARGS" | cut -d' ' -f2-)

if [ -z "$CYCLOTRON_DIR" ] || [ -z "$CONFIG_FILE" ]; then
    echo "Usage: $0 \"<cyclotron_dir> <config_file>\"" >&2
    echo "Got: cyclotron_dir='$CYCLOTRON_DIR', config_file='$CONFIG_FILE'" >&2
    exit 1
fi

# Run cyclotron with the provided config
exec cargo run --manifest-path "$CYCLOTRON_DIR/Cargo.toml" "$CONFIG_FILE"
