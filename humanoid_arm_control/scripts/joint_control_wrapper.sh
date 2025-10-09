#!/bin/bash
# Wrapper to fix Qt/snap conflicts before launching Python

# Remove snap library paths that conflict with Qt
unset LD_PRELOAD
unset QT_QPA_PLATFORM_PLUGIN_PATH

# Filter out snap paths from LD_LIBRARY_PATH
if [ -n "$LD_LIBRARY_PATH" ]; then
    export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/snap/' | tr '\n' ':' | sed 's/:$//')
fi

# Run the actual Python script
exec python3 "$(dirname "$0")/joint_control.py" "$@"
