#!/bin/bash
# make_executables.sh - sets the execute bit on every .sh in the repo (git clones on some
# systems lose it). Linux / macOS only: Windows has no execute bit.
cd "$(dirname "$0")"
find . -name "*.sh" -not -path "./venv/*" -not -path "./build/*" -exec chmod +x {} \; -print
echo "Permissions set."
