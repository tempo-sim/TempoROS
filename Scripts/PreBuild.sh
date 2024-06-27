#!/usr/bin/env bash

set -e

# Simply call the individual scripts from the same directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
bash "$SCRIPT_DIR/GenROSIDL.sh" "$@"
bash "$SCRIPT_DIR/GenROSBP.sh" "$3"
