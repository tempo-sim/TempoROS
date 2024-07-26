#!/usr/bin/env bash

# This script should be run once, after cloning TempoROS.
# It will keep the third party libraries in sync with the rest of TempoROS.

set -e

TEMPO_ROOT=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

if [ -z "$GIT_DIR" ]; then
	GIT_DIR=$(git rev-parse --git-common-dir);
	if [ $? -ne 0 ]; then
		GIT_DIR=.git
	fi
fi

ADD_COMMAND_TO_HOOK() {
  COMMAND=$1
  HOOK=$2
  HOOK_FILE="$GIT_DIR/hooks/$HOOK"

  if [ ! -f "$HOOK_FILE" ]; then
    touch "$HOOK_FILE"
    chmod +x "$HOOK_FILE"
  fi

  if ! grep -qF "$COMMAND" "$HOOK_FILE"; then
    echo "$COMMAND" >> "$HOOK_FILE"
  fi
}

SYNCDEPS="$TEMPO_ROOT/Scripts/SyncDeps.sh"

# Put SyncDeps.sh script in appropriate git hooks
if [ -d "$GIT_DIR/hooks" ]; then
  ADD_COMMAND_TO_HOOK "$SYNCDEPS" post-checkout
  ADD_COMMAND_TO_HOOK "$SYNCDEPS" post-merge
fi

# Run the steps once (adding -force if specified)
echo -e "Checking ThirdParty dependencies...\n"
if [ "$1" = "-force" ]; then
  SYNCDEPS="$SYNCDEPS -force"
fi
eval "$SYNCDEPS"
