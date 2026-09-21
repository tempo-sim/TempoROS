#!/usr/bin/env bash

# This script should be run once, after cloning TempoROS.
# It enables the TempoROS plugin in your project and keeps the third party libraries in sync with
# the rest of TempoROS.
#
# Flags:
#   -force       Re-download third party dependencies even if they are already up to date.
#   -if-enabled  Do nothing unless TempoROS is already enabled, and never enable it. Tempo's
#                Setup.sh passes this so that setting up Tempo does not opt a project into ROS.

set -e

TEMPO_ROOT=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

FORCE=0
IF_ENABLED=0
for ARG in "$@"; do
  case "$ARG" in
    -force)
      FORCE=1
      ;;
    -if-enabled)
      IF_ENABLED=1
      ;;
  esac
done

IS_PLUGIN_ENABLED="$TEMPO_ROOT/Scripts/IsPluginEnabled.sh"

# 0 enabled, 1 disabled, 2 undetermined (no .uproject above us, or no jq).
PLUGIN_STATE=0
"$IS_PLUGIN_ENABLED" TempoROS || PLUGIN_STATE=$?

if [ "$IF_ENABLED" -eq 1 ]; then
  if [ "$PLUGIN_STATE" -eq 1 ]; then
    echo "Skipping TempoROS setup because the TempoROS plugin is not enabled in this project."
    echo "To use ROS, run $TEMPO_ROOT/Setup.sh"
    exit 0
  fi
elif [ "$PLUGIN_STATE" -eq 1 ]; then
  # TempoROS is opt-in ("EnabledByDefault": false in TempoROS.uplugin), and running this script is
  # the opt-in. Say so in the .uproject rather than asking the user to hand-edit it.
  PROJECT_ROOT=$("$TEMPO_ROOT/Scripts/FindProjectRoot.sh")
  UPROJECT_FILE=$(find "$PROJECT_ROOT" -maxdepth 1 -name "*.uproject" -print -quit)
  TEMP_UPROJECT=$(mktemp)
  # Unreal writes .uproject files with tabs, so --tab keeps the diff to the lines we changed.
  jq --tab --arg name "TempoROS" '
    .Plugins = (.Plugins // [])
    | if any(.Plugins[]; (.Name // "") == $name)
      then .Plugins = [.Plugins[] | if (.Name // "") == $name then .Enabled = true else . end]
      else .Plugins += [{"Name": $name, "Enabled": true}]
      end' "$UPROJECT_FILE" > "$TEMP_UPROJECT"
  mv "$TEMP_UPROJECT" "$UPROJECT_FILE"
  echo -e "Enabled the TempoROS plugin in $(basename "$UPROJECT_FILE")\n"

  # TempoROSBridge (from Tempo) adapts Tempo's own API onto ROS. TempoROS does not depend on it
  # and will not enable it, but a project that has both almost certainly wants both.
  BRIDGE_DESCRIPTOR=$(find "$PROJECT_ROOT" \
    \( -name Intermediate -o -name Saved -o -name Binaries -o -name .git \) -prune \
    -o -name "TempoROSBridge.uplugin" -print -quit)
  if [ -n "$BRIDGE_DESCRIPTOR" ]; then
    BRIDGE_STATE=0
    "$IS_PLUGIN_ENABLED" TempoROSBridge || BRIDGE_STATE=$?
    if [ "$BRIDGE_STATE" -eq 1 ]; then
      echo -e "This project also has TempoROSBridge, which exposes Tempo's API over ROS. It is opt-in too."
      echo -e "To use it, add it to the \"Plugins\" array in $(basename "$UPROJECT_FILE"):\n"
      echo -e "\t{ \"Name\": \"TempoROSBridge\", \"Enabled\": true }\n"
    fi
  fi
fi

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
    # A hook needs a shebang to run on its own. Prompting from it needs the
    # terminal on stdin, but opening it must not fail the hook (and so the
    # checkout) where there is no controlling terminal, e.g. CI.
    echo -e "#!/usr/bin/env bash\n" > "$HOOK_FILE"
    echo 'if { : < /dev/tty; } 2>/dev/null; then exec < /dev/tty; fi' >> "$HOOK_FILE"
    chmod +x "$HOOK_FILE"
  fi

  if ! grep -qF "$COMMAND" "$HOOK_FILE"; then
    echo "$COMMAND" >> "$HOOK_FILE"
  fi
}

SYNCDEPS="$TEMPO_ROOT/Scripts/SyncDeps.sh"

# Put SyncDeps.sh script in appropriate git hooks
if [ -d "$GIT_DIR/hooks" ]; then
  ADD_COMMAND_TO_HOOK "\"$SYNCDEPS\"" post-checkout
  ADD_COMMAND_TO_HOOK "\"$SYNCDEPS\"" post-merge
fi

# Run the steps once (adding -force if specified)
echo -e "Checking ThirdParty dependencies...\n"
SYNCDEPS_ARGS=()
if [ "$FORCE" -eq 1 ]; then
  SYNCDEPS_ARGS+=("-force")
fi
"$SYNCDEPS" "${SYNCDEPS_ARGS[@]}"
