#!/usr/bin/env bash
# Copyright Tempo Simulation, LLC. All Rights Reserved

# Answers "will Unreal build this plugin for this project?" without invoking Unreal, mirroring
# UnrealBuildTool's Plugins.IsPluginEnabledForTarget:
#   1. A non-optional entry in the .uproject's "Plugins" array wins outright.
#   2. Otherwise a plugin the .uproject enables may pull this one in by referencing it. Unreal
#      resolves those references recursively; we only follow one level, which covers the case
#      this exists for (a project that enables TempoROSBridge but never names TempoROS).
#   3. Otherwise the plugin's own descriptor decides: "EnabledByDefault": false makes it opt-in,
#      and anything else leaves a project plugin enabled by default.
#
# Usage: IsPluginEnabled.sh <PluginName>
# Exit:  0 enabled, 1 disabled, 2 undetermined (no .uproject above us — e.g. a bare checkout).

set -e

PLUGIN_NAME="$1"
if [ -z "$PLUGIN_NAME" ]; then
  echo "Usage: $(basename "$0") <PluginName>" >&2
  exit 2
fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

if ! which jq &> /dev/null; then
  echo "Couldn't find jq" >&2
  exit 2
fi

PROJECT_ROOT=$("$SCRIPT_DIR"/FindProjectRoot.sh 2>/dev/null) || exit 2
UPROJECT_FILE=$(find "$PROJECT_ROOT" -maxdepth 1 -name "*.uproject" -print -quit)
if [ -z "$UPROJECT_FILE" ]; then
  exit 2
fi

# A plugin reference is enabled unless it says otherwise. Note the results are jq *strings*: a
# bare boolean false would make jq's "//" fall through to the default as if the key were absent.
REFERENCE_VERDICT='first(.Plugins[]?
    | select((.Name // "") == $name)
    | select((.Optional // false) | not)
    | (if (has("Enabled") | not) or .Enabled then "true" else "false" end)) // "unset"'

# Every *.uplugin Unreal would scan for this project. Skips the directories it ignores, and the
# descriptors DisableConflictingPlugins.sh has renamed out of Unreal's sight.
FIND_UPLUGINS() {
  find "$PROJECT_ROOT" \
    \( -name Intermediate -o -name Saved -o -name Binaries -o -name DerivedDataCache -o -name .git \) -prune \
    -o -name "*.uplugin" -print0
}

STRIP_CR() {
  local VALUE="$1"
  echo "${VALUE%$'\r'}"
}

# 1. The .uproject's own verdict, if it has one.
PROJECT_VERDICT=$(STRIP_CR "$(jq -r --arg name "$PLUGIN_NAME" "$REFERENCE_VERDICT" "$UPROJECT_FILE")")
if [ "$PROJECT_VERDICT" = "true" ]; then
  exit 0
elif [ "$PROJECT_VERDICT" = "false" ]; then
  exit 1
fi

# 2. Not named by the project. A plugin the project does enable may still require it.
while IFS= read -r -d '' UPLUGIN_FILE; do
  REFERRER=$(basename "$UPLUGIN_FILE" .uplugin)
  if [ "$REFERRER" = "$PLUGIN_NAME" ]; then
    continue
  fi
  REFERRER_VERDICT=$(STRIP_CR "$(jq -r --arg name "$REFERRER" "$REFERENCE_VERDICT" "$UPROJECT_FILE")")
  if [ "$REFERRER_VERDICT" != "true" ]; then
    continue
  fi
  if [ "$(STRIP_CR "$(jq -r --arg name "$PLUGIN_NAME" "$REFERENCE_VERDICT" "$UPLUGIN_FILE")")" = "true" ]; then
    exit 0
  fi
done < <(FIND_UPLUGINS)

# 3. Nothing references it, so its own descriptor decides.
PLUGIN_DESCRIPTOR=""
while IFS= read -r -d '' UPLUGIN_FILE; do
  if [ "$(basename "$UPLUGIN_FILE" .uplugin)" = "$PLUGIN_NAME" ]; then
    PLUGIN_DESCRIPTOR="$UPLUGIN_FILE"
    break
  fi
done < <(FIND_UPLUGINS)

if [ -z "$PLUGIN_DESCRIPTOR" ]; then
  exit 2
fi

ENABLED_BY_DEFAULT=$(STRIP_CR "$(jq -r 'if has("EnabledByDefault") then (.EnabledByDefault | tostring) else "unset" end' "$PLUGIN_DESCRIPTOR")")
if [ "$ENABLED_BY_DEFAULT" = "false" ]; then
  exit 1
fi

# Unset or true: a plugin in the project's Plugins folder is enabled by default.
exit 0
