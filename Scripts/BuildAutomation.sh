#!/usr/bin/env bash

# Builds the TempoROS.Automation.csproj project, which provides a custom copy handler
# for packaging that properly handles symbolic links in ROS third party dependencies.

set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

UNREAL_ENGINE_PATH=$("$SCRIPT_DIR"/FindUnreal.sh)
UNREAL_ENGINE_PATH="${UNREAL_ENGINE_PATH//\\//}"

# Get engine release (e.g. 5.4)
if [ -f "$UNREAL_ENGINE_PATH/Engine/Intermediate/Build/BuildRules/UE5RulesManifest.json" ]; then
  RELEASE_WITH_HOTFIX=$(jq -r '.EngineVersion' "$UNREAL_ENGINE_PATH/Engine/Intermediate/Build/BuildRules/UE5RulesManifest.json")
  RELEASE="${RELEASE_WITH_HOTFIX%.*}"
fi

# Find dotnet
if [[ "$OSTYPE" = "msys" ]]; then
  DOTNET=$(find "$UNREAL_ENGINE_PATH/Engine/Binaries/ThirdParty/DotNet" -type f -name dotnet.exe)
elif [[ "$OSTYPE" = "darwin"* ]]; then
  DOTNETS=$(find "$UNREAL_ENGINE_PATH/Engine/Binaries/ThirdParty/DotNet" -type f -name dotnet)
  ARCH=$(uname -m)
  if [[ "$ARCH" = "arm64" ]]; then
    DOTNET=$(echo "${DOTNETS[@]}" | grep -E "mac-arm64/dotnet")
  elif [[ "$ARCH" = "i386" ]]; then
    DOTNET=$(echo "${DOTNETS[@]}" | grep -E "mac-x64/dotnet")
  fi
elif [[ "$OSTYPE" = "linux-gnu"* ]]; then
  DOTNETS=$(find "$UNREAL_ENGINE_PATH/Engine/Binaries/ThirdParty/DotNet" -type f -name dotnet)
  if [[ "$RELEASE" == "5.4" ]]; then
    # In UE 5.4 there is only one dotnet on Linux. 5.5 added arm64 support.
    DOTNET="$DOTNETS"
  else
    ARCH=$(uname -m)
    if [[ "$ARCH" = "arm64" ]]; then
      DOTNET=$(echo "${DOTNETS[@]}" | grep -E "linux-arm64/dotnet")
    elif [[ "$ARCH" = "x86_64" ]]; then
      DOTNET=$(echo "${DOTNETS[@]}" | grep -E "linux-x64/dotnet")
    fi
  fi
fi

if [ -z ${DOTNET+x} ]; then
  echo -e "Unable to build TempoROS automation. Couldn't find dotnet.\n"
  exit 1
fi

"$DOTNET" build "$SCRIPT_DIR/TempoROS.Automation.csproj" -p:UNREAL_ENGINE_PATH="$UNREAL_ENGINE_PATH"
