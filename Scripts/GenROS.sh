#!/usr/bin/env bash

set -e

ENGINE_DIR="${1//\\//}"
PROJECT_FILE="${2//\\//}"
PROJECT_ROOT="${3//\\//}"
TARGET_NAME="$4"
TARGET_CONFIG="$5"
TARGET_PLATFORM="$6"
RCLCPP_DIR="${7//\\//}"

if [[ "$OSTYPE" = "msys" ]]; then
  PLATFORM_FOLDER="Windows"
elif [[ "$OSTYPE" = "darwin"* ]]; then
  PLATFORM_FOLDER="Mac"
elif [[ "$OSTYPE" = "linux-gnu"* ]]; then
  PLATFORM_FOLDER="Linux"
fi

if [ -z ${PLATFORM_FOLDER+x} ]; then
  echo "Unrecognized platform."
  exit 1
fi

GENTOOL="$RCLCPP_DIR/Binaries/$PLATFORM_FOLDER/rosidl"
export PYTHONPATH="$RCLCPP_DIR/Libraries/$PLATFORM_FOLDER/python3.9/site-packages"

# Check for jq
if ! which jq &> /dev/null; then
  echo "Couldn't find jq"
  if [[ "$OSTYPE" = "msys" ]]; then
    echo "Install (on Windows): curl -L -o /usr/bin/jq.exe https://github.com/stedolan/jq/releases/latest/download/jq-win64.exe)"
  elif [[ "$OSTYPE" = "darwin"* ]]; then
    echo "Install (on Mac): brew install jq"
  elif [[ "$OSTYPE" = "linux-gnu"* ]]; then
    echo "Install (on Linux): sudo apt-get install jq"
  fi
  exit 1
fi

# The Linux binaries were compiled from the Windows cross-compiler
# and they don't have their permissions set correctly.
if [[ "$OSTYPE" = "linux-gnu"* ]]; then
  chmod +x "$GENTOOL"
fi

# The temp directory where we will store the generated code, instead of just copying it directly to the source
# folder, thereby avoiding copying files that have not changed and triggering unnecessary rebuilds.
TEMP=$(mktemp -d)
GEN_TEMP_DIR="$TEMP/Generated"
mkdir -p "$GEN_TEMP_DIR"

# Function to refresh stale files
REPLACE_IF_STALE () {
  local FRESH="$1"
  local POSSIBLY_STALE="$2"
  
  if [ ! -f "$POSSIBLY_STALE" ]; then
    mkdir -p "$(dirname "$POSSIBLY_STALE")"
    cp -p "$FRESH" "$POSSIBLY_STALE"
  else
    if ! diff --brief "$FRESH" "$POSSIBLY_STALE" > /dev/null 2>&1; then
      cp -f "$FRESH" "$POSSIBLY_STALE"
    fi
  fi
}

GEN_MODULE_MSG_AND_SRVS() {
  local MODULE_PATH="$1"
  local SOURCE_DIR="$MODULE_PATH/Private/"
  local DEST_DIR="$MODULE_PATH/Private/ROSGenerated"
  mkdir -p "$DEST_DIR"
  local MODULE_NAME="$2"
  local PACKAGE_NAME=$(echo "$MODULE_NAME" | tr '[:upper:]' '[:lower:]')
  
  local MODULE_GEN_TEMP_DIR
  MODULE_GEN_TEMP_DIR="$GEN_TEMP_DIR/$MODULE_NAME"
  mkdir -p "$MODULE_GEN_TEMP_DIR/$PACKAGE_NAME"
  if [[ "$OSTYPE" = "msys" ]]; then
    GENTOOL=$(cygpath -m "$GENTOOL")
    MODULE_GEN_TEMP_DIR=$(cygpath -m "$MODULE_GEN_TEMP_DIR")
  fi
  cd "$SOURCE_DIR"
  for FILE in $(find . \( -name "*.msg" -o -name "*.srv" \) -type f); do
    if [[ "$OSTYPE" = "msys" ]]; then
      FILE=$(cygpath -m "$FILE")
    fi
    RELATIVE_PATH="${FILE#./}"
    eval "$GENTOOL" generate --type cpp "$PACKAGE_NAME" "$SOURCE_DIR:$RELATIVE_PATH" -o "$MODULE_GEN_TEMP_DIR/$PACKAGE_NAME" 1> /dev/null
  done

  cd "$MODULE_GEN_TEMP_DIR"
  REFRESHED_FILES=""
  # Replace any stale  files.
  for GENERATED_FILE in $(find . -type f); do    
    # Construct relative path
    RELATIVE_PATH="${GENERATED_FILE#./}"
    local POSSIBLY_STALE_FILE="$DEST_DIR/$RELATIVE_PATH"
    REPLACE_IF_STALE "$GENERATED_FILE" "$POSSIBLY_STALE_FILE"
    REFRESHED_FILES="$REFRESHED_FILES $POSSIBLY_STALE_FILE"
  done
  
  # Remove any generated files that were not refreshed this time.
  for FILE in $(find "$DEST_DIR" -type f); do
    if [[ ! $REFRESHED_FILES =~ $FILE ]]; then
      rm "$FILE"
    fi
  done
}

echo "Generating ROS IDL code..."

# Find dotnet
cd "$ENGINE_DIR"
if [[ "$OSTYPE" = "msys" ]]; then
  DOTNET=$(find ./Binaries/ThirdParty/DotNet -type f -name dotnet.exe)
elif [[ "$OSTYPE" = "darwin"* ]]; then
  DOTNETS=$(find ./Binaries/ThirdParty/DotNet -type f -name dotnet)
  ARCH=$(arch)
  if [[ "$ARCH" = "arm64" ]]; then
    DOTNET=$(echo "${DOTNETS[@]}" | grep -E "mac-arm64/dotnet")
  elif [[ "$ARCH" = "i386" ]]; then
    DOTNET=$(echo "${DOTNETS[@]}" | grep -E "mac-x64/dotnet")
  fi
elif [[ "$OSTYPE" = "linux-gnu"* ]]; then
  DOTNET=$(find ./Binaries/ThirdParty/DotNet -type f -name dotnet)
fi

if [ -z ${DOTNET+x} ]; then
  echo -e "Unable generate ROS IDL code. Couldn't find dotnet.\n"
  exit 1
fi

# Then dump a json describing all module dependencies.
eval "$DOTNET" "./Binaries/DotNET/UnrealBuildTool/UnrealBuildTool.dll" -Mode=JsonExport "$TARGET_NAME" "$TARGET_PLATFORM" "$TARGET_CONFIG" -Project="$PROJECT_FILE" -OutputFile="$TEMP/TempoModules.json" -NoMutex > /dev/null 2>&1

# Iterate over all the modules to ROS IDL code
for BUILD_CS_FILE in $(find "$PROJECT_ROOT" -name '*.Build.cs' -type f); do
  BUILD_CS_FILENAME="$(basename "$BUILD_CS_FILE")"
  MODULE_NAME="${BUILD_CS_FILENAME%%.*}"
  MODULE_PATH="$(dirname "$BUILD_CS_FILE")"
  MODULE_TYPE=$(jq -r --arg TargetName "$TARGET_NAME" --arg ModuleName "$MODULE_NAME" \
    'select(.Name == $TargetName)["Modules"][$ModuleName]["Type"]' < "$TEMP/TempoModules.json")
  if [ "$MODULE_TYPE" != "CPlusPlus" ]; then
    continue
  fi

  GEN_MODULE_MSG_AND_SRVS "$MODULE_PATH" "$MODULE_NAME"
done

rm -rf "$TEMP"

echo "Done"
