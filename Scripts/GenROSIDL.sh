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
SRC_TEMP_DIR="$TEMP/Source"
mkdir "$SRC_TEMP_DIR"
INCLUDES_TEMP_DIR="$TEMP/Includes"
mkdir -p "$INCLUDES_TEMP_DIR"
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

PASCAL_TO_SNAKE() {
    echo "$1" | sed -E 's/([A-Z])([A-Z])([a-z])/\1_\2\3/g' | sed -E 's/([a-z])([A-Z])/\1_\2/g' | tr '[:upper:]' '[:lower:]'
}

SNAKE_TO_PASCAL() {
    local input="$1"
    # Remove __ and anything after
    input="${input%%__*}"
    # Capitalize first letter of each word and remove underscores
    echo "$input" | awk 'BEGIN{FS=OFS="_"} {for(i=1;i<=NF;i++) $i=toupper(substr($i,1,1)) tolower(substr($i,2))} 1' | tr -d '_'
}

GEN_MODULE_MSG_AND_SRVS() {
  local INCLUDE_DIR="$1"
  local MODULE_PATH="$2"
  local PRIVATE_DEST_DIR="$MODULE_PATH/Private"
  local PUBLIC_DEST_DIR="$MODULE_PATH/Public"
  local MODULE_NAME="$3"
  local SOURCE_DIR="$INCLUDE_DIR"/"$MODULE_NAME"
  local PACKAGE_NAME
  # PascalCase module name to snake_case
  PACKAGE_NAME=$(PASCAL_TO_SNAKE "$MODULE_NAME")

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
    GEN_COMMAND="$GENTOOL generate --type cpp --type-support cpp $PACKAGE_NAME $SOURCE_DIR:$RELATIVE_PATH -o $MODULE_GEN_TEMP_DIR/$PACKAGE_NAME"
    for SUBDIR in "$INCLUDE_DIR"/*/ ; do
        if [ -d "$SUBDIR" ]; then
            GEN_COMMAND+=" -I $(realpath "$SUBDIR")"
        fi
    done
    eval "$GEN_COMMAND" 1> /dev/null
  done

  cd "$MODULE_GEN_TEMP_DIR"
  REFRESHED_FILES=""
  # Replace any stale  files.
  for GENERATED_FILE in $(find . -type f); do    
    # Construct relative path
    RELATIVE_PATH="${GENERATED_FILE#./}"
    if [[ "$RELATIVE_PATH" = "$PACKAGE_NAME/cpp/msg"* ]]; then
      local EXTENSION="msg"
    elif [[ "$RELATIVE_PATH" = "$PACKAGE_NAME/cpp/srv"* ]]; then
      local EXTENSION="srv"
    elif [[ "$RELATIVE_PATH" = "$PACKAGE_NAME/cpp/tmp"* ]]; then
      continue
    else
      echo "Unmatched relative path: $RELATIVE_PATH"
      exit 1
    fi
    local FILENAME
    FILENAME=$(basename "$RELATIVE_PATH")
    FILENAME_NO_EXT=${FILENAME%.*}
    FILENAME_NO_EXT=$(SNAKE_TO_PASCAL "$FILENAME_NO_EXT")
    ORIGINAL_FILENAME="$FILENAME_NO_EXT.$EXTENSION"
    MODULE_SRC_TEMP_DIR="$SRC_TEMP_DIR/$MODULE_NAME"
    # Remove /cpp from RELATIVE_PATH
    RELATIVE_PATH=${RELATIVE_PATH//\/cpp/}
    # If the original file was in Public, the generated files go in public
    if find "$MODULE_SRC_TEMP_DIR/Public" -name "$ORIGINAL_FILENAME" | grep -q .; then
      local POSSIBLY_STALE_FILE="$PUBLIC_DEST_DIR/$RELATIVE_PATH"
    else
      local POSSIBLY_STALE_FILE="$PRIVATE_DEST_DIR/$RELATIVE_PATH"
    fi
    
    REPLACE_IF_STALE "$GENERATED_FILE" "$POSSIBLY_STALE_FILE"
    REFRESHED_FILES="$REFRESHED_FILES $POSSIBLY_STALE_FILE"
  done

  # Remove any generated files that were not refreshed this time.
  if [ -d "$PUBLIC_DEST_DIR/$PACKAGE_NAME" ]; then
    for FILE in $(find "$PUBLIC_DEST_DIR/$PACKAGE_NAME" -type f); do
      if [[ ! $REFRESHED_FILES =~ $FILE ]]; then
        rm "$FILE"
      fi
    done
  fi
  
  if [ -d "$PUBLIC_DEST_DIR/$PACKAGE_NAME" ]; then
    find "$PUBLIC_DEST_DIR/$PACKAGE_NAME" -type d -empty -delete
  fi
  
    # Remove any generated files that were not refreshed this time.
    if [ -d "$PRIVATE_DEST_DIR/$PACKAGE_NAME" ]; then
      for FILE in $(find "$PRIVATE_DEST_DIR/$PACKAGE_NAME" -type f); do
        if [[ ! $REFRESHED_FILES =~ $FILE ]]; then
          rm "$FILE"
        fi
      done
    fi
    
    if [ -d "$PRIVATE_DEST_DIR/$PACKAGE_NAME" ]; then
      find "$PRIVATE_DEST_DIR/$PACKAGE_NAME" -type d -empty -delete
    fi
}

echo "Generating ROS IDL code..."

# First iterate through all the modules to:
# - Copy all ROS IDL files to a temp source directory - one per module!
# - Check that no module names are repeated
# - Check that all ROS IDL files are in the correct locations
for BUILD_CS_FILE in $(find "$PROJECT_ROOT" -name '*.Build.cs' -type f); do
  BUILD_CS_FILENAME="$(basename "$BUILD_CS_FILE")"
  MODULE_NAME="${BUILD_CS_FILENAME%%.*}"
  MODULE_PATH="$(dirname "$BUILD_CS_FILE")"
  MODULE_SRC_TEMP_DIR="$SRC_TEMP_DIR/$MODULE_NAME"
  if [ -d "$MODULE_SRC_TEMP_DIR" ]; then
    echo "Multiple modules named $MODULE_NAME found. Please rename one."
    exit 1
  fi
  mkdir -p "$MODULE_SRC_TEMP_DIR/Public"
  mkdir -p "$MODULE_SRC_TEMP_DIR/Private"
  if [ -d "$MODULE_PATH/Public" ]; then
    if [ -d "$MODULE_PATH/Public/msg" ]; then
      if find "$MODULE_PATH/Public/msg" -type f -not -name "*.msg" | grep -q .; then
        echo "Found non-msg file in $MODULE_PATH/Public/msg"
        exit 1
      fi
    fi
    if [ -d "$MODULE_PATH/Public/srv" ]; then
      if find "$MODULE_PATH/Public/srv" -type f -not -name "*.srv" | grep -q .; then
        echo "Found non-srv file in $MODULE_PATH/Public/srv"
        exit 1
      fi
    fi
    for PUBLIC_FILE in $(find "$MODULE_PATH/Public" \( -name "*.msg" -o -name "*.srv" \) -type f); do
      RELATIVE_PATH="${PUBLIC_FILE#$MODULE_PATH}"
      RELATIVE_PATH="${RELATIVE_PATH:1}"
      MODULE_SRC_TEMP_DIR_DEST="$MODULE_SRC_TEMP_DIR/$RELATIVE_PATH"
      mkdir -p "$(dirname $MODULE_SRC_TEMP_DIR_DEST)"
      cp "$PUBLIC_FILE" "$MODULE_SRC_TEMP_DIR/$RELATIVE_PATH"
    done
  fi
  if [ -d "$MODULE_PATH/Private" ]; then
    if [ -d "$MODULE_PATH/Private/msg" ]; then
      if find "$MODULE_PATH/Private/msg" -type f -not -name "*.msg" | grep -q .; then
        echo "Found non-msg file in $MODULE_PATH/Private/msg"
        exit 1
      fi
    fi
    if [ -d "$MODULE_PATH/Private/srv" ]; then
      if find "$MODULE_PATH/Private/srv" -type f -not -name "*.srv" | grep -q .; then
        echo "Found non-srv file in $MODULE_PATH/Private/srv"
        exit 1
      fi
    fi
    for PRIVATE_FILE in $(find "$MODULE_PATH/Private" \( -name "*.msg" -o -name "*.srv" \) -type f); do
      RELATIVE_PATH="${PRIVATE_FILE#$MODULE_PATH}"
      RELATIVE_PATH="${RELATIVE_PATH:1}"
      MODULE_SRC_TEMP_DIR_DEST="$MODULE_SRC_TEMP_DIR/$RELATIVE_PATH"
      mkdir -p "$(dirname "$MODULE_SRC_TEMP_DIR_DEST")"
      cp "$PRIVATE_FILE" "$MODULE_SRC_TEMP_DIR/$RELATIVE_PATH"
    done
  fi
done

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

SYNC_MSGS_SRVS() {
  SRC="$1"
  DEST="$2"
  if [[ "$OSTYPE" = "msys" ]]; then
    # robocopy uses non-zero exit codes even for successful exit conditions.
    set +e
    SRC=$(cygpath -m "$SRC")
    DEST=$(cygpath -m "$DEST")
    robocopy "$SRC" "$DEST" "*.srv" "*.msg" -s > /dev/null 2>&1
    set -e
  else
    rsync -av --include="*/" --include="*.msg" --include="*.srv" --exclude="*" "$SRC" "$DEST" > /dev/null 2>&1
  fi
}

GET_MODULE_INCLUDES_PUBLIC_ONLY() {
  local MODULE_NAME="$1"
  local INCLUDES_DIR="$2"
  PUBLIC_DEPENDENCIES=$(jq -r --arg TargetName "$TARGET_NAME" --arg ModuleName "$MODULE_NAME" \
    'select(.Name == $TargetName)["Modules"][$ModuleName]["PublicDependencyModules"][]' < "$TEMP/TempoModules.json")
  for PUBLIC_DEPENDENCY in $PUBLIC_DEPENDENCIES; do
    PUBLIC_DEPENDENCY=$(echo "$PUBLIC_DEPENDENCY" | sed 's/\[rn]//')
    if [ -d "$SRC_TEMP_DIR/$PUBLIC_DEPENDENCY" ]; then # Only consider project modules
      if [ -d "$INCLUDES_DIR/$PUBLIC_DEPENDENCY" ]; then
        # We already have this dependency - but still add its public dependencies.
        GET_MODULE_INCLUDES_PUBLIC_ONLY "$PUBLIC_DEPENDENCY" "$INCLUDES_DIR"
      else
        # This is a new dependency - add its public ROS IDL files and those of its public dependencies.
        mkdir -p "$INCLUDES_DIR/$PUBLIC_DEPENDENCY"
        SYNC_MSGS_SRVS "$SRC_TEMP_DIR/$PUBLIC_DEPENDENCY/Public/" "$INCLUDES_DIR/$PUBLIC_DEPENDENCY"
        GET_MODULE_INCLUDES_PUBLIC_ONLY "$PUBLIC_DEPENDENCY" "$INCLUDES_DIR"
      fi
    fi
  done
}

GET_MODULE_INCLUDES() {  
  local MODULE_NAME="$1"
  local INCLUDES_DIR="$2"
  # First copy everything from this modules public and private folders.
  mkdir -p "$INCLUDES_DIR/$MODULE_NAME"
  SYNC_MSGS_SRVS "$SRC_TEMP_DIR/$MODULE_NAME/Public/" "$INCLUDES_DIR/$MODULE_NAME"
  SYNC_MSGS_SRVS "$SRC_TEMP_DIR/$MODULE_NAME/Private/" "$INCLUDES_DIR/$MODULE_NAME"
  PUBLIC_DEPENDENCIES=$(jq -r --arg TargetName "$TARGET_NAME" --arg ModuleName "$MODULE_NAME" \
    'select(.Name == $TargetName)["Modules"][$ModuleName]["PublicDependencyModules"][]' < "$TEMP/TempoModules.json")
  PRIVATE_DEPENDENCIES=$(jq -r --arg TargetName "$TARGET_NAME" --arg ModuleName "$MODULE_NAME" \
    'select(.Name == $TargetName)["Modules"][$ModuleName]["PrivateDependencyModules"][]' < "$TEMP/TempoModules.json")
  for PUBLIC_DEPENDENCY in $PUBLIC_DEPENDENCIES; do
    PUBLIC_DEPENDENCY=$(echo "$PUBLIC_DEPENDENCY" | sed 's/\[rn]//')
    if [ -d "$SRC_TEMP_DIR/$PUBLIC_DEPENDENCY" ]; then # Only consider project modules
      if [ -d "$INCLUDES_DIR/$PUBLIC_DEPENDENCY" ]; then
        # We already have this dependency - but still add its public dependencies.
        GET_MODULE_INCLUDES_PUBLIC_ONLY "$PUBLIC_DEPENDENCY" "$INCLUDES"
      else
        # This is a new dependency - add its public ROS IDL files and those of its public dependencies.
        mkdir -p "$INCLUDES_DIR/$PUBLIC_DEPENDENCY"
        SYNC_MSGS_SRVS "$SRC_TEMP_DIR/$PUBLIC_DEPENDENCY/Public/" "$INCLUDES_DIR/$PUBLIC_DEPENDENCY"
        GET_MODULE_INCLUDES_PUBLIC_ONLY "$PUBLIC_DEPENDENCY" "$INCLUDES_DIR"
      fi
    fi
  done

  for PRIVATE_DEPENDENCY in $PRIVATE_DEPENDENCIES; do   
    PRIVATE_DEPENDENCY=$(echo "$PRIVATE_DEPENDENCY" | sed 's/\[rn]//')
    if [[ -d "$SRC_TEMP_DIR/$PRIVATE_DEPENDENCY" ]]; then # Only consider project modules
      if [[ -d "$INCLUDES_DIR/$PRIVATE_DEPENDENCY" ]]; then
        # We already have this dependency - but still add its public dependencies.
        GET_MODULE_INCLUDES_PUBLIC_ONLY "$MODULE_NAME" "$INCLUDES"
      else
        # This is a new dependency - add its public ROS IDL files and those of its public dependencies.
        mkdir -p "$INCLUDES_DIR/$PRIVATE_DEPENDENCY"
        SYNC_MSGS_SRVS "$SRC_TEMP_DIR/$PRIVATE_DEPENDENCY/Public/" "$INCLUDES_DIR/$PRIVATE_DEPENDENCY" > /dev/null 2>&1
        GET_MODULE_INCLUDES_PUBLIC_ONLY "$PRIVATE_DEPENDENCY" "$INCLUDES_DIR"
      fi
    fi
  done
}

# Lastly, iterate over all the modules again to generate the ROS IDL code
for BUILD_CS_FILE in $(find "$PROJECT_ROOT" -name '*.Build.cs' -type f); do
  BUILD_CS_FILENAME="$(basename "$BUILD_CS_FILE")"
  MODULE_NAME="${BUILD_CS_FILENAME%%.*}"
  MODULE_PATH="$(dirname "$BUILD_CS_FILE")"
  MODULE_TYPE=$(jq -r --arg TargetName "$TARGET_NAME" --arg ModuleName "$MODULE_NAME" \
    'select(.Name == $TargetName)["Modules"][$ModuleName]["Type"]' < "$TEMP/TempoModules.json")
  if [ "$MODULE_TYPE" != "CPlusPlus" ]; then
    continue
  fi
  MODULE_INCLUDES_TEMP_DIR="$INCLUDES_TEMP_DIR/$MODULE_NAME"
  GET_MODULE_INCLUDES "$MODULE_NAME" "$MODULE_INCLUDES_TEMP_DIR"
  GEN_MODULE_MSG_AND_SRVS "$MODULE_INCLUDES_TEMP_DIR" "$MODULE_PATH" "$MODULE_NAME"
done

rm -rf "$TEMP"

echo "Done"
