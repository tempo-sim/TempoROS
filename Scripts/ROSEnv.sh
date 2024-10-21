# Copyright Tempo Simulation, LLC. All Rights Reserved

# Source this file to set environment variables to use the ros2 CLI that comes with TempoROS
# (e.g. ros2 topic list, ros2 topic echo, etc)

SHELL=$(ps -p $$ | awk '$1 != "PID" {print $(NF)}')

if [[ "$SHELL" =~ "zsh" ]]; then
  # In zsh BASH_SOURCE is the directory where the script is called from, not the sourced file
  TEMPOROS_ROOT=${0:a:h}/..
else
  TEMPOROS_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"/..
fi

TEMPOROS_ROOT=$(readlink -f "$TEMPOROS_ROOT")

FIND_UPROJECT() {
    local START_DIR
    START_DIR=$(dirname "$1")
    local CURRENT_DIR="$START_DIR"
    local UPROJECT_FILE

    while [[ "$CURRENT_DIR" != "/" ]]; do
        UPROJECT_FILE=$(find "$CURRENT_DIR" -maxdepth 1 -name "*.uproject" -print -quit)
        if [[ -n "$UPROJECT_FILE" ]]; then
            echo "$UPROJECT_FILE"
            return 0
        fi
        CURRENT_DIR=$(dirname "$CURRENT_DIR")
    done

    echo "No .uproject file found" >&2
    return 1
}

ACTIVATE_VENV() {
  if [[ "$OSTYPE" != "msys" ]]; then
    UPROJECT_FILE=$(FIND_UPROJECT "$TEMPOROS_ROOT")
    PROJECT_ROOT=$(dirname "$UPROJECT_FILE")
    
    if [ ! -f "$PROJECT_ROOT/TempoEnv/bin/activate" ]; then
      echo "No Tempo Python virtual environment found. Please build (without TEMPO_SKIP_PREBUILD) first."
    fi
    
    source "$PROJECT_ROOT/TempoEnv/bin/activate"
  fi
}

if [[ "$OSTYPE" = "msys" ]]; then
  export AMENT_PREFIX_PATH="$RCLCPP_DIR/Binaries/Windows"
  export PYTHONPATH=$PYTHONPATH:"$TEMPOROS_ROOT/Source/ThirdParty/rclcpp/Libraries/Windows/python3.11/site-packages"
  export PATH=$PATH:"$TEMPOROS_ROOT/Source/ThirdParty/rclcpp/Binaries/Windows"
elif [[ "$OSTYPE" = "darwin"* ]]; then
  export AMENT_PREFIX_PATH="$RCLCPP_DIR/Libraries/Mac"
  export PYTHONPATH=$PYTHONPATH:"$TEMPOROS_ROOT/Source/ThirdParty/rclcpp/Libraries/MAc/python3.11/site-packages"
  export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:"$TEMPOROS_ROOT/Source/ThirdParty/rclcpp/Libraries/Mac"
  alias ros2="python $TEMPOROS_ROOT/Source/ThirdParty/rclcpp/Binaries/Mac/ros2"
  ACTIVATE_VENV
elif [[ "$OSTYPE" = "linux-gnu"* ]]; then
  export AMENT_PREFIX_PATH="$RCLCPP_DIR/Libraries/Linux"
  export PYTHONPATH=$PYTHONPATH:"$TEMPOROS_ROOT/Source/ThirdParty/rclcpp/Libraries/Linux/python3.11/site-packages"
  export LD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:"$TEMPOROS_ROOT/Source/ThirdParty/rclcpp/Libraries/Linux"
  alias ros2="python $TEMPOROS_ROOT/Source/ThirdParty/rclcpp/Binaries/Linux/ros2"
  ACTIVATE_VENV
fi
