# Copyright Tempo Simulation, LLC. All Rights Reserved

# Source this file to set environment variables to use the ros2 CLI that comes with TempoROS
# (e.g. ros2 topic list, ros2 topic echo, etc)

SHELL=$(ps -p $$ | awk '$1 != "PID" {print $(NF)}')

if [[ "$SHELL" =~ "zsh" ]]; then
  # In zsh this is the way to check if we're being sourced or run
  # https://unix.stackexchange.com/questions/73008/how-can-a-zsh-script-test-whether-it-is-being-sourced
  if [[ "$ZSH_EVAL_CONTEXT" == 'toplevel' ]]; then
      echo "Don't run ROSEnv.sh, source it."
      exit 1
  fi
  # In zsh BASH_SOURCE is the directory where the script is called from, not the sourced file
  # https://unix.stackexchange.com/questions/76505/unix-portable-way-to-get-scripts-absolute-path-in-zsh
  TEMPOROS_ROOT=${0:a:h}/..
else
  if [ "${0##*/}" = "ROSEnv.sh" ]; then
      echo "Don't run ROSEnv.sh, source it."
      exit 1
  fi
  TEMPOROS_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"/..
fi

TEMPOROS_ROOT=$(readlink -f "$TEMPOROS_ROOT")

FIND_PROJECT_ROOT() {
    local START_DIR
    START_DIR=$(dirname "$1")
    local CURRENT_DIR="$START_DIR"
    local UPROJECT_FILE

    while [[ "$CURRENT_DIR" != "/" ]]; do
        UPROJECT_FILE=$(find "$CURRENT_DIR" -maxdepth 1 -name "*.uproject" -print -quit)
        if [[ -n "$UPROJECT_FILE" ]]; then
            dirname "$UPROJECT_FILE"
            return 0
        fi
        CURRENT_DIR=$(dirname "$CURRENT_DIR")
    done

    echo "No .uproject file found" >&2
    return 1
}

# Activates the Tempo virtual environment, which points to Unreal's python and contains ROS package
# dependencies, if it is found.
ACTIVATE_PYTHON_VENV() {
  if [[ "$OSTYPE" != "msys" ]]; then
    PROJECT_ROOT=$(FIND_PROJECT_ROOT "$TEMPOROS_ROOT")
    
    if [ ! -f "$PROJECT_ROOT/TempoEnv/bin/activate" ]; then
      echo "No Tempo Python virtual environment found. Please build (without TEMPO_SKIP_PREBUILD) first."
    fi
    
    source "$PROJECT_ROOT/TempoEnv/bin/activate"
  fi
}

# Sets the specified environment variable to the specified value, or appends the specified value
# if the environment variable is set but does not already contain the specified value.
SET_OR_APPEND_ENV() {
  ENV_VAR="$1"
  VALUE="$2"
  eval "CURRENT_VALUE=\$$ENV_VAR"
  if [ -n "$CURRENT_VALUE" ]; then
      # Append the new value only if it's not already present
      if [[ "$CURRENT_VALUE" != *"$VALUE"* ]]; then
          eval "$ENV_VAR='$CURRENT_VALUE:$VALUE'"
          export "${ENV_VAR?}"
      fi
  else
      # If variable doesn't exist or is empty, set it
      eval "$ENV_VAR='$VALUE'"
      export "${ENV_VAR?}"
  fi
}

if [ -n "$AMENT_PREFIX_PATH" ]; then
  echo "It looks like you've already sourced a ROS environment. Please source in a new shell."
  return
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RCLCPP_DIR="$TEMPOROS_ROOT/Source/ThirdParty/rclcpp"

if [[ "$OSTYPE" = "msys" ]]; then
  SET_OR_APPEND_ENV "AMENT_PREFIX_PATH" "$RCLCPP_DIR/Binaries/Windows"
  SET_OR_APPEND_ENV "PYTHONPATH" "$RCLCPP_DIR/Libraries/Windows/python3.11/site-packages"
  SET_OR_APPEND_ENV "PATH" "$RCLCPP_DIR/Binaries/Windows"
elif [[ "$OSTYPE" = "darwin"* ]]; then
  SET_OR_APPEND_ENV "AMENT_PREFIX_PATH" "$RCLCPP_DIR/Libraries/Mac"
  SET_OR_APPEND_ENV "PYTHONPATH" "$RCLCPP_DIR/Libraries/Mac/python3.11/site-packages"
  SET_OR_APPEND_ENV "DYLD_LIBRARY_PATH" "$RCLCPP_DIR/Libraries/Mac"
  SET_OR_APPEND_ENV "PATH" "$RCLCPP_DIR/Binaries/Mac"
  # On Mac ros2 is a Python program, with a shebang reflecting the build machine's environment.
  # So, make an alias to run it with the local Python (from the virtual environment) explicitly.
  alias ros2="python $RCLCPP_DIR/Binaries/Mac/ros2"
  ACTIVATE_PYTHON_VENV
elif [[ "$OSTYPE" = "linux-gnu"* ]]; then
  SET_OR_APPEND_ENV "AMENT_PREFIX_PATH" "$RCLCPP_DIR/Libraries/Linux"
  SET_OR_APPEND_ENV "PYTHONPATH" "$RCLCPP_DIR/Libraries/Linux/python3.11/site-packages"
  SET_OR_APPEND_ENV "LD_LIBRARY_PATH" "$RCLCPP_DIR/Libraries/Linux"
  SET_OR_APPEND_ENV "PATH" "$RCLCPP_DIR/Binaries/Linux"
  # On Linux ros2 is a Python program, with a shebang reflecting the build machine's environment.
  # So, make an alias to run it with the local Python (from the virtual environment) explicitly.
  alias ros2="python $RCLCPP_DIR/Binaries/Linux/ros2"
  ACTIVATE_PYTHON_VENV
fi
