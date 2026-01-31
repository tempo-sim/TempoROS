"""
Generate C++ from ROS IDL (.msg/.srv) files for Tempo plugin modules.
Copyright Tempo Simulation, LLC. All Rights Reserved
"""

import argparse
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Dict, Set, Optional, List


class ROSIDLGenerator:
    def __init__(self, args):
        self.engine_dir = Path(args.engine_dir)
        self.project_file = Path(args.project_file)
        self.project_root = Path(args.project_root)
        self.plugin_root = Path(args.plugin_root)
        self.target_name = args.target_name
        self.target_config = args.target_config
        self.target_platform = args.target_platform
        self.rclcpp_dir = Path(args.rclcpp_dir)

        self.platform_folder = self.get_platform_folder()
        self.temp_dir = None
        self.src_temp_dir = None
        self.includes_temp_dir = None
        self.gen_temp_dir = None
        self.module_info = {}
        self.python_executable = None  # Path to Python executable for rosidl

    def get_platform_folder(self) -> str:
        """Determine platform folder name"""
        system = platform.system()
        if system == "Windows":
            return "Windows"
        elif system == "Darwin":
            return "Mac"
        elif system == "Linux":
            return "Linux"
        else:
            raise RuntimeError(f"Unrecognized platform: {system}")

    def get_unreal_python(self) -> Path:
        """Get the path to Unreal's Python executable"""
        system = platform.system()
        if system == "Windows":
            return self.engine_dir / "Binaries/ThirdParty/Python3/Win64/python.exe"
        elif system == "Darwin":
            return self.engine_dir / "Binaries/ThirdParty/Python3/Mac/bin/python3"
        elif system == "Linux":
            return self.engine_dir / "Binaries/ThirdParty/Python3/Linux/bin/python3"
        else:
            raise RuntimeError(f"Unsupported platform: {system}")

    def setup_environment(self):
        """Set up PYTHONPATH and AMENT_PREFIX_PATH for rosidl"""
        # Use Unreal's Python directly - required packages are bundled in rclcpp's site-packages
        self.python_executable = self.get_unreal_python()

        # Set up PYTHONPATH to include rclcpp's site-packages (contains pyyaml, empy, lark, netifaces)
        # Prepend to any existing PYTHONPATH
        site_packages = str(self.rclcpp_dir / "Libraries" / self.platform_folder / "python3.11/site-packages")
        existing_pythonpath = os.environ.get("PYTHONPATH", "")
        if existing_pythonpath:
            os.environ["PYTHONPATH"] = site_packages + os.pathsep + existing_pythonpath
        else:
            os.environ["PYTHONPATH"] = site_packages

        if platform.system() == "Windows":
            os.environ["AMENT_PREFIX_PATH"] = str(self.rclcpp_dir / "Binaries" / self.platform_folder)
        else:
            os.environ["AMENT_PREFIX_PATH"] = str(self.rclcpp_dir / "Libraries" / self.platform_folder)

    def get_gentool(self) -> str:
        """Get the path to the rosidl tool"""
        if platform.system() == "Windows":
            return str(self.rclcpp_dir / "Binaries" / self.platform_folder / "rosidl-script.py")
        else:
            return str(self.rclcpp_dir / "Binaries" / self.platform_folder / "rosidl")

    def setup_temp_dirs(self):
        """Create temporary directory structure"""
        self.temp_dir = Path(tempfile.mkdtemp())
        self.src_temp_dir = self.temp_dir / "Source"
        self.includes_temp_dir = self.temp_dir / "Includes"
        self.gen_temp_dir = self.temp_dir / "Generated"

        self.src_temp_dir.mkdir()
        self.includes_temp_dir.mkdir()
        self.gen_temp_dir.mkdir()

    def cleanup_temp_dirs(self):
        """Remove temporary directories"""
        if self.temp_dir and self.temp_dir.exists():
            shutil.rmtree(self.temp_dir)

    def replace_if_stale(self, fresh: Path, possibly_stale: Path):
        """Copy file only if it doesn't exist or differs from source"""
        if not possibly_stale.exists():
            possibly_stale.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(fresh, possibly_stale)
        else:
            if not self._files_identical(fresh, possibly_stale):
                shutil.copy2(fresh, possibly_stale)

    @staticmethod
    def _files_identical(file1: Path, file2: Path) -> bool:
        """Compare two files for equality"""
        if file1.stat().st_size != file2.stat().st_size:
            return False

        chunk_size = 8192
        with open(file1, 'rb') as f1, open(file2, 'rb') as f2:
            while True:
                chunk1 = f1.read(chunk_size)
                chunk2 = f2.read(chunk_size)
                if chunk1 != chunk2:
                    return False
                if not chunk1:
                    return True

    def find_dotnet(self) -> Path:
        """Locate the dotnet executable in the engine directory"""
        system = platform.system()

        if system == "Windows":
            pattern = "**/dotnet.exe"
        else:
            pattern = "**/dotnet"

        dotnet_dir = self.engine_dir / "Binaries/ThirdParty/DotNet"
        if not dotnet_dir.exists():
            raise RuntimeError(f"DotNet directory not found: {dotnet_dir}")

        dotnets = list(dotnet_dir.glob(pattern))
        if not dotnets:
            raise RuntimeError(f"dotnet executable not found in {dotnet_dir}")

        if system == "Windows":
            return dotnets[0]

        # On Mac/Linux, find the appropriate architecture
        machine = platform.machine().lower()

        if system == "Darwin":
            if machine == "arm64":
                arch_pattern = "mac-arm64"
            else:
                arch_pattern = "mac-x64"
        elif system == "Linux":
            # Check engine version for 5.4 compatibility
            release = self._get_engine_release()
            if release == "5.4":
                return dotnets[0]

            if machine in ["arm64", "aarch64"]:
                arch_pattern = "linux-arm64"
            else:
                arch_pattern = "linux-x64"

        for dotnet in dotnets:
            if arch_pattern in str(dotnet):
                return dotnet

        raise RuntimeError(f"Could not find dotnet for platform {system}/{machine}")

    def _get_engine_release(self) -> Optional[str]:
        """Get the engine release version (e.g., '5.4')"""
        manifest_path = self.engine_dir / "Intermediate/Build/BuildRules/UE5RulesManifest.json"
        if manifest_path.exists():
            try:
                with open(manifest_path) as f:
                    data = json.load(f)
                    version = data.get("EngineVersion", "")
                    # Extract major.minor from version like "5.4.1"
                    parts = version.split(".")
                    if len(parts) >= 2:
                        return f"{parts[0]}.{parts[1]}"
            except (json.JSONDecodeError, KeyError) as e:
                print(f"Warning: Could not parse engine version: {e}", file=sys.stderr)
        return None

    def export_module_dependencies(self) -> Path:
        """Run UnrealBuildTool to export module dependency information"""
        dotnet = self.find_dotnet()
        ubt_dll = self.engine_dir / "Binaries/DotNET/UnrealBuildTool/UnrealBuildTool.dll"

        if not ubt_dll.exists():
            raise RuntimeError(f"UnrealBuildTool not found: {ubt_dll}")

        output_file = self.temp_dir / "TempoModules.json"

        cmd = [
            str(dotnet),
            str(ubt_dll),
            "-Mode=JsonExport",
            self.target_name,
            self.target_platform,
            self.target_config,
            f"-Project={self.project_file}",
            f"-OutputFile={output_file}",
            "-NoMutex"
        ]

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                check=True
            )
        except subprocess.CalledProcessError as e:
            print(f"Error: UnrealBuildTool failed with exit code {e.returncode}", file=sys.stderr)
            if e.stdout:
                print(f"stdout: {e.stdout}", file=sys.stderr)
            if e.stderr:
                print(f"stderr: {e.stderr}", file=sys.stderr)
            raise RuntimeError("Failed to export module dependencies")

        if not output_file.exists():
            raise RuntimeError(f"UnrealBuildTool did not create output file: {output_file}")

        return output_file

    def filter_project_modules(self, json_file: Path) -> Dict:
        """Extract C++ project modules from the JSON export"""
        with open(json_file) as f:
            data = json.load(f)

        if "Modules" not in data:
            raise RuntimeError("Invalid module JSON: 'Modules' key not found")

        modules = data["Modules"]
        filtered = {}

        # Normalize paths for comparison
        project_root_normalized = str(self.project_root).replace("\\", "/")
        if not project_root_normalized.startswith("/"):
            project_root_normalized = "/" + project_root_normalized

        for module_name, module_data in modules.items():
            if module_data.get("Type") != "CPlusPlus":
                continue

            module_dir = module_data.get("Directory", "")
            module_dir_normalized = module_dir.replace("\\", "/")
            if not module_dir_normalized.startswith("/"):
                module_dir_normalized = "/" + module_dir_normalized

            if module_dir_normalized.startswith(project_root_normalized):
                filtered[module_name] = {
                    "Directory": module_dir,
                    "PublicDependencyModules": module_data.get("PublicDependencyModules", []),
                    "PrivateDependencyModules": module_data.get("PrivateDependencyModules", [])
                }

        return filtered

    def pascal_to_snake(self, name: str) -> str:
        """Convert PascalCase to snake_case"""
        # Insert underscore before sequences like ABc -> A_Bc
        s1 = re.sub(r'([A-Z])([A-Z])([a-z])', r'\1_\2\3', name)
        # Insert underscore before lowercase followed by uppercase
        s2 = re.sub(r'([a-z])([A-Z])', r'\1_\2', s1)
        return s2.lower()

    def snake_to_pascal(self, name: str) -> str:
        """Convert snake_case to PascalCase"""
        # Remove __ and anything after
        name = name.split('__')[0]
        # Capitalize first letter of each word and join
        return ''.join(word.capitalize() for word in name.split('_'))

    def validate_idl_directories(self, module_path: Path):
        """Validate that msg/srv directories only contain .msg/.srv files"""
        for subdir, extension in [("msg", ".msg"), ("srv", ".srv")]:
            for visibility in ["Public", "Private"]:
                dir_path = module_path / visibility / subdir
                if dir_path.exists():
                    for f in dir_path.iterdir():
                        if f.is_file() and f.suffix != extension:
                            raise RuntimeError(f"Found non-{extension} file in {dir_path}: {f.name}")

    def copy_module_idl_files(self, module_name: str, module_path: Path):
        """Copy .msg/.srv files from module to temp directory"""
        module_src_temp_dir = self.src_temp_dir / module_name

        if module_src_temp_dir.exists():
            raise RuntimeError(f"Multiple modules named {module_name} found. Please rename one.")

        (module_src_temp_dir / "Public").mkdir(parents=True)
        (module_src_temp_dir / "Private").mkdir(parents=True)

        # Validate directories
        self.validate_idl_directories(module_path)

        # Copy public IDL files
        public_dir = module_path / "Public"
        if public_dir.exists():
            for idl_file in public_dir.rglob("*.msg"):
                self._copy_idl_file(idl_file, module_path, module_src_temp_dir)
            for idl_file in public_dir.rglob("*.srv"):
                self._copy_idl_file(idl_file, module_path, module_src_temp_dir)

        # Copy private IDL files
        private_dir = module_path / "Private"
        if private_dir.exists():
            for idl_file in private_dir.rglob("*.msg"):
                self._copy_idl_file(idl_file, module_path, module_src_temp_dir)
            for idl_file in private_dir.rglob("*.srv"):
                self._copy_idl_file(idl_file, module_path, module_src_temp_dir)

    def _copy_idl_file(self, idl_file: Path, module_path: Path, module_src_temp_dir: Path):
        """Copy a single IDL file preserving relative path"""
        relative_path = idl_file.relative_to(module_path)
        dest_file = module_src_temp_dir / relative_path
        dest_file.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(idl_file, dest_file)

    def sync_msg_srv_files(self, src: Path, dest: Path):
        """Copy .msg and .srv files from src to dest, preserving directory structure"""
        if not src.exists():
            return

        for pattern in ["*.msg", "*.srv"]:
            for idl_file in src.rglob(pattern):
                relative_path = idl_file.relative_to(src)
                dest_file = dest / relative_path
                dest_file.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(idl_file, dest_file)

    def get_module_includes_public_only(self, module_name: str, includes_dir: Path, visited: Set[str]):
        """Recursively get public dependencies of a module"""
        if module_name in visited:
            return

        visited.add(module_name)

        public_deps = self.module_info.get(module_name, {}).get("PublicDependencyModules", [])

        for dep in public_deps:
            dep = dep.strip().rstrip('\r')

            # Only consider project modules
            if not (self.src_temp_dir / dep).exists():
                continue

            dep_dir = includes_dir / dep
            if not dep_dir.exists():
                # New dependency - add its public IDL files
                dep_dir.mkdir(parents=True, exist_ok=True)
                self.sync_msg_srv_files(self.src_temp_dir / dep / "Public", dep_dir)

            # Recursively add its public dependencies
            self.get_module_includes_public_only(dep, includes_dir, visited)

    def get_module_includes(self, module_name: str, includes_dir: Path):
        """Get all includes needed for a module (public and private dependencies)"""
        # Copy this module's IDL files (both public and private)
        module_dir = includes_dir / module_name
        module_dir.mkdir(parents=True, exist_ok=True)
        self.sync_msg_srv_files(self.src_temp_dir / module_name / "Public", module_dir)
        self.sync_msg_srv_files(self.src_temp_dir / module_name / "Private", module_dir)

        visited = set()

        # Add public dependencies
        public_deps = self.module_info.get(module_name, {}).get("PublicDependencyModules", [])
        for dep in public_deps:
            dep = dep.strip().rstrip('\r')

            if not (self.src_temp_dir / dep).exists():
                continue

            dep_dir = includes_dir / dep
            if not dep_dir.exists():
                dep_dir.mkdir(parents=True, exist_ok=True)
                self.sync_msg_srv_files(self.src_temp_dir / dep / "Public", dep_dir)

            self.get_module_includes_public_only(dep, includes_dir, visited)

        # Add private dependencies
        private_deps = self.module_info.get(module_name, {}).get("PrivateDependencyModules", [])
        for dep in private_deps:
            dep = dep.strip().rstrip('\r')

            if not (self.src_temp_dir / dep).exists():
                continue

            dep_dir = includes_dir / dep
            if not dep_dir.exists():
                dep_dir.mkdir(parents=True, exist_ok=True)
                self.sync_msg_srv_files(self.src_temp_dir / dep / "Public", dep_dir)

            self.get_module_includes_public_only(dep, includes_dir, visited)

    def get_ros_package_name(self, module_name: str, module_path: Path) -> str:
        """Get ROS package name, either from ros_info.json or derived from module name"""
        ros_info_file = module_path / "ros_info.json"
        if ros_info_file.exists():
            try:
                with open(ros_info_file) as f:
                    data = json.load(f)
                    custom_name = data.get("custom_package_name", "")
                    if custom_name:
                        return self.pascal_to_snake(custom_name)
            except (json.JSONDecodeError, KeyError):
                pass
        return self.pascal_to_snake(module_name)

    def remove_stale_package_dirs(self, module_path: Path, package_name: str):
        """Remove stale generated package directories (when user renamed the package)"""
        for visibility in ["Public", "Private"]:
            visibility_dir = module_path / visibility
            if not visibility_dir.exists():
                continue
            for subdir in visibility_dir.iterdir():
                if subdir.is_dir() and subdir.name != package_name:
                    # Check if this looks like a generated ROS package
                    if (subdir / "msg" / "detail").exists() or (subdir / "srv" / "detail").exists():
                        print(f"Removing stale generated ROS package: {subdir}")
                        shutil.rmtree(subdir)

    def run_rosidl(self, package_name: str, source_dir: Path, include_dir: Path,
                   module_gen_temp_dir: Path) -> bool:
        """Run rosidl for each .msg/.srv file in source_dir. Returns True if any files were generated."""
        gentool = self.get_gentool()
        has_rosidl_files = False

        # Find all .msg and .srv files
        idl_files = list(source_dir.rglob("*.msg")) + list(source_dir.rglob("*.srv"))
        if not idl_files:
            return False

        # Build environment for subprocess - inherit current env and add our paths
        env = os.environ.copy()

        for idl_file in idl_files:
            has_rosidl_files = True
            relative_path = idl_file.relative_to(source_dir)

            # Build the command using Unreal's Python with bundled packages
            cmd = [
                str(self.python_executable),
                gentool,
                "generate",
                "--type", "cpp",
                "--type-support", "cpp",
                "--type-support", "introspection_cpp",
                "--type-support", "fastrtps_cpp",
                package_name,
                f"{source_dir}:{relative_path}",
                "-o", str(module_gen_temp_dir / package_name)
            ]

            # Add include directories
            for subdir in include_dir.iterdir():
                if subdir.is_dir():
                    cmd.extend(["-I", str(subdir.resolve())])

            try:
                subprocess.run(cmd, check=True, capture_output=True, text=True, env=env)
            except subprocess.CalledProcessError as e:
                print(f"Error running rosidl for {idl_file}:", file=sys.stderr)
                print(f"Command: {' '.join(cmd)}", file=sys.stderr)
                if e.stdout:
                    print(f"stdout: {e.stdout}", file=sys.stderr)
                if e.stderr:
                    print(f"stderr: {e.stderr}", file=sys.stderr)
                raise

        return has_rosidl_files

    def copy_generated_files(self, module_gen_temp_dir: Path, module_path: Path,
                             package_name: str, module_name: str) -> Set[Path]:
        """Copy generated files to module Public/Private directories. Returns set of refreshed files."""
        private_dest_dir = module_path / "Private"
        public_dest_dir = module_path / "Public"
        module_src_temp_dir = self.src_temp_dir / module_name
        refreshed_files = set()

        package_gen_dir = module_gen_temp_dir / package_name
        if not package_gen_dir.exists():
            return refreshed_files

        for generated_file in package_gen_dir.rglob("*"):
            if not generated_file.is_file():
                continue

            relative_path = generated_file.relative_to(package_gen_dir)
            relative_str = str(relative_path)

            # Skip tmp directories
            if "/tmp/" in relative_str or "\\tmp\\" in relative_str or "tmp/" in relative_str:
                continue

            # Determine extension (msg or srv)
            if "/msg/" in relative_str or "\\msg\\" in relative_str or "msg/" in relative_str:
                extension = "msg"
            elif "/srv/" in relative_str or "\\srv\\" in relative_str or "srv/" in relative_str:
                extension = "srv"
            else:
                continue

            # Get original filename to determine public/private
            filename = generated_file.name
            filename_no_ext = filename.rsplit('.', 1)[0]
            filename_no_ext = self.snake_to_pascal(filename_no_ext)
            original_filename = f"{filename_no_ext}.{extension}"

            # De-duplicate cpp file names by type support
            if "introspection_cpp" in relative_str and "__type_support.cpp" in filename:
                relative_str = relative_str.replace("__type_support.cpp", "__introspection_cpp_type_support.cpp")
            if "fastrtps_cpp" in relative_str and "__type_support.cpp" in filename:
                relative_str = relative_str.replace("__type_support.cpp", "__fastrtps_cpp_type_support.cpp")

            # Remove cpp/, introspection_cpp/, fastrtps_cpp/ from path (can be at start or middle)
            # Handle paths starting with these prefixes
            for prefix in ["cpp/", "introspection_cpp/", "fastrtps_cpp/"]:
                prefix_backslash = prefix.replace("/", "\\")
                if relative_str.startswith(prefix):
                    relative_str = relative_str[len(prefix):]
                elif relative_str.startswith(prefix_backslash):
                    relative_str = relative_str[len(prefix_backslash):]
                # Also handle them in the middle of paths
                relative_str = relative_str.replace("/" + prefix, "/")
                relative_str = relative_str.replace("\\" + prefix_backslash, "\\")

            relative_path = Path(relative_str)

            # If the original file was in Public, generated files go in Public
            public_original = list((module_src_temp_dir / "Public").rglob(original_filename))
            if public_original:
                dest_file = public_dest_dir / package_name / relative_path
            else:
                dest_file = private_dest_dir / package_name / relative_path

            self.replace_if_stale(generated_file, dest_file)
            refreshed_files.add(dest_file)

        return refreshed_files

    def remove_stale_files(self, dest_dir: Path, package_name: str, refreshed_files: Set[Path]):
        """Remove generated files that were not refreshed this time"""
        package_dir = dest_dir / package_name
        if not package_dir.exists():
            return

        for f in package_dir.rglob("*"):
            if f.is_file() and f not in refreshed_files:
                f.unlink()

        # Remove empty directories
        self._remove_empty_dirs(package_dir)

    @staticmethod
    def _remove_empty_dirs(directory: Path):
        """Recursively remove empty directories"""
        if not directory.exists():
            return

        for dirpath, dirnames, filenames in os.walk(directory, topdown=False):
            path = Path(dirpath)
            if not filenames and not dirnames and path != directory:
                path.rmdir()

    def generate_force_export_file(self, module_path: Path, package_name: str):
        """Generate the force_export_rosidl_typesupport_symbols.cpp file"""
        # Find all typesupport header files
        typesupport_files = []
        for visibility in ["Public", "Private"]:
            search_dir = module_path / visibility
            if search_dir.exists():
                typesupport_files.extend(search_dir.rglob("*__rosidl_typesupport_*_cpp.hpp"))

        if not typesupport_files:
            return

        includes = []
        symbols = []

        for f in typesupport_files:
            # Get relative path from Public or Private
            try:
                relative_path = f.relative_to(module_path / "Public")
            except ValueError:
                relative_path = f.relative_to(module_path / "Private")

            includes.append(f'#include "{relative_path}"')

            # Determine if service or message
            relative_str = str(relative_path)
            if "/srv/" in relative_str or "\\srv\\" in relative_str:
                service_or_message = "service"
                srv_or_msg = "srv"
            else:
                service_or_message = "message"
                srv_or_msg = "msg"

            # Extract typesupport type
            basename = f.stem
            # Format: name__rosidl_typesupport_TYPE_cpp
            match = re.search(r'rosidl_typesupport_(\w+)_cpp', basename)
            if match:
                typesupport = match.group(1)
            else:
                continue

            # Extract service/message name
            service_or_message_name = self.snake_to_pascal(basename.split("__")[0])

            symbol = f"(void*)&rosidl_typesupport_{typesupport}_cpp__get_{service_or_message}_type_support_handle__{package_name}__{srv_or_msg}__{service_or_message_name},"
            symbols.append(symbol)

        if not symbols:
            return

        content = f"""// Copyright Tempo Simulation, LLC. All Rights Reserved

/* This file was generated by a TempoROS prebuild step and should not be modified. */

#if defined(_MSC_VER)
 #pragma optimize( "", off )
#elif defined(__clang__)
 #pragma clang optimize off
#endif

{chr(10).join(sorted(set(includes)))}

#if defined(__GNUC__)
__attribute__((used))
#if not defined(__clang__)
__attribute__((optimize("O0"))
#endif
#endif
volatile void* {package_name}_rosidl_typesupport_symbols[] = {{
{chr(10).join(sorted(set(symbols)))}
}};
#if defined(_MSC_VER)
#pragma optimize( "", on )
#elif defined(__clang__)
#pragma clang optimize on
#endif
"""

        # Write to temp file first then replace if stale
        temp_file = self.gen_temp_dir / f"{package_name}_force_export_rosidl_typesupport_symbols.cpp"
        with open(temp_file, 'w', encoding='utf-8') as f:
            f.write(content)

        dest_file = module_path / "Private" / package_name / f"{package_name}_force_export_rosidl_typesupport_symbols.cpp"
        self.replace_if_stale(temp_file, dest_file)

    def generate_module_idl(self, module_name: str, module_path: Path, include_dir: Path, package_name: str):
        """Generate all IDL code for a module"""
        source_dir = include_dir / module_name
        module_gen_temp_dir = self.gen_temp_dir / module_name
        module_gen_temp_dir.mkdir(parents=True, exist_ok=True)

        # Run rosidl
        has_rosidl_files = self.run_rosidl(package_name, source_dir, include_dir, module_gen_temp_dir)

        if not has_rosidl_files:
            return

        # Copy generated files
        refreshed_files = self.copy_generated_files(module_gen_temp_dir, module_path, package_name, module_name)

        # Remove stale files
        self.remove_stale_files(module_path / "Public", package_name, refreshed_files)
        self.remove_stale_files(module_path / "Private", package_name, refreshed_files)

        # Generate force export file
        self.generate_force_export_file(module_path, package_name)

    def run(self):
        """Main execution flow"""
        try:
            print("Generating ROS IDL code...")

            # Setup
            self.setup_environment()
            self.setup_temp_dirs()

            # Export module dependencies
            json_file = self.export_module_dependencies()
            self.module_info = self.filter_project_modules(json_file)

            if not self.module_info:
                print("Warning: No C++ project modules found", file=sys.stderr)
                return

            # Copy all module IDL files to temp directory
            for module_name, module_data in self.module_info.items():
                module_path = Path(module_data["Directory"].strip('"').replace("\\", "/"))
                self.copy_module_idl_files(module_name, module_path)

            # Generate IDL code for each module
            for module_name, module_data in self.module_info.items():
                module_path = Path(module_data["Directory"].strip('"').replace("\\", "/"))
                package_name = self.get_ros_package_name(module_name, module_path)

                # Remove stale package directories
                self.remove_stale_package_dirs(module_path, package_name)

                module_includes_dir = self.includes_temp_dir / module_name
                self.get_module_includes(module_name, module_includes_dir)
                self.generate_module_idl(module_name, module_path, module_includes_dir, package_name)

            print("Done")

        finally:
            self.cleanup_temp_dirs()


def main():
    parser = argparse.ArgumentParser(description="Generate ROS IDL code for Tempo plugin modules")
    parser.add_argument("engine_dir", help="Unreal Engine directory")
    parser.add_argument("project_file", help="Project file path")
    parser.add_argument("project_root", help="Project root directory")
    parser.add_argument("plugin_root", help="Plugin root directory")
    parser.add_argument("target_name", help="Build target name")
    parser.add_argument("target_config", help="Build configuration")
    parser.add_argument("target_platform", help="Target platform")
    parser.add_argument("rclcpp_dir", help="rclcpp third party directory")

    args = parser.parse_args()

    # Check for TEMPO_SKIP_PREBUILD
    skip_prebuild = os.environ.get("TEMPO_SKIP_PREBUILD", "")
    if skip_prebuild and skip_prebuild != "0":
        print(f"Skipping TempoROS prebuild steps because TEMPO_SKIP_PREBUILD is {skip_prebuild}")
        return 0

    try:
        generator = ROSIDLGenerator(args)
        generator.run()
        return 0
    except Exception as e:
        print(f"Fatal error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
