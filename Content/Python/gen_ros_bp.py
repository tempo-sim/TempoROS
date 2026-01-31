"""
Generate Blueprint function libraries for modules using TempoROS.
Copyright Tempo Simulation, LLC. All Rights Reserved
"""

import os
import re
import shutil
import sys
import tempfile
from pathlib import Path
from typing import List, Tuple, Set


class ROSBlueprintGenerator:
    def __init__(self, project_root: str):
        self.project_root = Path(project_root)
        self.temp_dir = None
        self.gen_temp_dir = None
        self.bp_keyword = "TempoROS__BPSupport"

    def setup_temp_dirs(self):
        """Create temporary directory structure"""
        self.temp_dir = Path(tempfile.mkdtemp())
        self.gen_temp_dir = self.temp_dir / "Generated"
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

    def find_modules(self) -> List[Tuple[str, Path]]:
        """Find all modules by scanning for *.Build.cs files"""
        modules = []
        for build_cs in self.project_root.rglob("*.Build.cs"):
            module_name = build_cs.stem.split('.')[0]
            module_path = build_cs.parent
            modules.append((module_name, module_path))
        return modules

    def extract_message_type(self, line: str) -> str:
        """Extract message type from template parameter, e.g. TToROSConverter<FString> -> FString"""
        match = re.search(r'[^<]*<([^>]*)>', line)
        if match:
            return match.group(1)
        return ""

    def sanitize_unreal_type(self, type_name: str) -> str:
        """Strip F/U/A prefix if followed by uppercase (e.g. FString -> String)"""
        if len(type_name) >= 2 and type_name[0] in ('U', 'F', 'A') and type_name[1].isupper():
            return type_name[1:]
        return type_name

    def scan_headers_for_bp_support(self, public_dir: Path) -> Tuple[List[Tuple[str, str]], List[Tuple[str, str]], Set[str]]:
        """
        Scan all .h files in public_dir for TempoROS__BPSupport lines.
        Returns (publishers, subscriptions, include_files)
        - publishers: list of (message_type, sanitized_type) from TToROSConverter lines
        - subscriptions: list of (message_type, sanitized_type) from TFromROSConverter lines
        - include_files: set of relative header paths that need to be included
        """
        publishers = []
        subscriptions = []
        include_files = set()

        if not public_dir.exists():
            return publishers, subscriptions, include_files

        for header_file in public_dir.rglob("*.h"):
            try:
                with open(header_file, 'r', encoding='utf-8', errors='ignore') as f:
                    content = f.read()
            except IOError:
                continue

            has_publisher = False
            has_subscription = False

            for line in content.split('\n'):
                if self.bp_keyword not in line:
                    continue

                message_type = self.extract_message_type(line)
                if not message_type:
                    continue

                sanitized_type = self.sanitize_unreal_type(message_type)

                if 'TToROSConverter' in line or 'TImplicitToROSConverter' in line:
                    publishers.append((message_type, sanitized_type))
                    has_publisher = True
                elif 'TFromROSConverter' in line or 'TImplicitFromROSConverter' in line:
                    subscriptions.append((message_type, sanitized_type))
                    has_subscription = True

            if has_publisher or has_subscription:
                relative_path = header_file.relative_to(public_dir)
                include_files.add(str(relative_path))

        return publishers, subscriptions, include_files

    def generate_library_content(self, module_name: str, publishers: List[Tuple[str, str]],
                                  subscriptions: List[Tuple[str, str]], include_files: Set[str]) -> str:
        """Generate the Blueprint function library header content"""
        module_api = f"{module_name.upper()}_API"

        # Build includes
        includes_str = ""
        for include_file in sorted(include_files):
            includes_str += f'\n#include "{include_file}"'

        # Build delegates (only for subscriptions)
        delegates_str = ""
        seen_subscription_types = set()
        for message_type, sanitized_type in subscriptions:
            if sanitized_type not in seen_subscription_types:
                seen_subscription_types.add(sanitized_type)
                delegates_str += f"""
DECLARE_DYNAMIC_DELEGATE_OneParam(FTempoROS{sanitized_type}Received, {message_type}, Value);
"""

        # Build methods
        methods_str = ""

        # Publisher methods
        seen_publisher_types = set()
        for message_type, sanitized_type in publishers:
            if sanitized_type not in seen_publisher_types:
                seen_publisher_types.add(sanitized_type)
                methods_str += f"""
    UFUNCTION(BlueprintCallable, Category = "TempoROS", meta=(AutoCreateRefTerm="QOSProfile"))
    static bool Add{sanitized_type}Publisher(UTempoROSNode* Node, const FString& Topic, const FROSQOSProfile& QOSProfile=FROSQOSProfile(), bool bPrependNodeName=true)
    {{
        return Node->AddPublisher<{message_type}>(Topic, QOSProfile, bPrependNodeName);
    }}

    UFUNCTION(BlueprintCallable, Category = "TempoROS")
    static bool Publish{sanitized_type}(UTempoROSNode* Node, const FString& Topic, const {message_type}& Message)
    {{
        return Node->Publish<{message_type}>(Topic, Message);
    }}
"""

        # Subscription methods
        seen_subscription_types_for_methods = set()
        for message_type, sanitized_type in subscriptions:
            if sanitized_type not in seen_subscription_types_for_methods:
                seen_subscription_types_for_methods.add(sanitized_type)
                methods_str += f"""
    UFUNCTION(BlueprintCallable, Category = "TempoROS")
    static bool Add{sanitized_type}Subscription(UTempoROSNode* Node, const FString& Topic, const FTempoROS{sanitized_type}Received& TempoROSMessageReceivedEvent)
    {{
        return Node->AddSubscription<{message_type}>(Topic, TROSSubscriptionDelegate<{message_type}>::CreateLambda([TempoROSMessageReceivedEvent](const {message_type}& Value)
        {{
            TempoROSMessageReceivedEvent.ExecuteIfBound(Value);
        }}));
    }}
"""

        # Build final content
        content = f"""// Copyright Tempo Simulation, LLC. All Rights Reserved

/* This file was generated by a TempoROS prebuild step and should not be modified. */

#pragma once
{includes_str}

#include "TempoROSNode.h"

#include "TempoROSTypes.h"

#include "Kismet/BlueprintFunctionLibrary.h"

#include "{module_name}TempoROSBlueprintFunctionLibrary.generated.h"
{delegates_str}
UCLASS(BlueprintType)
class {module_api} U{module_name}TempoROSBlueprintFunctionLibrary : public UBlueprintFunctionLibrary
{{
    GENERATED_BODY()
{methods_str}}};
"""
        return content

    def generate_module_bp(self, module_name: str, module_path: Path):
        """Generate Blueprint function library for a module"""
        library_filename = f"{module_name}TempoROSBlueprintFunctionLibrary.h"
        public_dir = module_path / "Public"
        dest_file = public_dir / library_filename

        if not public_dir.exists():
            # No public directory, remove any existing library file
            if dest_file.exists():
                dest_file.unlink()
            return

        publishers, subscriptions, include_files = self.scan_headers_for_bp_support(public_dir)

        if not include_files:
            # Nothing to generate for this module
            if dest_file.exists():
                dest_file.unlink()
            return

        # Generate content
        content = self.generate_library_content(module_name, publishers, subscriptions, include_files)

        # Write to temp file first
        module_gen_temp_dir = self.gen_temp_dir / module_name
        module_gen_temp_dir.mkdir(parents=True, exist_ok=True)
        temp_file = module_gen_temp_dir / library_filename

        with open(temp_file, 'w', encoding='utf-8') as f:
            f.write(content)

        # Replace only if stale
        self.replace_if_stale(temp_file, dest_file)

    def run(self):
        """Main execution flow"""
        try:
            print("Generating ROS IDL BP Support...")

            self.setup_temp_dirs()

            modules = self.find_modules()

            for module_name, module_path in modules:
                self.generate_module_bp(module_name, module_path)

            print("Done")

        finally:
            self.cleanup_temp_dirs()


def main():
    if len(sys.argv) < 2:
        print("Usage: gen_ros_bp.py <project_root>", file=sys.stderr)
        return 1

    # Check for TEMPO_SKIP_PREBUILD
    skip_prebuild = os.environ.get("TEMPO_SKIP_PREBUILD", "")
    if skip_prebuild and skip_prebuild != "0":
        print(f"Skipping TempoROS Blueprint function generation because TEMPO_SKIP_PREBUILD is {skip_prebuild}")
        return 0

    project_root = sys.argv[1]

    try:
        generator = ROSBlueprintGenerator(project_root)
        generator.run()
        return 0
    except Exception as e:
        print(f"Fatal error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
