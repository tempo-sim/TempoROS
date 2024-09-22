// Copyright Tempo Simulation, LLC. All Rights Reserved.

using System;
using System.Collections.Generic;
using System.IO;
using UnrealBuildTool;

public class rclcpp : ModuleRules
{
    public class ModuleDepPaths
    {
        public readonly string[] HeaderPaths;
        public readonly string[] LibraryPaths;
        public readonly string[] RuntimeLibraryPaths;

        public ModuleDepPaths(string[] headerPaths, string[] libraryPaths, string[] runtimeLibraryPaths)
        {
            HeaderPaths = headerPaths;
            LibraryPaths = libraryPaths;
            RuntimeLibraryPaths = runtimeLibraryPaths;
        }
    }

    private IEnumerable<string> FindFilesInDirectory(string dir, string suffix = "")
    {
        return Directory.EnumerateFiles(dir, "*." + suffix, SearchOption.AllDirectories);
    }

    public ModuleDepPaths GatherDeps()
    {
        List<string> HeaderPaths = new List<string>();
        List<string> LibraryPaths = new List<string>();
        List<string> RuntimeLibraryPaths = new List<string>();
        
        HeaderPaths.Add(Path.Combine(ModuleDirectory, "Includes"));

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            LibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Windows"), "lib"));
            RuntimeLibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Binaries", "Windows"), "dll"));
            RuntimeLibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Binaries", "Windows", "share"), "*"));
        }
        else if (Target.Platform == UnrealTargetPlatform.Mac)
        {
            LibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Mac"), "dylib"));
            RuntimeLibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Mac"), "dylib"));
            RuntimeLibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Mac", "share"), "*"));
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            LibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Linux"), "so"));
            RuntimeLibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Linux"), "so*"));
            RuntimeLibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Linux", "share"), "*"));
        }
        else
        {
            Console.WriteLine("Unsupported target platform for module rclcpp.");
        }

        return new ModuleDepPaths(HeaderPaths.ToArray(), LibraryPaths.ToArray(), RuntimeLibraryPaths.ToArray());
    }
    
    public rclcpp(ReadOnlyTargetRules Target) : base(Target)
    {
        Type = ModuleType.External;
        
        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Public"));
        
        ModuleDepPaths moduleDepPaths = GatherDeps();
        PublicIncludePaths.AddRange(moduleDepPaths.HeaderPaths);
        PublicAdditionalLibraries.AddRange(moduleDepPaths.LibraryPaths);
        PublicRuntimeLibraryPaths.Add(Path.Combine(ModuleDirectory, "Binaries", "Windows"));
        foreach (string libraryPath in moduleDepPaths.RuntimeLibraryPaths)
        {
            RuntimeDependencies.Add(libraryPath);
        }

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "Core"
            }
            );
        
        if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            PublicDefinitions.Add("_LIBCPP_HAS_NO_RTTI=1");
        }

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            PublicDefinitions.Add("EPROSIMA_ALL_DYN_LINK=1");
            // Either of these supposedly silence a build warning. But they do not actually work.
            // https://developercommunity.visualstudio.com/t/unable-to-remove-warning-for-cxx17-deprecated-feat/273815
            // PublicDefinitions.Add("_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS=1");
            // PublicDefinitions.Add("_SILENCE_CXX17_CODECVT_HEADER_DEPRECATION_WARNING=1");
            PrivateDefinitions.Add("ROSIDL_TYPESUPPORT_CPP_BUILDING_DLL=1");
            PrivateDefinitions.Add("ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_BUILDING_DLL=1");
        }

        PublicDefinitions.Add("RCLCPP_INTRA_PROCESS_DISABLED=1");

        // rclcpp and all modules that depend on it must enable exceptions.
        bEnableExceptions = true;

        AddEngineThirdPartyPrivateStaticDependencies(Target, "OpenSSL");
        AddEngineThirdPartyPrivateStaticDependencies(Target, "zlib");
        AddEngineThirdPartyPrivateStaticDependencies(Target, "UElibPNG");
        AddEngineThirdPartyPrivateDynamicDependencies(Target, "Python3");
    }
}
