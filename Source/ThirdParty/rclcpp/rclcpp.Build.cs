// Copyright Tempo Simulation, LLC. All Rights Reserved.

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
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
            LibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Windows"), "dll"));
            RuntimeLibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Windows"), "dll"));
        }
        else if (Target.Platform == UnrealTargetPlatform.Mac)
        {
            LibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Mac"), "dylib"));
            RuntimeLibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Mac"), "dylib"));
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            LibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Linux"), "so"));
            RuntimeLibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Linux"), "so*"));
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

        PublicDefinitions.Add("RCLCPP_INTRA_PROCESS_DISABLED=1");

        bEnableExceptions = true;
        
        AddEngineThirdPartyPrivateStaticDependencies(Target, "OpenSSL");
        AddEngineThirdPartyPrivateStaticDependencies(Target, "zlib");
        AddEngineThirdPartyPrivateStaticDependencies(Target, "UElibPNG");
        AddEngineThirdPartyPrivateDynamicDependencies(Target, "Python3");
    }
}
