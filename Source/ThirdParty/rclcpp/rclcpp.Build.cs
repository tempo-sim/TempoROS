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
        public readonly string[] GeneratedLibraryPaths;

        public ModuleDepPaths(string[] headerPaths, string[] libraryPaths, string[] generatedLibraryPaths)
        {
            HeaderPaths = headerPaths;
            LibraryPaths = libraryPaths;
            GeneratedLibraryPaths = generatedLibraryPaths;
        }
    }

    private IEnumerable<string> FindFilesInDirectory(string dir, string suffix = "")
    {
        return Directory.EnumerateFiles(dir, "*." + suffix, SearchOption.AllDirectories).Where(file => !Path.GetDirectoryName(file).Contains("Python3.framework"));
    }

    public ModuleDepPaths GatherDeps()
    {
        List<string> HeaderPaths = new List<string>();
        List<string> LibraryPaths = new List<string>();
        List<string> GeneratedLibraryPaths = new List<string>();
        
        HeaderPaths.Add(Path.Combine(ModuleDirectory, "Includes"));

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            LibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Windows"), "dll"));
        }
        else if (Target.Platform == UnrealTargetPlatform.Mac)
        {
            LibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Mac"), "dylib"));
            // GeneratedLibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Generated"), "dylib"));
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            LibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Linux"), "so"));
        }
        else
        {
            Console.WriteLine("Unsupported target platform for module rclcpp.");
        }

        return new ModuleDepPaths(HeaderPaths.ToArray(), LibraryPaths.ToArray(), GeneratedLibraryPaths.ToArray());
    }
    
    public rclcpp(ReadOnlyTargetRules Target) : base(Target)
    {
        Type = ModuleType.External;
        
        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Public"));
        
        ModuleDepPaths moduleDepPaths = GatherDeps();
        PublicIncludePaths.AddRange(moduleDepPaths.HeaderPaths);
        PublicAdditionalLibraries.AddRange(moduleDepPaths.LibraryPaths);
        foreach (string libraryPath in moduleDepPaths.LibraryPaths)
        {
            RuntimeDependencies.Add(libraryPath);
        }
        // PublicDelayLoadDLLs.AddRange(moduleDepPaths.GeneratedLibraryPaths);
        // PublicRuntimeLibraryPaths.Add(Path.Combine(ModuleDirectory, "Generated"));
        // foreach (string libraryPath in moduleDepPaths.GeneratedLibraryPaths)
        // {
        //     RuntimeDependencies.Add(libraryPath);
        // }
        
        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "Core"
            }
            );
        
        PublicDefinitions.Add("__STDC_VERSION__=202002L");
        PublicDefinitions.Add("RCLCPP_INTRA_PROCESS_DISABLED=1");
    }
}
