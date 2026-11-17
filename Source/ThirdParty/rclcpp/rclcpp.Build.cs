// Copyright Tempo Simulation, LLC. All Rights Reserved.

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using Microsoft.Extensions.Logging;
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

    // A versioned shared library comes as a chain of names: libX.dylib -> libX.N.dylib -> libX.N.M.P.dylib
    // (libX.so -> libX.so.N -> libX.so.N.M.P on Linux). Staging copies each symlink as a regular file, so
    // staging every name would ship each such library two or three times over. The loader only ever asks
    // for one of them - the install name (Mac) or SONAME (Linux) recorded in the library itself - so read
    // that out of the file and keep just the name that matches it, and staging yields exactly one file per
    // library. A library whose recorded name cannot be read, or matches none of its names, is kept whole,
    // with a warning.
    //
    // This deliberately only drops names, never remaps one: RuntimeDependencies.Add(Path, SourcePath)
    // copies SourcePath to Path at build time, which here would write through a symlink in the source tree.
    private static IEnumerable<string> OneNamePerLibrary(IEnumerable<string> paths, ILogger logger)
    {
        List<string> all = paths.Select(p => Path.GetFullPath(p)).Distinct().ToList();

        Dictionary<string, string> linkTargets = new Dictionary<string, string>();
        foreach (string p in all)
        {
            string target = new FileInfo(p).LinkTarget;
            if (target != null)
            {
                linkTargets[p] = Path.GetFullPath(Path.Combine(Path.GetDirectoryName(p), target));
            }
        }

        string Resolve(string p)
        {
            HashSet<string> seen = new HashSet<string>();
            while (linkTargets.TryGetValue(p, out string next) && seen.Add(p))
            {
                p = next;
            }
            return p;
        }

        foreach (IGrouping<string, string> library in all.GroupBy(Resolve))
        {
            if (library.Count() > 1)
            {
                string loaderName = ReadLoaderName(library.Key);
                List<string> matches = library.Where(p => Path.GetFileName(p) == loaderName).ToList();
                if (matches.Count == 1)
                {
                    yield return matches[0];
                    continue;
                }

                if (loaderName == null)
                {
                    logger.LogWarning("rclcpp: staging all {Count} names of {Library}: could not read its install name / SONAME",
                        library.Count(), library.Key);
                }
                else
                {
                    logger.LogWarning("rclcpp: staging all {Count} names of {Library}: its install name / SONAME '{LoaderName}' matches {Matches} of them",
                        library.Count(), library.Key, loaderName, matches.Count);
                }
            }

            foreach (string p in library)
            {
                yield return p;
            }
        }
    }

    // The file name the loader asks for when something links against this library: the last component
    // of a Mach-O's LC_ID_DYLIB install name, or an ELF's DT_SONAME. Null if it has none or is unreadable.
    // Read straight from the file, rather than via otool/readelf, so it works when cross-compiling too.
    private static string ReadLoaderName(string path)
    {
        try
        {
            using FileStream stream = File.OpenRead(path);
            using BinaryReader reader = new BinaryReader(stream);

            string ReadString(long offset)
            {
                stream.Seek(offset, SeekOrigin.Begin);
                List<byte> bytes = new List<byte>();
                for (byte b = reader.ReadByte(); b != 0 && bytes.Count < 1024; b = reader.ReadByte())
                {
                    bytes.Add(b);
                }
                return Path.GetFileName(System.Text.Encoding.UTF8.GetString(bytes.ToArray()));
            }

            uint magic = reader.ReadUInt32();

            if (magic == 0x464C457F) // "\x7FELF"
            {
                if (reader.ReadByte() != 2 || reader.ReadByte() != 1) // 64-bit, little-endian
                {
                    return null;
                }
                // Go through the program headers, as the dynamic loader does, rather than the section
                // headers, which a library may have stripped. PT_DYNAMIC locates the dynamic table, whose
                // DT_STRTAB is a virtual address; the PT_LOAD segments map that back to a file offset.
                stream.Seek(0x20, SeekOrigin.Begin);
                long programHeaders = reader.ReadInt64();
                stream.Seek(0x36, SeekOrigin.Begin);
                int programHeaderSize = reader.ReadUInt16();
                int programHeaderCount = reader.ReadUInt16();

                List<(long Offset, long Address, long Size)> loads = new List<(long, long, long)>();
                long dynamicOffset = -1;
                long dynamicSize = 0;
                for (int i = 0; i < programHeaderCount; i++)
                {
                    stream.Seek(programHeaders + (long)i * programHeaderSize, SeekOrigin.Begin);
                    uint type = reader.ReadUInt32();
                    reader.ReadUInt32(); // p_flags
                    long offset = reader.ReadInt64();
                    long address = reader.ReadInt64();
                    reader.ReadInt64(); // p_paddr
                    long fileSize = reader.ReadInt64();
                    if (type == 1) // PT_LOAD
                    {
                        loads.Add((offset, address, fileSize));
                    }
                    else if (type == 2) // PT_DYNAMIC
                    {
                        dynamicOffset = offset;
                        dynamicSize = fileSize;
                    }
                }
                if (dynamicOffset < 0)
                {
                    return null;
                }

                long soname = -1;
                long stringTable = -1;
                for (long entry = dynamicOffset; entry + 16 <= dynamicOffset + dynamicSize; entry += 16)
                {
                    stream.Seek(entry, SeekOrigin.Begin);
                    long tag = reader.ReadInt64();
                    long value = reader.ReadInt64();
                    if (tag == 0) // DT_NULL
                    {
                        break;
                    }
                    if (tag == 14) // DT_SONAME: offset into the string table
                    {
                        soname = value;
                    }
                    else if (tag == 5) // DT_STRTAB: virtual address of the string table
                    {
                        stringTable = value;
                    }
                }
                if (soname < 0 || stringTable < 0)
                {
                    return null;
                }
                foreach ((long Offset, long Address, long Size) load in loads)
                {
                    if (stringTable >= load.Address && stringTable < load.Address + load.Size)
                    {
                        return ReadString(load.Offset + (stringTable - load.Address) + soname);
                    }
                }
                return null;
            }

            // A universal binary holds one Mach-O per architecture, all with the same install name, behind
            // a big-endian header. Take the first.
            long start = 0;
            if (magic == 0xBEBAFECA || magic == 0xBFBAFECA) // FAT_MAGIC, FAT_MAGIC_64
            {
                stream.Seek(16, SeekOrigin.Begin);
                byte[] offset = reader.ReadBytes(magic == 0xBEBAFECA ? 4 : 8);
                foreach (byte b in offset)
                {
                    start = (start << 8) | b;
                }
                stream.Seek(start, SeekOrigin.Begin);
                magic = reader.ReadUInt32();
            }
            if (magic != 0xFEEDFACF) // MH_MAGIC_64
            {
                return null;
            }
            stream.Seek(start + 16, SeekOrigin.Begin);
            uint commandCount = reader.ReadUInt32();
            long command = start + 32;
            for (uint i = 0; i < commandCount; i++)
            {
                stream.Seek(command, SeekOrigin.Begin);
                uint id = reader.ReadUInt32();
                uint size = reader.ReadUInt32();
                if (id == 0xD) // LC_ID_DYLIB
                {
                    return ReadString(command + reader.ReadUInt32());
                }
                if (size == 0)
                {
                    return null;
                }
                command += size;
            }
            return null;
        }
        catch (Exception)
        {
            return null;
        }
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
            // Link only the top-level ROS libraries. Nested dylibs are Python packages' private
            // dependencies (e.g. numpy/.dylibs), whose install names are delocate's /DLC/...
            // placeholders: linking them records those absolute, nonexistent paths in the module,
            // which then fails to load. They are still staged below for the Python bindings.
            LibraryPaths.AddRange(Directory.EnumerateFiles(Path.Combine(ModuleDirectory, "Libraries", "Mac"), "*.dylib", SearchOption.TopDirectoryOnly));
            RuntimeLibraryPaths.AddRange(OneNamePerLibrary(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Mac"), "dylib"), Target.Logger));
            RuntimeLibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Mac", "share"), "*"));
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            LibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Linux"), "so"));
            RuntimeLibraryPaths.AddRange(OneNamePerLibrary(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Linux"), "so*"), Target.Logger));
            RuntimeLibraryPaths.AddRange(FindFilesInDirectory(Path.Combine(ModuleDirectory, "Libraries", "Linux", "share"), "*"));
        }
        else
        {
            Target.Logger.LogWarning("rclcpp: unsupported target platform {Platform}", Target.Platform);
        }

        return new ModuleDepPaths(HeaderPaths.ToArray(), LibraryPaths.ToArray(), RuntimeLibraryPaths.ToArray());
    }
    
    public rclcpp(ReadOnlyTargetRules Target) : base(Target)
    {
        Type = ModuleType.External;
        
        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Public"));
        
        ModuleDepPaths moduleDepPaths = GatherDeps();
        // The vendored ROS tree goes in as system includes (-isystem / -external:I) so its headers
        // are exempt from Unreal's warnings-as-errors. These are upstream sources we do not own and
        // cannot restyle, and they are not written to Unreal's warning settings: Fast-CDR's Cdr.h,
        // for one, tests `#if !__APPLE__ && !__FreeBSD__ && !__VXWORKS__` on macros that are simply
        // undefined on Linux, which -Wundef -Werror rejects. Suppressing warnings per-macro in
        // Patches/ would be a treadmill across ~226 packages. Real errors still surface normally.
        PublicSystemIncludePaths.AddRange(moduleDepPaths.HeaderPaths);
        PublicAdditionalLibraries.AddRange(moduleDepPaths.LibraryPaths);

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            PublicRuntimeLibraryPaths.Add(Path.Combine(ModuleDirectory, "Binaries", "Windows"));
        }

        // In a monolithic build (i.e. a packaged game) TempoROS is linked into the game executable itself, so the
        // Windows loader binds rclcpp's imports before any of our code - TempoROSBootstrap included - gets to run.
        // No module startup hook can ever be early enough. The executable's own directory is the one place the
        // loader searches unaided, so stage the dlls there. Every later load by name (the rmw implementation,
        // typesupport, and image_transport's pluginlib plugins) resolves from that same directory too.
        // The share directory is deliberately left where it is, since AMENT_PREFIX_PATH points at its parent.
        bool bStageDllsNextToExecutable = Target.Platform == UnrealTargetPlatform.Win64 &&
                                          Target.LinkType == TargetLinkType.Monolithic;

        foreach (string libraryPath in moduleDepPaths.RuntimeLibraryPaths)
        {
            if (bStageDllsNextToExecutable && libraryPath.EndsWith(".dll", StringComparison.OrdinalIgnoreCase))
            {
                RuntimeDependencies.Add(Path.Combine("$(TargetOutputDir)", Path.GetFileName(libraryPath)),
                    libraryPath, StagedFileType.NonUFS);
            }
            else
            {
                RuntimeDependencies.Add(libraryPath);
            }
        }

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "Core"
            }
            );

        // We need _LIBCPP_HAS_NO_RTTI to be set to avoid using RTTI in rclcpp, but it is already
        // set in UE 5.6 (but not 5.7) in v25_clang-18.1.0-rockylinux8/x86_64-unknown-linux-gnu/include/c++/v1/__config
        if (Target.Version.MajorVersion == 5 && Target.Version.MinorVersion != 6 && Target.Platform == UnrealTargetPlatform.Linux)
        {
            PublicDefinitions.Add("_LIBCPP_HAS_NO_RTTI=1");
        }

        // Xcode 26+ (Apple clang-1700.3+) no longer automatically defines _LIBCPP_HAS_NO_RTTI in
        // libc++'s __config when compiling with -fno-rtti. Older Xcode versions still define it,
        // so we check the clang build version to avoid a -Werror,-Wmacro-redefined error.
        // Note: clang must already be installed for UBT to reach this point.
        if (Target.Platform == UnrealTargetPlatform.Mac)
        {
            bool NeedNoRTTI = false;
            try
            {
                var Psi = new System.Diagnostics.ProcessStartInfo("clang", "--version")
                {
                    RedirectStandardOutput = true,
                    UseShellExecute = false
                };
                using var Proc = System.Diagnostics.Process.Start(Psi);
                string Output = Proc.StandardOutput.ReadToEnd();
                Proc.WaitForExit();
                // Output contains e.g. "(clang-1700.6.3.2)" - gate on MAJOR.MINOR >= 1700.3
                var Match = Regex.Match(Output, @"clang-(\d+)\.(\d+)\.");
                if (Match.Success
                    && int.TryParse(Match.Groups[1].Value, out int Major)
                    && int.TryParse(Match.Groups[2].Value, out int Minor)
                    && (Major > 1700 || (Major == 1700 && Minor >= 3)))
                {
                    NeedNoRTTI = true;
                }
                else if (!Match.Success)
                {
                    Target.Logger.LogWarning("rclcpp: could not parse clang version from: {Output}", Output);
                }
            }
            catch (Exception Ex)
            {
                Target.Logger.LogWarning("rclcpp: failed to query clang version: {Message}", Ex.Message);
            }
            if (NeedNoRTTI)
            {
                PublicDefinitions.Add("_LIBCPP_HAS_NO_RTTI=1");
            }
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
