using System;
using System.IO;
using AutomationTool;
using UnrealBuildTool;
using Microsoft.Extensions.Logging;
using EpicGames.Core;

public static class FileUtil
{
    public static void CopyPreservingSymlinks(string sourcePath, string destinationPath, ILogger Logger)
    {
        if (File.Exists(destinationPath))
        {
            return;
        }
        if (File.Exists(sourcePath))
        {
            var fileInfo = new FileInfo(sourcePath);
            
            // TODO: This won't work on Windows. Does it need to?
            if (!OperatingSystem.IsWindows() && fileInfo.LinkTarget != null)
            {
                // It's a symlink. First follow the link and copy the targeted file (recursively), then copy the link.
                CopyPreservingSymlinks(Path.Combine(Path.GetDirectoryName(sourcePath), fileInfo.LinkTarget), Path.Combine(Path.GetDirectoryName(destinationPath), Path.GetFileName(fileInfo.LinkTarget)), Logger);
                FileReference SourceFile = new FileReference(sourcePath);
                FileReference TargetFile = new FileReference(destinationPath);
                CommandUtils.CopyFileOrSymlink(SourceFile, TargetFile, CommandUtils.SymlinkMode.Retain);
            }
            else
            {
                // It's a regular file
                File.Copy(sourcePath, destinationPath, true);
            }
        }
        else
        {
            throw new FileNotFoundException("Source file not found", sourcePath);
        }
    }
}

[CustomStageCopyHandlerAttribute("TempoROSCopyHandler")]
public class TempoROSCopyHandler : CustomStageCopyHandler
{
    /// <summary>
    /// Called when copying files to the staging directory.
    /// </summary>
    /// <returns>true if the file copy can be handled.</returns>
    public override bool CanCopyFile(string SourceName)
    {
        return true;
    }

    /// <summary>
    /// Called when copying files to the staging directory.
    /// </summary>
    /// <returns>true if the file was handled. false to fallback on the default stage copy implementation</returns>
    public override bool StageFile(ILogger Logger, string SourceName, string TargetName)
    {
        if(File.Exists(SourceName))
        { 
            FileUtils.CopyPreservingSymlinks(SourceName, TargetName, Logger);
        }
        else
        {
            Logger.LogInformation("Skip copying file {SourceName} because it doesn't exist.", SourceName);
        }
        
        return true;
    }
}
