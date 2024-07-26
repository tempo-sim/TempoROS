// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class TempoROS : ModuleRules
{
	public TempoROS(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				// Unreal
				"Core",
				// Tempo
				"rclcpp",
			}
			);
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				// Unreal
				"CoreUObject",
				"DeveloperSettings",
				"Engine",
				"Slate",
				"SlateCore",
			}
			);
		
		if (Target.bBuildEditor)
		{
			PrivateDependencyModuleNames.Add("HotReload");
		}

		// All modules that depend on rclcpp must enable exceptions.
		bEnableExceptions = true;
	}
}
