// Copyright Tempo Simulation, LLC. All Rights Reserved

using UnrealBuildTool;

public class TempoROSBootstrap : ModuleRules
{
    public TempoROSBootstrap(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
            }
        );

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "Projects"
            }
        );
    }
}
