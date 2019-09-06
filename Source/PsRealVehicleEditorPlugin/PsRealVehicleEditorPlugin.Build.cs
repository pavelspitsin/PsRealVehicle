// Copyright 2016 Pushkin Studio. All Rights Reserved.

using System.IO;

namespace UnrealBuildTool.Rules
{
	public class PsRealVehicleEditorPlugin : ModuleRules
	{
		public PsRealVehicleEditorPlugin(ReadOnlyTargetRules Target) : base(Target)
		{
            PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

            PrivateIncludePaths.AddRange(
				new string[] {
					"PsRealVehicleEditorPlugin/Private",
					// ... add other private include paths required here ...
				});

			PublicDependencyModuleNames.AddRange(
				new string[]
				{
					"Core",
					"CoreUObject",
					"Engine",
					"PsRealVehiclePlugin"
					// ... add other public dependencies that you statically link with here ...
				});

			PrivateDependencyModuleNames.AddRange(
				new string[]
				{
					"AnimGraph",
					"BlueprintGraph",
					"AnimGraphRuntime",
					"UnrealEd"     // for FCompilerResultsLog
				});
		}
	}
}