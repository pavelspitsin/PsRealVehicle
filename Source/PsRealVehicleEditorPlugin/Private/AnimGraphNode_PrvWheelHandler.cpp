// Copyright 2016 Pushkin Studio. All Rights Reserved.

#include "AnimGraphNode_PrvWheelHandler.h"

#include "AnimGraphDefinitions.h"
#include "CompilerResultsLog.h"
#include "Kismet2/BlueprintEditorUtils.h"

/////////////////////////////////////////////////////
// UAnimGraphNode_PrvWheelHandler

#define LOCTEXT_NAMESPACE "A3Nodes"

UAnimGraphNode_PrvWheelHandler::UAnimGraphNode_PrvWheelHandler(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

FText UAnimGraphNode_PrvWheelHandler::GetControllerDescription() const
{
	return LOCTEXT("AnimGraphNode_PrvWheelHandler", "Wheel Handler for PrvVehicle");
}

FText UAnimGraphNode_PrvWheelHandler::GetTooltipText() const
{
	return LOCTEXT("AnimGraphNode_PrvWheelHandler_Tooltip", "This alters the wheel transform based on set up in Wheeled Vehicle. This only works when the owner is WheeledVehicle.");
}

FText UAnimGraphNode_PrvWheelHandler::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	FText NodeTitle;
	if (TitleType == ENodeTitleType::ListView || TitleType == ENodeTitleType::MenuTitle)
	{
		NodeTitle = GetControllerDescription();
	}
	else
	{
		// we don't have any run-time information, so it's limited to print
		// anymore than what it is it would be nice to print more data such as
		// name of bones for wheels, but it's not available in Persona
		NodeTitle = FText(LOCTEXT("AnimGraphNode_PrvWheelHandler_Title", "Prv Wheel Handler"));
	}
	return NodeTitle;
}

void UAnimGraphNode_PrvWheelHandler::ValidateAnimNodePostCompile(class FCompilerResultsLog& MessageLog, class UAnimBlueprintGeneratedClass* CompiledClass, int32 CompiledNodeIndex)
{
	// @FIXME
	// we only support vehicle anim instance
	/*if ( CompiledClass->IsChildOf(UVehicleAnimInstance::StaticClass())  == false )
	{
		MessageLog.Error(TEXT("@@ is only allowwed in VehicleAnimInstance. If this is for vehicle, please change parent to be VehicleAnimInstancen (Reparent Class)."), this);
	}*/
}

bool UAnimGraphNode_PrvWheelHandler::IsCompatibleWithGraph(const UEdGraph* TargetGraph) const
{
	UBlueprint* Blueprint = FBlueprintEditorUtils::FindBlueprintForGraph(TargetGraph);
	return (Blueprint != nullptr) /* && Blueprint->ParentClass->IsChildOf<UVehicleAnimInstance>()*/ && Super::IsCompatibleWithGraph(TargetGraph);
}

#undef LOCTEXT_NAMESPACE
