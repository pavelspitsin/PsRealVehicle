// Copyright 2016 Pushkin Studio. All Rights Reserved.

#include "PrvPlugin.h"

#include "AnimationRuntime.h"
#include "Animation/AnimInstanceProxy.h"

#include "Classes/PrvVehicle.h"
#include "Classes/PrvVehicleMovementComponent.h"


/////////////////////////////////////////////////////
// FAnimNode_PrvWheelHandler

FAnimNode_PrvWheelHandler::FAnimNode_PrvWheelHandler()
{
}

void FAnimNode_PrvWheelHandler::GatherDebugData(FNodeDebugData& DebugData)
{
	FString DebugLine = DebugData.GetNodeName(this);

	DebugLine += "(";
	AddDebugNodeData(DebugLine);
	DebugLine += ")";

	DebugData.AddDebugItem(DebugLine);
	for(const auto & WheelSim : WheelSimulators)
	{
		if (WheelSim.BoneReference.BoneIndex != INDEX_NONE)
		{
			DebugLine = FString::Printf(TEXT(" [Wheel Index : %d] Bone: %s , Rotation Offset : %s, Location Offset : %s"), 
				WheelSim.WheelIndex, *WheelSim.BoneReference.BoneName.ToString(), *WheelSim.RotOffset.ToString(), *WheelSim.LocOffset.ToString());
		}
		else
		{
			DebugLine = FString::Printf(TEXT(" [Wheel Index : %d] Bone: %s (invalid bone)"),
				WheelSim.WheelIndex, *WheelSim.BoneReference.BoneName.ToString());
		}

		DebugData.AddDebugItem(DebugLine);
	}

	ComponentPose.GatherDebugData(DebugData);
}

void FAnimNode_PrvWheelHandler::EvaluateBoneTransforms(USkeletalMeshComponent* SkelComp, FCSPose<FCompactPose>& MeshBases, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(OutBoneTransforms.Num() == 0);

	const FBoneContainer& BoneContainer = MeshBases.GetPose().GetBoneContainer();
	for(const auto & WheelSim : WheelSimulators)
	{
		if (WheelSim.BoneReference.IsValid(BoneContainer))
		{
			FCompactPoseBoneIndex WheelSimBoneIndex = WheelSim.BoneReference.GetCompactPoseIndex(BoneContainer);

			// the way we apply transform is same as FMatrix or FTransform
			// we apply scale first, and rotation, and translation
			// if you'd like to translate first, you'll need two nodes that first node does translate and second nodes to rotate. 
			FTransform NewBoneTM = MeshBases.GetComponentSpaceTransform(WheelSimBoneIndex);

			FAnimationRuntime::ConvertCSTransformToBoneSpace(SkelComp, MeshBases, NewBoneTM, WheelSimBoneIndex, BCS_ComponentSpace);
			
			// Apply rotation offset
			const FQuat BoneQuat(WheelSim.RotOffset);
			NewBoneTM.SetRotation(BoneQuat * NewBoneTM.GetRotation());

			// Apply loc offset
			NewBoneTM.AddToTranslation(WheelSim.LocOffset);

			// Convert back to Component Space.
			FAnimationRuntime::ConvertBoneSpaceTransformToCS(SkelComp, MeshBases, NewBoneTM, WheelSimBoneIndex, BCS_ComponentSpace);

			// add back to it
			OutBoneTransforms.Add(FBoneTransform(WheelSimBoneIndex, NewBoneTM));
		}
	}
}

bool FAnimNode_PrvWheelHandler::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) 
{
	// if both bones are valid
	for(const auto & WheelSim : WheelSimulators)
	{
		// if one of them is valid
		if (WheelSim.BoneReference.IsValid(RequiredBones) == true)
		{
			return true;
		}
	}

	return false;
}

void FAnimNode_PrvWheelHandler::InitializeBoneReferences(const FBoneContainer& RequiredBones) 
{
	for (auto & WheelSim : WheelSimulators)
	{
		WheelSim.BoneReference.Initialize(RequiredBones);
	}

	// sort by bone indices
	WheelSimulators.Sort([](FPrvWheelSimulator L, FPrvWheelSimulator R) { return L.BoneReference.BoneIndex < R.BoneReference.BoneIndex; });
}

void FAnimNode_PrvWheelHandler::UpdateInternal(const FAnimationUpdateContext& Context)
{
	if(VehicleSimComponent)
	{
		for(auto & WheelSim : WheelSimulators)
		{
			if (VehicleSimComponent->SuspensionData.IsValidIndex(WheelSim.WheelIndex))
			{
				const FSuspensionState& Wheel = VehicleSimComponent->SuspensionData[WheelSim.WheelIndex];

				if (Wheel.SuspensionInfo.bAnimateBoneRotation)
				{
					WheelSim.RotOffset.Pitch = Wheel.RotationAngle;
					WheelSim.RotOffset.Yaw = Wheel.SteeringAngle;
					WheelSim.RotOffset.Roll = 0.f;
				}

				if (Wheel.SuspensionInfo.bAnimateBoneOffset)
				{
					WheelSim.LocOffset.X = 0.f;
					WheelSim.LocOffset.Y = 0.f;
					WheelSim.LocOffset.Z = Wheel.SuspensionInfo.Length - Wheel.VisualLength;
				}
			}
		}
	}

	FAnimNode_SkeletalControlBase::UpdateInternal(Context);
}

void FAnimNode_PrvWheelHandler::Initialize(const FAnimationInitializeContext& Context)
{
	// TODO: only check vehicle anim instance
	// UPrvVehicleAnimInstance
	APrvVehicle* Vehicle = Cast<APrvVehicle> (Context.AnimInstanceProxy->GetSkelMeshComponent()->GetOwner());

	// we only support vehicle for this class
	if(Vehicle != nullptr)
	{
		VehicleSimComponent = Vehicle->GetVehicleMovementComponent();

		int32 NumOfwheels = VehicleSimComponent->SuspensionSetup.Num();
		if(NumOfwheels > 0)
		{
			WheelSimulators.Empty(NumOfwheels);
			WheelSimulators.AddZeroed(NumOfwheels);

			// now add wheel data
			for(int32 WheelIndex = 0; WheelIndex < WheelSimulators.Num(); ++WheelIndex)
			{
				FPrvWheelSimulator & WheelSim = WheelSimulators[WheelIndex];
				const FSuspensionInfo& WheelSetup = VehicleSimComponent->SuspensionSetup[WheelIndex];

				// set data
				WheelSim.WheelIndex = WheelIndex;
				WheelSim.BoneReference.BoneName = WheelSetup.BoneName;
				WheelSim.LocOffset = FVector::ZeroVector;
				WheelSim.RotOffset = FRotator::ZeroRotator;
			}
		}
	}

	FAnimNode_SkeletalControlBase::Initialize(Context);
}
