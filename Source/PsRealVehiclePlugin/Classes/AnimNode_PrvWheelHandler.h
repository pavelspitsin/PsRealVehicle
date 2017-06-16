// Copyright 2016 Pushkin Studio. All Rights Reserved.

#pragma once

#include "AnimNode_SkeletalControlBase.h"
#include "AnimNode_PrvWheelHandler.generated.h"

struct FPrvWheelSimulator
{
	int32					WheelIndex;
	FBoneReference			BoneReference;
	FBoneReference			SuspReference;
	FRotator				RotOffset;
	FVector					LocOffset;
};

/**
 *	Simple controller that replaces or adds to the translation/rotation of a single bone.
 */
USTRUCT()
struct PSREALVEHICLEPLUGIN_API FAnimNode_PrvWheelHandler : public FAnimNode_SkeletalControlBase
{
	GENERATED_USTRUCT_BODY()

	/** Current Asset being played **/
	UPROPERTY(transient)
	class UPrvVehicleMovementComponent* VehicleSimComponent;

	TArray<FPrvWheelSimulator> WheelSimulators;

	FAnimNode_PrvWheelHandler();

	// FAnimNode_Base interface
	virtual void GatherDebugData(FNodeDebugData& DebugData) override;
	// End of FAnimNode_Base interface

	// FAnimNode_SkeletalControlBase interface
	virtual void EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;
	virtual void UpdateInternal(const FAnimationUpdateContext& Context) override;
	virtual void Initialize(const FAnimationInitializeContext& Context) override;
	virtual bool CanUpdateInWorkerThread() const override;
	// End of FAnimNode_SkeletalControlBase interface

private:
	// FAnimNode_SkeletalControlBase interface
	virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;
	// End of FAnimNode_SkeletalControlBase interface
};
