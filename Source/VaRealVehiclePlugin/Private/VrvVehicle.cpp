// Copyright 2016 Vladimir Alyamkin. All Rights Reserved.

#include "VrvPlugin.h"

#include "DisplayDebugHelpers.h"

FName AVrvVehicle::VehicleMeshComponentName(TEXT("VehicleMesh"));
FName AVrvVehicle::VehicleMovementComponentName(TEXT("VehicleMovementComp"));

AVrvVehicle::AVrvVehicle(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	Mesh = CreateDefaultSubobject<USkeletalMeshComponent>(VehicleMeshComponentName);
	Mesh->SetCollisionProfileName(UCollisionProfile::Vehicle_ProfileName);
	Mesh->BodyInstance.bSimulatePhysics = true;
	Mesh->BodyInstance.bNotifyRigidBodyCollision = true;
	Mesh->BodyInstance.bUseCCD = true;
	Mesh->bBlendPhysics = true;
	Mesh->bGenerateOverlapEvents = true;
	Mesh->SetCanEverAffectNavigation(false);
	RootComponent = Mesh;

	// Construct advanced movement comp
	VehicleMovement = CreateDefaultSubobject<UVrvVehicleMovementComponent>(VehicleMovementComponentName);
	VehicleMovement->SetIsReplicated(true);		// Enable replication by default
	VehicleMovement->UpdatedComponent = GetMesh();
}


//////////////////////////////////////////////////////////////////////////
// Input handlers

void AVrvVehicle::MoveForward(float Val)
{
	GetVehicleMovementComponent()->SetThrottleInput(Val);
}

void AVrvVehicle::MoveRight(float Val)
{
	GetVehicleMovementComponent()->SetSteeringInput(Val);
}


//////////////////////////////////////////////////////////////////////////
// Debug

void AVrvVehicle::DisplayDebug(UCanvas* Canvas, const FDebugDisplayInfo& DebugDisplay, float& YL, float& YPos)
{
	static FName NAME_Vehicle = FName(TEXT("Vehicle"));

	Super::DisplayDebug(Canvas, DebugDisplay, YL, YPos);

	if (DebugDisplay.IsDisplayOn(NAME_Vehicle))
	{
		GetVehicleMovementComponent()->DrawDebug(Canvas, YL, YPos);
	}
}
