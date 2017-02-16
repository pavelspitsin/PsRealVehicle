// Copyright 2016 Pushkin Studio. All Rights Reserved.

#include "PrvPlugin.h"

#include "Engine/CollisionProfile.h"
#include "DisplayDebugHelpers.h"

FName APrvVehicle::VehicleMeshComponentName(TEXT("VehicleMesh"));
FName APrvVehicle::VehicleMovementComponentName(TEXT("VehicleMovementComp"));

APrvVehicle::APrvVehicle(const FObjectInitializer& ObjectInitializer)
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
	VehicleMovement = CreateDefaultSubobject<UPrvVehicleMovementComponent>(VehicleMovementComponentName);
	VehicleMovement->SetIsReplicated(true);		// Enable replication by default
	VehicleMovement->UpdatedComponent = GetMesh();
}


//////////////////////////////////////////////////////////////////////////
// Input handlers

void APrvVehicle::MoveForward(float Val)
{
	GetVehicleMovementComponent()->SetThrottleInput(Val);
}

void APrvVehicle::MoveRight(float Val)
{
	GetVehicleMovementComponent()->SetSteeringInput(Val);
}


//////////////////////////////////////////////////////////////////////////
// Debug

void APrvVehicle::DisplayDebug(UCanvas* Canvas, const FDebugDisplayInfo& DebugDisplay, float& YL, float& YPos)
{
	static FName NAME_Vehicle = FName(TEXT("Vehicle"));

	Super::DisplayDebug(Canvas, DebugDisplay, YL, YPos);

	if (DebugDisplay.IsDisplayOn(NAME_Vehicle) || GetVehicleMovementComponent()->IsDebug())
	{
		GetVehicleMovementComponent()->DrawDebug(Canvas, YL, YPos);
	}
}
