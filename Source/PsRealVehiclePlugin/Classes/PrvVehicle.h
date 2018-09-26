// Copyright 2016 Pushkin Studio. All Rights Reserved.

#pragma once

#include "GameFramework/Pawn.h"

#include "PrvVehicle.generated.h"

/**
 * Vehicle class with implemented custom physics
 */
UCLASS(abstract, config = Game, Blueprintable, BlueprintType)
class PSREALVEHICLEPLUGIN_API APrvVehicle : public APawn
{
	GENERATED_UCLASS_BODY()


	//////////////////////////////////////////////////////////////////////////
	// Input handlers

public:
	/** Move forward/back */
	UFUNCTION(BlueprintCallable, Category = "PsRealVehicle|Actions")
	virtual void MoveForward(float Val);

	/** Strafe right/left */	
	UFUNCTION(BlueprintCallable, Category = "PsRealVehicle|Actions")
	virtual void MoveRight(float Val);


	//////////////////////////////////////////////////////////////////////////
	// Movement physics replication

public:
	/** Custom method to call custom handler in Movement component */
	virtual void PostNetReceivePhysicState() override;


	//////////////////////////////////////////////////////////////////////////
	// Vehicle setup

private:
	/**  The main skeletal mesh associated with this Vehicle */
	UPROPERTY(Category = Vehicle, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	class USkeletalMeshComponent* Mesh;

	/** Vehicle simulation component */
	UPROPERTY(Category = AdvVehicle, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	class UPrvVehicleMovementComponent* VehicleMovement;

public:
	/** Name of the MeshComponent. Use this name if you want to prevent creation of the component (with ObjectInitializer.DoNotCreateDefaultSubobject). */
	static FName VehicleMeshComponentName;

	/** Name of the VehicleMovement. Use this name if you want to use a different class (with ObjectInitializer.SetDefaultSubobjectClass). */
	static FName VehicleMovementComponentName;

	/** Returns Mesh subobject **/
	FORCEINLINE USkeletalMeshComponent* GetMesh() const { return Mesh; }

	/** Util to get the wheeled vehicle movement component */
	FORCEINLINE UPrvVehicleMovementComponent* GetVehicleMovementComponent() const { return VehicleMovement; }
	FORCEINLINE UPrvVehicleMovementComponent* GetVehicleMovement() const { return VehicleMovement; }

	//~ Begin AActor Interface
	virtual void DisplayDebug(class UCanvas* Canvas, const FDebugDisplayInfo& DebugDisplay, float& YL, float& YPos) override;
	//~ End Actor Interface

};
