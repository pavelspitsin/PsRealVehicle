// Copyright 2016 Vladimir Alyamkin. All Rights Reserved.

#pragma once

#include "VrvVehicleMovementComponent.generated.h"

USTRUCT(BlueprintType)
struct FSuspensionInfo
{
	GENERATED_USTRUCT_BODY()

	/**  */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly)
	bool bInheritWheelBoneTransform;

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Suspension, meta = (EditCondition = "bInheritWheelBoneTransform"))
	FName BoneName;

	/** Suspension location in Actor space */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, meta = (EditCondition = "!bInheritWheelBoneTransform"))
	FVector Location;

	/** Suspension rotation in Actor space */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, meta = (EditCondition = "!bInheritWheelBoneTransform"))
	FRotator Rotation;

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Suspension)
	bool bRightTrack;

	/** Maximum length */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Suspension)
	float Length;

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Suspension)
	float CollisionRadius;

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Suspension)
	float Stiffness;

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Suspension)
	float Damping;

	/** Defaults */
	FSuspensionInfo()
	{
		bInheritWheelBoneTransform = true;

		Length = 25.f;
		CollisionRadius = 36.f;
		Stiffness = 4000000.f;
		Damping = 4000.f;
	}
};

USTRUCT(BlueprintType)
struct FSuspensionState
{
	GENERATED_USTRUCT_BODY()

	/**  */
	UPROPERTY(BlueprintReadWrite)
	FSuspensionInfo SuspensionInfo;

	/**  */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly)
	float PreviousLength;

	/**  */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly)
	FVector SuspensionForce;

	/**  */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly)
	FVector WheelCollisionLocation;

	/**  */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly)
	FVector WheelCollisionNormal;

	/**  */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly)
	bool WheelTouchedGround;

	/**  */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly)
	TEnumAsByte<EPhysicalSurface> SurfaceType;

	/** Defaults */
	FSuspensionState()
	{
		WheelTouchedGround = false;
	}
};

USTRUCT(BlueprintType)
struct FGearInfo
{
	GENERATED_USTRUCT_BODY()

	/**  */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly)
	float Ratio;
};

USTRUCT(BlueprintType)
struct FTrackInfo
{
	GENERATED_USTRUCT_BODY()

	/**  */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float Input;

	/**  */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float TorqueTransfer;

	/**  */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float LinearVelocity;

	/**  */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float AngularVelocity;

	/**  */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float DriveTorque;

	/**  */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float FrictionTorque;

	/**  */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float RollingFrictionTorque;

	/**  */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float BrakeRatio;

	/**  */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	FVector DriveForce;
};


/**
 * Component that uses Torque and Force to move tracked vehicles
 */
UCLASS()
class VAREALVEHICLEPLUGIN_API UVrvVehicleMovementComponent : public UPawnMovementComponent
{
	GENERATED_UCLASS_BODY()


	//////////////////////////////////////////////////////////////////////////
	// Initialization
	
	/** */
	virtual void InitializeComponent() override;

	/** */
	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;


	//////////////////////////////////////////////////////////////////////////
	// Physics initialization

	void CalculateMOI();
	void InitSuspension();
	void InitGears();


	//////////////////////////////////////////////////////////////////////////
	// Physics simulation

	void UpdateThrottle(float DeltaTime);
	void UpdateTracksVelocity(float DeltaTime);
	void UpdateHullVelocity(float DeltaTime);
	void UpdateEngine(float DeltaTime);
	void UpdateDriveForce(float DeltaTime);
	void UpdateSuspension(float DeltaTime);

	float ApplyBrake(float DeltaTime, float AngularVelocity, float BrakeRatio);


	/////////////////////////////////////////////////////////////////////////
	// Vehicle setup

	/** Kg */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = VehicleSetup)
	float SprocketMass;
	/** Cm */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = VehicleSetup)
	float SprocketRadius;
	/** Kg */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = VehicleSetup)
	float TrackMass;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = VehicleSetup)
	TArray<FSuspensionInfo> SuspensionSetup;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = VehicleSetup)
	TArray<FGearInfo> GearSetup;

	bool bAutoGear;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = VehicleSetup)
	float BrakeForce;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = VehicleSetup)
	float DifferentialRatio;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = VehicleSetup)
	float TransmissionEfficiency;

	/** Torque (Nm) at a given RPM */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = VehicleSetup)
	FRuntimeFloatCurve EngineTorqueCurve;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = VehicleSetup)
	float EngineExtraPowerRatio;


	/////////////////////////////////////////////////////////////////////////
	// Movement cache

	float FinalMOI;

	TArray<FSuspensionState> SuspensionData;

	int32 NeutralGear;
	int32 CurrentGear;
	bool bReverseGear;

	FTrackInfo LeftTrack;
	FTrackInfo RightTrack;
	
	float RightTrackTorque;
	float LeftTrackTorque;

	float HullAngularVelocity;

	float EngineRPM;
	float EngineTorque;
	float DriveTorque;

	/** How many wheels are touched the ground */
	int32 ActiveFrictionPoints;


	//////////////////////////////////////////////////////////////////////////
	// Vehicle control

public:
	/** Set the user input for the vehicle throttle */
	UFUNCTION(BlueprintCallable, Category="VaRealVehicle|Components|VehicleMovement")
	void SetThrottleInput(float Throttle);
	
	/** Set the user input for the vehicle steering */
	UFUNCTION(BlueprintCallable, Category="VaRealVehicle|Components|VehicleMovement")
	void SetSteeringInput(float Steering);

	/** Set the user input for handbrake */
	UFUNCTION(BlueprintCallable, Category="VaRealVehicle|Components|VehicleMovement")
	void SetHandbrakeInput(bool bNewHandbrake);


	//////////////////////////////////////////////////////////////////////////
	// Vehicle stats

public:
	/** How fast the vehicle is moving forward */
	UFUNCTION(BlueprintCallable, Category="VaRealVehicle|Components|VehicleMovement")
	float GetForwardSpeed() const;

	/** Get current engine's rotation speed */
	UFUNCTION(BlueprintCallable, Category="VaRealVehicle|Components|VehicleMovement")
	float GetEngineRotationSpeed() const;

	/** Get current engine's max rotation speed */
	UFUNCTION(BlueprintCallable, Category="VaRealVehicle|Components|VehicleMovement")
	float GetEngineMaxRotationSpeed() const;


	//////////////////////////////////////////////////////////////////////////
	// Data access

protected:
	/** Get the mesh this vehicle is tied to */
	class USkinnedMeshComponent* GetMesh();

	/** Get current gear */
	int32 GetCurrentGear() const;

	/** Get current gearbox ratio */
	FGearInfo GetGearInfo(int32 GearNum) const;


	//////////////////////////////////////////////////////////////////////////
	// Debug

public:
	/** Draw debug text for the wheels and suspension */
	virtual void DrawDebug(UCanvas* Canvas, float& YL, float& YPos);

	/** Draw debug primitives for the wheels and suspension */
	virtual void DrawDebugLines();

	/** */
	UFUNCTION(BlueprintCallable, Category = "VaRealVehicle|Components|VehicleMovement")
	void ShowDebug(bool bEnableDebug) { bShowDebug = bEnableDebug; }

	/** */
	UFUNCTION(BlueprintCallable, Category = "VaRealVehicle|Components|VehicleMovement")
	bool IsDebug() { return bShowDebug; }

protected:
	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Debug)
	bool bShowDebug;


	//////////////////////////////////////////////////////////////////////////
	// Internal data

protected:
	// What the player has the steering set to. Range -1...1
	UPROPERTY(Transient)
	float RawSteeringInput;

	// What the player has the accelerator set to. Range -1...1
	UPROPERTY(Transient)
	float RawThrottleInput;

	// True if the player is holding the handbrake
	UPROPERTY(Transient)
	uint32 bRawHandbrakeInput : 1;

	UPROPERTY(Transient)
	float ThrottleInput;

	UPROPERTY(Transient)
	float SteeringInput;

	UPROPERTY(Transient)
	float BrakeInput;

};
