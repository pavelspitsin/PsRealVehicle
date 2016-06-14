// Copyright 2016 Pushkin Studio. All Rights Reserved.

#pragma once

#include "PrvVehicleMovementComponent.generated.h"

USTRUCT(BlueprintType)
struct FSuspensionInfo
{
	GENERATED_USTRUCT_BODY()

	/** If yes, suspension location and rotation will be extracted from the bone transofrm */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	bool bInheritWheelBoneTransform;

	/** Bone name to get the wheel transform */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Suspension/*, meta = (EditCondition = "bInheritWheelBoneTransform")*/)
	FName BoneName;

	/** Suspension location in Actor space */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, meta = (EditCondition = "!bInheritWheelBoneTransform", DisplayName = "Suspension Location"))
	FVector Location;

	/** Suspension rotation in Actor space */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, meta = (EditCondition = "!bInheritWheelBoneTransform", DisplayName = "Suspension Rotation"))
	FRotator Rotation;

	/** Should wheel bone be animated with suspension compression offset? */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Animation)
	bool bAnimateBoneOffset;

	/** Should wheel bone be animated with wheel rotation? */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Animation)
	bool bAnimateBoneRotation;

	/** Is wheel belongs to the right track or the left one? */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Suspension)
	bool bRightTrack;

	/** Is wheel generates driving force [ignored by tracked vehicles] */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	bool bDrivingWheel;

	/** Is wheel is influenced by steering [ignored by tracked vehicles] */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	bool bSteeringWheel;

	/** Should the dust or not */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	bool bSpawnDust;

	/** If yes, wheel will use settings from below */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Suspension)
	bool bCustomWheelConfig;

	/** Wheel relative offset from its bone.
	 * Attn.! Ignored when suspension is not inherited from bone */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Suspension, meta = (EditCondition = "bCustomWheelConfig", DisplayName = "Wheel Offset"))
	FVector WheelBoneOffset;

	/** How far the wheel can go above the resting position */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Suspension, meta = (EditCondition = "bCustomWheelConfig", DisplayName = "Suspension Length"))
	float Length;	/** SuspensionMaxRaise */

	/** How far the wheel can drop below the resting position */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Suspension, meta = (EditCondition = "bCustomWheelConfig", DisplayName = "Suspension Max Drop"))
	float MaxDrop;

	/** Wheel [collision] radius */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Suspension, meta = (EditCondition = "bCustomWheelConfig"))
	float CollisionRadius;

	/** Wheel [collision] width */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Suspension, meta = (EditCondition = "bCustomWheelConfig"))
	float CollisionWidth;

	/** Wheel relative bone offset for animation */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Suspension, meta = (EditCondition = "bCustomWheelConfig"))
	FVector VisualOffset;

	/** How strong wheel reacts to compression [N/cm] */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Suspension, meta = (EditCondition = "bCustomWheelConfig"))
	float Stiffness;

	/** How fast wheel becomes stable on compression [N/(cm/s)] */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Suspension, meta = (EditCondition = "bCustomWheelConfig"))
	float CompressionDamping;

	/** How fast wheel becomes stable on decompression [N/(cm/s)] */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Suspension, meta = (EditCondition = "bCustomWheelConfig"))
	float DecompressionDamping;

	/** Defaults */
	FSuspensionInfo()
	{
		bInheritWheelBoneTransform = true;

		Location = FVector::ZeroVector;
		Rotation = FRotator::ZeroRotator;

		bAnimateBoneOffset = true;
		bAnimateBoneRotation = true;

		bRightTrack = false;
		bDrivingWheel = true;
		bSteeringWheel = false;
		bSpawnDust = false;

		bCustomWheelConfig = false;

		WheelBoneOffset = FVector::ZeroVector;
		Length = 25.f;
		MaxDrop = 10.f;
		CollisionRadius = 36.f;
		CollisionWidth = 20.f;
		VisualOffset = FVector::ZeroVector;
		Stiffness = 4000000.f;				// [N/cm]
		CompressionDamping = 4000000.f;		// [N/(cm/s)]
		DecompressionDamping = 4000000.f;	// [N/(cm/s)]
	}
};

USTRUCT(BlueprintType)
struct FSuspensionState
{
	GENERATED_USTRUCT_BODY()

	/** Wheel initial suspension config */
	UPROPERTY(BlueprintReadWrite)
	FSuspensionInfo SuspensionInfo;

	/** Effective suspension length on last tick */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly)
	float PreviousLength;

	/** Suspension length for visuals (including MaxDrop interval) */
	UPROPERTY(BlueprintReadOnly, Category = Visuals)
	float VisualLength;

	/** Current wheel rotation angle (pitch) */
	UPROPERTY(BlueprintReadOnly, Category = Visuals)
	float RotationAngle;

	/** Current wheel steering angle (yaw) */
	UPROPERTY(BlueprintReadOnly, Category = Visuals)
	float SteeringAngle;

	/** Force that was generated by suspension compression */
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
	FVector PreviousWheelCollisionVelocity;

	/**  */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float WheelLoad;

	/** Is wheel engaged into physics simulation */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly)
	bool WheelTouchedGround;

	/** */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly)
	TEnumAsByte<EPhysicalSurface> SurfaceType;

	/** */
	UPROPERTY(Transient)
	UParticleSystemComponent* DustPSC;

	/** Defaults */
	FSuspensionState()
	{
		PreviousLength = 0.f;
		VisualLength = 0.f;

		RotationAngle = 0.f;
		SteeringAngle = 0.f;

		SuspensionForce = FVector::ZeroVector;
		WheelCollisionLocation = FVector::ZeroVector;
		WheelCollisionNormal = FVector::UpVector;
		PreviousWheelCollisionVelocity = FVector::ZeroVector;

		WheelLoad = 0.f;
		WheelTouchedGround = false;

		SurfaceType = EPhysicalSurface::SurfaceType_Default;
	}
};

USTRUCT(BlueprintType)
struct FGearInfo
{
	GENERATED_USTRUCT_BODY()

	/** Determines the amount of torque multiplication */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float Ratio;

	/** Value of engineRevs/maxEngineRevs that is low enough to gear down */
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0", ClampMax = "1.0", UIMax = "1.0"))
	float DownRatio;

	/** Value of engineRevs/maxEngineRevs that is high enough to gear up */
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0", ClampMax = "1.0", UIMax = "1.0"))
	float UpRatio;

	/** Defaults */
	FGearInfo()
	{
		Ratio = 0.f;
		DownRatio = 0.15f;
		UpRatio = 0.9f;
	}
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

	/** Driving angular velocity */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float AngularVelocity;

	/** Real (effective) angual velocity */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float EffectiveAngularVelocity;

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

	/** Defaults */
	FTrackInfo()
	{
		Input = 0.f;
		TorqueTransfer = 0.f;

		LinearVelocity = 0.f;
		AngularVelocity = 0.f;
		EffectiveAngularVelocity = 0.f;

		DriveTorque = 0.f;
		FrictionTorque = 0.f;
		RollingFrictionTorque = 0.f;

		BrakeRatio = 0.f;
		DriveForce = FVector::ZeroVector;
	}
};

struct FAnimNode_PrvWheelHandler;

/**
 * Component that uses Torque and Force to move tracked vehicles
 */
UCLASS()
class PSREALVEHICLEPLUGIN_API UPrvVehicleMovementComponent : public UPawnMovementComponent
{
	GENERATED_UCLASS_BODY()

	// Let direct access for animation nodes
	friend FAnimNode_PrvWheelHandler;

protected:
	//////////////////////////////////////////////////////////////////////////
	// Initialization
	
	virtual void InitializeComponent() override;
	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;


	//////////////////////////////////////////////////////////////////////////
	// Physics initialization
	
	void CalculateMOI();
	void InitBodyPhysics();
	void InitSuspension();
	void InitGears();


	//////////////////////////////////////////////////////////////////////////
	// Physics simulation

	bool IsSleeping(float DeltaTime);
	void ResetSleep();

	/** [client] */
	UFUNCTION()
	void OnRep_IsSleeping();

	void UpdateSteering(float DeltaTime);
	void UpdateThrottle(float DeltaTime);
	void UpdateGearBox();
	void UpdateBrake();

	void UpdateTracksVelocity(float DeltaTime);
	void UpdateHullVelocity(float DeltaTime);
	void UpdateEngine();
	void UpdateDriveForce();

	void UpdateSuspension(float DeltaTime);
	void UpdateFriction(float DeltaTime);
	void UpdateLinearVelocity(float DeltaTime);
	void UpdateAngularVelocity(float DeltaTime);

	void AnimateWheels(float DeltaTime);

	float ApplyBrake(float DeltaTime, float AngularVelocity, float BrakeRatio);
	float CalculateFrictionCoefficient(FVector DirectionVelocity, FVector ForwardVector, FVector2D FrictionEllipse);

	/** Shift gear up or down 
	 * Attn.! It doesn't think about why it happend, so it should be done externally!) */
	void ShiftGear(bool bShiftUp);


	//////////////////////////////////////////////////////////////////////////
	// Network

	/** Pass current state to server */
	UFUNCTION(reliable, server, WithValidation)
	void ServerUpdateState(float InSteeringInput, float InThrottleInput, uint32 InHandbrakeInput, int32 InCurrentGear);

	// MAKE ALL CONFIG PUBLIC
public:

	/////////////////////////////////////////////////////////////////////////
	// Vehicle setup

	/** Is it a car? Tank by default */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Vehicle)
	bool bWheeledVehicle;

	/** Distance from forward wheels to back one */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Vehicle, meta = (editcondition = "bWheeledVehicle", ClampMin = "0.001", UIMin = "0.001"))
	float TransmissionLength;

	/** If true, mass will not be automatically computed and you must set it directly */
	UPROPERTY(meta = (DisplayName = "Override"))
	bool bOverrideMass;

	/** Mass of the body in KG. By default we compute this based on physical material and mass scale */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Vehicle, meta = (editcondition = "bOverrideMass", ClampMin = "0.001", UIMin = "0.001", DisplayName = "Vehicle Mass"))
	float OverrideVehicleMass;

	/** Use custom linear damping instead of build-in PhysX one */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Vehicle)
	bool bCustomLinearDamping;

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Vehicle, meta = (EditCondition = "bCustomLinearDamping"))
	FVector DryFrictionLinearDamping;

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Vehicle, meta = (EditCondition = "bCustomLinearDamping"))
	FVector FluidFrictionLinearDamping;

	/** Use custom angular damping instead of build-in PhysX one */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Vehicle)
	bool bCustomAngularDamping;

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Vehicle, meta = (EditCondition = "bCustomAngularDamping"))
	FVector DryFrictionAngularDamping;

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Vehicle, meta = (EditCondition = "bCustomAngularDamping"))
	FVector FluidFrictionAngularDamping;

	/** 'Drag' force added to reduce linear movement */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Vehicle)
	float LinearDamping;

	/** 'Drag' force added to reduce angular movement */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Vehicle, meta = (EditCondition = "!bCustomAngularDamping"))
	float AngularDamping;

	/** User specified offset for the center of mass of this object, from the calculated location */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Vehicle, meta = (DisplayName = "Center Of Mass Offset"))
	FVector COMOffset;

	/** Kg */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Vehicle)
	float SprocketMass;

	/** Cm */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Vehicle)
	float SprocketRadius;

	/** Kg */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Vehicle)
	float TrackMass;

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Vehicle)
	float SleepVelocity;

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Vehicle)
	float SleepDelay;


	/////////////////////////////////////////////////////////////////////////
	// Suspension setup

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Suspension)
	TArray<FSuspensionInfo> SuspensionSetup;

	/** Wheel relative offset from its bone.
	* Attn.! Ignored when suspension is not inherited from bone */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Suspension)
	FVector DefaultWheelBoneOffset;

	/** How far the wheel can go above the resting position */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Suspension)
	float DefaultLength;	/** SuspensionMaxRaise */

	/** How far the wheel can drop below the resting position */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Suspension)
	float DefaultMaxDrop;

	/** Wheel [collision] radius */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Suspension)
	float DefaultCollisionRadius;

	/** Wheel [collision] width */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Suspension)
	float DefaultCollisionWidth;

	/** Wheel relative bone offset for animation */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Suspension)
	FVector DefaultVisualOffset;

	/** How strong wheel reacts to compression */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Suspension)
	float DefaultStiffness;

	/** How fast wheel becomes stable on compression */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Suspension)
	float DefaultCompressionDamping;

	/** How fast wheel becomes stable on decompression */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Suspension)
	float DefaultDecompressionDamping;

	/** Global factor that applied to all wheels */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Suspension)
	float StiffnessFactor;

	/** Global factor that applied to all wheels */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Suspension)
	float CompressionDampingFactor;

	/** Global factor that applied to all wheels */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Suspension)
	float DecompressionDampingFactor;

	/** Discrete damping correction */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Suspension)
	bool bCustomDampingCorrection;

	/** Discrete damping correction factor: 0 - disabled correction, 1 - hypercorrection  */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Suspension, meta = (EditCondition = "bCustomDampingCorrection", ClampMin = "0.0", UIMin = "0.0", ClampMax = "1.0", UIMax = "1.0"))
	float DampingCorrectionFactor;

	/** If yes, damping will consider active friction points number */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Suspension)
	bool bAdaptiveDampingCorrection;

	/** How fast wheels are animated while going down */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Suspension)
	float DropFactor;

	/**	Should 'Hit' events fire when this object collides during physics simulation */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Suspension, meta = (DisplayName = "Simulation Generates Hit Events"))
	bool bNotifyRigidBodyCollision;


	/////////////////////////////////////////////////////////////////////////
	// Engine setup

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = EngineSetup)
	float ThrottleUpRatio;

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = EngineSetup)
	float ThrottleDownRatio;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = EngineSetup)
	float DifferentialRatio;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = EngineSetup)
	float TransmissionEfficiency;

	/** Torque (Nm) at a given RPM */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = EngineSetup)
	FRuntimeFloatCurve EngineTorqueCurve;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = EngineSetup)
	float EngineExtraPowerRatio;

	/** */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = EngineSetup)
	float TorqueTransferThrottleFactor;

	/** */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = EngineSetup)
	float TorqueTransferSteeringFactor;

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = EngineSetup)
	bool bLimitMaxSpeed;

	/** Cm/s */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = EngineSetup, meta = (editcondition = "bLimitMaxSpeed", ClampMin = "0.0", UIMin = "0.0"))
	float MaxSpeedLimit;


	/////////////////////////////////////////////////////////////////////////
	// Gear Box setup

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = GearBox)
	TArray<FGearInfo> GearSetup;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = GearBox)
	bool bAutoGear;

	/** Minimum time it takes the automatic transmission to initiate a gear change (seconds)*/
	UPROPERTY(EditAnywhere, Category = GearBox, meta = (editcondition = "bAutoGear", ClampMin = "0.0", UIMin = "0.0"))
	float GearAutoBoxLatency;


	/////////////////////////////////////////////////////////////////////////
	// Steering setup

	/** Use angular velocity hack instead of torque transfer */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = SteeringSetup)
	bool bAngularVelocitySteering;

	/** Steering rotation angular speed (basic, before throttle ratio is applied) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = SteeringSetup, meta = (editcondition = "bAngularVelocitySteering", ClampMin = "0.0", UIMin = "0.0"))
	float SteeringAngularSpeed;

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = SteeringSetup, meta = (editcondition = "bAngularVelocitySteering", ClampMin = "0.0", UIMin = "0.0"))
	float SteeringUpRatio;

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = SteeringSetup, meta = (editcondition = "bAngularVelocitySteering", ClampMin = "0.0", UIMin = "0.0"))
	float SteeringDownRatio;

	/** How strong throttle influences steering [-1, 1] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = SteeringSetup, meta = (editcondition = "bAngularVelocitySteering", UIMin = "-1.0", UIMax = "1.0"))
	float SteeringThrottleFactor;

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = SteeringSetup)
	bool bUseSteeringCurve;

	/** Maximum steering versus forward speed (km/h) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = SteeringSetup, meta = (editcondition = "bUseSteeringCurve"))
	FRuntimeFloatCurve SteeringCurve;


	/////////////////////////////////////////////////////////////////////////
	// Brake system

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = BrakeSystem)
	float BrakeForce;

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = BrakeSystem)
	bool bAutoBrake;

	/** How much brake applied by auto-brake system */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = BrakeSystem, meta = (editcondition = "bAutoBrake"))
	float AutoBrakeFactor;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = BrakeSystem, meta = (editcondition = "!bAngularVelocitySteering"))
	float SteeringBrakeTransfer;

	/** */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = BrakeSystem, meta = (editcondition = "!bAngularVelocitySteering"))
	float SteeringBrakeFactor;

	/** Attn.! Has almost no effect with Angular Velocity Steering System */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = BrakeSystem)
	bool bSteeringStabilizer;

	/** Minimum amount (ABS) of Hull angular velocity to use steering stabilizer */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = BrakeSystem, meta = (EditCondition = "bSteeringStabilizer"))
	float SteeringStabilizerMinimumHullVelocity;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = BrakeSystem, meta = (EditCondition = "bSteeringStabilizer"))
	float AutoBrakeStableTransfer;

	/** How much brake should be applied when stabilizer is working */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = BrakeSystem, meta = (EditCondition = "bSteeringStabilizer"))
	float SteeringStabilizerBrakeFactor;


	/////////////////////////////////////////////////////////////////////////
	// Friction

	/** Ellipse to calculate static friction coefficient
	 * http://hyperphysics.phy-astr.gsu.edu/hbase/frict2.html */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = PhysicsSimulation)
	FVector2D StaticFrictionCoefficientEllipse;

	/** Ellipse to calculate kinetic friction coefficient */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = PhysicsSimulation)
	FVector2D KineticFrictionCoefficientEllipse;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = PhysicsSimulation)
	float FrictionTorqueCoefficient;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = PhysicsSimulation)
	float RollingFrictionCoefficient;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = PhysicsSimulation)
	float RollingVelocityCoefficient;


	/////////////////////////////////////////////////////////////////////////
	// Movement cache

protected:
	float FinalMOI;

	TArray<FSuspensionState> SuspensionData;

	int32 NeutralGear;
	int32 CurrentGear;
	bool bReverseGear;

	float LastAutoGearShiftTime;
	float LastAutoGearHullVelocity;

	FTrackInfo LeftTrack;
	FTrackInfo RightTrack;
	
	float RightTrackTorque;
	float LeftTrackTorque;

	float HullAngularVelocity;

	// Cached RPM limits
	float MinEngineRPM;
	float MaxEngineRPM;

	float EngineRPM;
	float EngineTorque;
	float DriveTorque;

	// Angular steering speed
	float EffectiveSteeringAngularSpeed;

	/** How many wheels are touched the ground */
	int32 ActiveFrictionPoints;

	/** How many drive wheels are touched the ground */
	int32 ActiveDrivenFrictionPoints;

	UPROPERTY(Transient, ReplicatedUsing = OnRep_IsSleeping)
	uint32 bIsSleeping : 1;

	float SleepTimer;


	//////////////////////////////////////////////////////////////////////////
	// Vehicle control

public:
	/** Set the user input for the vehicle throttle */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	void SetThrottleInput(float Throttle);
	
	/** Set the user input for the vehicle steering */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	void SetSteeringInput(float Steering);

	/** Set the user input for handbrake */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	void SetHandbrakeInput(bool bNewHandbrake);

	/** Make movement possible */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	void EnableMovement();

	/** Make movement impossible */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	void DisableMovement();

	/** Is vehicle trying to move (movenent is enabled and it has player input) */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	bool IsMoving() const;

protected:
	/** */
	UPROPERTY(Transient, Replicated)
	bool bIsMovementEnabled;


	//////////////////////////////////////////////////////////////////////////
	// Vehicle stats

public:
	/** How fast the vehicle is moving forward */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	float GetForwardSpeed() const;

	/** Get current real throttle value */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	float GetThrottle() const;

	/** Get current engine's rotation speed */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	float GetEngineRotationSpeed() const;

	/** Get current engine's max rotation speed */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	float GetEngineMaxRotationSpeed() const;

	/** Get current engine torque */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	float GetEngineTorque() const;

	/** Get left track drive torque */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	float GetDriveTorqueLeft() const;

	/** Get right track drive torque */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	float GetDriveTorqueRight() const;

	/** Get left track angular velocity */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	float GetAngularVelocityLeft() const;

	/** Get right track angular velocity */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	float GetAngularVelocityRight() const;

	/** Get left track brake ratio */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	float GetBrakeRatioLeft() const;

	/** Get left track brake ratio */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	float GetBrakeRatioRight() const;

	/** Is vehicle is touching ground with wheels */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	bool HasTouchGround() const;
	

	//////////////////////////////////////////////////////////////////////////
	// Tracks info

public:
	/** Get left track internal data */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	void GetTrackInfoLeft(FTrackInfo& OutTrack) const;

	/** Get right track internal data */
	UFUNCTION(BlueprintCallable, Category="PsRealVehicle|Components|VehicleMovement")
	void GetTrackInfoRight(FTrackInfo& OutTrack) const;


	//////////////////////////////////////////////////////////////////////////
	// Data access

public:
	/** Get current gear */
	UFUNCTION(BlueprintCallable, Category = "PsRealVehicle|Components|VehicleMovement")
	int32 GetCurrentGear() const;

	/** Get neutral gear */
	UFUNCTION(BlueprintCallable, Category = "PsRealVehicle|Components|VehicleMovement")
	int32 GetNeutralGear() const;

	/** Get gear reverse state */
	UFUNCTION(BlueprintCallable, Category = "PsRealVehicle|Components|VehicleMovement")
	bool IsCurrentGearReverse() const;

	/** Get gearbox config */
	UFUNCTION(BlueprintCallable, Category = "PsRealVehicle|Components|VehicleMovement")
	FGearInfo GetGearInfo(int32 GearNum) const;

	/** Get current gearbox config */
	UFUNCTION(BlueprintCallable, Category = "PsRealVehicle|Components|VehicleMovement")
	FGearInfo GetCurrentGearInfo() const;

protected:
	/** Get the mesh this vehicle is tied to */
	class USkinnedMeshComponent* GetMesh();


	//////////////////////////////////////////////////////////////////////////
	// Effects

protected:
	/** */
	void UpdateWheelEffects(float DeltaTime);

	/** */
	UParticleSystemComponent* SpawnNewWheelEffect(FName InSocketName = NAME_None, FVector InSocketOffset = FVector::ZeroVector);

protected:
	/** */
	UPROPERTY(EditDefaultsOnly, Category = Effects)
	class UPrvVehicleDustEffect* DustEffect;


	//////////////////////////////////////////////////////////////////////////
	// Debug

public:
	/** Draw debug text for the wheels and suspension */
	virtual void DrawDebug(UCanvas* Canvas, float& YL, float& YPos);

	/** Draw debug primitives for the wheels and suspension */
	virtual void DrawDebugLines();

	/** */
	UFUNCTION(BlueprintCallable, Category = "PsRealVehicle|Components|VehicleMovement")
	void ShowDebug(bool bEnableDebug) { bShowDebug = bEnableDebug; }

	/** */
	UFUNCTION(BlueprintCallable, Category = "PsRealVehicle|Components|VehicleMovement")
	bool IsDebug() { return bShowDebug; }

protected:
	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Debug)
	bool bShowDebug;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Debug)
	bool bDebugAutoGearBox;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Debug)
	bool bDebugDampingCorrection;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Debug)
	bool bDebugCustomDamping;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Debug)
	bool bDebugSuspensionLimits;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Debug)
	bool bUseCustomVelocityCalculations;

	/**  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Debug)
	bool bForceNeverSleep;

	/** Make zero engine torque when MaxRPM is reached */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Debug)
	bool bLimitEngineTorque;


	//////////////////////////////////////////////////////////////////////////
	// Internal data

public:
	/** */
	UFUNCTION(BlueprintCallable, Category = "PsRealVehicle|Components|VehicleMovement")
	bool HasInput() const;

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
