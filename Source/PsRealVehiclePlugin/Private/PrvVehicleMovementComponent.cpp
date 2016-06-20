// Copyright 2016 Pushkin Studio. All Rights Reserved.

#include "PrvPlugin.h"

#include "Kismet/KismetMathLibrary.h"
#include "Kismet/KismetSystemLibrary.h"

UPrvVehicleMovementComponent::UPrvVehicleMovementComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	bAutoActivate = true;
	bWantsInitializeComponent = true;
	PrimaryComponentTick.bCanEverTick = true;

	bWheeledVehicle = false;
	TransmissionLength = 400.f;
	bOverrideMass = false;
	OverrideVehicleMass = 10000.f;
	LinearDamping = 0.5f;
	AngularDamping = 0.5f;
	COMOffset = FVector::ZeroVector;

	bCustomLinearDamping = false;
	DryFrictionLinearDamping = FVector::ZeroVector;
	FluidFrictionLinearDamping = FVector::ZeroVector;

	bCustomAngularDamping = false;
	DryFrictionAngularDamping = FVector::ZeroVector;
	FluidFrictionAngularDamping = FVector::ZeroVector;

	bLimitEngineTorque = true;
	bIsMovementEnabled = true;

	SprocketMass = 65.f;
	SprocketRadius = 25.f;
	TrackMass = 600.f;
	SleepVelocity = 5.f;
	SleepDelay = 2.f;

	bAngularVelocitySteering = true;
	SteeringAngularSpeed = 30.f;
	SteeringUpRatio = 1.f;
	SteeringDownRatio = 1.f;
	SteeringThrottleFactor = 0.f;

	bUseSteeringCurve = false;
	FRichCurve* SteeringCurveData = SteeringCurve.GetRichCurve();
	SteeringCurveData->AddKey(0.f, SteeringAngularSpeed);
	SteeringCurveData->AddKey(2000.f, SteeringAngularSpeed);	// 72 Km/h
	SteeringCurveData->AddKey(2500.f, 0.f);

	DefaultWheelBoneOffset = FVector::ZeroVector;
	DefaultLength = 25.f;
	DefaultMaxDrop = 10.f;
	DefaultCollisionRadius = 36.f;
	DefaultCollisionWidth = 20.f;
	DefaultVisualOffset = FVector::ZeroVector;
	DefaultStiffness = 4000000.f;				// [N/cm]
	DefaultCompressionDamping = 4000000.f;		// [N/(cm/s)]
	DefaultDecompressionDamping = 4000000.f;	// [N/(cm/s)]
	bCustomDampingCorrection = true;
	DampingCorrectionFactor = 1.f;
	bAdaptiveDampingCorrection = true;
	bNotifyRigidBodyCollision = true;

	GearSetup.AddDefaulted(1);	// Add at least one gear should exist
	bAutoGear = true;
	bAutoBrake = true;
	bSteeringStabilizer = true;
	SteeringStabilizerMinimumHullVelocity = 10.f;
	SpeedLimitBrakeFactor = 0.1f;
	
	GearAutoBoxLatency = 0.5f;
	LastAutoGearShiftTime = 0.f;
	LastAutoGearHullVelocity = 0.f;

	ThrottleUpRatio = 0.5f;
	ThrottleDownRatio = 1.f;

	BrakeForce = 30.f;
	AutoBrakeFactor = 1.f;
	SteeringBrakeTransfer = 0.7f;
	SteeringBrakeFactor = 1.f;
	AutoBrakeStableTransfer = 0.9f;
	SteeringStabilizerBrakeFactor = 0.2f;

	DifferentialRatio = 3.5f;
	TransmissionEfficiency = 0.9f;
	EngineExtraPowerRatio = 3.f;

	bLimitMaxSpeed = false;
	FRichCurve* MaxSpeedCurveData = MaxSpeedCurve.GetRichCurve();
	MaxSpeedCurveData->AddKey(0.f, 2000.f); // 72 Km/h

	TorqueTransferThrottleFactor = 1.f;
	TorqueTransferSteeringFactor = 1.f;

	StaticFrictionCoefficientEllipse = FVector2D(1.f, 1.f);
	KineticFrictionCoefficientEllipse = FVector2D(1.f, 1.f);

	FrictionTorqueCoefficient = 1.f;
	RollingFrictionCoefficient = 0.02f;
	RollingVelocityCoefficient = 0.000015f;

	StiffnessFactor = 1.f;
	CompressionDampingFactor = 1.f;
	DecompressionDampingFactor = 1.f;
	DropFactor = 5.f;

	// Init basic torque curve
	FRichCurve* TorqueCurveData = EngineTorqueCurve.GetRichCurve();
	TorqueCurveData->AddKey(0.f, 800.f);
	TorqueCurveData->AddKey(1400.f, 850.f);
	TorqueCurveData->AddKey(2800.f, 800.f);
	TorqueCurveData->AddKey(2810.f, 0.f);	// Torque should be zero at max RPM to prevent infinite acceleration

	// Nullify data
	NeutralGear = 0;
	CurrentGear = 0;
	bReverseGear = false;
	LastAutoGearShiftTime = 0.f;
	LastAutoGearHullVelocity = 0.f;
	RightTrackTorque = 0.f;
	LeftTrackTorque = 0.f;

	HullAngularVelocity = 0.f;
	EngineRPM = 0.f;
	EngineTorque = 0.f;
	DriveTorque = 0.f;

	EffectiveSteeringAngularSpeed = 0.f;
	ActiveFrictionPoints = 0;
	ActiveDrivenFrictionPoints = 0;
}


//////////////////////////////////////////////////////////////////////////
// Initialization

void UPrvVehicleMovementComponent::InitializeComponent()
{
	Super::InitializeComponent();

	InitBodyPhysics();
	CalculateMOI();
	InitSuspension();
	InitGears();

	// Cache RPM limits
	FRichCurve* TorqueCurveData = EngineTorqueCurve.GetRichCurve();
	TorqueCurveData->GetTimeRange(MinEngineRPM, MaxEngineRPM);
}

void UPrvVehicleMovementComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)
{
	// Notify server about player input
	APawn* MyOwner = UpdatedComponent ? Cast<APawn>(UpdatedComponent->GetOwner()) : NULL;
	if (MyOwner && MyOwner->IsLocallyControlled())
	{
		ServerUpdateState(RawSteeringInput, RawThrottleInput, bRawHandbrakeInput, GetCurrentGear());
	}

	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// Reset sleeping state each time we have any input
	if (HasInput())
	{
		ResetSleep();
	}

	// Check we're not sleeping (don't update physics state while sleeping)
	if (!IsSleeping(DeltaTime))
	{
		// Perform full simulation only on server and for local owner
		if ((GetOwner()->Role == ROLE_Authority) || (MyOwner && MyOwner->IsLocallyControlled()))
		{
			// Reset input if movement is disabled
			if (!bIsMovementEnabled)
			{
				SetSteeringInput(0.f);
				SetThrottleInput(0.f);
			}

			// Suspension
			UpdateSuspension(DeltaTime);
			UpdateFriction(DeltaTime);

			// Engine
			UpdateSteering(DeltaTime);
			UpdateThrottle(DeltaTime);

			// Control
			UpdateGearBox();
			UpdateBrake();

			// Movement
			UpdateTracksVelocity(DeltaTime);
			UpdateHullVelocity(DeltaTime);
			UpdateEngine();
			UpdateDriveForce();

			// Additional damping
			UpdateLinearVelocity(DeltaTime);
			UpdateAngularVelocity(DeltaTime);
		}
	}

	// @todo Network wheels animation
	AnimateWheels(DeltaTime);

	// Update dust VFX
	if (!IsRunningDedicatedServer())
	{
		UpdateWheelEffects(DeltaTime);
	}

	// Show debug
	if (bShowDebug)
	{
		DrawDebugLines();
	}
}


//////////////////////////////////////////////////////////////////////////
// Physics Initialization

void UPrvVehicleMovementComponent::CalculateMOI()
{
	float SprocketSquareRadius = (SprocketRadius * SprocketRadius);
	float SprocketMOI = (SprocketMass / 2) * SprocketSquareRadius;
	float TrackMOI = TrackMass * SprocketSquareRadius;

	FinalMOI = SprocketMOI + TrackMOI;

	UE_LOG(LogPrvVehicle, Warning, TEXT("Final MOI: %f"), FinalMOI);
	UE_LOG(LogPrvVehicle, Warning, TEXT("Vehicle mass: %f"), GetMesh()->GetMass());
}

void UPrvVehicleMovementComponent::InitBodyPhysics()
{
	auto VehicleMesh = GetMesh();

	if (bOverrideMass)
	{
		VehicleMesh->SetMassOverrideInKg(NAME_None, OverrideVehicleMass);
	}

	if (!bCustomLinearDamping)
	{
		VehicleMesh->SetLinearDamping(LinearDamping);
	}
	else
	{
		// Force zero physX damping
		VehicleMesh->SetLinearDamping(0.f);
	}

	if (!bCustomAngularDamping)
	{
		VehicleMesh->SetAngularDamping(AngularDamping);
	}
	else
	{
		// Force zero damping instead
		VehicleMesh->SetAngularDamping(0.f);
	}

	VehicleMesh->SetCenterOfMass(COMOffset);
}

void UPrvVehicleMovementComponent::InitSuspension()
{
	for (auto& SuspInfo : SuspensionSetup)
	{
		if (!SuspInfo.bCustomWheelConfig)
		{
			SuspInfo.WheelBoneOffset = DefaultWheelBoneOffset;
			SuspInfo.Length = DefaultLength;
			SuspInfo.MaxDrop = DefaultMaxDrop;
			SuspInfo.CollisionRadius = DefaultCollisionRadius;
			SuspInfo.CollisionWidth = DefaultCollisionWidth;
			SuspInfo.VisualOffset = DefaultVisualOffset;
			SuspInfo.Stiffness = DefaultStiffness;
			SuspInfo.CompressionDamping = DefaultCompressionDamping;
			SuspInfo.DecompressionDamping = DefaultDecompressionDamping;

			if (SuspInfo.bRightTrack)
			{
				SuspInfo.WheelBoneOffset.Y *= -1.f;
				SuspInfo.VisualOffset.Y *= -1.f;
			}
		}
		
		USkinnedMeshComponent* Mesh = GetMesh();
		if (Mesh)
		{
			if (SuspInfo.bInheritWheelBoneTransform)
			{
				FTransform WheelTransform = Mesh->GetSocketTransform(SuspInfo.BoneName, RTS_Actor);
				SuspInfo.Location = WheelTransform.GetLocation() + SuspInfo.WheelBoneOffset + FVector::UpVector * SuspInfo.Length;
				SuspInfo.Rotation = WheelTransform.GetRotation().Rotator();

				UE_LOG(LogPrvVehicle, Log, TEXT("Init suspension (%s): %s"), *SuspInfo.BoneName.ToString(), *SuspInfo.Location.ToString());
			}
			else
			{
				FTransform WheelTransform = Mesh->GetSocketTransform(SuspInfo.BoneName, RTS_Actor);
				SuspInfo.WheelBoneOffset = (SuspInfo.Location - FVector::UpVector * SuspInfo.Length) - WheelTransform.GetLocation();
			}
		}

		FSuspensionState SuspState;
		SuspState.SuspensionInfo = SuspInfo;
		SuspState.PreviousLength = SuspInfo.Length;

		if (SuspInfo.bSpawnDust)
		{
			SuspState.DustPSC = SpawnNewWheelEffect();
		}

		SuspensionData.Add(SuspState);
	}
}

void UPrvVehicleMovementComponent::InitGears()
{
	for (int32 i = 0; i < GearSetup.Num(); i++)
	{
		if (GearSetup[i].Ratio == 0.f)
		{
			NeutralGear = i;// (bAutoGear) ? i : FMath::Max(i + 1, GearSetup.Num());
			break;
		}
	}

	// Start with neutral gear
	CurrentGear = NeutralGear;

	UE_LOG(LogPrvVehicle, Warning, TEXT("Neutral gear: %d"), NeutralGear);
}


//////////////////////////////////////////////////////////////////////////
// Physics simulation

bool UPrvVehicleMovementComponent::IsSleeping(float DeltaTime)
{
	if (bForceNeverSleep)
	{
		return false;
	}

	if (!bIsSleeping && (SleepTimer < SleepDelay))
	{
		SleepTimer += DeltaTime;
		return false;
	}

	if (GetMesh()->GetPhysicsLinearVelocity().SizeSquared() < SleepVelocity && 
		GetMesh()->GetPhysicsAngularVelocity().SizeSquared() < SleepVelocity)
	{
		if (!bIsSleeping)
		{
			bIsSleeping = true;

			// Force update on server
			OnRep_IsSleeping();
		}
	}
	else
	{
		ResetSleep();
	}

	return bIsSleeping;
}

void UPrvVehicleMovementComponent::ResetSleep()
{
	bIsSleeping = false;
	SleepTimer = 0.f;
}

void UPrvVehicleMovementComponent::OnRep_IsSleeping()
{
	if (bIsSleeping)
	{
		SleepTimer = 0.f;
		GetMesh()->PutAllRigidBodiesToSleep();
	}
}

void UPrvVehicleMovementComponent::UpdateSteering(float DeltaTime)
{
	// Update steering input
	if (bAngularVelocitySteering)
	{
		if (RawSteeringInput != 0.f)
		{
			// -- [Car] --
			if (bWheeledVehicle)
			{
				SteeringInput = SteeringInput + FMath::Sign(RawSteeringInput) * (SteeringUpRatio * DeltaTime);
			}
			// -- [Tank] --
			else
			{
				SteeringInput = SteeringInput + ((bReverseGear) ? -1.f : 1.f) * FMath::Sign(RawSteeringInput) * (SteeringUpRatio * DeltaTime);
			}

			// Clamp steering to joystick values
			SteeringInput = FMath::Clamp(
				SteeringInput,
				(-1.f) * FMath::Abs(RawSteeringInput) - (ThrottleInput * SteeringThrottleFactor),
				FMath::Abs(RawSteeringInput) + (ThrottleInput * SteeringThrottleFactor));
		}
		else
		{
			SteeringInput = FMath::Sign(SteeringInput) * FMath::Max(0.f, (FMath::Abs(SteeringInput) - (SteeringDownRatio * DeltaTime)));
		}

		// No direct input to tracks
		LeftTrack.Input = 0.f;
		RightTrack.Input = 0.f;
	}
	else
	{
		SteeringInput = RawSteeringInput;
		LeftTrack.Input = SteeringInput;
		RightTrack.Input = -SteeringInput;
	}

	// Check steering curve usage
	if (bUseSteeringCurve)
	{
		const float CurrentSpeedCmS = UpdatedComponent->GetComponentVelocity().Size();
		FRichCurve* SteeringCurveData = SteeringCurve.GetRichCurve();

		// Check steering limitation (issue #51 magic)
		if (bLimitMaxSpeed)
		{
			if (SteeringInput != 0.f)
			{
				const float TargetSteeringAngularSpeed = FMath::Abs(SteeringInput * SteeringCurveData->Eval(0.f));
				const float AllowedSteeringAngularSpeed = SteeringCurveData->Eval(CurrentSpeedCmS);

				EffectiveSteeringAngularSpeed = FMath::Sign(SteeringInput) * FMath::Min(TargetSteeringAngularSpeed, AllowedSteeringAngularSpeed);
			}
			else
			{
				EffectiveSteeringAngularSpeed = 0.f;
			}
		}
		else
		{
			EffectiveSteeringAngularSpeed = SteeringInput * SteeringCurveData->Eval(CurrentSpeedCmS);
		}
	}
	else
	{
		EffectiveSteeringAngularSpeed = SteeringInput * SteeringAngularSpeed;
	}

	// Apply steering to angular velocity
	if (bAngularVelocitySteering)
	{
		// Move steering into angular velocity
		FVector LocalAngularVelocity = UpdatedComponent->GetComponentTransform().InverseTransformVectorNoScale(GetMesh()->GetPhysicsAngularVelocity());
		const float FrictionRatio = (float) ActiveDrivenFrictionPoints / FMath::Max(SuspensionData.Num(), 1);	// Dirty hack, it's not real, but good for visuals
		float TargetSteeringVelocity = EffectiveSteeringAngularSpeed * FrictionRatio;

		// -- [Car] --
		if (bWheeledVehicle)
		{
			// Simple model of angular speed for car
			const float TurnRadius = TransmissionLength / FMath::Sin(FMath::DegreesToRadians(TargetSteeringVelocity));
			TargetSteeringVelocity = FMath::RadiansToDegrees(GetForwardSpeed() / TurnRadius);
		}

		if (FMath::Abs(LocalAngularVelocity.Z) < FMath::Abs(TargetSteeringVelocity))
		{
			LocalAngularVelocity.Z = TargetSteeringVelocity;
			GetMesh()->SetPhysicsAngularVelocity(UpdatedComponent->GetComponentTransform().TransformVectorNoScale(LocalAngularVelocity));
		}
	}

	// -- [Car] --
	if (bWheeledVehicle)
	{
		// Update driving wheels for wheeled vehicles
		for (auto& SuspState : SuspensionData)
		{
			if (SuspState.SuspensionInfo.bSteeringWheel)
			{
				SuspState.SuspensionInfo.Rotation.Yaw = EffectiveSteeringAngularSpeed;
			}
		}
	}
}

void UPrvVehicleMovementComponent::UpdateThrottle(float DeltaTime)
{
	// -- [Car] --
	if (bWheeledVehicle)
	{
		LeftTrack.TorqueTransfer = FMath::Abs(RawThrottleInput) * TorqueTransferThrottleFactor;
		RightTrack.TorqueTransfer = FMath::Abs(RawThrottleInput) * TorqueTransferThrottleFactor;
	}
	// -- [Tank] --
	else
	{
		// Calc torque transfer based on input
		if (RawThrottleInput != 0.f)
		{
			LeftTrack.TorqueTransfer = FMath::Abs(RawThrottleInput) * TorqueTransferThrottleFactor + FMath::Max(0.f, LeftTrack.Input) * TorqueTransferSteeringFactor;
			RightTrack.TorqueTransfer = FMath::Abs(RawThrottleInput) * TorqueTransferThrottleFactor + FMath::Max(0.f, RightTrack.Input) * TorqueTransferSteeringFactor;
		}
		else
		{
			LeftTrack.TorqueTransfer = FMath::Abs(RawThrottleInput) * TorqueTransferThrottleFactor + LeftTrack.Input * TorqueTransferSteeringFactor;
			RightTrack.TorqueTransfer = FMath::Abs(RawThrottleInput) * TorqueTransferThrottleFactor + RightTrack.Input * TorqueTransferSteeringFactor;
		}
	}

	// Throttle shouldn't be instant
	if ((LeftTrack.TorqueTransfer != 0.f) || (RightTrack.TorqueTransfer != 0.f))
	{
		ThrottleInput += (ThrottleUpRatio * DeltaTime);
	}
	else
	{
		ThrottleInput -= (ThrottleDownRatio * DeltaTime);
	}

	// Limit throttle to [0; 1]
	ThrottleInput = FMath::Clamp(ThrottleInput, 0.f, 1.f);

	// Debug
	if (bShowDebug)
	{
		// Torque transfer balance
		DrawDebugString(GetWorld(), UpdatedComponent->GetComponentTransform().TransformPosition(FVector(0.f, -100.f, 0.f)), FString::SanitizeFloat(LeftTrack.TorqueTransfer), nullptr, FColor::White, 0.f);
		DrawDebugString(GetWorld(), UpdatedComponent->GetComponentTransform().TransformPosition(FVector(0.f, 100.f, 0.f)), FString::SanitizeFloat(RightTrack.TorqueTransfer), nullptr, FColor::White, 0.f);
	}
}

void UPrvVehicleMovementComponent::UpdateGearBox()
{
	if (!bAutoGear)
	{
		return;
	}

	// Cache previous gear
	const int32 PreviousGear = CurrentGear;

	// With auto-gear we shouldn't have neutral
	if (bAutoGear && (CurrentGear == NeutralGear))
	{
		if ((RawThrottleInput != 0.f) || (SteeringInput != 0.f))
		{
			ShiftGear((RawThrottleInput >= 0.f));

			if (bDebugAutoGearBox)
			{
				UE_LOG(LogPrvVehicle, Warning, TEXT("Switch from neutral: was %d, now %d"), PreviousGear, CurrentGear);
			}
		}
	}

	// Check velocity direction
	const float VelocityDirection = FVector::DotProduct(UpdatedComponent->GetForwardVector(), UpdatedComponent->GetComponentVelocity());
	const bool MovingForward = (VelocityDirection >= 0.f);
	const bool HasThrottleInput = (RawThrottleInput != 0.f);
	const bool HasAppropriateGear = ((RawThrottleInput > 0.f) == (!bReverseGear));

	// Force switch gears on input direction change
	if(HasThrottleInput && !HasAppropriateGear)
	{
		ShiftGear(!MovingForward);

		if (bDebugAutoGearBox)
		{
			UE_LOG(LogPrvVehicle, Warning, TEXT("Switch gear on direction change: was %d, now %d"), PreviousGear, CurrentGear);
		}
	}
	// Check that we can shift gear by time
	else if ((GetWorld()->GetTimeSeconds() - LastAutoGearShiftTime) > GearAutoBoxLatency)
	{
		const float CurrentRPMRatio = (EngineRPM - MinEngineRPM) / (MaxEngineRPM - MinEngineRPM);
		
		// Check we're shifring up or down
		if ((HullAngularVelocity < LastAutoGearHullVelocity) || (HullAngularVelocity < 10.f))
		{
			if (CurrentRPMRatio <= GetCurrentGearInfo().DownRatio)
			{
				// Shift down
				ShiftGear(bReverseGear);

				if (bDebugAutoGearBox)
				{	
					UE_LOG(LogPrvVehicle, Warning, TEXT("Switch gear down: was %d, now %d"), PreviousGear, CurrentGear);
				}
			}
		}
		else
		{
			if (CurrentRPMRatio >= GetCurrentGearInfo().UpRatio)
			{
				// Shift up
				ShiftGear(!bReverseGear);

				if (bDebugAutoGearBox)
				{
					UE_LOG(LogPrvVehicle, Warning, TEXT("Switch gear up: was %d, now %d"), PreviousGear, CurrentGear);
				}
			}
		}
	}

	LastAutoGearHullVelocity = HullAngularVelocity;
}

void UPrvVehicleMovementComponent::ShiftGear(bool bShiftUp)
{
	int32 PrevGear = CurrentGear;
	CurrentGear = FMath::Min(GearSetup.Num() - 1, FMath::Max(0, CurrentGear + ((bShiftUp) ? 1 : -1)));

	// Force gears limits on user input
	if (RawThrottleInput != 0.f)
	{
		bReverseGear = (RawThrottleInput < 0.f);

		if (bReverseGear)
		{
			CurrentGear = FMath::Max(0, FMath::Min(CurrentGear, NeutralGear - 1));
		}
		else
		{
			CurrentGear = FMath::Max(CurrentGear, NeutralGear);
		}
	}
	else
	{
		// Don't switch gear when we want to be neutral
		if (PrevGear >= NeutralGear)
		{
			CurrentGear = FMath::Max(CurrentGear, NeutralGear);
		}
		else
		{
			CurrentGear = FMath::Min(CurrentGear, NeutralGear);
		}

		bReverseGear = (CurrentGear < NeutralGear);
	}

	LastAutoGearShiftTime = GetWorld()->GetTimeSeconds();
}

void UPrvVehicleMovementComponent::UpdateBrake()
{
	// Check auto brake when we don't want to move
	if (bAutoBrake)
	{
		if (bAngularVelocitySteering && (RawThrottleInput == 0.f))
		{
			BrakeInput = AutoBrakeFactor;
		}
		else if ((RawThrottleInput == 0.f) && (SteeringInput == 0.f))
		{
			BrakeInput = AutoBrakeFactor;
		}
		else
		{
			// Check velocity direction
			const float VelocityDirection = FVector::DotProduct(UpdatedComponent->GetForwardVector(), UpdatedComponent->GetComponentVelocity());
			const bool MovingForward = (VelocityDirection >= 0.f);
			const bool HasThrottleInput = (RawThrottleInput != 0.f);
			const bool MovingThrottleInputDirection = (MovingForward == (RawThrottleInput > 0.f));
			const bool NonZeroAngularVelocity =
				(FMath::Sign(LeftTrack.AngularVelocity) != 0) &&
				(FMath::Sign(RightTrack.AngularVelocity) != 0);
			const bool WrongAngularVelocityDirection =
				(FMath::Sign(LeftTrack.AngularVelocity) != FMath::Sign(RawThrottleInput)) &&
				(FMath::Sign(RightTrack.AngularVelocity) != FMath::Sign(RawThrottleInput));

			// Brake when direction is changing
			if (HasThrottleInput && !MovingThrottleInputDirection && NonZeroAngularVelocity && WrongAngularVelocityDirection)
			{
				BrakeInput = AutoBrakeFactor;
			}
			else
			{
				BrakeInput = AutoBrakeFactor * bRawHandbrakeInput;
			}
		}
	}

	// Handbrake first
	LeftTrack.BrakeRatio = BrakeInput;
	RightTrack.BrakeRatio = BrakeInput;

	// -- [Tank] --
	if (!bWheeledVehicle)
	{
		// Shouldn't affect rotation on place
		if (RawThrottleInput != 0.f)
		{
			// Manual brake for rotation
			if ((LeftTrack.Input < 0.f) && (FMath::Abs(LeftTrack.AngularVelocity) >= FMath::Abs(RightTrack.AngularVelocity * SteeringBrakeTransfer)))
			{
				LeftTrack.BrakeRatio = (-1.f) * LeftTrack.Input * SteeringBrakeFactor;
			}
			else if ((RightTrack.Input < 0.f) && (FMath::Abs(RightTrack.AngularVelocity) >= FMath::Abs(LeftTrack.AngularVelocity * SteeringBrakeTransfer)))
			{
				RightTrack.BrakeRatio = (-1.f) * RightTrack.Input * SteeringBrakeFactor;
			}
		}
		// Without forward input we should brake if ang velocity is higher than wanted to be
		else
		{
			// @todo #22
		}
	}

	// Stabilize steering
	if (bSteeringStabilizer && (SteeringInput == 0.f) && !BrakeInput && 
		(HullAngularVelocity > SteeringStabilizerMinimumHullVelocity))		// Don't try to stabilize when we're to slow
	{
		if (FMath::Abs(LeftTrack.AngularVelocity * AutoBrakeStableTransfer) > FMath::Abs(RightTrack.AngularVelocity))
		{
			LeftTrack.BrakeRatio = SteeringStabilizerBrakeFactor;
		}
		else if (FMath::Abs(RightTrack.AngularVelocity * AutoBrakeStableTransfer) > FMath::Abs(LeftTrack.AngularVelocity))
		{
			RightTrack.BrakeRatio = SteeringStabilizerBrakeFactor;
		}
	}

	// Brake on speed limitation when steering
	if (LeftTrack.BrakeRatio == 0.f && RightTrack.BrakeRatio == 0.f && 
		bLimitMaxSpeed && 
		EffectiveSteeringAngularSpeed != 0.f)
	{
		const float CurrentSpeed = UpdatedComponent->GetComponentVelocity().Size();

		FRichCurve* MaxSpeedCurveData = MaxSpeedCurve.GetRichCurve();
		const float MaxSpeedLimit = MaxSpeedCurveData->Eval(FMath::Abs(EffectiveSteeringAngularSpeed));

		if (CurrentSpeed >= MaxSpeedLimit)
		{
			LeftTrack.BrakeRatio = SpeedLimitBrakeFactor;
			RightTrack.BrakeRatio = SpeedLimitBrakeFactor;
		}
	}
}

void UPrvVehicleMovementComponent::UpdateTracksVelocity(float DeltaTime)
{
	// Calc total torque
	RightTrackTorque = RightTrack.DriveTorque + RightTrack.FrictionTorque + RightTrack.RollingFrictionTorque;
	LeftTrackTorque = LeftTrack.DriveTorque + LeftTrack.FrictionTorque + LeftTrack.RollingFrictionTorque;

	// Update right track velocity
	const float RightAngularVelocity = RightTrack.AngularVelocity + RightTrackTorque / FinalMOI * DeltaTime;
	RightTrack.AngularVelocity = ApplyBrake(DeltaTime, RightAngularVelocity, RightTrack.BrakeRatio);
	RightTrack.LinearVelocity = RightTrack.AngularVelocity * SprocketRadius;

	// Update left track velocity
	const float LeftAngularVelocity = LeftTrack.AngularVelocity + LeftTrackTorque / FinalMOI * DeltaTime;
	LeftTrack.AngularVelocity = ApplyBrake(DeltaTime, LeftAngularVelocity, LeftTrack.BrakeRatio);
	LeftTrack.LinearVelocity = LeftTrack.AngularVelocity * SprocketRadius;

	// Update effective velocity
	if (bAngularVelocitySteering && !bWheeledVehicle)
	{
		// -- [Tank] --
		LeftTrack.EffectiveAngularVelocity = LeftTrack.AngularVelocity + (EffectiveSteeringAngularSpeed / SprocketRadius);
		RightTrack.EffectiveAngularVelocity = RightTrack.AngularVelocity - (EffectiveSteeringAngularSpeed / SprocketRadius);
	}
	else
	{
		LeftTrack.EffectiveAngularVelocity = LeftTrack.AngularVelocity;
		RightTrack.EffectiveAngularVelocity = RightTrack.AngularVelocity;
	}

	// Debug
	if (bShowDebug)
	{
		// Tracks torque
		DrawDebugString(GetWorld(), UpdatedComponent->GetComponentTransform().TransformPosition(FVector(0.f, -300.f, 0.f)), FString::SanitizeFloat(LeftTrackTorque), nullptr, FColor::White, 0.f);
		DrawDebugString(GetWorld(), UpdatedComponent->GetComponentTransform().TransformPosition(FVector(0.f, 300.f, 0.f)), FString::SanitizeFloat(RightTrackTorque), nullptr, FColor::White, 0.f);

		// Tracks torque
		DrawDebugString(GetWorld(), UpdatedComponent->GetComponentTransform().TransformPosition(FVector(0.f, -500.f, 0.f)), FString::SanitizeFloat(LeftTrack.AngularVelocity), nullptr, FColor::White, 0.f);
		DrawDebugString(GetWorld(), UpdatedComponent->GetComponentTransform().TransformPosition(FVector(0.f, 500.f, 0.f)), FString::SanitizeFloat(RightTrack.AngularVelocity), nullptr, FColor::White, 0.f);
	}
}

float UPrvVehicleMovementComponent::ApplyBrake(float DeltaTime, float AngularVelocity, float BrakeRatio)
{
	float BrakeVelocity = BrakeRatio * BrakeForce * DeltaTime;

	if (FMath::Abs(AngularVelocity) > FMath::Abs(BrakeVelocity))
	{
		return (AngularVelocity - (BrakeVelocity * FMath::Sign(AngularVelocity)));
	}

	return 0.f;
}

void UPrvVehicleMovementComponent::UpdateHullVelocity(float DeltaTime)
{
	HullAngularVelocity = (FMath::Abs(LeftTrack.AngularVelocity) + FMath::Abs(RightTrack.AngularVelocity)) / 2.f;
}

void UPrvVehicleMovementComponent::UpdateEngine()
{
	const FGearInfo CurrentGearInfo = GetCurrentGearInfo();

	// Update engine rotation speed (RPM)
	EngineRPM = OmegaToRPM((CurrentGearInfo.Ratio * DifferentialRatio) * HullAngularVelocity);
	EngineRPM = FMath::Clamp(EngineRPM, MinEngineRPM, MaxEngineRPM);

	// Calculate engine torque based on current RPM
	FRichCurve* TorqueCurveData = EngineTorqueCurve.GetRichCurve();
	float MaxEngineTorque = TorqueCurveData->Eval(EngineRPM);
	MaxEngineTorque *= 100.f; // From Meters to Cm

	// Check engine torque limitations
	const float CurrentSpeed = UpdatedComponent->GetComponentVelocity().Size();
	const bool LimitTorqueByRPM = bLimitEngineTorque && (EngineRPM == MaxEngineRPM);

	// Check steering limitation
	bool LimitTorqueBySpeed = false;
	if (bLimitMaxSpeed)
	{
		FRichCurve* MaxSpeedCurveData = MaxSpeedCurve.GetRichCurve();
		const float MaxSpeedLimit = MaxSpeedCurveData->Eval(FMath::Abs(EffectiveSteeringAngularSpeed));

		LimitTorqueBySpeed = (CurrentSpeed >= MaxSpeedLimit);
	}

	// Check we've reached the limit
	if (LimitTorqueBySpeed || LimitTorqueByRPM)
	{
		EngineTorque = 0.f;
	}
	else
	{
		EngineTorque = MaxEngineTorque * ThrottleInput;
	}

	// Gear box torque
	DriveTorque = EngineTorque * CurrentGearInfo.Ratio * DifferentialRatio * TransmissionEfficiency;
	DriveTorque *= (bReverseGear) ? -1.f : 1.f;
	DriveTorque *= EngineExtraPowerRatio;

	// Debug
	if (bShowDebug)
	{
		DrawDebugString(GetWorld(), UpdatedComponent->GetComponentTransform().TransformPosition(FVector(0.f, 0.f, 200.f)), FString::SanitizeFloat(EngineRPM), nullptr, FColor::Red, 0.f);
		DrawDebugString(GetWorld(), UpdatedComponent->GetComponentTransform().TransformPosition(FVector(0.f, 0.f, 250.f)), FString::SanitizeFloat(MaxEngineTorque), nullptr, FColor::White, 0.f);
		DrawDebugString(GetWorld(), UpdatedComponent->GetComponentTransform().TransformPosition(FVector(0.f, 0.f, 300.f)), FString::SanitizeFloat(DriveTorque), nullptr, FColor::Red, 0.f);
	}
}

void UPrvVehicleMovementComponent::UpdateDriveForce()
{
	// Drive force (right)
	RightTrack.DriveTorque = RightTrack.TorqueTransfer * DriveTorque;
	RightTrack.DriveForce = UpdatedComponent->GetForwardVector() * (RightTrack.DriveTorque / SprocketRadius);

	// Drive force (left)
	LeftTrack.DriveTorque = LeftTrack.TorqueTransfer * DriveTorque;
	LeftTrack.DriveForce = UpdatedComponent->GetForwardVector() * (LeftTrack.DriveTorque / SprocketRadius);
}

void UPrvVehicleMovementComponent::UpdateSuspension(float DeltaTime)
{
	// Refresh friction points counter
	const int32 ActiveWheelsNum = ActiveFrictionPoints;
	ActiveFrictionPoints = 0;
	ActiveDrivenFrictionPoints = 0;

	for (auto& SuspState : SuspensionData)
	{
		const FVector SuspUpVector = UpdatedComponent->GetComponentTransform().TransformVectorNoScale(UKismetMathLibrary::GetUpVector(SuspState.SuspensionInfo.Rotation));
		const FVector SuspWorldLocation = UpdatedComponent->GetComponentTransform().TransformPosition(SuspState.SuspensionInfo.Location);
		const FVector SuspTraceEndLocation = SuspWorldLocation - SuspUpVector * (SuspState.SuspensionInfo.Length + SuspState.SuspensionInfo.MaxDrop);

		// Make trace to touch the ground
		FHitResult Hit;
		TArray<AActor*> IgnoredActors;
		EDrawDebugTrace::Type DebugType = IsDebug() ? EDrawDebugTrace::ForOneFrame : EDrawDebugTrace::None;
		bool bHit = UKismetSystemLibrary::SphereTraceSingle_NEW(this, SuspWorldLocation, SuspTraceEndLocation, SuspState.SuspensionInfo.CollisionRadius,
			UEngineTypes::ConvertToTraceType(ECollisionChannel::ECC_Visibility), false, IgnoredActors, DebugType, Hit, true);

		// Something like true by default
		bool bHitValid = bHit;

		// Check that hit is valid (for non-spherical wheel)
		if (bHit)
		{
			// Check that collision is under suspension
			const FVector HitActorLocation = UpdatedComponent->GetComponentTransform().InverseTransformPosition(Hit.ImpactPoint);
			if (HitActorLocation.Z >= SuspState.SuspensionInfo.Location.Z)
			{
				if (bDebugSuspensionLimits)
				{
					UE_LOG(LogPrvVehicle, Warning, TEXT("Susp Hit Forced to Zero: Collision.Z: %f, Suspension.Z: %f"), HitActorLocation.Z, SuspState.SuspensionInfo.Location.Z);
				}

				// Force maximum compression
				Hit.ImpactPoint = SuspWorldLocation;
				Hit.ImpactNormal = SuspUpVector;
				Hit.Distance = 0.f;
			}

			// @todo Check that collision point is insider the wheel cylinder
		}

		// Process hit results
		if (bHitValid)
		{
			// Clamp suspension length because MaxDrop distance is for visuals only (non-effective compression)
			const float NewSuspensionLength = FMath::Clamp(Hit.Distance, 0.f, SuspState.SuspensionInfo.Length);

			const float SpringCompressionRatio = FMath::Clamp((SuspState.SuspensionInfo.Length - NewSuspensionLength) / SuspState.SuspensionInfo.Length, 0.f, 1.f);
			const float TargetVelocity = 0.f;		// @todo Target velocity can be different for wheeled vehicles

			// Original suspension velocity
			const float DiscreteSuspensionVelocity = (NewSuspensionLength - SuspState.PreviousLength) / DeltaTime;

			float SuspensionForce = 0.f;

			// Compression and decompression have different suspension quality
			float SuspensionDamping = 0.f;
			const float SuspensionStiffness = SuspState.SuspensionInfo.Stiffness * StiffnessFactor;

			if (DiscreteSuspensionVelocity < 0)
			{
				SuspensionDamping = SuspState.SuspensionInfo.CompressionDamping * CompressionDampingFactor;
			}
			else
			{
				SuspensionDamping = SuspState.SuspensionInfo.DecompressionDamping * DecompressionDampingFactor;
			}

			// Check we should correct the damping
			float SuspensionVelocity = DiscreteSuspensionVelocity;
			if (bCustomDampingCorrection && DampingCorrectionFactor != 0.f && DiscreteSuspensionVelocity != 0.f)
			{
				// Suspension velocity damping (because it works not discrete for DeltaTime)
				const float suspVel = DiscreteSuspensionVelocity / 100.f;
				const float k = SuspensionStiffness / 100.f;
				const float D = SuspensionDamping / 100.f;
				const float m = GetMesh()->GetMass();				// VehicleMass
				const float b = SuspensionDamping / (2.f * m);		// DampingCoefficient
				const float a_lin = FMath::Square(b) - (k / m);
				const float a = FMath::Sqrt(FMath::Max(1.f, a_lin));	// FrictionCoefficient
				const float A = suspVel / (2.f * a);				// InitialDampingEffect
				const float B = -A;
				const float dL_old = suspVel * DeltaTime;
				const float dL_new = FMath::Exp(-b * DeltaTime) * (A * FMath::Exp(a * DeltaTime) + B * FMath::Exp(-a * DeltaTime));
				const float Kl = dL_new / dL_old;
				SuspensionVelocity = suspVel * FMath::Pow(Kl, DampingCorrectionFactor);

				if (bDebugDampingCorrection)
				{
					if (a_lin < 1.f)
					{
						UE_LOG(LogPrvVehicle, Error, TEXT("a_lin is too small: %f"), a_lin);
					}

					UE_LOG(LogPrvVehicle, Warning, TEXT("DeltaTime: %f, suspVel: %f, k: %f, m: %f, D: %f, a: %f, b: %f, k/m: %f, A: %f, dL_old: %f, dL_new: %f, suspVelCorrected: %f"),
						DeltaTime, suspVel, k, m, D, a, b, (k / m), A, dL_old, dL_new, SuspensionVelocity);
				}
			}

			// Adaptive damping correction
			if (bAdaptiveDampingCorrection)
			{
				const float D = SuspensionDamping / 100.f;
				const float m = GetMesh()->GetMass();				// VehicleMass

				const float AdaptiveExp = (1 - FMath::Exp((-D) * ActiveWheelsNum / m * DeltaTime));
				if (AdaptiveExp != 0.f)
				{
					const float AdaptiveSuspensionDamping = AdaptiveExp * m / (ActiveWheelsNum * DeltaTime);

					if (bDebugDampingCorrection)
					{
						UE_LOG(LogPrvVehicle, Warning, TEXT("SuspensionDamping: %f, AdaptiveSuspensionDamping: %f, ActiveWheelsNum: %d"),
							SuspensionDamping, (AdaptiveSuspensionDamping * 100.f), ActiveWheelsNum);
					}

					SuspensionDamping = AdaptiveSuspensionDamping * 100.f;
				}
				else if (bDebugDampingCorrection)
				{
					UE_LOG(LogPrvVehicle, Warning, TEXT("SuspensionDamping: %f, AdaptiveExp: 0"), SuspensionDamping);
				}
			}
			
			// Apply suspension force
			SuspensionForce = (TargetVelocity - SuspensionVelocity) * SuspensionDamping + SpringCompressionRatio * SuspensionStiffness;
			SuspState.SuspensionForce = SuspensionForce * SuspUpVector;

			SuspState.WheelCollisionLocation = Hit.ImpactPoint;
			SuspState.WheelCollisionNormal = Hit.ImpactNormal;
			SuspState.PreviousLength = NewSuspensionLength;
			SuspState.WheelTouchedGround = true;
			SuspState.SurfaceType = UGameplayStatics::GetSurfaceType(Hit);

			if (SuspState.VisualLength < Hit.Distance)
			{
				SuspState.VisualLength = FMath::Lerp(SuspState.VisualLength, Hit.Distance, FMath::Clamp(DeltaTime * DropFactor, 0.f, 1.f));
			}
			else
			{
				SuspState.VisualLength = Hit.Distance;
			}

			// Current wheel touches ground
			ActiveFrictionPoints++;

			// Active driving wheels are calculated separately (has sense for cars only)
			if (!bWheeledVehicle || SuspState.SuspensionInfo.bDrivingWheel)
			{
				ActiveDrivenFrictionPoints++;
			}
		}
		else
		{
			// If there is no collision then suspension is relaxed
			SuspState.SuspensionForce = FVector::ZeroVector;
			SuspState.WheelCollisionLocation = FVector::ZeroVector;
			SuspState.WheelCollisionNormal = FVector::UpVector;
			SuspState.PreviousLength = SuspState.SuspensionInfo.Length;
			SuspState.VisualLength = FMath::Lerp(SuspState.VisualLength, SuspState.SuspensionInfo.Length + SuspState.SuspensionInfo.MaxDrop, FMath::Clamp(DeltaTime * DropFactor, 0.f, 1.f));		// @todo Make it non-momental
			SuspState.WheelTouchedGround = false;
			SuspState.SurfaceType = EPhysicalSurface::SurfaceType_Default;
		}

		// Add suspension force if spring compressed
		if (!SuspState.SuspensionForce.IsZero())
		{
			GetMesh()->AddForceAtLocation(SuspState.SuspensionForce, SuspWorldLocation);
		}

		// Push suspension force to environment
		if (bHit)
		{
			UPrimitiveComponent* PrimitiveComponent = Hit.Component.Get();
			if (PrimitiveComponent)
			{
				// Generate hit event
				if (bNotifyRigidBodyCollision)
				{
					GetMesh()->DispatchBlockingHit(*GetOwner(), Hit);
				}

				// Push the force
				if (PrimitiveComponent->IsSimulatingPhysics())
				{
					PrimitiveComponent->AddForceAtLocation(-SuspState.SuspensionForce, SuspWorldLocation);
				}
			}
		}

		// Debug
		if (bShowDebug)
		{
			// Suspension force
			DrawDebugLine(GetWorld(), SuspWorldLocation, SuspWorldLocation + SuspState.SuspensionForce * 0.0001f, FColor::Green, false, /*LifeTime*/ 0.f, /*DepthPriority*/ 0,  /*Thickness*/ 4.f);

			// Suspension length
			DrawDebugPoint(GetWorld(), SuspWorldLocation, 5.f, FColor(200, 0, 230), false, /*LifeTime*/ 0.f);
			DrawDebugLine(GetWorld(), SuspWorldLocation, SuspWorldLocation - SuspUpVector * SuspState.PreviousLength, FColor::Blue, false, 0.f, 0, 4.f);
			DrawDebugLine(GetWorld(), SuspWorldLocation, SuspWorldLocation - SuspUpVector * SuspState.SuspensionInfo.Length, FColor::Red, false, 0.f, 0, 2.f);

			// Draw wheel
			if (bHit)
			{
				FColor WheelColor = bHitValid ? FColor::Cyan : FColor::White;
				FVector LineOffset = UpdatedComponent->GetComponentTransform().GetRotation().RotateVector(FVector(0.f, SuspState.SuspensionInfo.CollisionWidth / 2.f, 0.f));
				LineOffset = SuspState.SuspensionInfo.Rotation.RotateVector(LineOffset);
				DrawDebugCylinder(GetWorld(), Hit.Location - LineOffset, Hit.Location + LineOffset, SuspState.SuspensionInfo.CollisionRadius, 16, WheelColor, false, /*LifeTime*/ 0.f, 100);
			}
		}
	}
}

void UPrvVehicleMovementComponent::UpdateFriction(float DeltaTime)
{
	// Reset tracks friction
	RightTrack.FrictionTorque = 0.f;
	RightTrack.RollingFrictionTorque = 0.f;
	LeftTrack.FrictionTorque = 0.f;
	LeftTrack.RollingFrictionTorque = 0.f;

	// Process suspension
	for (auto& SuspState : SuspensionData)
	{
		if (SuspState.WheelTouchedGround)
		{
			// Cache current track info
			FTrackInfo* WheelTrack = (SuspState.SuspensionInfo.bRightTrack) ? &RightTrack : &LeftTrack;

			/////////////////////////////////////////////////////////////////////////
			// Drive force

			// Calculate wheel load
			SuspState.WheelLoad = UKismetMathLibrary::ProjectVectorOnToVector(SuspState.SuspensionForce, SuspState.WheelCollisionNormal).Size();

			// Wheel forward vector
			const FVector WheelDirection = SuspState.SuspensionInfo.Rotation.RotateVector(GetMesh()->GetForwardVector());

			// Get Velocity at location
			FVector WorldPointVelocity = FVector::ZeroVector;
			if (bUseCustomVelocityCalculations)
			{
				const FVector PlaneLocalVelocity = GetOwner()->GetTransform().InverseTransformVectorNoScale(GetMesh()->GetPhysicsLinearVelocity());
				const FVector PlaneAngularVelocity = GetOwner()->GetTransform().InverseTransformVectorNoScale(GetMesh()->GetPhysicsAngularVelocity());
				const FVector LocalCOM = GetOwner()->GetTransform().InverseTransformPosition(GetMesh()->GetCenterOfMass());
				const FVector LocalCollisionLocation = GetOwner()->GetTransform().InverseTransformPosition(SuspState.WheelCollisionLocation);
				const FVector LocalPointVelocity = PlaneLocalVelocity + FVector::CrossProduct(FMath::DegreesToRadians(PlaneAngularVelocity), (LocalCollisionLocation - LocalCOM));
				WorldPointVelocity = GetOwner()->GetTransform().TransformVectorNoScale(LocalPointVelocity);
			}
			else
			{
				WorldPointVelocity = GetMesh()->GetPhysicsLinearVelocityAtPoint(SuspState.WheelCollisionLocation);
			}

			// Calculate wheel velocity relative to track (with simple Kalman filter)
			const FVector WheelCollisionVelocity = (WorldPointVelocity + SuspState.PreviousWheelCollisionVelocity) / 2.f;

			// Cache last velocity
			SuspState.PreviousWheelCollisionVelocity = WheelCollisionVelocity;

			// Apply linear friction
			FVector WheelVelocity = FVector::ZeroVector - WheelCollisionVelocity;

			// Add driving force
			if (!bWheeledVehicle || SuspState.SuspensionInfo.bDrivingWheel)
			{
				WheelVelocity += (WheelDirection * WheelTrack->LinearVelocity);
			}

			const FVector RelativeWheelVelocity = UKismetMathLibrary::ProjectVectorOnToPlane(WheelVelocity, SuspState.WheelCollisionNormal);

			// Get friction coefficients
			float MuStatic = CalculateFrictionCoefficient(RelativeWheelVelocity, WheelDirection, StaticFrictionCoefficientEllipse);
			float MuKinetic = CalculateFrictionCoefficient(RelativeWheelVelocity, WheelDirection, KineticFrictionCoefficientEllipse);

			// Mass and friction forces
			const float VehicleMass = GetMesh()->GetMass();
			const FVector FrictionXVector = UKismetMathLibrary::ProjectVectorOnToPlane(GetMesh()->GetForwardVector(), SuspState.WheelCollisionNormal).GetSafeNormal();
			const FVector FrictionYVector = UKismetMathLibrary::ProjectVectorOnToPlane(GetMesh()->GetRightVector(), SuspState.WheelCollisionNormal).GetSafeNormal();

			// Current wheel force contbution
			const FVector WheelBalancedForce = (ActiveFrictionPoints != 0) ? (RelativeWheelVelocity * VehicleMass / DeltaTime / ActiveFrictionPoints) : FVector::ZeroVector;

			// @temp For non-driving wheels X friction is disabled
			float LongitudeFrictionFactor = 1.f;
			if (bWheeledVehicle && !SuspState.SuspensionInfo.bDrivingWheel)
			{
				LongitudeFrictionFactor = 0.f;
			}

			// Full friction forces
			const FVector FullStaticFrictionForce =
				UKismetMathLibrary::ProjectVectorOnToVector(WheelBalancedForce, FrictionXVector) * StaticFrictionCoefficientEllipse.X  * LongitudeFrictionFactor +
				UKismetMathLibrary::ProjectVectorOnToVector(WheelBalancedForce, FrictionYVector) * StaticFrictionCoefficientEllipse.Y;
			const FVector FullKineticFrictionForce =
				UKismetMathLibrary::ProjectVectorOnToVector(WheelBalancedForce, FrictionXVector) * KineticFrictionCoefficientEllipse.X * LongitudeFrictionFactor +
				UKismetMathLibrary::ProjectVectorOnToVector(WheelBalancedForce, FrictionYVector) * KineticFrictionCoefficientEllipse.Y;

			// Drive Force from transmission torque
			const FVector TransmissionDriveForce = UKismetMathLibrary::ProjectVectorOnToPlane(WheelTrack->DriveForce, SuspState.WheelCollisionNormal);

			// Full drive forces
			const FVector FullStaticDriveForce = TransmissionDriveForce * StaticFrictionCoefficientEllipse.X * LongitudeFrictionFactor;
			const FVector FullKineticDriveForce = TransmissionDriveForce * KineticFrictionCoefficientEllipse.X * LongitudeFrictionFactor;

			// Full forces
			const FVector FullStaticForce = FullStaticDriveForce + FullStaticFrictionForce;
			const FVector FullKineticForce = FullKineticDriveForce + FullKineticFrictionForce;

			// We want to apply higher friction if forces are bellow static friction limit
			bool bUseKineticFriction = FullStaticForce.Size() >= (SuspState.WheelLoad * MuStatic);
			const FVector FullFrictionNormalizedForce = bUseKineticFriction ? FullKineticFrictionForce.GetSafeNormal() : FullStaticFrictionForce.GetSafeNormal();
			const FVector ApplicationForce = bUseKineticFriction
				? FullKineticForce.GetClampedToMaxSize(SuspState.WheelLoad * MuKinetic)
				: FullStaticForce.GetClampedToMaxSize(SuspState.WheelLoad * MuStatic);

			// Apply force to mesh
			GetMesh()->AddForceAtLocation(ApplicationForce, SuspState.WheelCollisionLocation);


			/////////////////////////////////////////////////////////////////////////
			// Friction torque

			// Friction should work agains real movement
			float FrictionDirectionMultiplier = FMath::Sign(WheelTrack->AngularVelocity) * FMath::Sign(WheelTrack->TorqueTransfer) * ((bReverseGear) ? (-1.f) : 1.f);
			if (FrictionDirectionMultiplier == 0.f) FrictionDirectionMultiplier = 1.f;

			// How much of friction force would effect transmission
			const FVector TransmissionFrictionForce = UKismetMathLibrary::ProjectVectorOnToVector(ApplicationForce, FullFrictionNormalizedForce) * (-1.f) * (TrackMass + SprocketMass) / VehicleMass * FrictionDirectionMultiplier;
			const FVector WorldFrictionForce = UpdatedComponent->GetComponentTransform().InverseTransformVectorNoScale(TransmissionFrictionForce);
			const float TrackFrictionTorque = UKismetMathLibrary::ProjectVectorOnToVector(WorldFrictionForce, FVector::ForwardVector).X * SprocketRadius;

			// Add torque to track
			WheelTrack->FrictionTorque += (TrackFrictionTorque * FrictionTorqueCoefficient);


			/////////////////////////////////////////////////////////////////////////
			// Rolling friction torque

			// @todo Make this a force instead of torque!
			const float ReverseVelocitySign = (-1.f) * FMath::Sign(WheelTrack->LinearVelocity);
			const float TrackRollingFrictionTorque = SuspState.WheelLoad * RollingFrictionCoefficient * ReverseVelocitySign +
				SuspState.WheelLoad * (WheelTrack->LinearVelocity * RollingVelocityCoefficient) * ReverseVelocitySign;

			// Add torque to track
			WheelTrack->RollingFrictionTorque += TrackRollingFrictionTorque;


			/////////////////////////////////////////////////////////////////////////
			// Debug

			if (bShowDebug)
			{
				// Friction type
				if (bUseKineticFriction)
				{
					DrawDebugString(GetWorld(), SuspState.WheelCollisionLocation, TEXT("Kinetic"), nullptr, FColor::Blue, 0.f);
				}
				else
				{
					DrawDebugString(GetWorld(), SuspState.WheelCollisionLocation, TEXT("Static"), nullptr, FColor::Red, 0.f);
				}

				// Force application
				DrawDebugLine(GetWorld(), SuspState.WheelCollisionLocation, SuspState.WheelCollisionLocation + ApplicationForce * 0.0001f, FColor::Cyan, false, 0.f, 0, 10.f);

				// Wheel velocity vectors
				DrawDebugLine(GetWorld(), SuspState.WheelCollisionLocation, SuspState.WheelCollisionLocation + WheelCollisionVelocity, FColor::Yellow, false, 0.f, 0, 8.f);
				DrawDebugLine(GetWorld(), SuspState.WheelCollisionLocation, SuspState.WheelCollisionLocation + RelativeWheelVelocity, FColor::Blue, false, 0.f, 0, 8.f);
			}
		}
		else 
		{
			// Reset wheel load
			SuspState.WheelLoad = 0.f;
		}
	}
}

float UPrvVehicleMovementComponent::CalculateFrictionCoefficient(FVector DirectionVelocity, FVector ForwardVector, FVector2D FrictionEllipse)
{
	// dot(A,B)
	float DirectionDotProduct = FVector::DotProduct(DirectionVelocity.GetSafeNormal(), ForwardVector);

	FVector2D MuVector;
	// x = r1 * dot(A,B)
	MuVector.X = FrictionEllipse.X * DirectionDotProduct;
	// y = r2 * sqrt(1 - dot(A,B)^2 )
	MuVector.Y = FrictionEllipse.Y * FMath::Sqrt(1.f - FMath::Square(DirectionDotProduct));

	return MuVector.Size();
}

void UPrvVehicleMovementComponent::UpdateLinearVelocity(float DeltaTime)
{
	if (bCustomLinearDamping)
	{
		const FVector LocalLinearVelocity = UpdatedComponent->GetComponentTransform().InverseTransformVectorNoScale(GetMesh()->GetPhysicsLinearVelocity());
		const FVector SignVector = FVector(FMath::Sign(LocalLinearVelocity.X), FMath::Sign(LocalLinearVelocity.Y), FMath::Sign(LocalLinearVelocity.Z));
		FVector NewLinearVelocity = LocalLinearVelocity - DeltaTime * (SignVector * DryFrictionLinearDamping + FluidFrictionLinearDamping * LocalLinearVelocity);

		// Clamp to zero vector in per-component basis
		NewLinearVelocity.X = SignVector.X * FMath::Max(0.f, SignVector.X * NewLinearVelocity.X);
		NewLinearVelocity.Y = SignVector.Y * FMath::Max(0.f, SignVector.Y * NewLinearVelocity.Y);
		NewLinearVelocity.Z = SignVector.Z * FMath::Max(0.f, SignVector.Z * NewLinearVelocity.Z);

		GetMesh()->SetPhysicsLinearVelocity(UpdatedComponent->GetComponentTransform().TransformVectorNoScale(NewLinearVelocity));

		if (bDebugCustomDamping)
		{
			UE_LOG(LogPrvVehicle, Error, TEXT("Linear damping WAS: %s, NOW: %s"), *LocalLinearVelocity.ToString(), *NewLinearVelocity.ToString());
		}
	}
}

void UPrvVehicleMovementComponent::UpdateAngularVelocity(float DeltaTime)
{
	if (bCustomAngularDamping)
	{
		const FVector LocalAngularVelocity = UpdatedComponent->GetComponentTransform().InverseTransformVectorNoScale(GetMesh()->GetPhysicsAngularVelocity());
		const FVector SignVector = FVector(FMath::Sign(LocalAngularVelocity.X), FMath::Sign(LocalAngularVelocity.Y), FMath::Sign(LocalAngularVelocity.Z));
		FVector NewAngularVelocity = LocalAngularVelocity - DeltaTime * (SignVector * DryFrictionAngularDamping + FluidFrictionAngularDamping * LocalAngularVelocity);

		// Clamp to zero vector in per-component basis
		NewAngularVelocity.X = SignVector.X * FMath::Max(0.f, SignVector.X * NewAngularVelocity.X);
		NewAngularVelocity.Y = SignVector.Y * FMath::Max(0.f, SignVector.Y * NewAngularVelocity.Y);
		NewAngularVelocity.Z = SignVector.Z * FMath::Max(0.f, SignVector.Z * NewAngularVelocity.Z);

		GetMesh()->SetPhysicsAngularVelocity(UpdatedComponent->GetComponentTransform().TransformVectorNoScale(NewAngularVelocity));

		if (bDebugCustomDamping)
		{
			UE_LOG(LogPrvVehicle, Error, TEXT("Angular damping WAS: %s, NOW: %s"), *LocalAngularVelocity.ToString(), *NewAngularVelocity.ToString());
		}
	}
}

void UPrvVehicleMovementComponent::AnimateWheels(float DeltaTime)
{
	for (auto& SuspState : SuspensionData)
	{
		FTrackInfo* WheelTrack = (SuspState.SuspensionInfo.bRightTrack) ? &RightTrack : &LeftTrack;

		SuspState.RotationAngle -= FMath::RadiansToDegrees(WheelTrack->EffectiveAngularVelocity) * DeltaTime * (SprocketRadius / SuspState.SuspensionInfo.CollisionRadius);
		SuspState.SteeringAngle = SuspState.SuspensionInfo.Rotation.Yaw;
	}
}


//////////////////////////////////////////////////////////////////////////
// Network

bool UPrvVehicleMovementComponent::ServerUpdateState_Validate(float InSteeringInput, float InThrottleInput, uint32 InHandbrakeInput, int32 InCurrentGear)
{
	return true;
}

void UPrvVehicleMovementComponent::ServerUpdateState_Implementation(float InSteeringInput, float InThrottleInput, uint32 InHandbrakeInput, int32 InCurrentGear)
{
	SetSteeringInput(InSteeringInput);
	SetThrottleInput(InThrottleInput);

	bRawHandbrakeInput = InHandbrakeInput;
	CurrentGear = InCurrentGear;
}


//////////////////////////////////////////////////////////////////////////
// Vehicle control

void UPrvVehicleMovementComponent::SetThrottleInput(float Throttle)
{
	RawThrottleInput = FMath::Clamp(Throttle, -1.0f, 1.0f);
}

void UPrvVehicleMovementComponent::SetSteeringInput(float Steering)
{
	RawSteeringInput = FMath::Clamp(Steering, -1.0f, 1.0f);
}

void UPrvVehicleMovementComponent::SetHandbrakeInput(bool bNewHandbrake)
{
	bRawHandbrakeInput = bNewHandbrake;
}

void UPrvVehicleMovementComponent::EnableMovement()
{
	bIsMovementEnabled = true;
}

void UPrvVehicleMovementComponent::DisableMovement()
{
	bIsMovementEnabled = false;
}

bool UPrvVehicleMovementComponent::IsMoving() const
{
	return bIsMovementEnabled && HasInput();
}


//////////////////////////////////////////////////////////////////////////
// Vehicle stats

float UPrvVehicleMovementComponent::GetForwardSpeed() const
{
	const float VelocityDirection = FVector::DotProduct(UpdatedComponent->GetForwardVector(), UpdatedComponent->GetComponentVelocity());
	return UpdatedComponent->GetComponentVelocity().Size() * ((VelocityDirection >= 0.f) ? 1.f : -1.f);
}

float UPrvVehicleMovementComponent::GetThrottle() const
{
	return ThrottleInput;
}

float UPrvVehicleMovementComponent::GetEngineRotationSpeed() const
{
	return EngineRPM;
}

float UPrvVehicleMovementComponent::GetEngineMaxRotationSpeed() const
{
	return MaxEngineRPM;
}

float UPrvVehicleMovementComponent::GetEngineTorque() const
{
	return EngineTorque;
}

float UPrvVehicleMovementComponent::GetDriveTorqueLeft() const
{
	return LeftTrack.DriveTorque;
}

float UPrvVehicleMovementComponent::GetDriveTorqueRight() const
{
	return RightTrack.DriveTorque;
}

float UPrvVehicleMovementComponent::GetAngularVelocityLeft() const
{
	return LeftTrack.AngularVelocity;
}

float UPrvVehicleMovementComponent::GetAngularVelocityRight() const
{
	return RightTrack.AngularVelocity;
}

float UPrvVehicleMovementComponent::GetBrakeRatioLeft() const
{
	return LeftTrack.BrakeRatio;
}

float UPrvVehicleMovementComponent::GetBrakeRatioRight() const
{
	return RightTrack.BrakeRatio;
}

bool UPrvVehicleMovementComponent::HasTouchGround() const
{
	return (ActiveFrictionPoints > 0);
}


//////////////////////////////////////////////////////////////////////////
// Data access

USkinnedMeshComponent* UPrvVehicleMovementComponent::GetMesh()
{
	return Cast<USkinnedMeshComponent>(UpdatedComponent);
}

void UPrvVehicleMovementComponent::GetTrackInfoLeft(FTrackInfo& OutTrack) const
{
	OutTrack = LeftTrack;
}

void UPrvVehicleMovementComponent::GetTrackInfoRight(FTrackInfo& OutTrack) const
{
	OutTrack = RightTrack;
}

int32 UPrvVehicleMovementComponent::GetCurrentGear() const
{
	return CurrentGear;
}

int32 UPrvVehicleMovementComponent::GetNeutralGear() const
{
	return NeutralGear;
}

bool UPrvVehicleMovementComponent::IsCurrentGearReverse() const
{
	return (CurrentGear < NeutralGear);
}

FGearInfo UPrvVehicleMovementComponent::GetGearInfo(int32 GearNum) const
{
	// Check that requested gear is valid
	if (GearNum < 0 || GearNum >= GearSetup.Num())
	{
		UE_LOG(LogPrvVehicle, Error, TEXT("Invalid gear index: %d from %d"), GearNum, GearSetup.Num());
		return FGearInfo();
	}

	return GearSetup[GearNum];
}

FGearInfo UPrvVehicleMovementComponent::GetCurrentGearInfo() const
{
	return GetGearInfo(CurrentGear);
}


//////////////////////////////////////////////////////////////////////////
// Effects

void UPrvVehicleMovementComponent::UpdateWheelEffects(float DeltaTime)
{
	if (DustEffect)
	{
		const float CurrentSpeed = UpdatedComponent->GetComponentVelocity().Size();

		// Process suspension
		for (auto& SuspState : SuspensionData)
		{
			if (SuspState.SuspensionInfo.bSpawnDust)
			{
				// Get vfx corresponding the surface
				UParticleSystem* WheelFX = DustEffect->GetDustFX(SuspState.SurfaceType, CurrentSpeed);

				// Check current one is active
				const bool bIsVfxActive = SuspState.DustPSC != nullptr && !SuspState.DustPSC->bWasDeactivated && !SuspState.DustPSC->bWasCompleted;

				// Check wheel is touched ground (should spawn dust)
				if (SuspState.WheelTouchedGround)
				{
					UParticleSystem* CurrentFX = SuspState.DustPSC != nullptr ? SuspState.DustPSC->Template : nullptr;

					// Check we need to spawn dust or change the effect
					if (WheelFX != nullptr && (CurrentFX != WheelFX || !bIsVfxActive))
					{
						if (SuspState.DustPSC == nullptr || !SuspState.DustPSC->bWasDeactivated)
						{
							if (SuspState.DustPSC != nullptr)
							{
								SuspState.DustPSC->SetActive(false);
								SuspState.DustPSC->bAutoDestroy = true;
							}

							SuspState.DustPSC = SpawnNewWheelEffect();
						}

						// Update effect location
						SuspState.DustPSC->SetRelativeRotation(SuspState.WheelCollisionNormal.Rotation());
						SuspState.DustPSC->SetWorldLocation(SuspState.WheelCollisionLocation);

						// Reactivate effect
						SuspState.DustPSC->SetTemplate(WheelFX);
						SuspState.DustPSC->ActivateSystem();
					}
					// Deactivate if no suitable VFX is found for surface type
					else if (WheelFX == nullptr && bIsVfxActive)
					{
						SuspState.DustPSC->SetActive(false);
					}
				}
				// Deactivate particles on ground untouch
				else if (bIsVfxActive)
				{
					SuspState.DustPSC->SetActive(false);
				}

				// Update effect location
				SuspState.DustPSC->SetRelativeRotation(SuspState.WheelCollisionNormal.Rotation());
				SuspState.DustPSC->SetWorldLocation(SuspState.WheelCollisionLocation);
			}
		}
	}
}

UParticleSystemComponent* UPrvVehicleMovementComponent::SpawnNewWheelEffect(FName InSocketName, FVector InSocketOffset)
{
	UParticleSystemComponent* DustPSC = NewObject<UParticleSystemComponent>(this);
	DustPSC->bAutoActivate = true;
	DustPSC->bAutoDestroy = false;
	DustPSC->RegisterComponentWithWorld(GetWorld());
	DustPSC->AttachToComponent(GetMesh(), FAttachmentTransformRules::SnapToTargetIncludingScale, InSocketName);
	DustPSC->SetRelativeLocation(InSocketOffset);

	return DustPSC;
}


//////////////////////////////////////////////////////////////////////////
// Debug

void UPrvVehicleMovementComponent::DrawDebug(UCanvas* Canvas, float& YL, float& YPos)
{
	// Force draw debug lines
	bShowDebug = true;

	// @todo 
}

void UPrvVehicleMovementComponent::DrawDebugLines()
{
	if (bIsSleeping)
	{
		DrawDebugString(GetWorld(), GetMesh()->GetCenterOfMass(), TEXT("SLEEP"), nullptr, FColor::Red, 0.f);
	}
	else
	{
		DrawDebugPoint(GetWorld(), GetMesh()->GetCenterOfMass(), 25.f, FColor::Yellow, false, /*LifeTime*/ 0.f);
	}
}


//////////////////////////////////////////////////////////////////////////
// Internal data

bool UPrvVehicleMovementComponent::HasInput() const
{
	return (RawThrottleInput != 0.f) || (RawSteeringInput != 0.f) || (bRawHandbrakeInput != 0);
}


//////////////////////////////////////////////////////////////////////////
// Replication

void UPrvVehicleMovementComponent::GetLifetimeReplicatedProps(TArray< FLifetimeProperty > & OutLifetimeProps) const
{
	Super::GetLifetimeReplicatedProps(OutLifetimeProps);

	DOREPLIFETIME(UPrvVehicleMovementComponent, bIsSleeping);
	DOREPLIFETIME(UPrvVehicleMovementComponent, bIsMovementEnabled);
}
