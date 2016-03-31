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

	SprocketMass = 65.f;
	SprocketRadius = 25.f;
	TrackMass = 600.f;

	bAutoGear = true;

	BrakeForce = 30.f;
	SteeringBrakeFactor = 1.f;
	SteeringBrakeTransfer = 0.7f;
	AutoBrakeStableTransfer = 0.9f;

	DifferentialRatio = 3.5f;
	TransmissionEfficiency = 0.9f;
	EngineExtraPowerRatio = 3.f;

	StaticFrictionCoefficientEllipse = FVector2D(1.f, 0.85f);
	KineticFrictionCoefficientEllipse = FVector2D(0.5f, 0.45f);

	FrictionTorqueCoefficient = 1.f;
	RollingFrictionCoefficient = 0.02f;
	RollingVelocityCoefficient = 0.000015f;

	DampingFactor = 1.f;
	StiffnessFactor = 1.f;

	// Init basic torque curve
	FRichCurve* TorqueCurveData = EngineTorqueCurve.GetRichCurve();
	TorqueCurveData->AddKey(0.f, 800.f);
	TorqueCurveData->AddKey(1400.f, 850.f);
	TorqueCurveData->AddKey(2800.f, 800.f);
	TorqueCurveData->AddKey(2810.f, 0.f);
}

//////////////////////////////////////////////////////////////////////////
// Initialization

void UPrvVehicleMovementComponent::InitializeComponent()
{
	Super::InitializeComponent();

	CalculateMOI();
	InitSuspension();
	InitGears();
}

void UPrvVehicleMovementComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UpdateSuspension(DeltaTime);
	UpdateFriction(DeltaTime);

	UpdateThrottle(DeltaTime);
	UpdateBrake(DeltaTime);

	UpdateTracksVelocity(DeltaTime);
	UpdateHullVelocity(DeltaTime);
	UpdateEngine(DeltaTime);
	UpdateDriveForce(DeltaTime);

	// @todo Reset input

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

void UPrvVehicleMovementComponent::InitSuspension()
{
	for (auto& SuspInfo : SuspensionSetup)
	{
		if (SuspInfo.bInheritWheelBoneTransform)
		{
			USkinnedMeshComponent* Mesh = GetMesh();
			if (Mesh)
			{
				FTransform WheelTransform = Mesh->GetSocketTransform(SuspInfo.BoneName, RTS_Actor);
				SuspInfo.Location = WheelTransform.GetLocation();
				SuspInfo.Rotation = WheelTransform.GetRotation().Rotator();

				UE_LOG(LogPrvVehicle, Log, TEXT("Init suspension (%s): %s"), *SuspInfo.BoneName.ToString(), *SuspInfo.Location.ToString());
			}
		}

		FSuspensionState SuspState;
		SuspState.SuspensionInfo = SuspInfo;
		SuspState.PreviousLength = SuspInfo.Length;

		SuspensionData.Add(SuspState);
	}
}

void UPrvVehicleMovementComponent::InitGears()
{
	for (int32 i = 0; i < GearSetup.Num(); i++)
	{
		if (GearSetup[i].Ratio == 0.f)
		{
			NeutralGear = (bAutoGear) ? i : FMath::Max(i + 1, GearSetup.Num());
			break;
		}
	}

	UE_LOG(LogPrvVehicle, Warning, TEXT("Neutral gear: %d"), NeutralGear);
}


//////////////////////////////////////////////////////////////////////////
// Physics simulation

void UPrvVehicleMovementComponent::UpdateThrottle(float DeltaTime)
{
	// @todo Throttle shouldn't be instant
	ThrottleInput = 1.f;// RawThrottleInput;

	// Calc torque transfer based on input
	LeftTrack.TorqueTransfer = 1 * (FMath::Abs(RawThrottleInput) + LeftTrack.Input);
	RightTrack.TorqueTransfer = 1 * (FMath::Abs(RawThrottleInput) + RightTrack.Input);

	// Debug
	if (bShowDebug)
	{
		// Torque transfer balance
		DrawDebugString(GetWorld(), UpdatedComponent->GetComponentTransform().TransformPosition(FVector(0.f, -100.f, 0.f)), FString::SanitizeFloat(LeftTrack.TorqueTransfer), nullptr, FColor::White, 0.f);
		DrawDebugString(GetWorld(), UpdatedComponent->GetComponentTransform().TransformPosition(FVector(0.f, 100.f, 0.f)), FString::SanitizeFloat(RightTrack.TorqueTransfer), nullptr, FColor::White, 0.f);
	}
}

void UPrvVehicleMovementComponent::UpdateBrake(float DeltaTime)
{
	// Handbrake first
	LeftTrack.BrakeRatio = bRawHandbrakeInput;
	RightTrack.BrakeRatio = bRawHandbrakeInput;

	// Manual brake for rotation
	if ((LeftTrack.Input < 0.f) && (LeftTrack.AngularVelocity >= (RightTrack.AngularVelocity * SteeringBrakeTransfer)))
	{
		LeftTrack.BrakeRatio = (-1.f) * LeftTrack.Input * SteeringBrakeFactor;
	}
	else if ((RightTrack.Input < 0.f) && (RightTrack.AngularVelocity >= (LeftTrack.AngularVelocity * SteeringBrakeTransfer)))
	{
		RightTrack.BrakeRatio = (-1.f) * RightTrack.Input * SteeringBrakeFactor;
	}

	// Stabilize steering
	if (bAutoGear && (SteeringInput == 0.f) && !bRawHandbrakeInput)
	{
		if ((LeftTrack.AngularVelocity * AutoBrakeStableTransfer) > RightTrack.AngularVelocity)
		{
			LeftTrack.BrakeRatio = 1.f;
		}
		else if ((RightTrack.AngularVelocity * AutoBrakeStableTransfer) > LeftTrack.AngularVelocity)
		{
			RightTrack.BrakeRatio = 1.f;
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

void UPrvVehicleMovementComponent::UpdateEngine(float DeltaTime)
{
	const FGearInfo CurrentGearInfo = GetGearInfo(GetCurrentGear());

	// @todo Cache engine RPM limits
	float MinRPM, MaxRPM;
	FRichCurve* TorqueCurveData = EngineTorqueCurve.GetRichCurve();
	TorqueCurveData->GetTimeRange(MinRPM, MaxRPM);

	// Update engine rotation speed (RPM)
	EngineRPM = OmegaToRPM((CurrentGearInfo.Ratio * DifferentialRatio) * HullAngularVelocity);
	EngineRPM = FMath::Clamp(EngineRPM, MinRPM, MaxRPM);

	// Calculate engine torque based on current RPM
	float MaxEngineTorque = TorqueCurveData->Eval(EngineRPM);
	MaxEngineTorque *= 100.f; // From Meters to Cm
	EngineTorque = MaxEngineTorque * ThrottleInput;

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

void UPrvVehicleMovementComponent::UpdateDriveForce(float DeltaTime)
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
	ActiveFrictionPoints = 0;

	for (auto& SuspState : SuspensionData)
	{
		const FVector SuspUpVector = UpdatedComponent->GetComponentTransform().TransformVectorNoScale(UKismetMathLibrary::GetUpVector(SuspState.SuspensionInfo.Rotation));
		const FVector SuspWorldLocation = UpdatedComponent->GetComponentTransform().TransformPosition(SuspState.SuspensionInfo.Location);
		const FVector SuspTraceEndLocation = SuspWorldLocation - SuspUpVector * SuspState.SuspensionInfo.Length;

		// Make trace to touch the ground
		FHitResult Hit;
		TArray<AActor*> IgnoredActors;
		EDrawDebugTrace::Type DebugType = IsDebug() ? EDrawDebugTrace::ForOneFrame : EDrawDebugTrace::None;
		bool bHit = UKismetSystemLibrary::SphereTraceSingle_NEW(this, SuspWorldLocation, SuspTraceEndLocation, SuspState.SuspensionInfo.CollisionRadius,
			UEngineTypes::ConvertToTraceType(ECollisionChannel::ECC_Visibility), false, IgnoredActors, DebugType, Hit, true);

		// Process hit results
		if (bHit)
		{
			const float NewSuspensionLength = (SuspWorldLocation - Hit.Location).Size();

			const float SpringCompressionRatio = FMath::Clamp((SuspState.SuspensionInfo.Length - NewSuspensionLength) / SuspState.SuspensionInfo.Length, 0.f, 1.f);
			const float TargetVelocity = 0.f;		// @todo Target velocity can be different for wheeled vehicles
			const float SuspensionVelocity = (NewSuspensionLength - SuspState.PreviousLength) / DeltaTime;
			const float SuspensionForce = (TargetVelocity - SuspensionVelocity) * SuspState.SuspensionInfo.Damping * DampingFactor + 
				SpringCompressionRatio * SuspState.SuspensionInfo.Stiffness * StiffnessFactor;

			SuspState.SuspensionForce = SuspensionForce * SuspUpVector;

			SuspState.WheelCollisionLocation = Hit.ImpactPoint;
			SuspState.WheelCollisionNormal = Hit.ImpactNormal;
			SuspState.PreviousLength = NewSuspensionLength;
			SuspState.WheelTouchedGround = true;
			SuspState.SurfaceType = UGameplayStatics::GetSurfaceType(Hit);

			ActiveFrictionPoints++;
		}
		else
		{
			// If there is no collision then suspension is relaxed
			SuspState.SuspensionForce = FVector::ZeroVector;
			SuspState.WheelCollisionLocation = FVector::ZeroVector;
			SuspState.WheelCollisionNormal = FVector::UpVector;
			SuspState.PreviousLength = SuspState.SuspensionInfo.Length;
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
			if (PrimitiveComponent && PrimitiveComponent->IsSimulatingPhysics())
			{
				PrimitiveComponent->AddForceAtLocation(-SuspState.SuspensionForce, SuspWorldLocation);
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
			const FVector WheelDirection = GetMesh()->GetForwardVector();

			// Calculate wheel velocity relative to track
			const FVector WheelCollisionVelocity = GetMesh()->GetPhysicsLinearVelocityAtPoint(SuspState.WheelCollisionLocation);
			const FVector WheelVelocity = WheelCollisionVelocity - WheelDirection * WheelTrack->LinearVelocity;
			const FVector RelativeWheelVelocity = UKismetMathLibrary::ProjectVectorOnToPlane(WheelVelocity, SuspState.WheelCollisionNormal);

			// Get friction coefficients
			float MuStatic = CalculateFrictionCoefficient(RelativeWheelVelocity, WheelDirection, StaticFrictionCoefficientEllipse);
			float MuKinetic = CalculateFrictionCoefficient(RelativeWheelVelocity, WheelDirection, KineticFrictionCoefficientEllipse);

			// Mass and friction forces
			const float VehicleMass = GetMesh()->GetMass();
			const FVector FrictionXVector = UKismetMathLibrary::ProjectVectorOnToPlane(GetMesh()->GetForwardVector(), SuspState.WheelCollisionNormal).GetSafeNormal();
			const FVector FrictionYVector = UKismetMathLibrary::ProjectVectorOnToPlane(GetMesh()->GetRightVector(), SuspState.WheelCollisionNormal).GetSafeNormal();

			// Current wheel force contbution
			const FVector WheelBalancedForce = ((RelativeWheelVelocity * -1) * VehicleMass / DeltaTime / ActiveFrictionPoints);

			// Full friction forces
			const FVector FullStaticFrictionForce = 
				UKismetMathLibrary::ProjectVectorOnToVector(WheelBalancedForce, FrictionXVector) * StaticFrictionCoefficientEllipse.X +
				UKismetMathLibrary::ProjectVectorOnToVector(WheelBalancedForce, FrictionYVector) * StaticFrictionCoefficientEllipse.Y;
			const FVector FullKineticFrictionForce =
				UKismetMathLibrary::ProjectVectorOnToVector(WheelBalancedForce, FrictionXVector) * KineticFrictionCoefficientEllipse.X +
				UKismetMathLibrary::ProjectVectorOnToVector(WheelBalancedForce, FrictionYVector) * KineticFrictionCoefficientEllipse.Y;

			// Drive Force from transmission torque
			const FVector TransmissionDriveForce = UKismetMathLibrary::ProjectVectorOnToPlane(WheelTrack->DriveForce, SuspState.WheelCollisionNormal);

			// Full drive forces
			const FVector FullStaticDriveForce = TransmissionDriveForce * StaticFrictionCoefficientEllipse.X;
			const FVector FullKineticDriveForce = TransmissionDriveForce * KineticFrictionCoefficientEllipse.X;

			// Full forces
			const FVector FullStaticForce = FullStaticDriveForce + FullStaticFrictionForce;
			const FVector FullKineticForce = FullKineticDriveForce + FullKineticFrictionForce;

			// We want to apply higher friction if forces are bellow static friction limit
			bool bUseKineticFriction = FullStaticForce.Size() >= (SuspState.WheelLoad * MuStatic);
			const FVector FullFrictionNormalizedForce = bUseKineticFriction ? FullKineticFrictionForce.GetSafeNormal() : FullStaticFrictionForce.GetSafeNormal();
			const FVector ApplicationForce = bUseKineticFriction 
				? FullKineticForce.GetClampedToSize(0.f, SuspState.WheelLoad * MuKinetic)
				: FullStaticForce.GetClampedToSize(0.f, SuspState.WheelLoad * MuStatic);

			// Apply force to mesh
			GetMesh()->AddForceAtLocation(ApplicationForce, SuspState.WheelCollisionLocation);


			/////////////////////////////////////////////////////////////////////////
			// Friction torque
			
			// How much of friction force would effect transmission
			const FVector TransmissionFrictionForce = UKismetMathLibrary::ProjectVectorOnToVector(ApplicationForce, FullFrictionNormalizedForce) * (-1.f) * (TrackMass + SprocketMass) / VehicleMass;
			const FVector WorldFrictionForce = UpdatedComponent->GetComponentTransform().InverseTransformVectorNoScale(TransmissionFrictionForce);
			const float TrackFrictionTorque = UKismetMathLibrary::ProjectVectorOnToVector(WorldFrictionForce, FVector::ForwardVector).X * SprocketRadius;
		
			// @todo Make this a force instead of torque!
			const float ReverseVelocitySign = (-1.f) * FMath::Sign(WheelTrack->LinearVelocity);
			const float TrackRollingFrictionTorque = SuspState.WheelLoad * RollingFrictionCoefficient * ReverseVelocitySign +
				SuspState.WheelLoad * (WheelTrack->LinearVelocity * RollingVelocityCoefficient) * ReverseVelocitySign;

			// Add torque to track
			WheelTrack->FrictionTorque += (TrackFrictionTorque * FrictionTorqueCoefficient);
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


//////////////////////////////////////////////////////////////////////////
// Vehicle control

void UPrvVehicleMovementComponent::SetThrottleInput(float Throttle)
{
	RawThrottleInput = FMath::Clamp(Throttle, -1.0f, 1.0f);
}

void UPrvVehicleMovementComponent::SetSteeringInput(float Steering)
{
	RawSteeringInput = FMath::Clamp(Steering, -1.0f, 1.0f);

	// Update tracks coefficients
	SteeringInput = RawSteeringInput;
	LeftTrack.Input = SteeringInput;
	RightTrack.Input = -SteeringInput;
}

void UPrvVehicleMovementComponent::SetHandbrakeInput(bool bNewHandbrake)
{
	bRawHandbrakeInput = bNewHandbrake;
}


//////////////////////////////////////////////////////////////////////////
// Vehicle stats

float UPrvVehicleMovementComponent::GetForwardSpeed() const
{
	return 0.0f;
}

float UPrvVehicleMovementComponent::GetEngineRotationSpeed() const
{
	return 0.0f;
}

float UPrvVehicleMovementComponent::GetEngineMaxRotationSpeed() const
{
	return 0.0f;
}


//////////////////////////////////////////////////////////////////////////
// Data access

USkinnedMeshComponent* UPrvVehicleMovementComponent::GetMesh()
{
	return Cast<USkinnedMeshComponent>(UpdatedComponent);
}

int32 UPrvVehicleMovementComponent::GetCurrentGear() const
{
	return CurrentGear;
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
	// Center of mass
	DrawDebugPoint(GetWorld(), GetMesh()->GetCenterOfMass(), 25.f, FColor::Yellow, false, /*LifeTime*/ 0.f);
}
