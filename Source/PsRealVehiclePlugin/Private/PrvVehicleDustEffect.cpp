// Copyright 2016 Pushkin Studio. All Rights Reserved.

#include "PrvPlugin.h"

bool UPrvVehicleDustEffect::bAllowToUseEffect = false;

UPrvVehicleDustEffect::UPrvVehicleDustEffect(const FObjectInitializer& ObjectInitializer) 
	: Super(ObjectInitializer)
{
	bShowForOwnerOnly = true;
}

UParticleSystem* UPrvVehicleDustEffect::GetDustFX(EPhysicalSurface SurfaceType, float CurrentSpeed)
{
	for (auto DustEffect : DustEffects)
	{
		if (DustEffect.SurfaceType == SurfaceType && 
			CurrentSpeed >= DustEffect.ActivationMinSpeed)
		{
			return DustEffect.DustFX;
		}
	}

	if (CurrentSpeed >= DefaultMinSpeed)
	{
		return DefaultFX;
	}

	return nullptr;
}

void UPrvVehicleDustEffect::SetAllowToUseEffect(bool bValue)
{
	UPrvVehicleDustEffect::bAllowToUseEffect = bValue;
}

bool UPrvVehicleDustEffect::GetAllowToUseEffect()
{
	return UPrvVehicleDustEffect::bAllowToUseEffect;
}
