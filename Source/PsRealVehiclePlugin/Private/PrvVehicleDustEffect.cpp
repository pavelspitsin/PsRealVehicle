// Copyright 2016 Pushkin Studio. All Rights Reserved.

#include "PrvPlugin.h"

UPrvVehicleDustEffect::UPrvVehicleDustEffect(const FObjectInitializer& ObjectInitializer) 
	: Super(ObjectInitializer)
{

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
