// Copyright 2016 Vladimir Alyamkin. All Rights Reserved.

#include "VaRealVehiclePluginPrivatePCH.h"

class FVaRealVehiclePlugin : public IVaRealVehiclePlugin
{
	/** IModuleInterface implementation */
	virtual void StartupModule() override
	{

	}

	virtual void ShutdownModule() override
	{

	}
};

IMPLEMENT_MODULE( FVaRealVehiclePlugin, VaRealVehiclePlugin )

DEFINE_LOG_CATEGORY(LogVaRealVehicle);
