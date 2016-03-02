// Copyright 2016 Pushkin Studio. All Rights Reserved.

#include "PrvPlugin.h"

class FPsRealVehiclePlugin : public IPsRealVehiclePlugin
{
	/** IModuleInterface implementation */
	virtual void StartupModule() override
	{

	}

	virtual void ShutdownModule() override
	{

	}
};

IMPLEMENT_MODULE( FPsRealVehiclePlugin, PsRealVehiclePlugin )

DEFINE_LOG_CATEGORY(LogPrvVehicle);
