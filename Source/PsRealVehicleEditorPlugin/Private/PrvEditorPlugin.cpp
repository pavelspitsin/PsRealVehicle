// Copyright 2016 Pushkin Studio. All Rights Reserved.

#include "PrvEditorPlugin.h"

class FPsRealVehicleEditorPlugin : public IPsRealVehicleEditorPlugin
{
	/** IModuleInterface implementation */
	virtual void StartupModule() override
	{
	}

	virtual void ShutdownModule() override
	{
	}
};

IMPLEMENT_MODULE(FPsRealVehicleEditorPlugin, PsRealVehicleEditorPlugin)

DEFINE_LOG_CATEGORY(LogPrvVehicleEditor);
