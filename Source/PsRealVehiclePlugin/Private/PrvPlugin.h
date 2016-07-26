// Copyright 2016 Pushkin Studio. All Rights Reserved.

#pragma once

#include "CoreUObject.h"
#include "Engine.h"

#include "Net/UnrealNetwork.h"

// You should place include statements to your module's private header files here.  You only need to
// add includes for headers that are used in most of your module's source files though.
#include "ModuleManager.h"

DECLARE_STATS_GROUP(TEXT("Prv Movement"), STATGROUP_MovementPhysics, STATCAT_Advanced);

DECLARE_LOG_CATEGORY_EXTERN(LogPrvVehicle, Log, All);

#include "IPsRealVehiclePlugin.h"

#include "PsRealVehiclePluginClasses.h"
