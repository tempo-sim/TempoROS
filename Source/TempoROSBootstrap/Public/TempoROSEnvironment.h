// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "CoreMinimal.h"

/// Set an environment variable for this process.
/**
 * Inline so both TempoROSBootstrap and TempoROS get their own copy; it is small, and a shared
 * definition would have to be exported from the bootstrap module to be linkable from the other.
 */
inline void SetEnvironmentVar(const TCHAR* VariableName, const TCHAR* Value)
{
#if PLATFORM_WINDOWS
	// On Windows only, SetEnvironmentVar does not seem to work properly, but this does.
	_putenv_s(TCHAR_TO_UTF8(VariableName), TCHAR_TO_UTF8(Value));
#else
	FPlatformMisc::SetEnvironmentVar(VariableName, Value);
#endif
}
