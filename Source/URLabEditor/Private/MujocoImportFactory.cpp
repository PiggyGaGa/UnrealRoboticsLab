// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "MujocoImportFactory.h"
#include "MujocoGenerationAction.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Engine/Blueprint.h"
#include "Misc/FeedbackContext.h"
#include "Misc/Paths.h"
#include "Misc/ScopedSlowTask.h"
#include "RenderingThread.h"
#include "ShaderCompiler.h"
#include "URLabEditorLogging.h"

UMujocoImportFactory::UMujocoImportFactory()
{
	Formats.Add(TEXT("xml;MuJoCo XML File"));
	SupportedClass = UBlueprint::StaticClass();
	bCreateNew = false;
	bEditorImport = true;
}

bool UMujocoImportFactory::FactoryCanImport(const FString& Filename)
{
	return FPaths::GetExtension(Filename).Equals(TEXT("xml"), ESearchCase::IgnoreCase);
}

UObject* UMujocoImportFactory::FactoryCreateFile(UClass* InClass, UObject* InParent, FName InName, EObjectFlags Flags, const FString& Filename, const TCHAR* Parms, FFeedbackContext* Warn, bool& bOutOperationCanceled)
{
	// Create blueprint based on AMjArticulation
	UClass* ParentClass = AMjArticulation::StaticClass();

	// Defensive: FKismetEditorUtilities::CreateBlueprint asserts when
	// an existing UBlueprint with the same name is still in memory
	// (Kismet2.cpp:424 -- FindObject<UBlueprint>(Outer, Name) == 0).
	// Callers that want to overwrite should drive ImportXmlSync with
	// force_reimport=true, which destroys the existing asset first; if
	// we get here with a stale BP anyway, bail with a logged error
	// rather than crashing the editor.
	if (UBlueprint* Existing = FindObject<UBlueprint>(InParent, *InName.ToString()))
	{
		UE_LOG(LogURLabEditor, Error,
			TEXT("MujocoImportFactory: refusing to overwrite existing Blueprint '%s' in '%s'. "
				 "Call ImportXmlSync with force_reimport=true (which destroys the existing asset) "
				 "before importing again."),
			*InName.ToString(),
			InParent ? *InParent->GetPathName() : TEXT("<no outer>"));
		bOutOperationCanceled = true;
		return nullptr;
	}

	// Create the Blueprint Asset
	UBlueprint* NewBP = FKismetEditorUtilities::CreateBlueprint(
		ParentClass,
		InParent,
		InName,
		BPTYPE_Normal,
		UBlueprint::StaticClass(),
		UBlueprintGeneratedClass::StaticClass());

	if (NewBP)
	{
		FScopedSlowTask SlowTask(3.f, NSLOCTEXT("URLab", "ImportingMuJoCo", "Importing MuJoCo model..."));
		SlowTask.MakeDialog(/*bShowCancelButton=*/false);

		SlowTask.EnterProgressFrame(1.f, NSLOCTEXT("URLab", "ImportStep1", "Reading XML and importing meshes..."));

		// Set XML Path in CDO so it persists (use original path, not _ue variant)
		AMjArticulation* CDO = Cast<AMjArticulation>(NewBP->GeneratedClass->GetDefaultObject());
		if (CDO)
		{
			CDO->MuJoCoXMLFile.FilePath = Filename;
			CDO->MarkPackageDirty();
		}

		SlowTask.EnterProgressFrame(1.f, NSLOCTEXT("URLab", "ImportStep2", "Building Blueprint components..."));

		// Generate components directly from the source XML. Referenced mesh files
		// are imported by UE_Assimp; Python preprocessing is not required.
		UMujocoGenerationAction* Generator = NewObject<UMujocoGenerationAction>();
		Generator->GenerateForBlueprint(NewBP, Filename);

		SlowTask.EnterProgressFrame(1.f, NSLOCTEXT("URLab", "ImportStep3", "Compiling Blueprint..."));

		// Compile to save changes and ensure components are valid
		FKismetEditorUtilities::CompileBlueprint(NewBP);

		// Wait for all shaders to finish compiling and flush render commands.
		// Material instances created during import trigger async shader compilation.
		// If the content browser renders thumbnails before shaders are ready,
		// the render thread crashes (UE-23902).
		if (GShaderCompilingManager)
		{
			GShaderCompilingManager->FinishAllCompilation();
		}
		FlushRenderingCommands();
	}

	bOutOperationCanceled = false;
	return NewBP;
}
