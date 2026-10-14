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

// MujocoMeshImporter.cpp — Mesh and material import methods for UMujocoGenerationAction.

#include "MujocoGenerationAction.h"
#include "URLabEditorLogging.h"
#include "AssimpFunctionLibrary.h"
#include "AIMesh.h"
#include "AINode.h"
#include "AIScene.h"
#include "Engine/StaticMesh.h"
#include "Materials/Material.h"
#include "MeshDescription.h"
#include "MeshDescriptionBuilder.h"
#include "StaticMeshAttributes.h"
#include "AssetToolsModule.h"
#include "Factories/FbxFactory.h"
#include "Factories/FbxImportUI.h"
#include "Factories/FbxStaticMeshImportData.h"
#include "AssetImportTask.h"
#include "FileHelpers.h"
#include "IImageWrapperModule.h"
#include "IImageWrapper.h"
#include "ImageUtils.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "Misc/PackageName.h"
#include "ObjectTools.h"
#include "PackageTools.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"
#include "UObject/StrongObjectPtr.h"

namespace
{
	constexpr double MuJoCoMetresToUnrealCentimetres = 100.0;

	FVector ToUnrealVector(const aiVector3D& Vector)
	{
		return FVector(Vector.x, Vector.y, Vector.z);
	}

	bool AppendAssimpNode(
		UAINode* Node,
		FMeshDescriptionBuilder& Builder,
		const FPolygonGroupID PolygonGroup,
		const FTransform& CoordinateCorrection,
		int32& OutVertexCount,
		int32& OutTriangleCount)
	{
		if (!IsValid(Node))
		{
			return false;
		}

		const FTransform NodeTransform = Node->GetRootTransform();
		TArray<UAIMesh*> NodeMeshes;
		Node->GetNodeMeshes(NodeMeshes);

		for (UAIMesh* AssimpMesh : NodeMeshes)
		{
			if (!IsValid(AssimpMesh) || !AssimpMesh->Mesh || !AssimpMesh->Mesh->HasPositions())
			{
				continue;
			}

			const aiMesh* SourceMesh = AssimpMesh->Mesh;
			TArray<FVertexID> VertexIDs;
			VertexIDs.Reserve(SourceMesh->mNumVertices);

			for (uint32 VertexIndex = 0; VertexIndex < SourceMesh->mNumVertices; ++VertexIndex)
			{
				const FVector Position = CoordinateCorrection.TransformPosition(
					NodeTransform.TransformPosition(ToUnrealVector(SourceMesh->mVertices[VertexIndex])))
					* MuJoCoMetresToUnrealCentimetres;
				VertexIDs.Add(Builder.AppendVertex(Position));
			}

			for (uint32 FaceIndex = 0; FaceIndex < SourceMesh->mNumFaces; ++FaceIndex)
			{
				const aiFace& Face = SourceMesh->mFaces[FaceIndex];
				if (Face.mNumIndices != 3
					|| !VertexIDs.IsValidIndex(static_cast<int32>(Face.mIndices[0]))
					|| !VertexIDs.IsValidIndex(static_cast<int32>(Face.mIndices[1]))
					|| !VertexIDs.IsValidIndex(static_cast<int32>(Face.mIndices[2])))
				{
					continue;
				}

				FVertexInstanceID Instances[3];
				for (int32 Corner = 0; Corner < 3; ++Corner)
				{
					const uint32 VertexIndex = Face.mIndices[Corner];
					Instances[Corner] = Builder.AppendInstance(VertexIDs[VertexIndex]);

					FVector Normal = FVector::UpVector;
					if (SourceMesh->HasNormals())
					{
						Normal = CoordinateCorrection.TransformVectorNoScale(
							NodeTransform.TransformVectorNoScale(ToUnrealVector(SourceMesh->mNormals[VertexIndex]))).GetSafeNormal();
						Builder.SetInstanceNormal(Instances[Corner], Normal);
					}

					if (SourceMesh->HasTangentsAndBitangents())
					{
						const FVector Tangent = CoordinateCorrection.TransformVectorNoScale(
							NodeTransform.TransformVectorNoScale(ToUnrealVector(SourceMesh->mTangents[VertexIndex]))).GetSafeNormal();
						const FVector Bitangent = CoordinateCorrection.TransformVectorNoScale(
							NodeTransform.TransformVectorNoScale(ToUnrealVector(SourceMesh->mBitangents[VertexIndex]))).GetSafeNormal();
						const float BinormalSign = FVector::DotProduct(FVector::CrossProduct(Normal, Tangent), Bitangent) < 0.0 ? -1.0f : 1.0f;
						Builder.SetInstanceTangentSpace(Instances[Corner], Normal, Tangent, BinormalSign);
					}

					if (SourceMesh->HasTextureCoords(0))
					{
						const aiVector3D& UV = SourceMesh->mTextureCoords[0][VertexIndex];
						Builder.SetInstanceUV(Instances[Corner], FVector2D(UV.x, UV.y), 0);
					}
				}

				Builder.AppendTriangle(Instances[0], Instances[1], Instances[2], PolygonGroup);
				++OutTriangleCount;
			}

			OutVertexCount += static_cast<int32>(SourceMesh->mNumVertices);
		}

		for (UAINode* Child : Node->GetChildNodes())
		{
			AppendAssimpNode(Child, Builder, PolygonGroup, CoordinateCorrection, OutVertexCount, OutTriangleCount);
		}

		return OutTriangleCount > 0;
	}

	UStaticMesh* ImportMeshWithAssimp(
		const FString& SourcePath,
		const FString& PackageName,
		const FString& ObjectName,
		UStaticMesh* ExistingMesh)
	{
		// The realtime preset requests tangent generation, which Assimp logs as
		// an error for perfectly valid meshes without UVs (common for STL). UE's
		// StaticMesh build generates the missing tangent basis, so omit that flag.
		const int32 ImportFlags =
			(UAssimpFunctionLibrary::ProcessPreset_TargetRealtime_Quality()
				& ~UAssimpFunctionLibrary::PostProcess_CalcTangentSpace())
			| UAssimpFunctionLibrary::PostProcess_MakeLeftHanded()
			| UAssimpFunctionLibrary::PostProcess_FlipUVs();

		TStrongObjectPtr<UAIScene> Scene(UAssimpFunctionLibrary::ImportScene(
			SourcePath, GetTransientPackage(), ImportFlags, /*DisableAutoSpaceChange=*/true));
		if (!Scene.IsValid() || !IsValid(Scene->GetRootNode()))
		{
			return nullptr;
		}

		FMeshDescription MeshDescription;
		FStaticMeshAttributes MeshAttributes(MeshDescription);
		MeshAttributes.Register();
		FMeshDescriptionBuilder Builder;
		Builder.SetMeshDescription(&MeshDescription);
		Builder.SetNumUVLayers(1);
		const FPolygonGroupID PolygonGroup = Builder.AppendPolygonGroup();
		const FString Extension = FPaths::GetExtension(SourcePath).ToLower();
		const bool bAssimpProducesYUp = Extension == TEXT("dae")
			|| Extension == TEXT("gltf") || Extension == TEXT("glb")
			|| Extension == TEXT("fbx");
		// aiProcess_MakeLeftHanded reflects Assimp's Z axis. MuJoCo and UE are
		// both Z-up, but converting MuJoCo's right-handed frame to UE's
		// left-handed frame must reflect Y instead: (x, y, z) -> (x, -y, z).
		// A 180-degree X rotation changes Assimp's Z reflection into that Y
		// reflection. Formats that Assimp first normalizes to Y-up need only the
		// corresponding 90-degree X correction.
		const double CorrectionAngle = bAssimpProducesYUp ? HALF_PI : PI;
		const FTransform CoordinateCorrection(
			FQuat(FVector(1.0, 0.0, 0.0), CorrectionAngle));

		int32 VertexCount = 0;
		int32 TriangleCount = 0;
		if (!AppendAssimpNode(Scene->GetRootNode(), Builder, PolygonGroup, CoordinateCorrection, VertexCount, TriangleCount))
		{
			UE_LOG(LogURLabEditor, Error, TEXT("Assimp imported no triangles from '%s'"), *SourcePath);
			return nullptr;
		}

		UPackage* Package = CreatePackage(*PackageName);
		if (!Package)
		{
			return nullptr;
		}

		UStaticMesh* StaticMesh = ExistingMesh;
		const bool bIsNewAsset = !IsValid(StaticMesh);
		if (bIsNewAsset)
		{
			StaticMesh = NewObject<UStaticMesh>(Package, *ObjectName, RF_Public | RF_Standalone);
		}
		else
		{
			StaticMesh->Modify();
		}
		StaticMesh->GetStaticMaterials().Reset();
		StaticMesh->GetStaticMaterials().Add(FStaticMaterial(UMaterial::GetDefaultMaterial(MD_Surface)));

		UStaticMesh::FBuildMeshDescriptionsParams BuildParams;
		BuildParams.bBuildSimpleCollision = false;
		const TArray<const FMeshDescription*> MeshDescriptions = {&MeshDescription};
		if (!StaticMesh->BuildFromMeshDescriptions(MeshDescriptions, BuildParams))
		{
			UE_LOG(LogURLabEditor, Error, TEXT("Failed to build StaticMesh '%s' from Assimp data"), *ObjectName);
			StaticMesh->ClearFlags(RF_Public | RF_Standalone);
			return nullptr;
		}

		StaticMesh->CalculateExtendedBounds();
		StaticMesh->MarkPackageDirty();
		if (bIsNewAsset)
		{
			FAssetRegistryModule::AssetCreated(StaticMesh);
		}

		FSavePackageArgs SaveArgs;
		SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
		SaveArgs.SaveFlags = SAVE_NoError;
		const FString PackageFilename = FPackageName::LongPackageNameToFilename(
			PackageName, FPackageName::GetAssetPackageExtension());
		if (!UPackage::SavePackage(Package, StaticMesh, *PackageFilename, SaveArgs))
		{
			UE_LOG(LogURLabEditor, Warning, TEXT("Built mesh '%s' but could not save package '%s'"), *ObjectName, *PackageFilename);
		}

		const FBox ImportedBounds = StaticMesh->GetBoundingBox();
		UE_LOG(LogURLabEditor, Log,
			TEXT("Assimp imported '%s' as '%s' (%d vertices, %d triangles, bounds min=%s max=%s size=%s)"),
			*SourcePath, *StaticMesh->GetPathName(), VertexCount, TriangleCount,
			*ImportedBounds.Min.ToString(), *ImportedBounds.Max.ToString(), *ImportedBounds.GetSize().ToString());
		return StaticMesh;
	}
}

UStaticMesh* UMujocoGenerationAction::ImportSingleMesh(const FString& SourcePath, const FString& DestinationPath, const FString& AssetName)
{
	if (SourcePath.IsEmpty() || DestinationPath.IsEmpty())
	{
		return nullptr;
	}

	const FString FileName = FPaths::GetBaseFilename(SourcePath);
	FString ObjectName = ObjectTools::SanitizeObjectName(AssetName.IsEmpty() ? FileName : AssetName);
	if (ObjectName.IsEmpty())
	{
		ObjectName = TEXT("ImportedMesh");
	}
	FString PackageName = FPaths::Combine(DestinationPath, ObjectName);
	PackageName = UPackageTools::SanitizePackageName(PackageName);
	const FString ObjectPath = PackageName + TEXT(".") + ObjectName;

	UStaticMesh* ExistingMesh = LoadObject<UStaticMesh>(nullptr, *ObjectPath);

	UE_LOG(LogURLabEditor, Log, TEXT("Importing mesh from: %s to %s"), *SourcePath, *DestinationPath);

	if (!FPaths::FileExists(SourcePath))
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Source mesh file does not exist: %s"), *SourcePath);
		return nullptr;
	}

	if (UStaticMesh* AssimpMesh = ImportMeshWithAssimp(SourcePath, PackageName, ObjectName, ExistingMesh))
	{
		if (ValidateMesh(AssimpMesh, ObjectName))
		{
			return AssimpMesh;
		}
	}

	UE_LOG(LogURLabEditor, Warning,
		TEXT("Assimp import failed for '%s'; trying Unreal's native importer as a fallback"), *SourcePath);

	// Try import with MikkTSpace first (best quality)
	UStaticMesh* ImportedMesh = AttemptMeshImport(SourcePath, DestinationPath, EFBXNormalGenerationMethod::MikkTSpace);

	// Validate mesh
	if (ImportedMesh && ValidateMesh(ImportedMesh, FileName))
	{
		UE_LOG(LogURLabEditor, Log, TEXT("Successfully imported mesh '%s' with MikkTSpace"), *FileName);
		return ImportedMesh;
	}

	// MikkTSpace failed or mesh invalid - try fallback with BuiltIn normals
	if (ImportedMesh)
	{
		UE_LOG(LogURLabEditor, Warning, TEXT("Mesh '%s' has issues with MikkTSpace, attempting fallback with BuiltIn normals"), *FileName);
	}
	else
	{
		UE_LOG(LogURLabEditor, Warning, TEXT("Failed to import mesh '%s' with MikkTSpace, attempting fallback"), *FileName);
	}

	ImportedMesh = AttemptMeshImport(SourcePath, DestinationPath, EFBXNormalGenerationMethod::BuiltIn);

	if (ImportedMesh && ValidateMesh(ImportedMesh, FileName))
	{
		UE_LOG(LogURLabEditor, Warning, TEXT("Successfully imported mesh '%s' with BuiltIn normals (fallback)"), *FileName);
		return ImportedMesh;
	}

	UE_LOG(LogURLabEditor, Error, TEXT("Failed to import mesh '%s' - all import methods failed"), *FileName);
	return nullptr;
}

UStaticMesh* UMujocoGenerationAction::AttemptMeshImport(const FString& SourcePath, const FString& DestinationPath, EFBXNormalGenerationMethod::Type NormalMethod)
{
	IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();

	// Configure Automated Import Task
	UAssetImportTask* ImportTask = NewObject<UAssetImportTask>();
	ImportTask->Filename = SourcePath;
	ImportTask->DestinationPath = DestinationPath;
	ImportTask->bAutomated = true;
	ImportTask->bSave = true;
	ImportTask->bReplaceExisting = true;
	ImportTask->bReplaceExistingSettings = true;

	// Configure FBX Factory only for FBX/OBJ
	FString Extension = FPaths::GetExtension(SourcePath).ToLower();

	if (Extension == "fbx" || Extension == "obj" || Extension == "t3d")
	{
		// Configure FBX Factory
		UFbxFactory* FbxFactory = NewObject<UFbxFactory>();
		ImportTask->Factory = FbxFactory;

		// Configure FBX Import UI settings
		UFbxImportUI* ImportUI = NewObject<UFbxImportUI>();
		ImportUI->bImportMesh = true;
		ImportUI->bImportTextures = false;
		ImportUI->bImportMaterials = false;
		ImportUI->bAutomatedImportShouldDetectType = false;
		ImportUI->MeshTypeToImport = FBXIT_StaticMesh;

		// Robust Static Mesh Settings
		ImportUI->StaticMeshImportData->bCombineMeshes = true;
		ImportUI->StaticMeshImportData->bRemoveDegenerates = true;
		ImportUI->StaticMeshImportData->bComputeWeightedNormals = true;
		ImportUI->StaticMeshImportData->bGenerateLightmapUVs = true;
		ImportUI->StaticMeshImportData->NormalImportMethod = EFBXNormalImportMethod::FBXNIM_ComputeNormals;
		ImportUI->StaticMeshImportData->NormalGenerationMethod = NormalMethod;

		// Additional settings to fix degenerate geometry (especially from OBJ files)
		ImportUI->StaticMeshImportData->bAutoGenerateCollision = false; // We handle collision separately
		ImportUI->StaticMeshImportData->bBuildReversedIndexBuffer = true;
		ImportUI->StaticMeshImportData->bBuildNanite = false; // Nanite requires clean geometry

		// Vertex welding - critical for fixing overlapping vertices that cause degenerate tangents
		// Note: There's no direct bWeldVertices in UE 5.7, but bRemoveDegenerates handles this

		// Apply UI to Factory
		FbxFactory->ImportUI = ImportUI;
		FbxFactory->EnableShowOption();
	}
	else
	{
		// For other formats (GLTF, GLB, etc.), let Unreal's asset tools find the appropriate factory.
		// We don't manually set the factory, so ImportAssetTasks will automatically detect the correct one.
		// Note: We lose the fine-grained settings (like NormalGenerationMethod), but GLTF importers
		// usually rely on the file's inherent data which is often cleaner than OBJ.
		ImportTask->Factory = nullptr;

		UE_LOG(LogURLabEditor, Log, TEXT("Using automated factory detection for mesh: %s"), *SourcePath);
	}

	// Run Import
	TArray<UAssetImportTask*> ImportTasks;
	ImportTasks.Add(ImportTask);
	AssetTools.ImportAssetTasks(ImportTasks);

	// Retrieve Result
	TArray<UObject*> ImportedAssets;
	for (UObject* Obj : ImportTask->GetObjects())
	{
		if (Obj)
			ImportedAssets.Add(Obj);
	}

	// Log all imported assets for debugging
	UE_LOG(LogURLabEditor, Log, TEXT("[ImportSingleMesh] Import returned %d objects:"), ImportedAssets.Num());
	for (int32 i = 0; i < ImportedAssets.Num(); ++i)
	{
		UObject* Obj = ImportedAssets[i];
		UE_LOG(LogURLabEditor, Log, TEXT("  [%d] %s (%s) at %s"),
			i, *Obj->GetName(), *Obj->GetClass()->GetName(), *Obj->GetPathName());
	}

	// Search all imported assets for a StaticMesh (GLB imports may return textures first)
	UStaticMesh* Mesh = nullptr;
	for (UObject* Obj : ImportedAssets)
	{
		Mesh = Cast<UStaticMesh>(Obj);
		if (Mesh)
			break;
	}

	// If not found in direct results, search subfolder paths that Interchange may use
	if (!Mesh)
	{
		FString MeshName = FPaths::GetBaseFilename(SourcePath);

		// Try various subfolder patterns Interchange uses
		TArray<FString> SearchPaths = {
			FString::Printf(TEXT("%s/%s/StaticMeshes/%s.%s"), *DestinationPath, *MeshName, *MeshName, *MeshName),
			FString::Printf(TEXT("%s/%s/StaticMeshes/%s"), *DestinationPath, *MeshName, *MeshName),
			FString::Printf(TEXT("%s/%s.%s"), *DestinationPath, *MeshName, *MeshName),
		};

		for (const FString& SearchPath : SearchPaths)
		{
			Mesh = LoadObject<UStaticMesh>(nullptr, *SearchPath);
			if (Mesh)
			{
				UE_LOG(LogURLabEditor, Log, TEXT("[ImportSingleMesh] Found mesh at: %s"), *SearchPath);
				break;
			}
			else
			{
				UE_LOG(LogURLabEditor, Log, TEXT("[ImportSingleMesh] Not found at: %s"), *SearchPath);
			}
		}

		// Last resort: use asset registry to find any StaticMesh in the destination folder
		if (!Mesh)
		{
			FString SearchDir = FString::Printf(TEXT("%s/%s"), *DestinationPath, *MeshName);
			UE_LOG(LogURLabEditor, Log, TEXT("[ImportSingleMesh] Searching asset registry under: %s"), *SearchDir);

			IAssetRegistry& AssetRegistry = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry").Get();
			TArray<FAssetData> Assets;
			AssetRegistry.GetAssetsByPath(FName(*SearchDir), Assets, true);

			for (const FAssetData& Asset : Assets)
			{
				UE_LOG(LogURLabEditor, Log, TEXT("  Registry: %s (%s)"), *Asset.AssetName.ToString(), *Asset.AssetClassPath.ToString());
				if (Asset.AssetClassPath.GetAssetName() == TEXT("StaticMesh"))
				{
					Mesh = Cast<UStaticMesh>(Asset.GetAsset());
					if (Mesh)
					{
						UE_LOG(LogURLabEditor, Log, TEXT("[ImportSingleMesh] Found mesh via registry: %s"), *Asset.GetObjectPathString());
						break;
					}
				}
			}
		}
	}

	if (Mesh)
	{
		// Clear Interchange-created materials that may reference stripped textures.
		// Our import pipeline assigns MI_ material instances on the SCS template,
		// but the static mesh asset retains Interchange materials in its slots.
		// These can crash the render thread when browsing/thumbnailing (UE-23902).
		for (FStaticMaterial& Mat : Mesh->GetStaticMaterials())
		{
			Mat.MaterialInterface = UMaterial::GetDefaultMaterial(MD_Surface);
		}

		// Force rebuild bounds - critical for fixing 0x0x0 size issue
		Mesh->Build();
		Mesh->CalculateExtendedBounds();

		UPackage* Package = Mesh->GetOutermost();
		FEditorFileUtils::PromptForCheckoutAndSave({Package}, false, false);

		UE_LOG(LogURLabEditor, Log, TEXT("Imported mesh '%s' - Bounds: %s"),
			*FPaths::GetBaseFilename(SourcePath),
			*Mesh->GetBoundingBox().GetSize().ToString());

		return Mesh;
	}

	return nullptr;
}

bool UMujocoGenerationAction::ValidateMesh(UStaticMesh* Mesh, const FString& MeshName)
{
	if (!Mesh)
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Mesh validation failed: Mesh is null"));
		return false;
	}

	// Check if mesh has render data
	if (!Mesh->GetRenderData())
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Mesh '%s' has no render data"), *MeshName);
		return false;
	}

	// Check LOD 0 exists
	if (Mesh->GetRenderData()->LODResources.Num() == 0)
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Mesh '%s' has no LOD resources"), *MeshName);
		return false;
	}

	const FStaticMeshLODResources& LOD0 = Mesh->GetRenderData()->LODResources[0];

	// Check vertex buffer
	if (LOD0.VertexBuffers.StaticMeshVertexBuffer.GetNumVertices() == 0)
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Mesh '%s' has empty vertex buffer"), *MeshName);
		return false;
	}

	// Check index buffer
	if (LOD0.IndexBuffer.GetNumIndices() == 0)
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Mesh '%s' has empty index buffer"), *MeshName);
		return false;
	}

	// Log mesh statistics
	int32 NumVertices = LOD0.VertexBuffers.StaticMeshVertexBuffer.GetNumVertices();
	int32 NumTriangles = LOD0.IndexBuffer.GetNumIndices() / 3;
	int32 NumUVChannels = LOD0.VertexBuffers.StaticMeshVertexBuffer.GetNumTexCoords();

	UE_LOG(LogURLabEditor, Log, TEXT("Mesh '%s' validation: %d vertices, %d triangles, %d UV channels"),
		*MeshName, NumVertices, NumTriangles, NumUVChannels);

	// Warn if no UV channel 0
	if (NumUVChannels == 0)
	{
		UE_LOG(LogURLabEditor, Warning, TEXT("Mesh '%s' has no UV channels - materials may not display correctly"), *MeshName);
	}

	return true;
}

UTexture2D* UMujocoGenerationAction::ImportSingleTexture(const FString& SourcePath, const FString& DestinationPath)
{
	if (SourcePath.IsEmpty() || DestinationPath.IsEmpty())
	{
		return nullptr;
	}

	FString FileName = FPaths::GetBaseFilename(SourcePath);
	FString PackageName = FPaths::Combine(DestinationPath, FileName);
	PackageName = UPackageTools::SanitizePackageName(PackageName);

	// Check if texture already exists
	UTexture2D* ExistingTexture = LoadObject<UTexture2D>(nullptr, *PackageName);
	if (ExistingTexture)
	{
		return ExistingTexture;
	}

	UE_LOG(LogURLabEditor, Log, TEXT("Importing texture from: %s to %s"), *SourcePath, *DestinationPath);

	// Load texture file from disk
	TArray<uint8> FileData;
	if (!FFileHelper::LoadFileToArray(FileData, *SourcePath))
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Failed to load texture file: %s"), *SourcePath);
		return nullptr;
	}

	// Determine image format from extension
	FString Extension = FPaths::GetExtension(SourcePath).ToLower();
	EImageFormat ImageFormat = EImageFormat::Invalid;

	if (Extension == TEXT("png"))
	{
		ImageFormat = EImageFormat::PNG;
	}
	else if (Extension == TEXT("jpg") || Extension == TEXT("jpeg"))
	{
		ImageFormat = EImageFormat::JPEG;
	}
	else if (Extension == TEXT("tga"))
	{
		ImageFormat = EImageFormat::TGA;
	}
	else if (Extension == TEXT("bmp"))
	{
		ImageFormat = EImageFormat::BMP;
	}
	else
	{
		UE_LOG(LogURLabEditor, Warning, TEXT("Unsupported texture format: %s"), *Extension);
		return nullptr;
	}

	// Decode image
	IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName("ImageWrapper"));
	TSharedPtr<IImageWrapper> ImageWrapper = ImageWrapperModule.CreateImageWrapper(ImageFormat);

	if (!ImageWrapper.IsValid() || !ImageWrapper->SetCompressed(FileData.GetData(), FileData.Num()))
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Failed to decode texture: %s"), *SourcePath);
		return nullptr;
	}

	// Get raw image data
	TArray<uint8> RawData;
	if (!ImageWrapper->GetRaw(ERGBFormat::BGRA, 8, RawData))
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Failed to get raw texture data: %s"), *SourcePath);
		return nullptr;
	}

	// Create package
	UPackage* Package = CreatePackage(*PackageName);
	Package->FullyLoad();

	// Create texture
	UTexture2D* NewTexture = NewObject<UTexture2D>(Package, FName(*FileName), RF_Public | RF_Standalone);

	// Set texture properties
	NewTexture->Source.Init(ImageWrapper->GetWidth(), ImageWrapper->GetHeight(), 1, 1, TSF_BGRA8, RawData.GetData());
	NewTexture->SRGB = true;
	NewTexture->CompressionSettings = TextureCompressionSettings::TC_Default;
	NewTexture->MipGenSettings = TextureMipGenSettings::TMGS_FromTextureGroup;
	NewTexture->UpdateResource();

	// Save package
	Package->MarkPackageDirty();
	FAssetRegistryModule::AssetCreated(NewTexture);

	FString PackageFileName = FPackageName::LongPackageNameToFilename(PackageName, FPackageName::GetAssetPackageExtension());
	FSavePackageArgs SaveArgs;
	SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
	SaveArgs.SaveFlags = SAVE_NoError;
	UPackage::SavePackage(Package, NewTexture, *PackageFileName, SaveArgs);

	UE_LOG(LogURLabEditor, Log, TEXT("Successfully imported texture: %s"), *FileName);
	return NewTexture;
}

UMaterialInstanceConstant* UMujocoGenerationAction::CreateMaterialInstance(
	const FString& MeshName,
	const FMuJoCoMaterialData& MaterialData,
	const TMap<FString, UTexture2D*>& TextureAssets,
	const FString& DestinationPath)
{
	// Load master material
	UMaterial* MasterMaterial = LoadObject<UMaterial>(nullptr, TEXT("/UnrealRoboticsLab/Materials/M_MuJoCo_Master.M_MuJoCo_Master"));
	if (!MasterMaterial)
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Failed to load master material: /UnrealRoboticsLab/Materials/M_MuJoCo_Master"));
		return nullptr;
	}

	// Create material instance package. Strip path separators from the
	// instance name — Unreal FNames don't tolerate '/', which corrupts the
	// asset path for namespaced MJCF materials (e.g. "obstacles/box1").
	FString InstanceName = FString::Printf(TEXT("MI_%s"), *MeshName);
	InstanceName = InstanceName.Replace(TEXT("/"), TEXT("_"));
	FString PackageName = FPaths::Combine(DestinationPath, InstanceName);
	PackageName = UPackageTools::SanitizePackageName(PackageName);

	// Check if material instance already exists — reuse during the same import session
	// (multiple geoms referencing the same material), but don't skip parameter setup
	UMaterialInstanceConstant* ExistingInstance = LoadObject<UMaterialInstanceConstant>(nullptr, *PackageName);
	if (ExistingInstance)
	{
		UE_LOG(LogURLabEditor, Log, TEXT("Reusing existing material instance: %s"), *InstanceName);
		return ExistingInstance;
	}

	UE_LOG(LogURLabEditor, Log, TEXT("Creating material instance: %s"), *InstanceName);

	// Create package
	UPackage* Package = CreatePackage(*PackageName);
	Package->FullyLoad();

	// Create material instance
	UMaterialInstanceConstant* MaterialInstance = NewObject<UMaterialInstanceConstant>(
		Package,
		FName(*InstanceName),
		RF_Public | RF_Standalone);

	MaterialInstance->SetParentEditorOnly(MasterMaterial);

	// Helper lambda to set a scalar parameter with override enabled
	auto SetScalar = [&](const TCHAR* Name, float Value) {
		FMaterialParameterInfo Info(Name);
		MaterialInstance->SetScalarParameterValueEditorOnly(Info, Value);
	};

	// Helper lambda to set a vector parameter with override enabled
	auto SetVector = [&](const TCHAR* Name, const FLinearColor& Value) {
		FMaterialParameterInfo Info(Name);
		MaterialInstance->SetVectorParameterValueEditorOnly(Info, Value);
	};

	// Helper lambda to set a texture parameter with override enabled
	auto SetTexture = [&](const TCHAR* Name, UTexture* Tex) {
		FMaterialParameterInfo Info(Name);
		MaterialInstance->SetTextureParameterValueEditorOnly(Info, Tex);

		// Also directly add to TextureParameterValues to ensure override is enabled
		FTextureParameterValue TexParam;
		TexParam.ParameterInfo = Info;
		TexParam.ParameterValue = Tex;
		TexParam.ExpressionGUID = FGuid(); // Will be resolved by UE
		MaterialInstance->TextureParameterValues.Add(TexParam);

		UE_LOG(LogURLabEditor, Log, TEXT("  [SetTexture] Set '%s' = '%s' (TextureParameterValues count: %d)"),
			Name, *Tex->GetName(), MaterialInstance->TextureParameterValues.Num());
	};

	// Set base color
	SetVector(TEXT("BaseColor"), MaterialData.Rgba);

	// Set texture parameters
	bool bHasBaseColorTexture = false;
	if (!MaterialData.BaseColorTextureName.IsEmpty())
	{
		if (TextureAssets.Contains(MaterialData.BaseColorTextureName))
		{
			UTexture2D* BaseColorTex = TextureAssets[MaterialData.BaseColorTextureName];
			if (BaseColorTex)
			{
				SetTexture(TEXT("BaseColorTexture"), BaseColorTex);
				bHasBaseColorTexture = true;
				UE_LOG(LogURLabEditor, Log, TEXT("  Texture '%s' applied to material '%s' (tex=%s)"),
					*MaterialData.BaseColorTextureName, *InstanceName, *BaseColorTex->GetName());
			}
		}
		else
		{
			UE_LOG(LogURLabEditor, Warning, TEXT("  Texture '%s' referenced by material '%s' but not found in imported textures (%d entries)"),
				*MaterialData.BaseColorTextureName, *InstanceName, TextureAssets.Num());
		}
	}

	// Bind each PBR slot if a texture was parsed for it. We bind all four
	// (Normal, ORM, Roughness, Metallic) eagerly — the static-switch pass
	// below tells the master material which ones are actually live.
	//
	// ORM is the packed-channel workflow MuJoCo materials normally use
	// (R=AO, G=Roughness, B=Metallic). When ORM is present, separate
	// Roughness / Metallic textures are typically redundant; we still bind
	// them so a master-material variant that wants them can opt-in via its
	// own switch wiring.
	bool bHasNormalTex = false;
	bool bHasORMTex = false;
	bool bHasRoughnessTex = false;
	bool bHasMetallicTex = false;

	if (!MaterialData.NormalTextureName.IsEmpty() && TextureAssets.Contains(MaterialData.NormalTextureName))
	{
		if (UTexture2D* Tex = TextureAssets[MaterialData.NormalTextureName])
		{
			SetTexture(TEXT("NormalTexture"), Tex);
			bHasNormalTex = true;
		}
	}
	if (!MaterialData.ORMTextureName.IsEmpty() && TextureAssets.Contains(MaterialData.ORMTextureName))
	{
		if (UTexture2D* Tex = TextureAssets[MaterialData.ORMTextureName])
		{
			SetTexture(TEXT("ORMTexture"), Tex);
			bHasORMTex = true;
		}
	}
	if (!MaterialData.RoughnessTextureName.IsEmpty() && TextureAssets.Contains(MaterialData.RoughnessTextureName))
	{
		if (UTexture2D* Tex = TextureAssets[MaterialData.RoughnessTextureName])
		{
			SetTexture(TEXT("RoughnessTexture"), Tex);
			bHasRoughnessTex = true;
		}
	}
	if (!MaterialData.MetallicTextureName.IsEmpty() && TextureAssets.Contains(MaterialData.MetallicTextureName))
	{
		if (UTexture2D* Tex = TextureAssets[MaterialData.MetallicTextureName])
		{
			SetTexture(TEXT("MetallicTexture"), Tex);
			bHasMetallicTex = true;
		}
	}

	// Static-switch update. Each `bUse*Texture` switch lets the master
	// material gate one of the PBR slots so the sample/branch is compiled
	// out when no texture was bound — same trick as ``bUseTexture`` for
	// BaseColor, which prevents the null-texture render-thread crash
	// (UE-23902). Switches the master material doesn't declare are silently
	// skipped, so this is forward-compatible with the master being
	// re-wired incrementally.
	{
		FStaticParameterSet StaticParams;
		MaterialInstance->GetStaticParameterValues(StaticParams);

		auto SetSwitch = [&](const TCHAR* Name, bool Value) {
			for (FStaticSwitchParameter& Param : StaticParams.StaticSwitchParameters)
			{
				if (Param.ParameterInfo.Name == Name)
				{
					Param.Value = Value;
					Param.bOverride = true;
					return;
				}
			}
		};

		SetSwitch(TEXT("bUseTexture"), bHasBaseColorTexture);
		SetSwitch(TEXT("bUseNormalTexture"), bHasNormalTex);
		SetSwitch(TEXT("bUseORMTexture"), bHasORMTex);
		SetSwitch(TEXT("bUseRoughnessTexture"), bHasRoughnessTex);
		SetSwitch(TEXT("bUseMetallicTexture"), bHasMetallicTex);

		MaterialInstance->UpdateStaticPermutation(StaticParams);
	}

	// Force update and save
	MaterialInstance->UpdateStaticPermutation();
	MaterialInstance->PostEditChange();

	// Save package
	Package->MarkPackageDirty();
	FAssetRegistryModule::AssetCreated(MaterialInstance);

	FString PackageFileName = FPackageName::LongPackageNameToFilename(PackageName, FPackageName::GetAssetPackageExtension());
	FSavePackageArgs SaveArgs;
	SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
	SaveArgs.SaveFlags = SAVE_NoError;
	UPackage::SavePackage(Package, MaterialInstance, *PackageFileName, SaveArgs);

	UE_LOG(LogURLabEditor, Log, TEXT("Successfully created material instance: %s"), *InstanceName);
	return MaterialInstance;
}
