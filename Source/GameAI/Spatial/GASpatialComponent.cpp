#include "GASpatialComponent.h"
#include "GameAI/Pathfinding/GAPathComponent.h"
#include "GameAI/Grid/GAGridMap.h"
#include "Kismet/GameplayStatics.h"
#include "Math/MathFwd.h"
#include "GASpatialFunction.h"
#include "ProceduralMeshComponent.h"



UGASpatialComponent::UGASpatialComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	SampleDimensions = 8000.0f;		// should cover the bulk of the test map
}


const AGAGridActor* UGASpatialComponent::GetGridActor() const
{
	AGAGridActor* Result = GridActor.Get();
	if (Result)
	{
		return Result;
	}
	else
	{
		AActor* GenericResult = UGameplayStatics::GetActorOfClass(this, AGAGridActor::StaticClass());
		if (GenericResult)
		{
			Result = Cast<AGAGridActor>(GenericResult);
			if (Result)
			{
				// Cache the result
				// Note, GridActor is marked as mutable in the header, which is why this is allowed in a const method
				GridActor = Result;
			}
		}

		return Result;
	}
}

UGAPathComponent* UGASpatialComponent::GetPathComponent() const
{
	UGAPathComponent* Result = PathComponent.Get();
	if (Result)
	{
		return Result;
	}
	else
	{
		AActor* Owner = GetOwner();
		if (Owner)
		{
			// Note, the UGAPathComponent and the UGASpatialComponent are both on the controller
			Result = Owner->GetComponentByClass<UGAPathComponent>();
			if (Result)
			{
				PathComponent = Result;
			}
		}
		return Result;
	}
}

APawn* UGASpatialComponent::GetOwnerPawn() const
{
	AActor* Owner = GetOwner();
	if (Owner)
	{
		APawn* Pawn = Cast<APawn>(Owner);
		if (Pawn)
		{
			return Pawn;
		}
		else
		{
			AController* Controller = Cast<AController>(Owner);
			if (Controller)
			{
				return Controller->GetPawn();
			}
		}
	}

	return NULL;
}

static void CameFromToPath(const TMap<FCellRef, FVector>& CameFrom, const FVector& start, const FVector& goal, TArray<FVector>& path, const AGAGridActor* GridActor)
{
	path.Empty();

	FVector current = goal;
	while (current != start)
	{
		path.Add(current);
		FCellRef CurrentCell;
		if ((GridActor->GetCellRef(current, false).IsValid()) || !CameFrom.Contains(CurrentCell))
		{
			// Unable to reconstruct the path
			return;
		}
		current = CameFrom[CurrentCell];
	}

	// Add the start point to the path
	path.Add(start);

	// Reverse the path to have it in the correct order
	Algo::Reverse(path);
}


bool UGASpatialComponent::ChoosePosition(bool PathfindToPosition, bool Debug)
{
	bool Result = false;
	const APawn* OwnerPawn = GetOwnerPawn();
	const AGAGridActor* Grid = GetGridActor();
	FCellRef BestCell;

	if (SpatialFunctionReference.Get() == NULL)
	{
		UE_LOG(LogTemp, Warning, TEXT("UGASpatialComponent has no SpatialFunctionReference assigned."));
		return false;
	}

	// Don't worry too much about the Unreal-ism below. Technically our SpatialFunctionReference is not ACTUALLY
	// a spatial function instance, rather it's a class, which happens to have a lot of data in it.
	// Happily, Unreal creates, under the hood, a default object for every class, that lets you access that data
	// as if it were a normal instance
	const UGASpatialFunction* SpatialFunction = SpatialFunctionReference->GetDefaultObject<UGASpatialFunction>();

	// The below is to create a GridMap (which you will fill in) based on a bounding box centered around the OwnerPawn

	FBox2D Box(EForceInit::ForceInit);
	FIntRect CellRect;
	FVector2D PawnLocation(OwnerPawn->GetActorLocation());
	Box += PawnLocation;
	Box = Box.ExpandBy(SampleDimensions / 2.0f);
	if (GridActor->GridSpaceBoundsToRect2D(Box, CellRect))
	{
		// Super annoying, by the way, that FIntRect is not blueprint accessible, because it forces us instead
		// to make a separate bp-accessible FStruct that represents _exactly the same thing_.
		FGridBox GridBox(CellRect);

		// This is the grid map I'm going to fill with values
		FGAGridMap GridMap(Grid, GridBox, 0.0f);

		// Fill in this distance map using Dijkstra!
		FGAGridMap DistanceMap(Grid, GridBox, FLT_MAX);


		// ~~~ STEPS TO FILL IN FOR ASSIGNMENT 3 ~~~

		// Step 1: Run Dijkstra's to determine which cells we should even be evaluating (the GATHER phase)
		// (You should add a Dijkstra() function to the UGAPathComponent())
		// I would recommend adding a method to the path component which looks something like
		// bool UGAPathComponent::Dijkstra(const FVector &StartPoint, FGAGridMap &DistanceMapOut) const;
		AActor* Owner = GetOwnerPawn();
		FVector StartPoint = Owner->GetActorLocation();
		UGAPathComponent* PathComp = GetPathComponent();
		TMap<FCellRef, FVector> CameFrom;
		PathComp->Dijkstra(StartPoint, DistanceMap, CameFrom);


		// Step 2: For each layer in the spatial function, evaluate and accumulate the layer in GridMap
		// Note, only evaluate accessible cells found in step 1
		for (const FFunctionLayer& Layer : SpatialFunction->Layers)
		{
			UE_LOG(LogTemp, Warning, TEXT("In Layers"));
			// figure out how to evaluate each layer type, and accumulate the value in the GridMap
			EvaluateLayer(Layer, GridMap, DistanceMap, BestCell);
			UE_LOG(LogTemp, Warning, TEXT("Value: %d"), 3);
		}
		UE_LOG(LogTemp, Warning, TEXT("Best Cell: (%d, %d), Best Value: %f"), BestCell.X, BestCell.Y);
		// Step 3: pick the best cell in GridMap

		// Let's pretend for now we succeeded.
		Result = true;

		if (PathfindToPosition)
		{
			// Step 4: Go there!
			// This will involve reconstructing the path and then getting it into the UGAPathComponent
			// Depending on what your cached Dijkstra data looks like, the path reconstruction might be implemented here
			// or in the UGAPathComponent
			FVector BestCellPosition;

			// Assuming BestCell is already calculated somewhere in your code
			// For example:
			// BestCell = FindBestCell();

			// Get the position of the BestCell
			float BestCValue = 0;
			if (BestCell.IsValid())
			{
				BestCellPosition = Grid->GetCellPosition(BestCell);

				GridMap.GetValue(BestCell, BestCValue);
			}
			UE_LOG(LogTemp, Warning, TEXT("Best Cell: (%d, %d), Best Value: %f"), BestCell.X, BestCell.Y, BestCValue);
			UE_LOG(LogTemp, Warning, TEXT("Destination: %s"), *BestCellPosition.ToString());

			PathComp->SetDestination(BestCellPosition, CameFrom);



			// Reconstruct the path from the start point to the BestCell
			/*TArray<FVector> PathPoints;
			TMap<FCellRef, FVector> CameFrom;
			CameFromToPath(CameFrom, StartPoint, BestCellPosition, PathPoints, Grid);

			// Print out the values of PathPoints
			UE_LOG(LogTemp, Warning, TEXT("PathPoints after CameFromToPath:"));
			for (const FVector& Point : PathPoints)
			{
				UE_LOG(LogTemp, Warning, TEXT("Point: %s"), *Point.ToString());
			}

			// Convert the array of points to Steps array
			PathComp->Steps.SetNum(PathPoints.Num());
			int32 i = 0;
			for (const FVector& Point : PathPoints)
			{
				FCellRef CellRef = Grid->GetCellRef(Point);
				ECellData CellData = Grid->GetCellData(CellRef);
				// Check if the cell is traversable
				if (CellData == ECellData::CellDataTraversable)
				{
					PathComp->Steps[i].Set(FVector2D(Point.X, Point.Y), CellRef);
					++i;
				}*/



		}


		if (Debug)
		{
			// Note: this outputs (basically) the results of the position selection
			// However, you can get creative with the debugging here. For example, maybe you want
			// to be able to examine the values of a specific layer in the spatial function
			// You could create a separate debug map above (where you're doing the evaluations) and
			// cache it off for debug rendering. Ideally you'd be able to control what layer you wanted to 
			// see from blueprint

			GridActor->DebugGridMap = GridMap;
			GridActor->RefreshDebugTexture();
			GridActor->DebugMeshComponent->SetVisibility(true);		//cheeky!
		}
	}

	return Result;
}


void UGASpatialComponent::EvaluateLayer(const FFunctionLayer& Layer, FGAGridMap& GridMap, FGAGridMap& DistanceMap, FCellRef& BestCell) const
{


	float BestValue = -FLT_MAX;

	for (int32 Y = GridMap.GridBounds.MinY; Y < GridMap.GridBounds.MaxY; Y++)
	{
		for (int32 X = GridMap.GridBounds.MinX; X < GridMap.GridBounds.MaxX; X++)
		{
			AActor* OwnerPawn = GetOwnerPawn();
			const AGAGridActor* Grid = GetGridActor();
			APawn* PlayerPawn = UGameplayStatics::GetPlayerPawn(this, 0);
			FVector PlayerPosition = PlayerPawn->GetActorLocation();
			FCellRef CellRef(X, Y);
			float CellValue;


			if (EnumHasAllFlags(Grid->GetCellData(CellRef), ECellData::CellDataTraversable))
			{

				// evaluate me!

				float Value = 0.0f;
				switch (Layer.Input)
				{
				case ESpatialInput::SI_None:
					// No input, so value remains 0
					break;
				case ESpatialInput::SI_TargetRange:
					// Evaluate distance to target and set as value
					Value = FVector::DistSquared(Grid->GetCellPosition(CellRef), PlayerPosition);
					break;
				case ESpatialInput::SI_PathDistance:
					// Retrieve path distance from the pre-calculated distance map
					float PathDistanceValue;
					Value = DistanceMap.GetValue(CellRef, PathDistanceValue);
					break;
				case ESpatialInput::SI_LOS:
					// Cast a ray to check line of sight
					UWorld* World = GetWorld();
					FHitResult HitResult;
					FCollisionQueryParams Params;
					FVector Start = Grid->GetCellPosition(CellRef);  // Ray start from center of the cell
					FVector End = PlayerPawn->GetActorLocation();    // Ray end at player's location
					Start.Z = End.Z;  // Hack to align the ray with the player's Z position
					Params.AddIgnoredActor(PlayerPawn);  // Ignore the player pawn
					Params.AddIgnoredActor(OwnerPawn);   // Ignore the owner pawn (AI)

					bool bHitSomething = World->LineTraceSingleByChannel(HitResult, Start, End, ECollisionChannel::ECC_Visibility, Params);
					if (!bHitSomething)
					{
						// No obstruction found, clear LOS
						Value = 1.0f;
					}
					else
					{
						// Obstruction found, no LOS
						Value = 0.0f;
					}
					// Add cases for additional input types if needed
				}

				// Apply response curve to the value
				float ModifiedValue = Layer.ResponseCurve.GetRichCurveConst()->Eval(Value);

				// Apply operation based on layer operation type
				switch (Layer.Op)
				{
				case ESpatialOp::SO_None:
					// No operation, just set the modified value
					GridMap.SetValue(CellRef, ModifiedValue);
					break;
				case ESpatialOp::SO_Add:
					// Add modified value to the existing value in the grid map
					if (GridMap.GetValue(CellRef, Value))
					{
						GridMap.SetValue(CellRef, ModifiedValue + GridMap.GetValue(CellRef, Value));
					}
					break;
				case ESpatialOp::SO_Multiply:
					// Multiply modified value with the existing value in the grid map
					if (GridMap.GetValue(CellRef, Value))
					{
						GridMap.SetValue(CellRef, ModifiedValue * GridMap.GetValue(CellRef, Value));
					}
					break;
					// Add cases for additional operation types if needed
				}

				if (GridMap.GetValue(CellRef, CellValue))
				{
					// Check if this cell has a higher value than the current best
					if (CellValue > BestValue)
					{
						BestValue = CellValue;
						BestCell = CellRef;
						UE_LOG(LogTemp, Warning, TEXT("Best Cell: (%d, %d), Best Value: %f"), BestCell.X, BestCell.Y, BestValue);

					}
				}
				UE_LOG(LogTemp, Warning, TEXT("Best Cell: (%d, %d), Best Value: %f"), BestCell.X, BestCell.Y, BestValue);



				// HERE ARE SOME ADDITIONAL HINTS

				// Here's how to get the player's pawn
				// APawn *PlayerPawn = UGameplayStatics::GetPlayerPawn(this, 0);

				// Here's how to cast a ray

				// UWorld* World = GetWorld();
				// FHitResult HitResult;
				// FCollisionQueryParams Params;
				// FVector Start = Grid->GetCellPosition(CellRef);		// need a ray start
				// FVector End = PlayerPawn->GetActorLocation();		// need a ray end
				// Start.Z = End.Z;		// Hack: we don't have Z information in the grid actor -- take the player's z value and raycast against that
				// Add any actors that should be ignored by the raycast by calling
				// Params.AddIgnoredActor(PlayerPawn);			// Probably want to ignore the player pawn
				// Params.AddIgnoredActor(OwnerPawn);			// Probably want to ignore the AI themself
				// bool bHitSomething = World->LineTraceSingleByChannel(HitResult, Start, End, ECollisionChannel::ECC_Visibility, Params);
				// If bHitSomething is false, then we have a clear LOS
			}
		}
	}
}

