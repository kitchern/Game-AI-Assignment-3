#include "GAPathComponent.h"
#include "GameFramework/NavMovementComponent.h"
#include "Kismet/GameplayStatics.h"
#include "Algo/Reverse.h"

UGAPathComponent::UGAPathComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	State = GAPS_None;
	bDestinationValid = false;
	ArrivalDistance = 100.0f;
	bRebuildPathRequested = false;

	// A bit of Unreal magic to make TickComponent below get called
	PrimaryComponentTick.bCanEverTick = true;
}


const AGAGridActor* UGAPathComponent::GetGridActor() const
{
	if (!GridActor.IsValid()) {
		UE_LOG(LogTemp, Warning, TEXT("Invalid GridActor"));
	}

	if (GridActor.Get())
	{
		return GridActor.Get();
	}
	else
	{
		AGAGridActor* Result = NULL;
		AActor* GenericResult = UGameplayStatics::GetActorOfClass(this, AGAGridActor::StaticClass());
		if (GenericResult)
		{
			Result = Cast<AGAGridActor>(GenericResult);
			if (Result)
			{
				// Cache the result
				// Note, GridActor is m5arked as mutable in the header, which is why this is allowed in a const method
				GridActor = Result;
			}
		}

		return Result;
	}

}

APawn* UGAPathComponent::GetOwnerPawn()
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


void UGAPathComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	// Check if a path rebuild is requested
	if (bDestinationValid && bRebuildPathRequested)
	{
		RefreshPath();
		// Reset the request flag
		bRebuildPathRequested = false;
	}

	// Check if the state is active
	if (State == EGAPathState::GAPS_Active)
	{
		FollowPath();
	}
	// Super important! Otherwise, unbelievably, the Tick event in Blueprint won't get called

	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UGAPathComponent::SetCameFrom(const TMap<FCellRef, FVector>& InCameFrom)
{
	CachedCameFrom = InCameFrom;
}


static void CameFromToPath(const TMap<FCellRef, FVector>& CameFrom, const FVector& start, const FVector& goal, TArray<FVector>& path, const AGAGridActor* GridActor)
{
	path.Empty();

	FVector current = goal;
	while (current != start)
	{
		path.Add(current);
		FCellRef CurrentCell;
		if ((GridActor->GetCellRef(current, false).IsValid()))
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

EGAPathState UGAPathComponent::GoThere(TMap<FCellRef, FVector>& CameFrom)
{
	const AGAGridActor* Grid = GetGridActor();

	if (!Grid)
	{
		// Handle the case where the grid is not available
		return GAPS_Invalid;
	}

	FIntRect CellRect;
	FGridBox GridBox(CellRect);
	FGAGridMap DistanceMap(Grid, GridBox, FLT_MAX);
	AActor* Owner = GetOwnerPawn();
	FVector StartPoint = Owner->GetActorLocation();
	FCellRef StartRef = GridActor->GetCellRef(StartPoint, true);
	TArray<FVector> RawPath;

	// Use Dijkstra's algorithm for pathfinding
	if (Dijkstra(StartPoint, DistanceMap, CameFrom))
	{
		// Reconstruct the path from the distance map
		FVector CurrentDestination = GetCurrentDestination();
		CameFromToPath(CameFrom, StartPoint, CurrentDestination, RawPath, GetGridActor());
		FCellRef CurrentDestinationRef = GridActor->GetCellRef(CurrentDestination, true);

		// Convert the array of points to Steps array
		Steps.SetNum(CameFrom.Num());
		int32 i = 0;
		for (const FVector& Point : RawPath)
		{
			FCellRef CellRef = GetGridActor()->GetCellRef(Point);
			ECellData CellData = GetGridActor()->GetCellData(CellRef);
			// Check if the cell is traversable
			if (CellData == ECellData::CellDataTraversable)
			{
				Steps[i].Set(FVector2D(Point.X, Point.Y), CellRef);
				++i;
			}
		}

		// Smooth the raw path
		State = GAPS_Active;
	}
	else
	{
		State = GAPS_Invalid;
	}

	return State;
}

EGAPathState UGAPathComponent::RefreshPath()
{
	AActor* Owner = GetOwnerPawn();
	FVector StartPoint = Owner->GetActorLocation();

	check(bDestinationValid);

	float DistanceToDestination = FVector::Dist(StartPoint, Destination);

	if (DistanceToDestination <= ArrivalDistance)
	{
		// Yay! We got there!
		State = GAPS_Finished;
	}
	else
	{
		// Replan the path in the event its needed
		if (State != GAPS_Active) // Check if not already actively following a path
		{
			State = GAPS_Active;
			TArray<FVector> PathPoints;
			TMap<FCellRef, FVector> CameFrom;
			return GoThere(CachedCameFrom); // Call the pathfinding function and return its result
		}
	}

	return State;
}


static void SmoothPath(const TArray<FVector>& OriginalPath, TArray<FVector>& SmoothedPath)
{
	SmoothedPath.Empty();

	if (OriginalPath.Num() < 2)
	{
		// Cannot smooth a path with less than two points
		return;
	}

	// Add the first point
	SmoothedPath.Add(OriginalPath[0]);

	for (int32 i = 1; i < OriginalPath.Num() - 1; ++i)
	{
		// Use linear interpolation to add intermediate points
		FVector SmoothedPoint = FMath::Lerp(OriginalPath[i - 1], OriginalPath[i + 1], 0.5f);
		SmoothedPath.Add(SmoothedPoint);
	}

	// Add the last point
	SmoothedPath.Add(OriginalPath.Last());
}

static bool IsCellValid(const AGAGridActor* GridActor, const FCellRef& Cell)
{
	if (!GridActor)
	{
		// Handle the case where the grid actor is not available
		return false;
	}

	// Check if the cell is within the grid bounds
	return Cell.X >= 0 && Cell.X < GridActor->XCount && Cell.Y >= 0 && Cell.Y < GridActor->YCount;
}



static void FindNeighbors(const AGAGridActor* GridActor, const FCellRef& CurrentCell, TArray<FCellRef>& Neighbors)
{
	Neighbors.Empty();

	if (!GridActor)
	{
		// Handle the case where the grid actor is not available
		return;
	}

	// Assuming a fixed grid step size
	const int32 GridStep = 1;

	// Add neighboring cell references based on the grid step size
	for (int32 YOffset = -1; YOffset <= 1; ++YOffset)
	{
		for (int32 XOffset = -1; XOffset <= 1; ++XOffset)
		{
			// Skip the current cell
			if (XOffset == 0 && YOffset == 0)
			{
				continue;
			}

			FCellRef NeighborCell(CurrentCell.X + XOffset * GridStep, CurrentCell.Y + YOffset * GridStep);
			if (IsCellValid(GridActor, NeighborCell))
			{
				Neighbors.Add(NeighborCell);
			}
		}
	}
}



static void dijkstra(const FVector& StartPoint, FGAGridMap& DistanceMap, TMap<FCellRef, FVector>& CameFrom, const AGAGridActor* GridActor)
{
	// Initialize all distances as INFINITE and set sptSet[] as false
	TMap<FCellRef, float> dist;
	TMap<FCellRef, bool> sptSet;

	// Initialize all distances as INFINITE and sptSet[] as false
	for (int32 Y = DistanceMap.GridBounds.MinY; Y <= DistanceMap.GridBounds.MaxY; ++Y)
	{
		for (int32 X = DistanceMap.GridBounds.MinX; X <= DistanceMap.GridBounds.MaxX; ++X)
		{
			FCellRef CellRef(X, Y);
			dist.Add(CellRef, FLT_MAX);
			sptSet.Add(CellRef, false);
		}
	}

	// Distance of source vertex from itself is always 0
	FCellRef SourceVertex = GridActor->GetCellRef(StartPoint);
	dist.Add(SourceVertex, 0);

	int32 Area = (DistanceMap.GridBounds.MaxX - DistanceMap.GridBounds.MinX + 1) *
		(DistanceMap.GridBounds.MaxY - DistanceMap.GridBounds.MinY + 1);

	// Find shortest path for all vertices
	for (int count = 0; count < Area - 1; count++)
	{
		// Pick the minimum distance vertex from the set of vertices not yet processed
		FCellRef u;
		float minDist = FLT_MAX;
		for (const auto& Pair : dist)
		{
			if (!sptSet[Pair.Key] && Pair.Value <= minDist)
			{
				minDist = Pair.Value;
				u = Pair.Key;
			}
		}

		// Mark the picked vertex as processed
		sptSet[u] = true;

		// Update dist value of the adjacent vertices of the picked vertex
		TArray<FCellRef> Neighbors;
		// Assuming FindNeighbors returns neighboring cell references
		FindNeighbors(GridActor, u, Neighbors);
		for (const FCellRef& Neighbor : Neighbors)
		{
			// Update dist[v] only if it's not in sptSet and there is a path from u to v
			if (!sptSet[Neighbor] && dist[u] != FLT_MAX)
			{
				// Assuming all edges have a weight of 1, so we increment the distance by 1
				dist[Neighbor] = dist[u] + 1;

				// Store the predecessor for path reconstruction
				FVector NeighborWorldPos = GridActor->GetCellPosition(Neighbor);
				CameFrom.Add(Neighbor, NeighborWorldPos);
			}
		}
	}

	// Copy the computed distances to DistanceMap
	for (const auto& Pair : dist)
	{
		DistanceMap.SetValue(Pair.Key, Pair.Value);
	}
}

EGAPathState UGAPathComponent::AStar()
{

	return GAPS_Finished;
}


bool UGAPathComponent::Dijkstra(const FVector& StartPoint, FGAGridMap& DistanceMapOut, TMap<FCellRef, FVector>& CameFrom)
{
	const AGAGridActor* Grid = GetGridActor();

	if (!Grid)
	{
		// Handle the case where the grid is not available
		return false;
	}

	// Initialize a grid to store distances

	// Run Dijkstra's algorithm
	dijkstra(StartPoint, DistanceMapOut, CameFrom, Grid);

	// Reconstruct the path from the start point to the destination
	/*FVector GoalPoint = Destination;
	TArray<FVector> PathPoints;
	CameFromToPath(CameFrom, StartPoint, GoalPoint, PathPoints, Grid);
	UE_LOG(LogTemp, Warning, TEXT("PathPoints after CameFromToPath:"));
	for (const FVector& Point : PathPoints)
	{
		UE_LOG(LogTemp, Warning, TEXT("Point: %s"), *Point.ToString());
	}

	// Convert the array of points to Steps array
	Steps.SetNum(PathPoints.Num());
	int32 i = 0;
	for (const FVector& Point : PathPoints)
	{
		FCellRef CellRef = Grid->GetCellRef(Point);
		ECellData CellData = Grid->GetCellData(CellRef);
		// Check if the cell is traversable
		if (CellData == ECellData::CellDataTraversable)
		{
			Steps[i].Set(FVector2D(Point.X, Point.Y), CellRef);
			++i;
		}
	}
	*/


	// Update the DistanceMapOut if needed
	// For now, let's just log a warning message
	UE_LOG(LogTemp, Warning, TEXT("Dijkstra's algorithm completed!"));

	/*UE_LOG(LogTemp, Warning, TEXT("Printing Steps:"));
	for (const auto& Step : Steps)
	{
		UE_LOG(LogTemp, Warning, TEXT("Step: Point(%f, %f), CellRef(%d, %d)"),
			Step.Point.X, Step.Point.Y, Step.CellRef.X, Step.CellRef.Y);
	}
	*/
	return true;
}

void UGAPathComponent::FollowPath()
{
	AActor* Owner = GetOwnerPawn();
	FVector StartPoint = Owner->GetActorLocation();

	check(State == GAPS_Active);
	check(Steps.Num() > 0);

	// Always follow the first step, assuming that we are refreshing the whole path every tick
	FVector V = FVector(Steps[0].Point, 0.0f) - StartPoint;
	V.Normalize();

	UNavMovementComponent* MovementComponent = Owner->FindComponentByClass<UNavMovementComponent>();
	if (MovementComponent)
	{
		MovementComponent->RequestPathMove(V);
	}
}



EGAPathState UGAPathComponent::SetDestination(const FVector& DestinationPoint, TMap<FCellRef, FVector>& CameFrom)
{
	Destination = DestinationPoint;

	State = GAPS_Invalid;
	bDestinationValid = true;

	const AGAGridActor* Grid = GetGridActor();
	if (Grid)
	{
		FCellRef CellRef = Grid->GetCellRef(Destination);
		if (CellRef.IsValid())
		{
			DestinationCell = CellRef;
			bDestinationValid = true;

			RequestPathRebuild();
			SetCameFrom(CameFrom);
			RefreshPath();
		}
	}

	return State;
}

void UGAPathComponent::RequestPathRebuild()
{
	bRebuildPathRequested = true;
}

void UGAPathComponent::SetDestinationAndRebuildPath(const FVector& DestinationPoint)
{
	Destination = DestinationPoint;
	bDestinationValid = true;
	RequestPathRebuild();
}



const FVector& UGAPathComponent::GetCurrentDestination() const
{
	return Destination;
}

bool UGAPathComponent::IsDestinationValid() const
{
	return bDestinationValid;
}

void UGAPathComponent::SetArrivalDistance(float NewArrivalDistance)
{
	ArrivalDistance = NewArrivalDistance;
}
