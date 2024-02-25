#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "GameAI/Grid/GAGridActor.h"
#include "GAPathComponent.generated.h"



USTRUCT(BlueprintType)
struct FPathStep
{
	GENERATED_USTRUCT_BODY()

	FPathStep() : Point(FVector2D::ZeroVector), CellRef(FCellRef::Invalid) {}

	void Set(const FVector2D& PointIn, const FCellRef& CellRefIn)
	{
		Point = PointIn;
		CellRef = CellRefIn;
	}

	UPROPERTY(BlueprintReadWrite)
	FVector2D Point;

	UPROPERTY(BlueprintReadWrite)
	FCellRef CellRef;
};

// Note the UMeta -- DisplayName is just a nice way to show the name in the editor
UENUM(BlueprintType)
enum EGAPathState
{
	GAPS_None			UMETA(DisplayName = "None"),
	GAPS_Active			UMETA(DisplayName = "Active"),
	GAPS_Finished		UMETA(DisplayName = "Finished"),
	GAPS_Invalid		UMETA(DisplayName = "Invalid"),
};


// Our custom path following component, which will rely on the data
// contained in the GridActor
// Note the meta-specific "BlueprintSpawnableComponnet". This will allow us
// to attach this component to any actor type in Blueprint. Otherwise it would
// only be attachable in code.

UCLASS(BlueprintType, Blueprintable, meta = (BlueprintSpawnableComponent))
class UGAPathComponent : public UActorComponent
{
	GENERATED_UCLASS_BODY()

private:
	TMap<FCellRef, FVector> CachedCameFrom;
	
public:

	// Note just a cached pointer
	UPROPERTY()
	mutable TSoftObjectPtr<AGAGridActor> GridActor;

	UFUNCTION(BlueprintCallable)
	const AGAGridActor* GetGridActor() const;

	// It is super easy to forget: this component will usually be attached to the CONTROLLER, not the pawn it's controlling
	// A lot of times we want access to the pawn (e.g. when sending signals to its movement component).
	UFUNCTION(BlueprintCallable, BlueprintPure)
	APawn* GetOwnerPawn();


	// State Update ------------------------

	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	void SetCameFrom(const TMap<FCellRef, FVector>& InCameFrom);

	EGAPathState GoThere(TMap<FCellRef, FVector>& CameFrom);

	EGAPathState RefreshPath();

	EGAPathState AStar();

	bool Dijkstra(const FVector& StartPoint, FGAGridMap& DistanceMapOut, TMap<FCellRef, FVector>& CameFrom);

	// bool Dijkstra(const FVector& StartPoint, FGAGridMap& DistanceMapOut);


	void FollowPath();

	// Parameters ------------------------

	// When I'm within this distance of my destination, my path is considered finished.
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	float ArrivalDistance;

	// Destination ------------------------

	UFUNCTION(BlueprintCallable)
	// EGAPathState SetDestination(const FVector& DestinationPoint);

	EGAPathState SetDestination(const FVector& DestinationPoint, TMap<FCellRef, FVector>& CameFrom);

	void RequestPathRebuild();

	void SetDestinationAndRebuildPath(const FVector& DestinationPoint);

	const FVector& GetCurrentDestination() const;

	bool IsDestinationValid() const;

	bool bRebuildPathRequested;

	void SetArrivalDistance(float NewArrivalDistance);

	UPROPERTY(BlueprintReadOnly)
	bool bDestinationValid;

	UPROPERTY(BlueprintReadOnly)
	FVector Destination;

	UPROPERTY(BlueprintReadOnly)
	FCellRef DestinationCell;

	// State ------------------------

	UPROPERTY(BlueprintReadOnly)
	TEnumAsByte<EGAPathState> State;

	UPROPERTY(BlueprintReadWrite)
	TArray<FPathStep> Steps;

};
