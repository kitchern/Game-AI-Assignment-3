#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "HashFunction.generated.h"

UCLASS()
class GAMEAI_API AHashFunction : public ACharacter
{
	GENERATED_BODY()

public:
	// Sets default values for this empty's properties
	AHashFunction();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;
};
