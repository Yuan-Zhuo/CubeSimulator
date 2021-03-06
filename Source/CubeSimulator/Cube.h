// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "../Physics/PhysicsSim.hpp"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
// last
#include "Cube.generated.h"

UCLASS()
class CUBESIMULATOR_API ACube : public AActor {
    GENERATED_BODY()

   public:
    // Sets default values for this actor's properties
    ACube();

   protected:
    // Called when the game starts or when spawned
    virtual void BeginPlay() override;

   public:
    // Called every frame
    virtual void Tick(float DeltaTime) override;

   private:
    UPROPERTY(VisibleAnywhere)
    UStaticMeshComponent* CubeMesh;
    CubePhysics* cube_physics;
};
