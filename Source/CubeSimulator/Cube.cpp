// Fill out your copyright notice in the Description page of Project Settings.
#pragma once

#include "Cube.h"

#include "../Physics/PhysicsSim.hpp"

Eigen::Vector3f FVector2Eigen(FVector vec) {
    return Eigen::Vector3f(vec.X, vec.Y, vec.Z);
}

Eigen::Vector3f FRotator2Eigen(FRotator rot) {
    return Eigen::Vector3f(rot.Pitch, rot.Yaw, rot.Roll);
}

Eigen::Quaternionf FQuat2Eigen(FQuat quat_) {
    return Eigen::Quaternionf(quat_.W, quat_.X, quat_.Y, quat_.Z);
}

FVector Eigen2FVector(Eigen::Vector3f vec) {
    return FVector(vec.x(), vec.y(), vec.z());
}

FRotator Eigen2FRotator(Eigen::Vector3f vec) {
    return FRotator(vec.x(), vec.y(), vec.z());
}

FQuat Eigen2FQuat(Eigen::Quaternionf quat_) {
    return FQuat(quat_.x(), quat_.y(), quat_.z(), quat_.w());
}

ACube::ACube() {
    PrimaryActorTick.bCanEverTick = true;

    CubeMesh =
        CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Free Fall Cube"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeVisualAsset(
        TEXT("/Game/StarterContent/Shapes/Shape_Cube.Shape_Cube"));
    if (CubeVisualAsset.Succeeded())
        CubeMesh->SetStaticMesh(CubeVisualAsset.Object);

    CubeMesh->SetupAttachment(RootComponent);

    CubeMesh->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);
}

void ACube::BeginPlay() {
    Super::BeginPlay();

    FVector NewLocation = GetActorLocation();
    FRotator NewRotation = GetActorRotation();
    FVector origin;
    FVector tmp;
    GetActorBounds(true, origin, tmp);
    float mass = 1.f;

    FVector boxExtent(50.f, 50.f, 50.f);

    UE_LOG(LogTemp, Display, TEXT("Location: %s"), *NewLocation.ToString());
    UE_LOG(LogTemp, Display, TEXT("Origin: %s"), *origin.ToString());
    UE_LOG(LogTemp, Display, TEXT("Rotation: %s"),
           *Eigen2FVector(FRotator2Eigen(NewRotation)).ToString());
    UE_LOG(LogTemp, Display, TEXT("BoxExtent: %s"), *boxExtent.ToString());

    cube_physics = new CubePhysics(
        mass, FVector2Eigen(boxExtent), FVector2Eigen(origin),
        FVector2Eigen(NewLocation), FQuat2Eigen(NewRotation.Quaternion()));
}

void ACube::Tick(float DeltaTime) {
    Super::Tick(DeltaTime);

    cube_physics->set_delta_time(DeltaTime);

    // collision detection
    TArray<struct FOverlapResult> OutOverlaps;

    // update point
    cube_physics->update_hit_point_set();

    // apply gravity
    cube_physics->resolve_gravity();

    FCollisionShape CollisionShape =
        FCollisionShape::MakeBox(Eigen2FVector(cube_physics->get_box_extent()));
    GetWorld()->OverlapMultiByChannel(
        OutOverlaps, Eigen2FVector(cube_physics->get_origin()),
        Eigen2FQuat(cube_physics->get_quat()),
        ECollisionChannel::ECC_WorldStatic, CollisionShape);
    for (auto&& Result : OutOverlaps)
        GEngine->AddOnScreenDebugMessage(
            -1, 5.f, FColor::White,
            FString::Printf(TEXT("Overlap now: %s"),
                            *Result.GetActor()->GetName()));

    FHitResult Hit;

    if (OutOverlaps.Num() > 0) {
        UE_LOG(LogTemp, Display, TEXT("Hit Now: %s"),
               *Eigen2FVector(cube_physics->get_origin()).ToString());
        cube_physics->resolve_hit_point();
    }

    // warmstart
    cube_physics->warm_start();

    // handle collision
    cube_physics->resolve_collision();

    // update state
    cube_physics->update_actor_state();

    // set
    FVector location = Eigen2FVector(cube_physics->get_location());
    FQuat quat = Eigen2FQuat(cube_physics->get_quat());

    GEngine->AddOnScreenDebugMessage(
        -1, 5.f, FColor::White,
        FString::Printf(TEXT("Cube Location: %s"), *location.ToString()));
    SetActorLocationAndRotation(location, quat.Rotator());
}
