// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "CoreMinimal.h"
#include "rclcpp/qos.hpp"
#include "rmw/qos_policy_kind.h"

#include "TempoROSTypes.generated.h"

// Note that zero converts to RMW_DURATION_UNSPECIFIED, which every RMW implementation interprets as
// "use my default" (which may or may not be infinite), *not* as a zero-length duration.
static rmw_time_t ToRMWTime(float Value)
{
	const uint64_t Sec = Value;
	const uint64_t NSec = 1e9 * (Value - Sec);
	return rmw_time_t{Sec, NSec};
}

// The name of the QOS policy the middleware reported an incompatibility for, for logging.
inline const TCHAR* QOSPolicyKindName(rmw_qos_policy_kind_t Kind)
{
	switch (Kind)
	{
	case RMW_QOS_POLICY_DURABILITY: return TEXT("durability");
	case RMW_QOS_POLICY_DEADLINE: return TEXT("deadline");
	case RMW_QOS_POLICY_LIVELINESS: return TEXT("liveliness");
	case RMW_QOS_POLICY_RELIABILITY: return TEXT("reliability");
	case RMW_QOS_POLICY_HISTORY: return TEXT("history");
	case RMW_QOS_POLICY_LIFESPAN: return TEXT("lifespan");
	case RMW_QOS_POLICY_DEPTH: return TEXT("depth");
	case RMW_QOS_POLICY_LIVELINESS_LEASE_DURATION: return TEXT("liveliness lease duration");
	case RMW_QOS_POLICY_AVOID_ROS_NAMESPACE_CONVENTIONS: return TEXT("avoid ROS namespace conventions");
	default: return TEXT("unknown");
	}
}

USTRUCT(BlueprintType)
struct FTwist
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector LinearVelocity {FVector::ZeroVector};

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector AngularVelocity {FVector::ZeroVector};
};

UENUM(Blueprintable, BlueprintType)
enum class ERMWImplementation: uint8
{
	FastRTPS = 0,
	CycloneDDS = 1,
};

UENUM(Blueprintable, BlueprintType)
enum class EROSQOSReliability: uint8
{
	BestEffort = 0,
	Reliable = 1,
	SystemDefault = 2,
};

UENUM(Blueprintable, BlueprintType)
enum class EROSQOSDurability: uint8
{
	TransientLocal = 0,
	Volatile = 1,
	SystemDefault = 2,
};

UENUM(Blueprintable, BlueprintType)
enum class EROSQOSLiveliness: uint8
{
	Automatic = 0,
	ManualByTopic = 1,
	SystemDefault = 2,
};

USTRUCT(BlueprintType)
struct FROSQOSProfile
{
	GENERATED_BODY();

	FROSQOSProfile() = default;
	FROSQOSProfile(int32 CustomQueueSize)
		: QueueSize(CustomQueueSize) {}

	// Store all samples, subject to the configured resource limits of the underlying middleware,
	// as opposed to only storing up to N samples, configurable via the queue size option.
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bLimitedQueueSize = true;

	// Number of samples that will be stored when not processed by the subscriber.
	// Ten is the ROS 2 default depth. Note that a queue size of zero does not mean "store nothing": zero is
	// RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT, which leaves the depth at the middleware's default of one.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta=(EditCondition=bLimitedQueueSize, EditConditionHides=true))
	int32 QueueSize = 10;

	// Best effort: attempt to deliver samples, but may lose them if the network is not robust.
	// Reliable: guarantee that samples are delivered, may retry multiple times.
	// Reliable is the ROS 2 default. Note that "system default" is *not* the ROS 2 default: it defers to
	// the DDS default, which is best effort for subscriptions (and reliable for publishers).
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	EROSQOSReliability Reliability = EROSQOSReliability::Reliable;

	// Transient local: the publisher becomes responsible for persisting samples for “late-joining” subscriptions.
	// Volatile: no attempt is made to persist samples.
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	EROSQOSDurability Durability = EROSQOSDurability::Volatile;

	// Should use a non-default deadline?
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bCustomDeadline = false;

	// The expected maximum amount of time between subsequent messages being published to a topic.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta=(EditCondition=bCustomDeadline, EditConditionHides=true))
	float Deadline = 0.0;

	// Should use a non-default lifespan?
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bCustomLifespan = false;

	// The maximum amount of time between the publishing and the reception of a message without the message being
	// considered stale or expired (expired messages are silently dropped and are effectively never received).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta=(EditCondition=bCustomLifespan, EditConditionHides=true))
	float Lifespan = 0.0;

	// Automatic: the system will consider all of the node’s publishers to be alive for another “lease duration” when any one of its publishers has published a message.
	// Manual by topic: the system will consider the publisher to be alive for another “lease duration” if it manually asserts that it is still alive (via a call to the publisher API).
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	EROSQOSLiveliness Liveliness = EROSQOSLiveliness::SystemDefault;

	// Should use a custom duration?
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bCustomLeaseDuration = false;

	// The lease duration to be used in non-default liveliness modes.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta=(EditCondition=bCustomLeaseDuration, EditConditionHides=true))
	float LeaseDuration = 0.0;

	// Whether to use shared memory transport, when available.
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bUseSharedMemory = false;

	FROSQOSProfile& CustomQueueSize(int32 Value)
	{
		checkf(!bUseSharedMemory || Value < 16, TEXT("Shared memory restricts other QOS options"));
		QueueSize = Value;
		bLimitedQueueSize = true;
		return *this;
	}

	FROSQOSProfile& KeepAll()
	{
		checkf(!bUseSharedMemory, TEXT("Shared memory restricts other QOS options"));
		bLimitedQueueSize = false;
		return *this;
	}

	FROSQOSProfile& BestEffort()
	{
		checkf(!bUseSharedMemory, TEXT("Shared memory restricts other QOS options"));
		Reliability = EROSQOSReliability::BestEffort;
		return *this;
	}

	FROSQOSProfile& Reliable()
	{
		Reliability = EROSQOSReliability::Reliable;
		return *this;
	}

	// A transient local *subscription* only receives the samples a publisher persisted before it joined if
	// it is also reliable: that replay is delivered over the reliable (heartbeat/ack) path, so a best
	// effort subscription is only ever sent samples published after it matched the publisher.
	FROSQOSProfile& TransientLocal()
	{
		Durability = EROSQOSDurability::TransientLocal;
		return *this;
	}

	FROSQOSProfile& Volatile()
	{
		Durability = EROSQOSDurability::Volatile;
		return *this;
	}

	FROSQOSProfile& AutomaticLiveliness()
	{
		Liveliness = EROSQOSLiveliness::Automatic;
		return *this;
	}

	FROSQOSProfile& ManualByTopicLiveliness()
	{
		checkf(!bUseSharedMemory, TEXT("Shared memory restricts other QOS options"));
		Liveliness = EROSQOSLiveliness::ManualByTopic;
		return *this;
	}

	FROSQOSProfile& SharedMemory()
	{
		bUseSharedMemory = true;
		// Shared memory restricts other QOS options.
		// https://github.com/ros2/rmw_cyclonedds/blob/humble/shared_memory_support.md#qos-settings
		Liveliness = EROSQOSLiveliness::Automatic;
		Deadline = 0.0;
		Reliability = EROSQOSReliability::Reliable;
		bLimitedQueueSize = true;
		QueueSize = FMath::Min(QueueSize, 16);
		return *this;
	}

	FROSQOSProfile& CustomDeadline(float Value)
	{
		Deadline = Value;
		bCustomDeadline = true;
		return *this;
	}

	FROSQOSProfile& CustomLifespan(float Value)
	{
		Lifespan = Value;
		bCustomLifespan = true;
		return *this;
	}

	FROSQOSProfile& CustomLeaseDuration(float Value)
	{
		LeaseDuration = Value;
		bCustomLeaseDuration = true;
		return *this;
	}

	rclcpp::QoS ToROS() const
	{
		rclcpp::QoS ROSQOS(QueueSize);

		if (!bLimitedQueueSize)
		{
			ROSQOS = ROSQOS.keep_all();
		}

		switch (Reliability)
		{
		case EROSQOSReliability::Reliable:
			{
				ROSQOS = ROSQOS.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
				break;
			}
		case EROSQOSReliability::BestEffort:
			{
				ROSQOS = ROSQOS.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
				break;
			}
		case EROSQOSReliability::SystemDefault:
			{
				ROSQOS = ROSQOS.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
				break;
			}
		}

		switch (Durability)
		{
		case EROSQOSDurability::Volatile:
			{
				ROSQOS = ROSQOS.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
				break;
			}
		case EROSQOSDurability::TransientLocal:
			{
				ROSQOS = ROSQOS.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
				break;
			}
		case EROSQOSDurability::SystemDefault:
			{
				ROSQOS = ROSQOS.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
				break;
			}
		}

		if (bCustomDeadline)
		{
			ROSQOS = ROSQOS.deadline(ToRMWTime(Deadline));
		}

		if (bCustomLifespan)
		{
			ROSQOS = ROSQOS.lifespan(ToRMWTime(Lifespan));
		}

		switch (Liveliness)
		{
		case EROSQOSLiveliness::Automatic:
			{
				ROSQOS = ROSQOS.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
				break;
			}
		case EROSQOSLiveliness::ManualByTopic:
			{
				ROSQOS = ROSQOS.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
				break;
			}
		case EROSQOSLiveliness::SystemDefault:
			{
				ROSQOS = ROSQOS.liveliness(RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT);
				break;
			}
		}

		if (bCustomLeaseDuration)
		{
			ROSQOS = ROSQOS.liveliness_lease_duration(ToRMWTime(LeaseDuration));
		}

		return ROSQOS;
	}
};
