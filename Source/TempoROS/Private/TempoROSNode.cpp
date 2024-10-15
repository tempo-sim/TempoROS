// Copyright Tempo Simulation, LLC. All Rights Reserved

#include "TempoROSNode.h"

UTempoROSNode* UTempoROSNodeBlueprintFunctionLibrary::CreateTempoROSNode(const FString& NodeName, UObject* Owner, bool bAutoTick)
{
	return UTempoROSNode::Create(NodeName, Owner, bAutoTick);
}

UTempoROSNode* UTempoROSNode::Create(const FString& NodeName, UObject* Outer, bool bAutoTick, const rclcpp::NodeOptions& NodeOptions)
{
	UTempoROSNode* NewNode = NewObject<UTempoROSNode>(Outer);
	UWorld* TickWithWorld = nullptr;
	if (bAutoTick)
	{
		if (UWorld* World = Outer->GetWorld())
		{
			TickWithWorld = World;
		}
		else
		{
			UE_LOG(LogTempoROS, Error, TEXT("Unable to find world to auto tick TempoROS node %s"), *NodeName);
		}
	}
	try
	{
		NewNode->Init(NodeName, NodeOptions, TickWithWorld);
	}
	catch (const std::exception& Exception)
	{
		UE_LOG(LogTempoROS, Error, TEXT("Failed to initialize node. Error: %hs"), Exception.what());
	}
	return NewNode;
}

void UTempoROSNode::Init(const FString& NodeName, const rclcpp::NodeOptions& NodeOptions, UWorld* TickWithWorld)
{
	Node = std::make_shared<rclcpp::Node>(TCHAR_TO_UTF8(*NodeName), NodeOptions);
	ImageTransport = std::make_unique<image_transport::ImageTransport>(Node);
	StaticTFPublisher = MakeUnique<FTempoStaticTFPublisher>(Node);
	DynamicTFPublisher = MakeUnique<FTempoDynamicTFPublisher>(Node);
	TFListener = MakeUnique<FTempoTFListener>(Node);
	
	if (TickWithWorld)
	{
		// ROS has nothing to do with movie scene sequences, but this event fires in exactly the right conditions:
		// After world time has been updated for the current frame, before Actor ticks have begun, and even when paused.
		TickWithWorld->AddMovieSceneSequenceTickHandler(FOnMovieSceneSequenceTick::FDelegate::CreateUObject(this, &UTempoROSNode::Tick));
	}
}

void UTempoROSNode::Tick(float DeltaTime) const
{
	rclcpp::spin_some(Node);
}

TSet<FString> UTempoROSNode::GetPublishedTopics() const
{
	TSet<FString> Topics;
	Publishers.GetKeys(Topics);
	return Topics;
}

void UTempoROSNode::RemovePublisher(const FString& Topic)
{
	if (!Publishers.Contains(Topic))
	{
		UE_LOG(LogTempoROS, Error, TEXT("Node did not have publisher for Topic %s"), *Topic);
		return;
	}
	Publishers.Remove(Topic);
}

void UTempoROSNode::RemoveSubscriptions(const FString& Topic)
{
	if (!Subscriptions.Contains(Topic))
	{
		UE_LOG(LogTempoROS, Error, TEXT("Node did not have subscription for Topic %s"), *Topic);
		return;
	}
	Subscriptions.Remove(Topic);
}
