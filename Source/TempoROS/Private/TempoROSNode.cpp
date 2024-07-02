// Copyright Tempo Simulation, LLC. All Rights Reserved

#include "TempoROSNode.h"

#include "rclcpp/utilities.hpp"

// Makes sure rclcpp is initialized before creating a node
class FNodeFactory
{
public:
	std::shared_ptr<rclcpp::Node> MakeNode(const FString& Name, const rclcpp::NodeOptions& Options)
	{
		if (!bInitialized)
		{
			Initialize();
		}
		return std::make_shared<rclcpp::Node>(TCHAR_TO_UTF8(*Name), Options);
	}

	static FNodeFactory& GetInstance() { return Instance; }

private:
	void Initialize()
	{
		rclcpp::init(0, nullptr);
		bInitialized = true;
	}
	
	bool bInitialized = false;

	static FNodeFactory Instance;
};

FNodeFactory FNodeFactory::Instance;

UTempoROSNode* UTempoROSNode::Create(const FString& NodeName, UObject* Outer, UWorld* TickWithWorld, const rclcpp::NodeOptions& NodeOptions)
{
	UTempoROSNode* NewNode = NewObject<UTempoROSNode>(Outer);
	NewNode->Init(NodeName, NodeOptions);
	if (TickWithWorld)
	{
		// ROS has nothing to do with movie scene sequences, but this event fires in exactly the right conditions:
		// After world time has been updated for the current frame, before Actor ticks have begun, and even when paused.
		TickWithWorld->AddMovieSceneSequenceTickHandler(FOnMovieSceneSequenceTick::FDelegate::CreateUObject(NewNode, &UTempoROSNode::Tick));
	}
	return NewNode;
}

void UTempoROSNode::Init(const FString& NodeName, const rclcpp::NodeOptions& NodeOptions)
{
	UE_LOG(LogTemp, Warning, TEXT("Making Node %s"), *NodeName);
	Node = FNodeFactory::GetInstance().MakeNode(NodeName, rclcpp::NodeOptions());
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

bool UTempoROSNode::RemovePublisher(const FString& Topic)
{
	if (!Publishers.Contains(Topic))
	{
		UE_LOG(LogTempoROS, Error, TEXT("Node did not have publisher for Topic %s"), *Topic);
		return false;
	}
	Publishers.Remove(Topic);
	return true;
}
