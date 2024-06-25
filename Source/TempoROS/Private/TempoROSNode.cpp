// Copyright Tempo Simulation, LLC. All Rights Reserved

#include "TempoROSNode.h"

#include "rclcpp/utilities.hpp"

class NodeFactory
{
public:
	static std::shared_ptr<rclcpp::Node> MakeNode(const FString& Name)
	{
		if (!bInitialized)
		{
			Initialize();
		}
		return std::make_shared<rclcpp::Node>(TCHAR_TO_UTF8(*Name), rclcpp::NodeOptions());
	}

private:
	static void Initialize()
	{
		rclcpp::init(0, nullptr);
		bInitialized = true;
	}
	static bool bInitialized;
};

bool NodeFactory::bInitialized = false;

UTempoROSNode::UTempoROSNode()
	: Node(NodeFactory::MakeNode("GreatNode")) { }

void UTempoROSNode::Tick() const
{
	rclcpp::spin_some(Node);
}
