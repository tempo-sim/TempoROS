# TempoROS
A plugin to integrate ROS and Unreal via `rclcpp`.

This plugin was written by Tempo Simulation, LLC, and is free for anyone to use under the Apache License 2.0. Interested in learning more about Tempo? Visit us [temposimulation.com](https://temposimulation.com), or check out the full suite of [Tempo Unreal Plugins](https://github.com/tempo-sim/Tempo).

`TempoROS`, unlike other Tempo plugins, is a standalone unit, and does not depend on the others (not even `TempoCore`). **You can use `TempoROS` even if you are not using other Tempo plugins in your project.**

If you enable `TempoROS` in a project where you **are** using the other Tempo plugins you should also enable `TempoROSBridge`.

> [!Note]
> `TempoROS`, like `TempoScripting`, includes a pre-build code generation step for ROS IDL files in your project. If you're not changing ROS IDL files, and you've built at least once, you can use the `TEMPO_SKIP_PREBUILD` environment variable to skip this step. Note that you may have to restart your IDE after changing this.

## Quick Start

### Clone TempoROS
> [!Warning]
> Skip this if you are using `TempoROS` as part of the rest of Tempo.

From your project's Plugins directory run<br />
`git submodule add https://github.com/tempo-sim/TempoROS.git`<br />

### First-Time Setup
> [!Warning]
> Skip this if you are using `TempoROS` as part of the rest of Tempo. Tempo's `Setup.sh` will take care of it.

Run the `Setup.sh` script (from the `TempoROS` root) once. This script will:
- Install third party dependencies (`rclcpp`)
- Add git hooks to keep the above in sync automatically as you check out different `TempoROS` commits
> [!Note]
> If you run `Setup.sh` again it shouldn't do anything, because it can tell it's already run. If something goes wrong you can force it to run again with the `-force` flag.

### Project Requirements
`TempoROS` imposes a two requirements on your project:
- You must enable exceptions by adding `bEnableExceptions = true;` to the `Build.cs` file for any module that depends on `TempoROS` or `rclcpp`.
- You must specify a custom stage copy handler by adding `CustomStageCopyHandler=TempoROSCopyHandler` to your `Config/DefaultGame.ini`.
- To run an Unreal packaged game with TempoROS on Windows, you must add the directory `<package_root>/UE/TempoSample/Plugins/Tempo/TempoROS/Source/ThirdParty/rclcpp/Binaries/Windows` to your `PATH` environment variable. We are searching for a way to remove this requirement.

### TempoROS in C++
Using `TempoROS` from C++ is very simple:
```
// Create a UTempoROSNode. UTempoROSNodes are UObjects, store them in a UPROPERTY().
ROSNode = UTempoROSNode::Create("MyNode", this);

// Add a publisher.
ROSNode->AddPublisher<FString>("my_topic", false /*bPrependNodeName*/);

// Add a subscription.
ROSNode->AddSubscription<FString>("my_topic", TROSSubscriptionDelegate<FString>::CreateLambda([](double FString& Message)
{
    UE_LOG(LogTemp, Display, TEXT("Got a message: %s"), *Message);
}));

// Publish a message.
ROSNode->Publish("my_topic", FString("Hello World!));
```
### TempoROS in Blueprint
Using `TempoROS` from Blueprint is also straightforward. This Blueprint is equivalent to the above C++:
<img width="1868" alt="Screenshot 2024-10-15 at 10 06 14 PM" src="https://github.com/user-attachments/assets/2e8df465-b940-43df-8823-c52b6eb12900">

## Design
`TempoROS` uses [rclcpp](https://github.com/ros2/rclcpp), ROS's C++ client library, to integrate ROS and Unreal at the C++ level. Other popular Unreal plugins for ROS use a custom API to interface with Unreal and then an external "bridge" library or process to translate messages to and from ROS messages.

`TempoROS`'s design has several notable advantages over the "bridge" alternative:
- It avoids unnecessary serialization, deserialization, and network transport to get data in and out of the Unreal project by using ROS messages directly.
- It leverages ROS client libraries, like `tf2` and `image_transport` enabling users to take advantage of their convenient APIs in the Unreal project.
- It achieves zero-copy transport of messages from the Unreal project to external ROS nodes, when the two are on the same machine, using shared memory.

`TempoROS`'s design was challenging to implement for several reasons:
- `rclcpp` relies on several assumptions about the layout of a C++ codebase and locations of compiled libraries, so `TempoROS` attempts to satisfy those assumptions in the Unreal project's environment, including by organizing generated code in just the right way and setting several environment variables to help `rclcpp` find the libraries it needs during compilation, linking, and runtime.
- `rclcpp` has a large number of [third party dependencies](https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos), several of which are also dependencies of Unreal. To avoid compatibility issues when combining the two systems, `TempoROS` uses a custom build of `rclcpp` linked against Unreal's third party libraries.
- `rclcpp` uses C++ features that are not enabled by default in Unreal C++ projects, including exceptions and RTTI (`dynamic_cast` and `type_id`). To resolve the former, users must enable exceptions in modules that depend on `TempoROS` (see Project Requirements). For the latter, we've modified `rclcpp` to remove any uses of RTTI from header files.

## User Guide
### Unreal Type Conversion
In order to publish and subscribe to ROS topics with native Unreal types (e.g. `FMyType`) in `TempoROS` you must:
- Define a `TImplicitToROSConverter` (for publishing) and/or a `TImplicitFromROSConverter` (for subscribing), which tell `TempoROS` which ROS type to convert your Unreal type to/from, and how to do so.
- Define a "type trait" for your Unreal type, with `DEFINE_TEMPOROS_MESSAGE_TYPE_TRAITS(FMyType)`.
- [Optional] If your Unreal type is a `BlueprintType`, you can add a special comment, `// TempoROS__BPSupport`, on the implicit converter definition line, and `TempoROS` will take care of the rest.

We've added several common types already in `TempoROSCommonConverters.h`. For example, here are the implicit converters for `FVector`:
```
template <>
struct TImplicitToROSConverter<FVector> : TToROSConverter<geometry_msgs::msg::Vector3, FVector> // TempoROS__BPSupport
{
	static ToType Convert(const FromType& TempoVector)
	{
		ToType ROSVector;
		ROSVector.x = 0.01 * TempoVector.X;
		ROSVector.y = -0.01 * TempoVector.Y;
		ROSVector.z = 0.01 * TempoVector.Z;
		return ROSVector;
	}
};

template <>
struct TImplicitFromROSConverter<FVector> : TFromROSConverter<geometry_msgs::msg::Vector3, FVector> // TempoROS__BPSupport
{
	static ToType Convert(const FromType& ROSVector)
	{
		return ToType(100.0 * ROSVector.x, -100.0 * ROSVector.y, 100.0 * ROSVector.z);
	}
};
```
The above implicit converters associate `FVector` with the ROS type `geometry_msgs::msg::Vector3` and handle conversion from Unreal's left-handed, centimeters coordinate frame to ROS's right-handed, meters one.

To be clear, there's nothing stopping you from creating native ROS types in C++ and publishing/subscribing to them too. But, isn't it nicer to keep everything consistent?
### Nodes
In ROS a "Node" is an object that can publish and subscribe to messages. `TempoROS` wraps ROS's underlying Node type in a `UObject`, `UTempoROSNode`. To create a `UTempoROSNode` from C++ or Blueprint you should use the `UTempoROSNode::Create` method (instead of `NewObject`). For example:
```
ROSNode = UTempoROSNode::Create("MyNode", this);
```
Like any `UObject`, you should retain ownership of the returned `UTempoROSNode` in a `UPROPERTY` class variable (otherwise Unreal will garbage collect it).

### Publishers
You can add a publisher to a `UTempoROSNode` like this:
```
ROSNode->AddPublisher<FMyType>("my_topic");
```
You can also specify two optional parameters:
- An `FROSQOSProfile`, to tell ROS how to handle different situations that can arise when sending messages
- A boolean, `bPrependNodeName`, default `True`

Then, you can publish a message on that topic like this:
```
ROSNode->Publish<FMyType>("my_topic", FMyType());
```
ROS topics can have exactly one message type. If you try to publish a message of one type on a channel you've defined with a different type, you will get a runtime error. If you try to publish a message on a topic and you've not defined that message's type traits (with `DEFINE_TEMPOROS_MESSAGE_TYPE_TRAITS`), you will get a runtime error. Don't forget to do it!

It is not possible to expose templated methods to Blueprint, so `TempoROS` automatically generates individual methods (e.g. `AddMyTypePublisher` and `PublishMyType`) for each type with Blueprint support.

### Subscriptions
You can add a subscription to a `UTempoROSNode` like this:
```
ROSNode->AddSubscription<FMyType>("my_topic", TROSSubscriptionDelegate<FMyType>::CreateUObject(this, &UMyClass::MyTypeHandler);
```
It is not possible to expose templated methods to Blueprint, so we generate individual methods (e.g. `AddMyTypeSubscription`) for each type with Blueprint support.

## Services
ROS also has the concept of services, which use publishers and subscribers under the hood but offer the client the simplicity of a simple call and response. To host a ROS service you must first define the service's request and response types, like this:
```
struct FMyService
{
	using Request = FMyRequestType;
	using Response = FMyResponseType;
};
```
Where `FMyRequestType` and `FMyResponseType` are either native ROS types or Unreal types for which you've defined a `TImplicitToROSConverter` (for responses) or `TImplicitFromROSConverter` (for requests).

Then, you can add the service to your `UTempoROSNode` like this:
```
ROSNode->AddService<FMyService>("my_service", TROSServiceDelegate<FMyService>::CreateUObject(this, &UMyClass::MyServiceHandler);
```
In the above example the handler's signature must be:
```
FMyResponseType UMyClass::MyServiceHandler(const FMyRequestType&)
```
### Custom Message & Service Types
ROS supports custom message and service types defined via their [IDL](https://design.ros2.org/articles/idl_interface_definition.html). To use custom message and service types in `TempoROS`, you should define them in a special folder, `msg` or `srv` in your module's `Public` or `Private` folders.

Then, `TempoROS` will generate the corresponding C++ code for your types in a pre-build step and store the generated code in a new folder, still under `Public` or `Private`, that is your module's name but `snake_cased`. You can `#include` the generated headers as you would any built-in ROS type.

You can find many examples of defining and using custom services and messages in the `TempoROSBridge` plugin.

### Clock Server
The `/clock` topic is special in ROS because it is the only topic for which there may only be one publisher. `TempoROS` includes a `UTempoROSClockServer` subsystem, which will automatically be created and will publish the simulation time to the `/clock` channel every frame. You don't have to do anything to enable this, but be sure not to publish anything on the `/clock` channel yourself.

### TF2
ROS includes [tf2](https://wiki.ros.org/tf2), a library that makes subscribing and publishing a graph of transforms more convenient. Some advantages over publishing raw transform messages include:
- It handles transform interpolation. You can ask tf2 for the transform from one frame to another at a specific time, and it will interpolate its internal time buffer of transforms to provide the most accurate response it can.
- It handles recursively propagating transforms through a graph. So, if one node publishes the transform from frame A to frame B, and another publishes the transform from frame B to frame C, anyone can ask tf2 for the transform from frame A to frame C.
- It differentiates between static and dynamic transforms, where static transforms are understood not to change, and therefore have their values "latched", meaning any node asking for a static transform (or chain of static transforms) can get an answer, even if that message containing that transform was published long ago.

To use tf2 in `TempoROS`, you should use `UTempoROSNode`'s `PublishDynamicTransform`, `PublishStaticTransform`, and `GetTransform` methods, which take native Unreal types and, of course, have full Blueprint support :).

### Image Transport Plugins
ROS has a built-in type for image messages (`sensor_msgs::msg::Image`), and there is nothing stopping you from publishing messages of this type directly to a topic.

However, raw image data can be heavy, so ROS also comes with an [image transport](http://wiki.ros.org/image_transport) library and several "image transport plugins" that can compress image data. When you use ROS's image transport to publish images, it will automatically translate your topic into several more topics, one for each transport plugin.

`TempoROS` handles this all behind the scenes. If you publish a `sensor_msgs::msg::Image` message, or any other type that is implicitly convertible to one, `UTempoROSNode` will automatically use ROS's image transport.

### Shared Memory Transport
`TempoROS` includes support for shared memory transport using ROS's [CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds) middleware and [iceoryx](https://github.com/eclipse-iceoryx/iceoryx), on Linux only. To enable this, you must:
- Choose `CycloneDDS` as the RMW Implementation in the TempoROS plugin settings
- Specify the path to a valid `CycloneDDS` xml config in the `CycloneDDS URI` setting
- Run the `roudi` server as a separate process on the same machine. `TempoROS` comes with a pre-build `roudi` (at `TempoROS/Source/ThirdParty/rclcpp/Binaries/Linux/iox-roudi`), but one from a pacakged ROS installation should also work. Note that you'll have to relax its compatibility check, with `iox-roudi -x minor`, as the one `TempoROS` linked against won't match a packaged ROS installation's exactly.
