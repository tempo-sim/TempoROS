# TempoROS
A plugin to integrate ROS 2 and Unreal via `rclcpp`.

This plugin was written by Tempo Simulation, LLC, and is free for anyone to use under the Apache License 2.0. Interested in learning more about Tempo? Visit us [temposimulation.com](https://temposimulation.com), or check out the full suite of [Tempo Unreal Plugins](https://github.com/tempo-sim/Tempo).

`TempoROS`, unlike other Tempo plugins, is a standalone unit. **You can use `TempoROS` even if you are not using other `Tempo` plugins in your project.**

Have a question? Find us on [![Discord](https://img.shields.io/badge/Discord-Join%20Server-5865F2?logo=discord&logoColor=white)](https://discord.gg/bKa2hnGYnw)

## Compatibility
- Linux (Ubuntu 22.04 and 24.04), MacOS (13.0 "Ventura" or newer, Apple silicon only), Windows 10 and 11
- Unreal Engine 5.4, and 5.5, and 5.6
- ROS 2 [humble](https://docs.ros.org/en/humble/index.html)

## Prerequisites
- Linux:
  - Unreal: Download and install from https://www.unrealengine.com/en-US/linux
  - `curl`: `sudo apt update && sudo apt install curl`
  - `jq`: `sudo apt update && sudo apt install jq` 
- Mac:
  - Unreal: Install using Epic Games Launcher
  - `jq`: `brew install jq`
- Windows:
  - Unreal: Install using Epic Games Launcher
  - [Git Bash](https://gitforwindows.org/) (Run all Tempo `*.sh` scripts from Git Bash)
  - `jq`: (Use Administrator Git Bash) `curl -L -o /usr/bin/jq.exe https://github.com/jqlang/jq/releases/latest/download/jq-win64.exe`

## Environment Variables
- `UNREAL_ENGINE_PATH`: On Linux only must be set to your Unreal Engine installation directory (the folder containing `Engine`). On Mac and Windows, TempoROS will attempt to automatically find Unreal via your uproject file, but you can still set this to override it.

## Quick Start
### Standalone Setup
> [!Warning]
> Skip this if you are using `TempoROS` as part of the rest of `Tempo`. `TempoROS` is a submodule of `Tempo`, and `Tempo`'s `Setup.sh` will call `TempoROS`'s `Setup.sh`.
- Clone `TempoROS`. From your project's Plugins directory:
  - If you **are** using git to track your Unreal project: `git submodule add https://github.com/tempo-sim/TempoROS.git`
  - If you **are not** using git to track your Unreal project: `git clone https://github.com/tempo-sim/TempoROS.git`
- Run the `Setup.sh` script (from the `TempoROS` root) once. This script will:
  - Install third party dependencies (`rclcpp` and its dependencies)
  - Add git hooks to update dependencies automatically

If you run `Setup.sh` again it shouldn't do anything. However you can always force it to run again with the `-force` flag.

### Using Bundled ROS Environment
Of course, `TempoROS` can connect to your local ROS installation. For quick CLI debugging, it also comes with its own minimal ROS environment. You can run `source ./Scripts/ROSEnv.sh` to activate it. Then you can use `ros2 topic list`, `ros2 topic echo`, etc.

### TempoROSBridge
If you enable `TempoROS` in a project where you **are** using the other Tempo plugins you should also enable [TempoROSBridge](https://github.com/tempo-sim/Tempo/tree/release/TempoROSBridge), `Tempo`'s plugin to adapt its existing API to ROS.

### Enable Exceptions
You must enable exceptions for any module that depends on `TempoROS` or `rclcpp` by adding `bEnableExceptions = true;` to its `Build.cs` file.

### Using TempoROS in C++
Using `TempoROS` from C++ is very simple:
```
// Create a UTempoROSNode. UTempoROSNodes are UObjects, store them in a UPROPERTY().
ROSNode = UTempoROSNode::Create("MyNode", this);

// Add a publisher.
ROSNode->AddPublisher<FString>("my_topic", false /*bPrependNodeName*/);

// Add a subscription.
ROSNode->AddSubscription<FString>("my_topic", TROSSubscriptionDelegate<FString>::CreateLambda([](const FString& Message)
{
    UE_LOG(LogTemp, Display, TEXT("%s"), *Message);
}));

// Publish a message.
ROSNode->Publish<FString>("my_topic", TEXT("Hello World!"));
```
### Using TempoROS in Blueprint
Using `TempoROS` from Blueprint is also straightforward. This Blueprint is equivalent to the above C++:

<img width="1868" alt="Screenshot 2024-10-15 at 10 06 14 PM" src="https://github.com/user-attachments/assets/2e8df465-b940-43df-8823-c52b6eb12900">

## Design
`TempoROS` uses [rclcpp](https://github.com/ros2/rclcpp), ROS's C++ client library, to integrate ROS and Unreal at the C++ level. Other popular Unreal plugins for ROS use a custom API to interface with Unreal and then an external "bridge" library or process to translate messages to and from ROS messages.

`TempoROS`'s design has several notable advantages over the "bridge" alternative:
- It avoids unnecessary serialization, deserialization, and network transport to get data in and out of the Unreal project by using ROS messages directly.
- It leverages ROS client libraries, like `tf2` and `image_transport` enabling users to take advantage of their convenient APIs in the Unreal project.
- It achieves zero-copy transport of messages from the Unreal project to an external ROS node, when the two are on the same machine, using shared memory.

`TempoROS`'s design was challenging to implement for several reasons:
- `rclcpp` relies on several assumptions about the layout of a C++ codebase and locations of compiled libraries, so `TempoROS` attempts to satisfy those assumptions in the Unreal project's environment, including by organizing generated code in just the right way and setting several environment variables to help `rclcpp` find the libraries it needs during compilation, linking, and runtime.
- `rclcpp` has a large number of [third party dependencies](https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos), several of which are also dependencies of Unreal. To avoid compatibility issues when combining the two systems, `TempoROS` uses a custom build of `rclcpp` linked against Unreal's third party libraries.
- `rclcpp` uses C++ features that are not enabled by default in Unreal C++ projects, including exceptions and RTTI (`dynamic_cast` and `type_id`). To resolve the former, users must enable exceptions in modules that depend on `TempoROS` (see Project Requirements). For the latter, `TempoROS` uses a modified `rclcpp` with any uses of RTTI from header files removed.

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
In the above example the handler's signature must be:
```
void MyTypeHandler(const FMyType&)
```
It is not possible to expose templated methods to Blueprint, so `TempoROS` automatically generates individual methods (e.g. `AddMyTypeSubscription`) for each type with Blueprint support.

### Services
ROS also has the concept of services, which use publishers and subscribers under the hood but offer the client the simplicity of a self-contained call and response. To host a ROS service you must first define the service's request and response types, like this:
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

Then, `TempoROS` will generate the corresponding C++ code for your types automatically in a pre-build step and store the generated code in a new folder, still under `Public` or `Private`. By default the generated folder's name, which is also the name of the ROS package containing your custom messages and services, is your module's name but `snake_cased`. You can override this name to give your package a custom name by adding a `ros_info.json` file at the same level as your module's `Build.cs` file, with these contents:

```
{
  "custom_package_name": "my_custom_package_name"
}
```

Once generated, you can `#include` the generated headers from C++ as you would any built-in ROS type. You can find many examples of defining and using custom services and messages in the [TempoROSBridge](https://github.com/tempo-sim/Tempo/tree/release/TempoROSBridge) plugin.

> [!Note]
> `TempoROS`, like `TempoScripting`, includes a pre-build code generation step for ROS IDL files in your project. If you are not changing ROS IDL files, and you've built at least once, you can use the `TEMPO_SKIP_PREBUILD` environment variable to skip this step. Note that you may have to restart your IDE after changing this.

**Windows Only**:
On Windows, you must add a few private preprocessor definitions to the `Build.cs` file for any module that defines custom ROS messages or services:
```
if (Target.Platform == UnrealTargetPlatform.Win64)
{
    PrivateDefinitions.Add("ROSIDL_TYPESUPPORT_FASTRTPS_CPP_BUILDING_DLL_<custom_package_name_or_module_name_snake_case>=1");
    PrivateDefinitions.Add("ROSIDL_TYPESUPPORT_CPP_BUILDING_DLL=1");
    PrivateDefinitions.Add("ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_BUILDING_DLL=1");
}
```

### Clock Server
The `/clock` topic is special in ROS because it is the only topic for which there may only be one publisher. `TempoROS` includes a `UTempoROSClockServer` subsystem, which will automatically be created and will publish the simulation time to the `/clock` channel every frame. You don't have to do anything to enable this, but be sure not to publish anything on the `/clock` channel yourself.

### TF2
ROS includes [tf2](https://wiki.ros.org/tf2), a library that makes subscribing and publishing a graph of transforms more convenient. Some advantages over publishing raw transform messages include:
- It handles transform interpolation. You can ask tf2 for the transform from one frame to another at a specific time, and it will interpolate its internal time buffer of transforms to provide the most accurate response it can.
- It handles recursively propagating transforms through a graph. So, if one node publishes the transform from frame A to frame B, and another publishes the transform from frame B to frame C, anyone can ask tf2 for the transform from frame A to frame C.
- It differentiates between static and dynamic transforms, where static transforms are understood not to change, and therefore have their values "latched", meaning any node asking for a static transform (or chain of static transforms) can get an answer, even if that message containing that transform was published long ago.

To use tf2 in `TempoROS`, you should use `UTempoROSNode`'s `PublishDynamicTransform`, `PublishStaticTransform`, and `GetTransform` methods, which take native Unreal types and, of course, have full Blueprint support :).

<img width="1664" alt="Screenshot 2024-10-16 at 8 58 44 AM" src="https://github.com/user-attachments/assets/63816782-c70f-4c68-860e-12522d41e0ce">

### Image Transport Plugins
ROS has a built-in type for image messages (`sensor_msgs::msg::Image`), and there is nothing stopping you from publishing messages of this type directly to a topic.

However, raw image data can be heavy, so ROS also comes with an [image transport](http://wiki.ros.org/image_transport) library and several "image transport plugins" that can compress image data. When you use ROS's image transport to publish images, it will automatically translate your topic into several more topics, one for each transport plugin.

`TempoROS` handles this all behind the scenes. If you publish a `sensor_msgs::msg::Image` message, or any other type that is implicitly convertible to one, `UTempoROSNode` will automatically use ROS's image transport.

### Shared Memory Transport
`TempoROS` includes support for shared memory transport using ROS's [CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds) middleware and [iceoryx](https://github.com/eclipse-iceoryx/iceoryx), on Linux only. To enable this, you must:
- Choose `CycloneDDS` as the RMW Implementation in the TempoROS plugin settings
- Specify the path to a valid `CycloneDDS` xml config in the `CycloneDDS URI` setting
- Run the `roudi` server as a separate process on the same machine. `TempoROS` comes with a pre-build `roudi` (at `TempoROS/Source/ThirdParty/rclcpp/Binaries/Linux/iox-roudi`), but one from a pacakged ROS installation should also work. Note that you'll have to relax its compatibility check, with `iox-roudi -x minor`, as the one `TempoROS` linked against won't match a packaged ROS installation's exactly.

## Packaging
You can use `TempoROS` as part of a packaged game. You can find a convenient script to package the project with the recommended settings in [TempoSample](https://github.com/tempo-sim/TempoSample/blob/main/Scripts/Package.sh).

To package an Unreal project with `TempoROS`, you must specify its custom stage copy handler by adding `CustomStageCopyHandler=TempoROSCopyHandler` to your `Config/DefaultGame.ini`. This allows the package process to correctly copy symbolic links in the `rclcpp` libraries on all platforms.

## Known Issues
- To run an Unreal packaged game with TempoROS on Windows, you must add the directory `<package_root>/<YourProjectName>/Plugins/Tempo/TempoROS/Source/ThirdParty/rclcpp/Binaries/Windows` to your `PATH` environment variable.
- The `Setup.sh` and prebuild code generation steps are very slow on Windows the first time they run.
- Sometimes the `GenROSIDL` prebuild steps fails with `TypeError: '>' not supported between instances of 'str' and 'int'` from `em.py`. Still debugging this, but for whatever reason it seems more likely to happen when using ssh.
