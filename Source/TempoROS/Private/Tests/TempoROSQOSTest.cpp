// Copyright Tempo Simulation, LLC. All Rights Reserved

#include "TempoROSTypes.h"

#include "Misc/AutomationTest.h"

// These are pure unit tests for FROSQOSProfile's mapping onto an rclcpp::QoS (and through it onto the
// rmw QOS profile the middleware actually sees). No node, no context, no world. They run headlessly via:
//   Scripts/Test.sh            (runs all "Tempo." automation tests)
//   Automation RunTests Tempo.ROS.QOS   (from the editor console)

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	constexpr EAutomationTestFlags TempoROSQOSTestFlags =
		EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTempoROSQOSDefaultsTest,
	"Tempo.ROS.QOS.Defaults", TempoROSQOSTestFlags)
bool FTempoROSQOSDefaultsTest::RunTest(const FString& Parameters)
{
	// A default profile must mean what a default profile means in ROS 2 (rmw_qos_profile_default):
	// keep the last ten samples, reliable, volatile, with the middleware's own durations.
	const rmw_qos_profile_t Profile = FROSQOSProfile().ToROS().get_rmw_qos_profile();

	TestEqual(TEXT("History"), static_cast<int32>(Profile.history), static_cast<int32>(RMW_QOS_POLICY_HISTORY_KEEP_LAST));
	TestEqual(TEXT("Depth"), static_cast<int32>(Profile.depth), 10);
	TestEqual(TEXT("Reliability"), static_cast<int32>(Profile.reliability), static_cast<int32>(RMW_QOS_POLICY_RELIABILITY_RELIABLE));
	TestEqual(TEXT("Durability"), static_cast<int32>(Profile.durability), static_cast<int32>(RMW_QOS_POLICY_DURABILITY_VOLATILE));
	TestEqual(TEXT("Liveliness"), static_cast<int32>(Profile.liveliness), static_cast<int32>(RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT));

	// RMW_DURATION_UNSPECIFIED (zero), i.e. "whatever the middleware defaults to", rather than a
	// zero-length deadline/lifespan/lease.
	TestTrue(TEXT("Deadline unspecified"), rmw_time_equal(Profile.deadline, RMW_DURATION_UNSPECIFIED));
	TestTrue(TEXT("Lifespan unspecified"), rmw_time_equal(Profile.lifespan, RMW_DURATION_UNSPECIFIED));
	TestTrue(TEXT("Lease duration unspecified"), rmw_time_equal(Profile.liveliness_lease_duration, RMW_DURATION_UNSPECIFIED));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTempoROSQOSTransientLocalTest,
	"Tempo.ROS.QOS.TransientLocal", TempoROSQOSTestFlags)
bool FTempoROSQOSTransientLocalTest::RunTest(const FString& Parameters)
{
	// TransientLocal() alone has to be enough to receive a latched sample. A transient local
	// subscription only gets the samples published before it joined if it is also reliable, since that
	// replay is delivered over the reliable path, so the default must not leave reliability unset.
	const rmw_qos_profile_t Latched = FROSQOSProfile().TransientLocal().ToROS().get_rmw_qos_profile();

	TestEqual(TEXT("Durability"), static_cast<int32>(Latched.durability), static_cast<int32>(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL));
	TestEqual(TEXT("Reliability"), static_cast<int32>(Latched.reliability), static_cast<int32>(RMW_QOS_POLICY_RELIABILITY_RELIABLE));

	// The explicit options still win.
	const rmw_qos_profile_t BestEffort = FROSQOSProfile().BestEffort().ToROS().get_rmw_qos_profile();
	TestEqual(TEXT("BestEffort reliability"), static_cast<int32>(BestEffort.reliability), static_cast<int32>(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT));

	const rmw_qos_profile_t SystemDefault = FROSQOSProfile().ToROS().get_rmw_qos_profile();
	TestEqual(TEXT("Volatile durability"), static_cast<int32>(SystemDefault.durability), static_cast<int32>(RMW_QOS_POLICY_DURABILITY_VOLATILE));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTempoROSQOSQueueSizeTest,
	"Tempo.ROS.QOS.QueueSize", TempoROSQOSTestFlags)
bool FTempoROSQOSQueueSizeTest::RunTest(const FString& Parameters)
{
	const rmw_qos_profile_t Custom = FROSQOSProfile(3).ToROS().get_rmw_qos_profile();
	TestEqual(TEXT("Constructed depth"), static_cast<int32>(Custom.depth), 3);
	TestEqual(TEXT("Constructed history"), static_cast<int32>(Custom.history), static_cast<int32>(RMW_QOS_POLICY_HISTORY_KEEP_LAST));

	const rmw_qos_profile_t Set = FROSQOSProfile().CustomQueueSize(5).ToROS().get_rmw_qos_profile();
	TestEqual(TEXT("CustomQueueSize depth"), static_cast<int32>(Set.depth), 5);

	const rmw_qos_profile_t All = FROSQOSProfile().KeepAll().ToROS().get_rmw_qos_profile();
	TestEqual(TEXT("KeepAll history"), static_cast<int32>(All.history), static_cast<int32>(RMW_QOS_POLICY_HISTORY_KEEP_ALL));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTempoROSQOSDurationsTest,
	"Tempo.ROS.QOS.Durations", TempoROSQOSTestFlags)
bool FTempoROSQOSDurationsTest::RunTest(const FString& Parameters)
{
	// Each duration option must set its own policy and leave the others at the middleware default.
	const rmw_qos_profile_t Deadline = FROSQOSProfile().CustomDeadline(1.5).ToROS().get_rmw_qos_profile();
	TestEqual(TEXT("Deadline sec"), static_cast<int32>(Deadline.deadline.sec), 1);
	TestEqual(TEXT("Deadline nsec"), static_cast<int32>(Deadline.deadline.nsec), 500000000);
	TestTrue(TEXT("Deadline leaves lifespan alone"), rmw_time_equal(Deadline.lifespan, RMW_DURATION_UNSPECIFIED));

	const rmw_qos_profile_t Lifespan = FROSQOSProfile().CustomLifespan(2.0).ToROS().get_rmw_qos_profile();
	TestEqual(TEXT("Lifespan sec"), static_cast<int32>(Lifespan.lifespan.sec), 2);
	TestTrue(TEXT("Lifespan leaves deadline alone"), rmw_time_equal(Lifespan.deadline, RMW_DURATION_UNSPECIFIED));

	const rmw_qos_profile_t Lease = FROSQOSProfile().ManualByTopicLiveliness().CustomLeaseDuration(3.0).ToROS().get_rmw_qos_profile();
	TestEqual(TEXT("Lease duration sec"), static_cast<int32>(Lease.liveliness_lease_duration.sec), 3);
	TestEqual(TEXT("Liveliness"), static_cast<int32>(Lease.liveliness), static_cast<int32>(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC));
	// The lease duration used to be written to the lifespan instead.
	TestTrue(TEXT("Lease duration leaves lifespan alone"), rmw_time_equal(Lease.lifespan, RMW_DURATION_UNSPECIFIED));

	return true;
}

#endif
