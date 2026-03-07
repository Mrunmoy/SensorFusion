#include <gtest/gtest.h>

#include "MocapCalibrationFlow.hpp"

TEST(MocapCalibrationFlowTest, IgnoresTPoseCommandBeforeStationaryCapture) {
    sf::MocapCalibrationFlow flow;
    flow.issue(sf::MocapCalibrationFlow::Command::CaptureTPose);

    EXPECT_EQ(flow.state(), sf::MocapCalibrationFlow::State::Uncalibrated);
    EXPECT_FALSE(flow.hasPendingCommand());
}

TEST(MocapCalibrationFlowTest, CapturesStationaryOnNextSample) {
    sf::MocapCalibrationFlow flow;
    flow.issue(sf::MocapCalibrationFlow::Command::CaptureStationary);

    const sf::Quaternion qRef = sf::Quaternion::fromAxisAngle(0.0f, 0.0f, 1.0f, 30.0f);
    EXPECT_TRUE(flow.processSample(qRef));
    EXPECT_EQ(flow.state(), sf::MocapCalibrationFlow::State::StationaryCalibrated);
    EXPECT_FALSE(flow.hasPendingCommand());
}

TEST(MocapCalibrationFlowTest, AppliesStationaryReferenceAfterCapture) {
    sf::MocapCalibrationFlow flow;
    const sf::Quaternion qRef = sf::Quaternion::fromAxisAngle(1.0f, 0.0f, 0.0f, 25.0f);
    flow.issue(sf::MocapCalibrationFlow::Command::CaptureStationary);
    ASSERT_TRUE(flow.processSample(qRef));

    sf::Quaternion rel = flow.apply(qRef);
    rel.normalize();
    EXPECT_NEAR(rel.w, 1.0f, 1e-4f);
    EXPECT_NEAR(rel.x, 0.0f, 1e-4f);
    EXPECT_NEAR(rel.y, 0.0f, 1e-4f);
    EXPECT_NEAR(rel.z, 0.0f, 1e-4f);
}

TEST(MocapCalibrationFlowTest, CapturesTPoseAfterStationaryAndUsesTPoseReference) {
    sf::MocapCalibrationFlow flow;
    const sf::Quaternion qStationary = sf::Quaternion::fromAxisAngle(0.0f, 1.0f, 0.0f, 15.0f);
    flow.issue(sf::MocapCalibrationFlow::Command::CaptureStationary);
    ASSERT_TRUE(flow.processSample(qStationary));

    const sf::Quaternion qTPose = sf::Quaternion::fromAxisAngle(0.0f, 0.0f, 1.0f, 45.0f);
    flow.issue(sf::MocapCalibrationFlow::Command::CaptureTPose);
    ASSERT_TRUE(flow.processSample(qTPose));
    EXPECT_EQ(flow.state(), sf::MocapCalibrationFlow::State::FullyCalibrated);

    sf::Quaternion rel = flow.apply(qTPose);
    rel.normalize();
    EXPECT_NEAR(rel.w, 1.0f, 1e-4f);
    EXPECT_NEAR(rel.x, 0.0f, 1e-4f);
    EXPECT_NEAR(rel.y, 0.0f, 1e-4f);
    EXPECT_NEAR(rel.z, 0.0f, 1e-4f);
}

TEST(MocapCalibrationFlowTest, ResetReturnsToUncalibratedState) {
    sf::MocapCalibrationFlow flow;
    flow.issue(sf::MocapCalibrationFlow::Command::CaptureStationary);
    ASSERT_TRUE(flow.processSample(sf::Quaternion::fromAxisAngle(0.0f, 0.0f, 1.0f, 10.0f)));
    EXPECT_EQ(flow.state(), sf::MocapCalibrationFlow::State::StationaryCalibrated);

    flow.issue(sf::MocapCalibrationFlow::Command::Reset);
    EXPECT_EQ(flow.state(), sf::MocapCalibrationFlow::State::Uncalibrated);
    EXPECT_FALSE(flow.hasPendingCommand());
}
