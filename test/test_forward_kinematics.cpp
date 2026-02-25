#include <gtest/gtest.h>
#include "ForwardKinematics.hpp"
#include "PoseSnapshot.hpp"
#include <cmath>

using namespace sf;

static constexpr float TOL = 0.01f;

TEST(ForwardKinTest, ConfigureEmptySkeleton) {
    ForwardKinematics fk;
    EXPECT_TRUE(fk.configure(nullptr, 0));
    EXPECT_EQ(fk.boneCount(), 0u);
}

TEST(ForwardKinTest, ConfigureSingleRoot) {
    Bone bones[] = {{ForwardKinematics::ROOT, 0x01, 0.3f}};
    ForwardKinematics fk;
    EXPECT_TRUE(fk.configure(bones, 1));
    EXPECT_EQ(fk.boneCount(), 1u);
}

TEST(ForwardKinTest, ConfigureNullPointerWithCountFails) {
    ForwardKinematics fk;
    EXPECT_FALSE(fk.configure(nullptr, 1));
}

TEST(ForwardKinTest, ConfigureTooManyBonesFails) {
    Bone bones[17];
    for (auto& b : bones) b = {ForwardKinematics::ROOT, 0, 0.1f};
    ForwardKinematics fk;
    EXPECT_FALSE(fk.configure(bones, 17));
}

TEST(ForwardKinTest, ConfigureInvalidParentFails) {
    // Child references a parent that comes after it (not topological)
    Bone bones[] = {
        {1, 0x01, 0.3f},  // parent=1 but it's index 0
        {ForwardKinematics::ROOT, 0x02, 0.3f},
    };
    ForwardKinematics fk;
    EXPECT_FALSE(fk.configure(bones, 2));
}

TEST(ForwardKinTest, RootAtOrigin) {
    Bone bones[] = {{ForwardKinematics::ROOT, 0x01, 0.5f}};
    ForwardKinematics fk;
    fk.configure(bones, 1);
    fk.setNodeOrientation(0x01, Quaternion{});
    fk.solve();

    auto j = fk.joint(0);
    EXPECT_NEAR(j.position.x, 0.0f, TOL);
    EXPECT_NEAR(j.position.y, 0.0f, TOL);
    EXPECT_NEAR(j.position.z, 0.0f, TOL);
}

TEST(ForwardKinTest, SingleBoneChainIdentity) {
    // Root + one child, identity orientation -> child at (0, length, 0)
    Bone bones[] = {
        {ForwardKinematics::ROOT, 0x01, 0.3f},
        {0, 0x02, 0.25f},
    };
    ForwardKinematics fk;
    fk.configure(bones, 2);
    fk.setNodeOrientation(0x01, Quaternion{});
    fk.setNodeOrientation(0x02, Quaternion{});
    fk.solve();

    auto j1 = fk.joint(1);
    EXPECT_NEAR(j1.position.x, 0.0f, TOL);
    EXPECT_NEAR(j1.position.y, 0.3f, TOL);  // parent bone length along +Y
    EXPECT_NEAR(j1.position.z, 0.0f, TOL);
}

TEST(ForwardKinTest, SingleBoneChain90Rotation) {
    // Parent rotated 90 deg about Z -> bone goes along -X instead of +Y
    Bone bones[] = {
        {ForwardKinematics::ROOT, 0x01, 0.3f},
        {0, 0x02, 0.25f},
    };
    ForwardKinematics fk;
    fk.configure(bones, 2);
    fk.setNodeOrientation(0x01, Quaternion::fromAxisAngle(0, 0, 1, 90.0f));
    fk.setNodeOrientation(0x02, Quaternion{});
    fk.solve();

    auto j1 = fk.joint(1);
    // 90 deg about Z rotates +Y to -X
    EXPECT_NEAR(j1.position.x, -0.3f, TOL);
    EXPECT_NEAR(j1.position.y, 0.0f, TOL);
    EXPECT_NEAR(j1.position.z, 0.0f, TOL);
}

TEST(ForwardKinTest, TwoBoneChainStraight) {
    // Three joints in a line along +Y
    // Joint positions: root(0,0,0) -> joint1(0,0.3,0) -> joint2(0,0.55,0)
    Bone bones[] = {
        {ForwardKinematics::ROOT, 0x01, 0.3f},  // root bone length 0.3
        {0, 0x02, 0.25f},                        // bone 1 length 0.25
        {1, 0x03, 0.2f},                         // bone 2 length 0.2
    };
    ForwardKinematics fk;
    fk.configure(bones, 3);
    fk.setNodeOrientation(0x01, Quaternion{});
    fk.setNodeOrientation(0x02, Quaternion{});
    fk.setNodeOrientation(0x03, Quaternion{});
    fk.solve();

    // Joint 1 at parent(root) bone length along +Y
    auto j1 = fk.joint(1);
    EXPECT_NEAR(j1.position.y, 0.3f, TOL);

    // Joint 2 at parent(bone1) length further along +Y
    auto j2 = fk.joint(2);
    EXPECT_NEAR(j2.position.x, 0.0f, TOL);
    EXPECT_NEAR(j2.position.y, 0.55f, TOL);  // 0.3 + 0.25
    EXPECT_NEAR(j2.position.z, 0.0f, TOL);
}

TEST(ForwardKinTest, TwoBoneChain90Elbow) {
    // Upper arm identity, forearm's parent is rotated 90 about Z at the elbow
    Bone bones[] = {
        {ForwardKinematics::ROOT, 0x01, 0.3f},   // upper arm
        {0, 0x02, 0.25f},                          // forearm
    };
    ForwardKinematics fk;
    fk.configure(bones, 2);
    fk.setNodeOrientation(0x01, Quaternion{});  // pointing +Y
    fk.setNodeOrientation(0x02, Quaternion::fromAxisAngle(0, 0, 1, 90.0f));
    fk.solve();

    // Elbow at (0, 0.3, 0) -- uses parent orientation (identity) to place child
    auto j1 = fk.joint(1);
    EXPECT_NEAR(j1.position.x, 0.0f, TOL);
    EXPECT_NEAR(j1.position.y, 0.3f, TOL);
}

TEST(ForwardKinTest, SetNodeOrientationUnknownNodeFails) {
    Bone bones[] = {{ForwardKinematics::ROOT, 0x01, 0.3f}};
    ForwardKinematics fk;
    fk.configure(bones, 1);
    EXPECT_FALSE(fk.setNodeOrientation(0xFF, Quaternion{}));
}

TEST(ForwardKinTest, ResetClearsState) {
    Bone bones[] = {{ForwardKinematics::ROOT, 0x01, 0.3f}};
    ForwardKinematics fk;
    fk.configure(bones, 1);
    EXPECT_EQ(fk.boneCount(), 1u);
    fk.reset();
    EXPECT_EQ(fk.boneCount(), 0u);
}

// --- PoseSnapshot tests ---

TEST(PoseSnapshotTest, DefaultIsZeroed) {
    PoseSnapshot pose;
    EXPECT_EQ(pose.activeJoints, 0);
    EXPECT_EQ(pose.timestampUs, 0u);
    EXPECT_NEAR(pose.positions[0].x, 0.0f, TOL);
    EXPECT_NEAR(pose.orientations[0].w, 1.0f, TOL);
}

TEST(PoseSnapshotTest, StoresJointData) {
    PoseSnapshot pose;
    pose.activeJoints = 2;
    pose.timestampUs = 123456;
    pose.positions[0] = {1.0f, 2.0f, 3.0f};
    pose.orientations[0] = Quaternion::fromAxisAngle(0, 0, 1, 45.0f);
    EXPECT_EQ(pose.activeJoints, 2);
    EXPECT_NEAR(pose.positions[0].x, 1.0f, TOL);
}

TEST(ForwardKinSizeTest, LessThan848Bytes) {
    EXPECT_LE(sizeof(ForwardKinematics), 848u);
}

TEST(PoseSnapshotSizeTest, LessThan472Bytes) {
    EXPECT_LE(sizeof(PoseSnapshot), 472u);
}
