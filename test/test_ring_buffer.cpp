#include <gtest/gtest.h>
#include "RingBuffer.hpp"

using namespace sf;

TEST(RingBufferTest, EmptyOnConstruction) {
    RingBuffer<int, 4> rb;
    EXPECT_TRUE(rb.isEmpty());
    EXPECT_FALSE(rb.isFull());
    EXPECT_EQ(rb.count(), 0u);
}

TEST(RingBufferTest, CapacityIsN) {
    RingBuffer<int, 8> rb;
    EXPECT_EQ(rb.capacity(), 8u);
}

TEST(RingBufferTest, PushOneItem) {
    RingBuffer<int, 4> rb;
    EXPECT_TRUE(rb.push(42));
    EXPECT_EQ(rb.count(), 1u);
    EXPECT_FALSE(rb.isEmpty());
}

TEST(RingBufferTest, PopOneItem) {
    RingBuffer<int, 4> rb;
    rb.push(42);
    int val;
    EXPECT_TRUE(rb.pop(val));
    EXPECT_EQ(val, 42);
    EXPECT_TRUE(rb.isEmpty());
}

TEST(RingBufferTest, PopFromEmptyReturnsFalse) {
    RingBuffer<int, 4> rb;
    int val;
    EXPECT_FALSE(rb.pop(val));
}

TEST(RingBufferTest, FillToCapacity) {
    RingBuffer<int, 4> rb;
    for (int i = 0; i < 4; ++i) {
        EXPECT_TRUE(rb.push(i));
    }
    EXPECT_TRUE(rb.isFull());
    EXPECT_EQ(rb.count(), 4u);
}

TEST(RingBufferTest, PushWhenFullReturnsFalse) {
    RingBuffer<int, 4> rb;
    for (int i = 0; i < 4; ++i) rb.push(i);
    EXPECT_FALSE(rb.push(99));
}

TEST(RingBufferTest, FIFOOrder) {
    RingBuffer<int, 4> rb;
    rb.push(10);
    rb.push(20);
    rb.push(30);

    int val;
    rb.pop(val); EXPECT_EQ(val, 10);
    rb.pop(val); EXPECT_EQ(val, 20);
    rb.pop(val); EXPECT_EQ(val, 30);
}

TEST(RingBufferTest, WrapAround) {
    RingBuffer<int, 3> rb;
    rb.push(1);
    rb.push(2);
    rb.push(3);  // full
    int val;
    rb.pop(val); EXPECT_EQ(val, 1);  // free one slot
    EXPECT_TRUE(rb.push(4));  // wraps
    rb.pop(val); EXPECT_EQ(val, 2);
    rb.pop(val); EXPECT_EQ(val, 3);
    rb.pop(val); EXPECT_EQ(val, 4);
    EXPECT_TRUE(rb.isEmpty());
}

TEST(RingBufferTest, CountAfterMixedOps) {
    RingBuffer<int, 4> rb;
    rb.push(1); rb.push(2); rb.push(3);
    EXPECT_EQ(rb.count(), 3u);
    int val;
    rb.pop(val);
    EXPECT_EQ(rb.count(), 2u);
    rb.push(4); rb.push(5);
    EXPECT_EQ(rb.count(), 4u);
    EXPECT_TRUE(rb.isFull());
}

TEST(RingBufferTest, CapacityOne) {
    RingBuffer<int, 1> rb;
    EXPECT_TRUE(rb.push(42));
    EXPECT_TRUE(rb.isFull());
    EXPECT_FALSE(rb.push(43));
    int val;
    EXPECT_TRUE(rb.pop(val));
    EXPECT_EQ(val, 42);
}

TEST(RingBufferTest, WorksWithStruct) {
    struct Data { float x; int y; };
    RingBuffer<Data, 4> rb;
    rb.push({1.5f, 10});
    rb.push({2.5f, 20});
    Data d;
    rb.pop(d);
    EXPECT_FLOAT_EQ(d.x, 1.5f);
    EXPECT_EQ(d.y, 10);
}

TEST(RingBufferTest, MultipleWrapArounds) {
    RingBuffer<int, 3> rb;
    // Fill and drain multiple times to stress wrap-around
    for (int round = 0; round < 5; ++round) {
        for (int i = 0; i < 3; ++i) {
            EXPECT_TRUE(rb.push(round * 10 + i));
        }
        EXPECT_TRUE(rb.isFull());
        for (int i = 0; i < 3; ++i) {
            int val;
            EXPECT_TRUE(rb.pop(val));
            EXPECT_EQ(val, round * 10 + i);
        }
        EXPECT_TRUE(rb.isEmpty());
    }
}

TEST(RingBufferTest, IsNotFullAfterOnePop) {
    RingBuffer<int, 2> rb;
    rb.push(1);
    rb.push(2);
    EXPECT_TRUE(rb.isFull());
    int val;
    rb.pop(val);
    EXPECT_FALSE(rb.isFull());
}
