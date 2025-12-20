#include "fifo.hpp"
#include <gtest/gtest.h>

class VarFIFOTest : public ::testing::Test {
   protected:
    dsa::VarFIFO<16, 4> fifo;

    void SetUp() override { fifo.clear(); }
};

TEST_F(VarFIFOTest, InitiallyEmpty) {
    EXPECT_EQ(fifo.get_capacity_used(), 0u);
    EXPECT_EQ(fifo.get_indices_used_count(), 0u);
}

TEST_F(VarFIFOTest, PushOneItem) {
    int item = 0x12345678;
    EXPECT_TRUE(
        fifo.push(reinterpret_cast<const uint8_t*>(&item), sizeof(item)));
    EXPECT_EQ(fifo.get_capacity_used(), sizeof(item));
    EXPECT_EQ(fifo.get_indices_used_count(), 1u);
}

TEST_F(VarFIFOTest, PushPopRoundTrip) {
    int a = 42;
    int b = 100;
    EXPECT_TRUE(fifo.push(reinterpret_cast<const uint8_t*>(&a), sizeof(a)));
    EXPECT_TRUE(fifo.push(reinterpret_cast<const uint8_t*>(&b), sizeof(b)));
    EXPECT_EQ(fifo.get_capacity_used(), sizeof(a) + sizeof(b));
    EXPECT_EQ(fifo.get_indices_used_count(), 2u);

    int out = 0;
    size_t out_size = 0;
    EXPECT_TRUE(fifo.pop(reinterpret_cast<uint8_t*>(&out), out_size));
    EXPECT_EQ(out, a);
    EXPECT_EQ(fifo.get_capacity_used(), sizeof(b));
    EXPECT_EQ(fifo.get_indices_used_count(), 1u);

    EXPECT_TRUE(fifo.pop(reinterpret_cast<uint8_t*>(&out), out_size));
    EXPECT_EQ(out, b);
    EXPECT_EQ(fifo.get_capacity_used(), 0u);
    EXPECT_EQ(fifo.get_indices_used_count(), 0u);
}

TEST_F(VarFIFOTest, FillIndicesAndCapacityLimits) {
    // Each int is 4 bytes; pool size is 16 -> 4 ints fill pool exactly.
    int v0 = 1, v1 = 2, v2 = 3, v3 = 4, v4 = 5;
    EXPECT_TRUE(fifo.push(reinterpret_cast<const uint8_t*>(&v0), sizeof(v0)));
    EXPECT_TRUE(fifo.push(reinterpret_cast<const uint8_t*>(&v1), sizeof(v1)));
    EXPECT_TRUE(fifo.push(reinterpret_cast<const uint8_t*>(&v2), sizeof(v2)));
    EXPECT_TRUE(fifo.push(reinterpret_cast<const uint8_t*>(&v3), sizeof(v3)));

    // Index buffer is full (MaxItemCount == 4), and capacity should be full (16).
    EXPECT_EQ(fifo.get_indices_used_count(), 4u);
    EXPECT_EQ(fifo.get_capacity_used(), 4 * sizeof(int));

    // Next push should fail due to index buffer full (and/or capacity).
    EXPECT_FALSE(fifo.push(reinterpret_cast<const uint8_t*>(&v4), sizeof(v4)));

    // Pop one item then push should succeed.
    int out = 0;
    size_t out_size = 0;
    EXPECT_TRUE(fifo.pop(reinterpret_cast<uint8_t*>(&out), out_size));
    EXPECT_EQ(out, v0);
    EXPECT_EQ(fifo.get_indices_used_count(), 3u);
    EXPECT_EQ(fifo.get_capacity_used(), 3 * sizeof(int));

    EXPECT_TRUE(fifo.push(reinterpret_cast<const uint8_t*>(&v4), sizeof(v4)));
    EXPECT_EQ(fifo.get_indices_used_count(), 4u);
    EXPECT_EQ(fifo.get_capacity_used(), 4 * sizeof(int));

    // Pop all to verify order.
    EXPECT_TRUE(fifo.pop(reinterpret_cast<uint8_t*>(&out), out_size));
    EXPECT_EQ(out, v1);
    EXPECT_TRUE(fifo.pop(reinterpret_cast<uint8_t*>(&out), out_size));
    EXPECT_EQ(out, v2);
    EXPECT_TRUE(fifo.pop(reinterpret_cast<uint8_t*>(&out), out_size));
    EXPECT_EQ(out, v3);
    EXPECT_TRUE(fifo.pop(reinterpret_cast<uint8_t*>(&out), out_size));
    EXPECT_EQ(out, v4);
    EXPECT_FALSE(fifo.pop(reinterpret_cast<uint8_t*>(&out), out_size));
    EXPECT_EQ(fifo.get_capacity_used(), 0u);
    EXPECT_EQ(fifo.get_indices_used_count(), 0u);
}

TEST_F(VarFIFOTest, MixedTypesDifferentSizes) {
    // Custom struct with fields of different sizes; sizeof(MyStruct) will account for padding/alignment.
    struct MyStruct {
        uint16_t a;
        uint32_t b;
    };

    uint16_t s1 = 0x1111;
    int i1 = 0x22222222;
    MyStruct s = {0x3333, 0x44444444};
    uint16_t s2 = 0x5555;
    MyStruct s3 = {0x6666, 0x77777777};

    const size_t expected_total =
        sizeof(s1) + sizeof(i1) + sizeof(s) + sizeof(s2);

    EXPECT_TRUE(fifo.push(reinterpret_cast<const uint8_t*>(&s1), sizeof(s1)));
    EXPECT_TRUE(fifo.push(reinterpret_cast<const uint8_t*>(&i1), sizeof(i1)));
    EXPECT_TRUE(fifo.push(reinterpret_cast<const uint8_t*>(&s), sizeof(s)));
    EXPECT_TRUE(fifo.push(reinterpret_cast<const uint8_t*>(&s2), sizeof(s2)));
    EXPECT_FALSE(fifo.push(reinterpret_cast<const uint8_t*>(&s3),
                           sizeof(s3)));  // Should fail, no space.

    EXPECT_EQ(fifo.get_capacity_used(), expected_total);
    EXPECT_EQ(fifo.get_indices_used_count(), 4u);

    uint16_t out_s1 = 0;
    int out_i1 = 0;
    MyStruct out_s = {0, 0};
    uint16_t out_s2 = 0;
    MyStruct out_s3 = {0, 0};
    size_t out_size = 0;

    EXPECT_TRUE(fifo.pop(reinterpret_cast<uint8_t*>(&out_s1), out_size));
    EXPECT_EQ(out_s1, s1);

    EXPECT_TRUE(fifo.pop(reinterpret_cast<uint8_t*>(&out_i1), out_size));
    EXPECT_EQ(out_i1, i1);

    EXPECT_FALSE(fifo.push(reinterpret_cast<const uint8_t*>(&s3),
                           sizeof(s3)));  // Still no space.

    EXPECT_TRUE(fifo.pop(reinterpret_cast<uint8_t*>(&out_s), out_size));
    EXPECT_EQ(out_s.a, s.a);
    EXPECT_EQ(out_s.b, s.b);

    EXPECT_TRUE(fifo.push(reinterpret_cast<const uint8_t*>(&s3),
                          sizeof(s3)));  // Now there should be space.

    EXPECT_TRUE(fifo.pop(reinterpret_cast<uint8_t*>(&out_s2), out_size));
    EXPECT_EQ(out_s2, s2);

    EXPECT_TRUE(fifo.pop(reinterpret_cast<uint8_t*>(&out_s3), out_size));
    EXPECT_EQ(out_s3.a, s3.a);
    EXPECT_EQ(out_s3.b, s3.b);

    // Ensure FIFO is empty at the end.
    EXPECT_EQ(fifo.get_capacity_used(), 0u);
    EXPECT_EQ(fifo.get_indices_used_count(), 0u);
}