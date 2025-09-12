#include <gtest/gtest.h>
#include "middleware_classes.hpp"
#include "middleware_interfaces.hpp"
#include "middleware_types.hpp"

namespace {

    using namespace MW_RTOS;

    class MockRTOSQueueTest : public ::testing::Test {
       protected:
        TestRTOS rtos;
        QueueHandle queue = nullptr;
        size_t queue_length = 5;
        size_t item_size = 8;
    };

    TEST_F(MockRTOSQueueTest, QueueCreateReturnsTrueAndSetsFields) {
        bool result = rtos.queue_create(queue, queue_length, item_size);

        EXPECT_TRUE(result);
        ASSERT_NE(queue, nullptr);
        EXPECT_EQ(queue->queue_length, queue_length);
        EXPECT_EQ(queue->item_size, item_size);
        EXPECT_EQ(queue->item_count, 0);
        EXPECT_EQ(queue->front_index, 0);
        EXPECT_EQ(queue->back_index, 0);
        ASSERT_NE(queue->byte_queue, nullptr);

        // Clean up
        free(queue->byte_queue);
        free(queue);
    }

    TEST_F(MockRTOSQueueTest, QueueCreateReturnsFalseIfMallocFails) {
        // Simulate malloc failure by passing too large sizes.
        QueueHandle bad_queue = nullptr;
        bool result = rtos.queue_create(bad_queue, 1000000, 1000000);

        EXPECT_FALSE(result);
    }

    TEST_F(MockRTOSQueueTest, QueuePushbackFloatSuccess) {
        MockRTOSQueue q;
        q.queue_length = 3;
        q.item_size = sizeof(float);
        q.item_count = 0;
        q.front_index = 0;
        q.back_index = 0;
        q.byte_queue =
            static_cast<uint8_t*>(malloc(q.queue_length * q.item_size));
        ASSERT_NE(q.byte_queue, nullptr);

        float value = 3.14f;
        bool result = rtos.queue_pushback(&q, &value);
        EXPECT_TRUE(result);
        EXPECT_EQ(q.item_count, 1);
        EXPECT_EQ(q.front_index, 0);
        EXPECT_EQ(q.back_index, 1);  // back_index should be incremented

        float stored = 0.0f;
        memcpy(&stored, q.byte_queue, sizeof(float));
        EXPECT_FLOAT_EQ(stored, value);

        free(q.byte_queue);
    }

    TEST_F(MockRTOSQueueTest, QueuePushbackFloatFullFails) {
        MockRTOSQueue q;
        q.queue_length = 2;
        q.item_size = sizeof(float);
        q.item_count = 2;  // Already full
        q.front_index = 0;
        q.back_index = 0;
        q.byte_queue =
            static_cast<uint8_t*>(malloc(q.queue_length * q.item_size));
        ASSERT_NE(q.byte_queue, nullptr);

        float value = 1.23f;
        // Save original state
        size_t orig_item_count = q.item_count;
        size_t orig_front_index = q.front_index;
        size_t orig_back_index = q.back_index;
        uint8_t* orig_byte_queue = q.byte_queue;

        bool result = rtos.queue_pushback(&q, &value);
        EXPECT_FALSE(result);
        // Assert no modification
        EXPECT_EQ(q.item_count, orig_item_count);
        EXPECT_EQ(q.front_index, orig_front_index);
        EXPECT_EQ(q.back_index, orig_back_index);
        EXPECT_EQ(q.byte_queue, orig_byte_queue);

        free(q.byte_queue);
    }

    struct CompositeType {
        uint8_t a;
        int16_t b;
        float c;
    };

    TEST_F(MockRTOSQueueTest, QueuePushbackCompositeTypeSuccess) {
        MockRTOSQueue q;
        q.queue_length = 2;
        q.item_size = sizeof(CompositeType);
        q.item_count = 1;
        q.front_index = 0;
        q.back_index = 1;
        q.byte_queue =
            static_cast<uint8_t*>(malloc(q.queue_length * q.item_size));
        ASSERT_NE(q.byte_queue, nullptr);

        // Existing value
        CompositeType existing = {10, 100, 1.23f};
        memcpy(q.byte_queue, &existing, sizeof(CompositeType));

        CompositeType value = {42, -1234, 2.718f};
        bool result = rtos.queue_pushback(&q, &value);
        EXPECT_TRUE(result);
        EXPECT_EQ(q.item_count, 2);
        EXPECT_EQ(q.front_index, 0);
        EXPECT_EQ(q.back_index, 0);

        // Check integrity of existing value
        CompositeType stored_existing {};
        memcpy(&stored_existing, q.byte_queue, sizeof(CompositeType));
        EXPECT_EQ(stored_existing.a, existing.a);
        EXPECT_EQ(stored_existing.b, existing.b);
        EXPECT_FLOAT_EQ(stored_existing.c, existing.c);

        // Check new value
        CompositeType stored_new {};
        memcpy(&stored_new, q.byte_queue + 1 * q.item_size,
               sizeof(CompositeType));
        EXPECT_EQ(stored_new.a, value.a);
        EXPECT_EQ(stored_new.b, value.b);
        EXPECT_FLOAT_EQ(stored_new.c, value.c);

        free(q.byte_queue);
    }

    TEST_F(MockRTOSQueueTest, QueuePushbackCompositeTypeFullFails) {
        MockRTOSQueue q;
        q.queue_length = 1;
        q.item_size = sizeof(CompositeType);
        q.item_count = 1;  // Already full
        q.front_index = 0;
        q.back_index = 0;
        q.byte_queue =
            static_cast<uint8_t*>(malloc(q.queue_length * q.item_size));
        ASSERT_NE(q.byte_queue, nullptr);

        CompositeType value = {1, 2, 3.0f};
        // Save original state
        size_t orig_item_count = q.item_count;
        size_t orig_front_index = q.front_index;
        size_t orig_back_index = q.back_index;
        uint8_t* orig_byte_queue = q.byte_queue;

        bool result = rtos.queue_pushback(&q, &value);
        EXPECT_FALSE(result);
        // Assert no modification
        EXPECT_EQ(q.item_count, orig_item_count);
        EXPECT_EQ(q.front_index, orig_front_index);
        EXPECT_EQ(q.back_index, orig_back_index);
        EXPECT_EQ(q.byte_queue, orig_byte_queue);

        free(q.byte_queue);
    }

    TEST_F(MockRTOSQueueTest, QueuePushbackWrapsBackIndex) {
        MockRTOSQueue q;
        q.queue_length = 3;
        q.item_size = sizeof(float);
        q.item_count = 2;
        q.front_index = 1;
        q.back_index = 2;
        q.byte_queue =
            static_cast<uint8_t*>(malloc(q.queue_length * q.item_size));
        ASSERT_NE(q.byte_queue, nullptr);

        // Fill first two slots with known values
        float first = 1.0f, second = 2.0f;
        memcpy(q.byte_queue + 0 * q.item_size, &first, sizeof(float));
        memcpy(q.byte_queue + 1 * q.item_size, &second, sizeof(float));

        float value = 3.14f;
        bool result = rtos.queue_pushback(&q, &value);
        EXPECT_TRUE(result);
        EXPECT_EQ(q.item_count, 3);
        EXPECT_EQ(q.front_index, 1);
        EXPECT_EQ(q.back_index, 0);  // back_index should wrap to 0

        // The value should be at slot 2 (where back_index was before push)
        float stored = 0.0f;
        memcpy(&stored, q.byte_queue + 2 * q.item_size, sizeof(float));
        EXPECT_FLOAT_EQ(stored, value);

        free(q.byte_queue);
    }

    TEST_F(MockRTOSQueueTest, QueueOverwriteEmptyQueueSuccess) {
        MockRTOSQueue q;
        q.queue_length = 3;
        q.item_size = sizeof(float);
        q.item_count = 0;
        q.front_index = 0;
        q.back_index = 0;
        q.byte_queue =
            static_cast<uint8_t*>(malloc(q.queue_length * q.item_size));
        ASSERT_NE(q.byte_queue, nullptr);

        float value = 5.55f;
        bool result = rtos.queue_overwrite(&q, &value);
        EXPECT_TRUE(result);
        EXPECT_EQ(q.item_count, 1);
        EXPECT_EQ(q.front_index, 0);
        EXPECT_EQ(q.back_index, 1);

        float stored = 0.0f;
        memcpy(&stored, q.byte_queue, sizeof(float));
        EXPECT_FLOAT_EQ(stored, value);

        free(q.byte_queue);
    }

    TEST_F(MockRTOSQueueTest, QueueOverwriteNonFullQueueSuccess) {
        MockRTOSQueue q;
        q.queue_length = 3;
        q.item_size = sizeof(float);
        q.item_count = 2;
        q.front_index = 0;
        q.back_index = 2;
        q.byte_queue =
            static_cast<uint8_t*>(malloc(q.queue_length * q.item_size));
        ASSERT_NE(q.byte_queue, nullptr);

        float old_value1 = 1.23f, old_value2 = 2.34f;
        memcpy(q.byte_queue + 0 * q.item_size, &old_value1, sizeof(float));
        memcpy(q.byte_queue + 1 * q.item_size, &old_value2, sizeof(float));

        float new_value = 9.87f;
        bool result = rtos.queue_overwrite(&q, &new_value);
        EXPECT_TRUE(result);
        EXPECT_EQ(q.item_count, 3);
        EXPECT_EQ(q.front_index, 0);
        EXPECT_EQ(q.back_index, 0);

        // Check integrity of existing values
        float stored1 = 0.0f, stored2 = 0.0f;
        memcpy(&stored1, q.byte_queue + 0 * q.item_size, sizeof(float));
        memcpy(&stored2, q.byte_queue + 1 * q.item_size, sizeof(float));
        EXPECT_FLOAT_EQ(stored1, old_value1);
        EXPECT_FLOAT_EQ(stored2, old_value2);

        // Check new value
        float stored_new = 0.0f;
        memcpy(&stored_new, q.byte_queue + 2 * q.item_size, sizeof(float));
        EXPECT_FLOAT_EQ(stored_new, new_value);

        free(q.byte_queue);
    }

    TEST_F(MockRTOSQueueTest, QueueOverwriteFullQueueSuccess) {
        MockRTOSQueue q;
        q.queue_length = 2;
        q.item_size = sizeof(float);
        q.item_count = 2;
        q.front_index = 0;
        q.back_index = 0;
        q.byte_queue =
            static_cast<uint8_t*>(malloc(q.queue_length * q.item_size));
        ASSERT_NE(q.byte_queue, nullptr);

        float old_value1 = 1.23f, old_value2 = 2.34f;
        memcpy(q.byte_queue + 0 * q.item_size, &old_value1, sizeof(float));
        memcpy(q.byte_queue + 1 * q.item_size, &old_value2, sizeof(float));

        float new_value = 7.77f;
        bool result = rtos.queue_overwrite(&q, &new_value);
        EXPECT_TRUE(result);
        EXPECT_EQ(q.item_count, 2);
        EXPECT_EQ(q.front_index, 1);
        EXPECT_EQ(q.back_index, 1);

        float stored = 0.0f;
        memcpy(&stored, q.byte_queue + 0 * q.item_size, sizeof(float));
        EXPECT_FLOAT_EQ(stored, new_value);

        free(q.byte_queue);
    }

    TEST_F(MockRTOSQueueTest, QueueOverwriteFromIsrEmptyQueueSuccess) {
        MockRTOSQueue q;
        q.queue_length = 3;
        q.item_size = sizeof(float);
        q.item_count = 0;
        q.front_index = 0;
        q.back_index = 0;
        q.byte_queue =
            static_cast<uint8_t*>(malloc(q.queue_length * q.item_size));
        ASSERT_NE(q.byte_queue, nullptr);

        float value = 9.99f;
        bool awaken = true;
        bool result = rtos.queue_overwrite_from_isr(&q, &value, &awaken);
        EXPECT_TRUE(result);
        EXPECT_FALSE(awaken);
        EXPECT_EQ(q.item_count, 1);
        EXPECT_EQ(q.front_index, 0);
        EXPECT_EQ(q.back_index, 1);

        float stored = 0.0f;
        memcpy(&stored, q.byte_queue, sizeof(float));
        EXPECT_FLOAT_EQ(stored, value);

        free(q.byte_queue);
    }

    TEST_F(MockRTOSQueueTest, QueueOverwriteFromIsrNonFullQueueSuccess) {
        MockRTOSQueue q;
        q.queue_length = 3;
        q.item_size = sizeof(float);
        q.item_count = 2;
        q.front_index = 0;
        q.back_index = 2;
        q.byte_queue =
            static_cast<uint8_t*>(malloc(q.queue_length * q.item_size));
        ASSERT_NE(q.byte_queue, nullptr);

        float old_value1 = 1.23f, old_value2 = 2.34f;
        memcpy(q.byte_queue + 0 * q.item_size, &old_value1, sizeof(float));
        memcpy(q.byte_queue + 1 * q.item_size, &old_value2, sizeof(float));

        float new_value = 8.76f;
        bool awaken = true;
        bool result = rtos.queue_overwrite_from_isr(&q, &new_value, &awaken);
        EXPECT_TRUE(result);
        EXPECT_FALSE(awaken);
        EXPECT_EQ(q.item_count, 3);
        EXPECT_EQ(q.front_index, 0);
        EXPECT_EQ(q.back_index, 0);

        // Check integrity of existing values
        float stored1 = 0.0f, stored2 = 0.0f;
        memcpy(&stored1, q.byte_queue + 0 * q.item_size, sizeof(float));
        memcpy(&stored2, q.byte_queue + 1 * q.item_size, sizeof(float));
        EXPECT_FLOAT_EQ(stored1, old_value1);
        EXPECT_FLOAT_EQ(stored2, old_value2);

        // Check new value
        float stored_new = 0.0f;
        memcpy(&stored_new, q.byte_queue + 2 * q.item_size, sizeof(float));
        EXPECT_FLOAT_EQ(stored_new, new_value);

        free(q.byte_queue);
    }

    TEST_F(MockRTOSQueueTest, QueueOverwriteFromIsrFullQueueSuccess) {
        MockRTOSQueue q;
        q.queue_length = 2;
        q.item_size = sizeof(float);
        q.item_count = 2;
        q.front_index = 0;
        q.back_index = 0;
        q.byte_queue =
            static_cast<uint8_t*>(malloc(q.queue_length * q.item_size));
        ASSERT_NE(q.byte_queue, nullptr);

        float old_value1 = 1.23f, old_value2 = 2.34f;
        memcpy(q.byte_queue + 0 * q.item_size, &old_value1, sizeof(float));
        memcpy(q.byte_queue + 1 * q.item_size, &old_value2, sizeof(float));

        float new_value = 11.11f;
        bool awaken = true;
        bool result = rtos.queue_overwrite_from_isr(&q, &new_value, &awaken);
        EXPECT_TRUE(result);
        EXPECT_FALSE(awaken);
        EXPECT_EQ(q.item_count, 2);
        EXPECT_EQ(q.front_index, 1);
        EXPECT_EQ(q.back_index, 1);

        float stored = 0.0f;
        memcpy(&stored, q.byte_queue + 0 * q.item_size, sizeof(float));
        EXPECT_FLOAT_EQ(stored, new_value);

        free(q.byte_queue);
    }

    TEST_F(MockRTOSQueueTest, QueuePushbackFromIsrSuccess) {
        MockRTOSQueue q;
        q.queue_length = 2;
        q.item_size = sizeof(float);
        q.item_count = 1;
        q.front_index = 0;
        q.back_index = 1;
        q.byte_queue =
            static_cast<uint8_t*>(malloc(q.queue_length * q.item_size));
        ASSERT_NE(q.byte_queue, nullptr);

        // Existing value
        float existing = 4.44f;
        memcpy(q.byte_queue, &existing, sizeof(float));

        float value = 6.66f;
        bool awaken = true;
        bool result = rtos.queue_pushback_from_isr(&q, &value, &awaken);
        EXPECT_TRUE(result);
        EXPECT_FALSE(awaken);  // Should be set to false
        EXPECT_EQ(q.item_count, 2);
        EXPECT_EQ(q.front_index, 0);
        EXPECT_EQ(q.back_index, 0);  // Should wrap around

        // Check integrity of existing value
        float stored_existing = 0.0f;
        memcpy(&stored_existing, q.byte_queue, sizeof(float));
        EXPECT_FLOAT_EQ(stored_existing, existing);

        // Check new value
        float stored_new = 0.0f;
        memcpy(&stored_new, q.byte_queue + 1 * q.item_size, sizeof(float));
        EXPECT_FLOAT_EQ(stored_new, value);

        free(q.byte_queue);
    }

    TEST_F(MockRTOSQueueTest, QueuePushbackFromIsrFailsIfFull) {
        MockRTOSQueue q;
        q.queue_length = 2;
        q.item_size = sizeof(float);
        q.item_count = 2;  // Already full
        q.front_index = 0;
        q.back_index = 0;
        q.byte_queue =
            static_cast<uint8_t*>(malloc(q.queue_length * q.item_size));
        ASSERT_NE(q.byte_queue, nullptr);

        float value = 8.88f;
        bool awaken = true;
        // Save original state
        size_t orig_item_count = q.item_count;
        size_t orig_front_index = q.front_index;
        size_t orig_back_index = q.back_index;
        uint8_t* orig_byte_queue = q.byte_queue;

        bool result = rtos.queue_pushback_from_isr(&q, &value, &awaken);
        EXPECT_FALSE(result);
        EXPECT_FALSE(awaken);  // Should be set to false
        EXPECT_EQ(q.item_count, orig_item_count);
        EXPECT_EQ(q.front_index, orig_front_index);
        EXPECT_EQ(q.back_index, orig_back_index);
        EXPECT_EQ(q.byte_queue, orig_byte_queue);

        free(q.byte_queue);
    }

}  // namespace