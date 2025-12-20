#ifndef __FIFO_HPP
#define __FIFO_HPP

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <span>
#include "uarm_lib.hpp"

namespace dsa {
    /**
     * @brief A simple ring buffer (circular buffer) implementation.
     * 
     * This is a fixed-size buffer that overwrites old data when full.
     * This is primarily used in VarFifo for storing indices of occupied slots.
     * However, it can be used in other scenarios where a FIFO buffer is needed.
     * 
     * Template Parameters:
     * - T: Type of elements stored in the buffer.
     * - N: Maximum number of elements the buffer can hold.
     */
    template <typename T, size_t N>
    class RingBuffer {
       private:
        std::array<T, N> buffer;
        size_t head;
        size_t tail;
        size_t count;

       public:
        RingBuffer() : head(0), tail(0), count(0) { buffer.fill(T {}); }

        bool is_empty() const { return count == 0; }

        bool is_full() const { return count == N; }

        size_t size() const { return count; }

        [[nodiscard]] bool push(const T& value) {
            if (is_full()) {
                return false;  // Buffer is full
            }
            buffer[head] = value;
            head = (head + 1) % N;
            ++count;
            return true;
        }

        [[nodiscard]] bool pop(T& value) {
            if (is_empty()) {
                return false;  // Buffer is empty
            }
            value = buffer[tail];
            tail = (tail + 1) % N;
            --count;
            return true;
        }

        void clear() {
            head = 0;
            tail = 0;
            count = 0;
            std::memset(buffer.data(), 0, sizeof(T) * N);
        }
    };

    struct VarFIFOIndex {
        size_t index;
        size_t size;
    };

    // TODO: Implement wrapping and proper popping from the front of the queue.
    /**
     * @brief A variable-size FIFO (First In, First Out) queue implementation.
     * 
     * Variable size items can be pushed and popped from the queue.
     * Items are stored in a fixed-size contiguous memory block. There is a maximum
     * total number of items is also limited. During a push, if the item cannot fit
     * in the pool or there are no available slots in the index buffer, it will be dropped.
     * This DS copies items on push, so their original copies can be used without
     * affecting the items in the FIFO.
     */
    template <size_t PoolSize, size_t MaxItemCount>
    class VarFIFO {
        static_assert(PoolSize > 0, "PoolSize must be greater than 0");
        static_assert(PoolSize >= MaxItemCount,
                      "MaxItemCount must be less than or equal to PoolSize");
        static_assert(MaxItemCount > 0, "MaxItemCount must be greater than 0");

       private:
        std::array<std::byte, PoolSize> pool;  // Changed to std::byte
        RingBuffer<VarFIFOIndex, MaxItemCount> index_buffer;
        size_t capacity_used = 0;
        size_t back_index = 0;
        size_t front_index = 0;

       public:
        VarFIFO() { pool.fill(std::byte {0}); }  // Initialize with std::byte{0}

        [[nodiscard]] bool push(std::span<const std::byte> item) {
            constexpr size_t MIN_ITEM_SIZE = 1;
            ASSERT(item.size() <= PoolSize, "Item too large for the pool.");
            ASSERT(item.size() >= MIN_ITEM_SIZE, "Item cannot be zero-sized.");

            // Check index buffer availability
            if (index_buffer.is_full()) {
                return false;
            }

            // Check pool capacity availability
            if ((PoolSize - capacity_used) < item.size()) {
                return false;
            }

            const size_t start = back_index;

            // Copy into pool with wrap-around if necessary
            if (start + item.size() <= PoolSize) {
                // contiguous copy
                std::copy(item.begin(), item.end(), pool.begin() + start);
            } else {
                const size_t first_chunk = PoolSize - start;
                std::copy(item.begin(), item.begin() + first_chunk,
                          pool.begin() + start);
                std::copy(item.begin() + first_chunk, item.end(), pool.begin());
            }

            // Record index and size
            VarFIFOIndex idx {start, item.size()};
            if (!index_buffer.push(idx)) {
                return false;
            }

            capacity_used += item.size();
            back_index = (start + item.size()) % PoolSize;
            return true;
        }

        [[nodiscard]] bool pop(std::span<std::byte> dst, size_t& out_size) {
            ASSERT(!dst.empty(), "Destination span cannot be empty.");

            VarFIFOIndex idx;
            if (!index_buffer.pop(idx)) {
                return false;  // nothing to pop
            }

            const size_t start = idx.index;
            const size_t sz = idx.size;

            ASSERT(dst.size() >= sz, "Destination span is too small.");

            // Copy out with wrap-around handling
            if (start + sz <= PoolSize) {
                std::copy(pool.begin() + start, pool.begin() + start + sz,
                          dst.begin());
            } else {
                const size_t first_chunk = PoolSize - start;
                std::copy(pool.begin() + start,
                          pool.begin() + start + first_chunk, dst.begin());
                std::copy(pool.begin(), pool.begin() + (sz - first_chunk),
                          dst.begin() + first_chunk);
            }

            // Update bookkeeping
            capacity_used -= sz;
            front_index = (start + sz) % PoolSize;
            out_size = sz;

            return true;
        }

        // Old overloads with uint8_t pointers for compatibility.
        // TODO: Phase out these overloads and use std::span<std::byte> instead.
        [[nodiscard]] bool push(const uint8_t* item, size_t item_size) {
            constexpr size_t MIN_ITEM_SIZE = 1;
            ASSERT(item_size <= PoolSize, "Item too large for the pool.");
            ASSERT(item_size >= MIN_ITEM_SIZE, "Item cannot be zero-sized.");

            // Check index buffer availability
            if (index_buffer.is_full()) {
                return false;
            }

            // Check pool capacity availability
            if ((PoolSize - capacity_used) < item_size) {
                return false;
            }

            const size_t start = back_index;

            // Copy into pool with wrap-around if necessary
            if (start + item_size <= PoolSize) {
                // contiguous copy
                std::memcpy(pool.data() + start, item, item_size);
            } else {
                const size_t first_chunk = PoolSize - start;
                std::memcpy(pool.data() + start, item, first_chunk);
                std::memcpy(pool.data(), item + first_chunk,
                            item_size - first_chunk);
            }

            // Record index and size
            VarFIFOIndex idx {start, item_size};
            if (!index_buffer.push(idx)) {
                return false;
            }

            capacity_used += item_size;
            back_index = (start + item_size) % PoolSize;
            return true;
        }

        [[nodiscard]] bool pop(uint8_t* dst, size_t& out_size) {
            ASSERT(dst != nullptr, "Destination pointer cannot be null.");

            VarFIFOIndex idx;
            if (!index_buffer.pop(idx)) {
                return false;  // nothing to pop
            }

            const size_t start = idx.index;
            const size_t sz = idx.size;

            // Copy out with wrap-around handling
            if (start + sz <= PoolSize) {
                std::memcpy(dst, pool.data() + start, sz);
            } else {
                const size_t first_chunk = PoolSize - start;
                std::memcpy(dst, pool.data() + start, first_chunk);
                std::memcpy(dst + first_chunk, pool.data(), sz - first_chunk);
            }

            // Update bookkeeping
            capacity_used -= sz;
            front_index = (start + sz) % PoolSize;
            out_size = sz;

            return true;
        }

        void clear() {
            pool.fill(std::byte {0});  // Clear with std::byte{0}
            index_buffer.clear();
            capacity_used = 0;
            front_index = 0;
            back_index = 0;
        }

        size_t get_indices_used_count() const { return index_buffer.size(); }
        size_t get_capacity_used() const { return capacity_used; }
    };

    /**
     * @brief A double variable-size FIFO (First In, First Out) queue implementation.
     * 
     * DoubleVarFIFO maintains two separate VarFIFO instances to allow for
     * concurrent push and pop operations. 
     */
    template <size_t PoolSize, size_t MaxItemCount>
    class DoubleVarFIFO {};
}  // namespace dsa

#endif