#ifndef __FIFO_HPP
#define __FIFO_HPP

#include <array>
#include <cstring>

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
        RingBuffer() : head(0), tail(0), count(0) { buffer.fill(0); }

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

    /**
     * @brief A variable-size FIFO (First In, First Out) queue implementation.
     * 
     * Variable size items can be pushed and popped from the queue.
     * Items are stored in a fixed-size contiguous memory block. There is a maximum
     * total number of items is also limited. During a push, if the item cannot fit
     * in the pool or there are no available slots in the index buffer, it will be dropped.
     * This DS does not manage the memory of the items, it only copies the data. There is
     * no wrap-around during pushes, so when the FIFO is full, it will drop new items until
     * there is space available.
     */
    template <size_t PoolSize, size_t MaxItemCount>
    class VarFIFO {
       private:
        std::array<uint8_t, PoolSize> pool;
        RingBuffer<VarFIFOIndex, MaxItemCount> index_buffer;
        size_t capacity_used = 0;
        size_t next_free_index = 0;

       public:
        VarFIFO() { pool.fill(0); }

        template <typename T>
        [[nodiscard]] bool push(T& item) {
            size_t item_size = sizeof(T);
            if (item_size + capacity_used > PoolSize ||
                index_buffer.size() >= MaxItemCount) {
                return false;  // Not enough space in pool or index buffer is full
            }

            // Find the next free index in the pool
            if (next_free_index + item_size > PoolSize) {
                return false;  // Not enough contiguous space at the end of the pool
            }

            // Copy the item into the pool
            std::memcpy(pool.data() + next_free_index, &item, item_size);

            // Update the index buffer
            VarFIFOIndex index_entry {next_free_index, item_size};
            if (!index_buffer.push(index_entry)) {
                return false;  // Should not happen as we checked earlier
            }

            // Update capacity used and next free index
            capacity_used += item_size;
            next_free_index += item_size;

            return true;
        }

        [[nodiscard]] bool pop(void* dst) {
            VarFIFOIndex index_entry;
            if (!index_buffer.pop(index_entry)) {
                return false;  // Buffer is empty
            }

            // Copy the item from the pool to the destination
            std::memcpy(dst, pool.data() + index_entry.index, index_entry.size);

            // Update capacity used
            capacity_used -= index_entry.size;
            // Note: next_free_index is not decremented to avoid fragmentation.
            // This is a simple implementation and does not wrap around memory.

            return true;
        }

        void clear() {
            pool.fill(0);
            index_buffer.clear();
            capacity_used = 0;
            next_free_index = 0;
        }

        size_t get_indices_used() const { return index_buffer.size(); }
        size_t get_capacity_used() const { return capacity_used; }
    };

    /**
     * @brief A double variable-size FIFO (First In, First Out) queue implementation.
     * 
     * DoubleVarFIFO maintains two separate VarFIFO instances to allow for
     * concurrent push and pop operations. 
     */
    template <size_t PoolSize, size_t MaxItemCount>
    class DoubleVarFIFO {}
}  // namespace dsa

#endif