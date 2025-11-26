#include "message_center.hpp"
#include "middleware_interfaces.hpp"

#include <array>
#include <cstring>
#include <optional>
#include <type_traits>
#include <vector>

struct Counters {
    size_t get_count = 0;
    size_t pub_count = 0;
    std::vector<uint8_t> return_value_bytes;
    std::vector<uint8_t> last_pub_bytes;
};

template <typename TopicRegistry>
class FakeMessageCenter : public mc2::MC2<TopicRegistry> {
   private:
    // keep a local reference to RTOS because base MC2::rtos is private
    MW_RTOS::IRTOS& rtos_ref;
    static constexpr size_t NUM_TOPICS = std::tuple_size_v<TopicRegistry>;
    std::array<Counters, NUM_TOPICS> counters_;

   public:
    FakeMessageCenter(MW_RTOS::IRTOS& _rtos)
        : mc2::MC2<TopicRegistry>(_rtos), rtos_ref(_rtos) {}

    // Set the bytes that should be copied into `message` when get_message<T>
    template <typename T>
    void set_get_return_value(const T& value) {
        static_assert(std::is_trivially_copyable_v<T>,
                      "set_get_return_value requires trivially copyable types");
        constexpr size_t idx = mc2::get_index<T, TopicRegistry>();
        auto& c = counters_.at(idx);
        c.return_value_bytes.resize(sizeof(T));
        std::memcpy(c.return_value_bytes.data(), &value, sizeof(T));
    }

    // Retrieve counters for a message type
    template <typename T>
    Counters* get_counters() {
        constexpr size_t idx = mc2::get_index<T, TopicRegistry>();
        return &counters_[idx];
    }

    // Fake init: do nothing, return true
    bool init() { return true; }

    // get_message: if a return_value exists for T, copy it into message and
    // increment get_count and return current tick. Otherwise return nullopt.
    template <typename T>
    std::optional<MW_RTOS::TickType> get_message(
        T& message, MW_RTOS::TickType ticks_to_wait = 0) {
        static_assert(std::is_trivially_copyable_v<T>,
                      "get_message in fake requires trivially copyable types");
        (void) ticks_to_wait;
        constexpr size_t idx = mc2::get_index<T, TopicRegistry>();
        auto& c = counters_.at(idx);
        ++c.get_count;
        if (c.return_value_bytes.size() >= sizeof(T)) {
            std::memcpy(&message, c.return_value_bytes.data(), sizeof(T));
            return std::make_optional(rtos_ref.get_current_tick());
        }
        return std::nullopt;
    }

    // peek_message has same semantics as get_message in the fake
    template <typename T>
    std::optional<MW_RTOS::TickType> peek_message(
        T& message, MW_RTOS::TickType ticks_to_wait = 0) {
        return get_message(message, ticks_to_wait);
    }

    // pub_message: increment pub_count, store last published bytes, return current tick
    template <typename T>
    std::optional<MW_RTOS::TickType> pub_message(
        T& message, MW_RTOS::TickType ticks_to_wait = 0) {
        static_assert(std::is_trivially_copyable_v<T>,
                      "pub_message in fake requires trivially copyable types");
        (void) ticks_to_wait;

        constexpr size_t idx = mc2::get_index<T, TopicRegistry>();
        auto& c = counters_.at(idx);
        ++c.pub_count;
        c.last_pub_bytes.resize(sizeof(T));
        std::memcpy(c.last_pub_bytes.data(), &message, sizeof(T));
        return std::make_optional(rtos_ref.get_current_tick());
    }

    // pub_message_from_isr: same as pub_message but doesn't use ticks param
    template <typename T>
    std::optional<MW_RTOS::TickType> pub_message_from_isr(
        T& message, bool* will_context_switch = nullptr) {
        static_assert(
            std::is_trivially_copyable_v<T>,
            "pub_message_from_isr in fake requires trivially copyable types");
        if (will_context_switch)
            *will_context_switch = false;
        return pub_message(message, 0);
    }
};