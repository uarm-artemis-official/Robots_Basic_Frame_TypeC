#ifndef ISR_INTERFACES_HPP
#define ISR_INTERFACES_HPP

#include <array>
#include <functional>
#include <type_traits>
#include "uarm_lib.hpp"

namespace isr {
    constexpr size_t MAX_REGISTERED_ROUTINES = 20;

    /**
     * @brief Base class for derived ISR classes.
     * 
     * @tparam ECallbacks Enum type for all supported events that routines can register for.
     * @tparam TISRRoutine Functor type for routine callbacks which are called when a specific
     * event occrs. These should be a std::function type with return and parameter types
     * determined by specific ISR implementation of run_isr_routines().
     * @tparam TInitFunc Functor type for init function which are called during initialization. 
     * These should be a std::function type with return and parameter types
     * determined by specific ISR implementation of init().
     * @tparam TISRState Any state information relevant to running routines when an ISR is 
     * triggered.
     */
    template <typename ECallbacks, typename TISRRoutine, typename TInitFunc,
              typename TISRState>
    class ISR {
       protected:
        std::array<std::pair<TISRRoutine, ECallbacks>, MAX_REGISTERED_ROUTINES>
            routines;
        size_t routines_size = 0;
        std::array<TInitFunc, MAX_REGISTERED_ROUTINES> init_funcs;
        size_t init_funcs_size = 0;
        bool initialized = false;

       public:
        template <typename TFunctor>
        bool register_routine(ECallbacks register_for,
                              TFunctor&& routine_func) {
            // The static assert is to prevent excessively large functors from being registered.
            // This is to hopefully ensure small buffer optimization (SBO) and prevent heap allocation.
            // A possible TODO would be to create a statically allocated type-erased container
            // similar to std::function and replace it in all used instances throughout the codebase.
            static_assert(
                sizeof(routine_func) <= 32,
                "Cannot register routine functors larger 32 bytes large.");
            ASSERT(routines_size < MAX_REGISTERED_ROUTINES,
                   "Registered too many ISR routines.");
            routines[routines_size++] =
                std::make_pair(std::function(routine_func), register_for);
            on_register_routine(routines_size - 1);
            return true;
        }

        template <typename TFunctor>
        bool register_init(TFunctor&& init_func) {
            static_assert(
                sizeof(init_func) <= 32,
                "Cannot register routine functors larger 32 bytes large.");
            ASSERT(init_funcs_size < MAX_REGISTERED_ROUTINES,
                   "Registered too many ISR routines.");
            init_funcs[init_funcs_size++] = std::function(init_func);
            on_register_init(init_funcs_size - 1);
            return true;
        }

        [[nodiscard]] virtual bool init() = 0;

        // Event handlers for successful registration of routines and init functions.
        [[nodiscard]] virtual bool on_register_init(size_t init_func_idx) = 0;
        [[nodiscard]] virtual bool on_register_routine(
            size_t routine_func_idx) = 0;

        virtual void run_isr_routines(ECallbacks callback_running,
                                      TISRState callback_state) = 0;

        [[nodiscard]] bool is_initialized() const { return initialized; }
    };
}  // namespace isr

#endif