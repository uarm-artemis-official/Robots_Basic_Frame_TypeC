#ifndef ISR_INTERFACES_HPP
#define ISR_INTERFACES_HPP

#include <array>
#include <type_traits>
#include "uarm_lib.hpp"

namespace isr {
    constexpr size_t MAX_REGISTERED_ROUTINES = 20;

    template <enum ECallbacks, typename TISRRoutine, typename TInitFunc,
              typename TMiddleware, typename TCallbackParam>
    class ISR {
        static_assert(std::is_invocable_r_v<void, TISRRoutine, TMiddleware,
                                            TCallbackParam>);
        static_assert(std::is_invocable_r_v<void, TInitFunc>);

       protected:
        std::array<std::pair<TISRRoutine, ECallbacks> MAX_REGISTERED_ROUTINES>
            routines;
        size_t routines_size;
        std::array<TISRRoutine, MAX_REGISTERED_ROUTINES> init_funcs;
        size_t init_funcs_size;

       public:
        bool register_routine(ECallbacks register_for,
                              TISRRoutine&& routine_func) {
            ASSERT(routines_size < MAX_REGISTERED_ROUTINES,
                   "Registered too many ISR routines.");
            routines[routines_size++] = {register_for, routine_func};
        }

        bool register_init(TInitFunc&& init_func) {
            init_funcs[init_funcs_size++] = init_func;
        }

        virtual [[nodiscard]] bool init() = 0;
        virtual void run_isr_routines(ECallbacks isr_running) = 0;
    };
}  // namespace isr

#endif