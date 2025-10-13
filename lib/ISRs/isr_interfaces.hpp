#ifndef ISR_INTERFACES_HPP
#define ISR_INTERFACES_HPP

#include <array>
#include <type_traits>
#include "uarm_lib.hpp"

namespace isr {
    constexpr size_t MAX_REGISTERED_ROUTINES = 20;

    template <typename ECallbacks, typename TISRRoutine, typename TInitFunc,
              typename TISRState>
    class ISR {
       protected:
        std::array<std::pair<TISRRoutine, ECallbacks>, MAX_REGISTERED_ROUTINES>
            routines;
        size_t routines_size;
        std::array<TInitFunc, MAX_REGISTERED_ROUTINES> init_funcs;
        size_t init_funcs_size;

       public:
        bool register_routine(ECallbacks register_for,
                              TISRRoutine&& routine_func) {
            ASSERT(routines_size < MAX_REGISTERED_ROUTINES,
                   "Registered too many ISR routines.");
            routines[routines_size++] = {register_for, routine_func};
            return true;
        }

        bool register_init(TInitFunc&& init_func) {
            ASSERT(init_funcs_size < MAX_REGISTERED_ROUTINES,
                   "Registered too many ISR routines.");
            init_funcs[init_funcs_size++] = init_func;
            return true;
        }

        [[nodiscard]] virtual bool init() = 0;
        virtual void run_isr_routines(ECallbacks callback_running,
                                      TISRState callback_state) = 0;
    };
}  // namespace isr

#endif