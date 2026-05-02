#ifndef __UARM_LIB_HPP
#define __UARM_LIB_HPP

#include <concepts>
#include <tuple>
#include <type_traits>
#include "uarm_types.hpp"

// TODO: Make type safe memcpy/memset?
#ifdef GTEST

#include <cassert>
#include <cstdlib>

#define ASSERT(cond, msg) \
    if (!(cond))          \
    assert(0 && (msg))
#define MALLOC(size) malloc(size)
#define FREE(ptr) free(ptr)

#else

#if defined(STM32F407xx)
#include "stm32f407xx.h"
#elif defined(STM32F427xx)
#include "stm32f427xx.h"
#else
#error "Unknown STM32 MCU. Please define either STM32F407xx or STM32F427xx."
#endif

__attribute__((noreturn)) __attribute__((weak)) void on_error(const char* msg) {
    (void) msg;
    __disable_irq();
    while (true) {}
}

#define ASSERT(cond, msg)        \
    if (!(cond)) {               \
        on_error(msg);           \
        __builtin_unreachable(); \
    }

#define MALLOC(size) pvPortMalloc(size)
#define FREE(ptr) vPortFree(ptr)

#endif

namespace uarm_lib {
    namespace typelist {
        template <template <typename> typename Checker, typename TL>
        auto concept_filter() {
            // We use a fold expression (...) over the comma operator
            // to build one giant tuple from many small ones.
            return []<size_t... indices>(std::index_sequence<indices...>) {
                return std::tuple_cat([]() {
                    if constexpr (Checker<std::tuple_element_t<indices,
                                                               TL>>::value) {
                        return std::tuple<std::tuple_element_t<indices, TL>> {};
                    } else {
                        return std::tuple<> {};
                    }
                }()...);
            }(std::make_index_sequence<std::tuple_size_v<TL>> {});
        }

        template <template <typename> typename Checker, typename TL>
        using concept_filter_t = decltype(concept_filter<Checker, TL>());

        template <typename TL, typename F>
        constexpr auto functor_map(F&& f) {
            constexpr size_t size = std::tuple_size_v<TL>;

            return [&]<size_t... Is>(std::index_sequence<Is...>) {
                return std::array {f(std::tuple_element_t<Is, TL> {})...};
            }(std::make_index_sequence<size> {});
        }  // namespace typelist
    }  // namespace typelist
}  // namespace uarm_lib

#endif