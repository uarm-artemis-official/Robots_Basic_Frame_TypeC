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
#include "FreeRTOS.h"
#include "error_handler.h"

#define ASSERT(cond, msg)        \
    if (!(cond)) {               \
        error_handler((msg));    \
        __builtin_unreachable(); \
    }

#define MALLOC(size) pvPortMalloc(size)
#define FREE(ptr) vPortFree(ptr)
#endif

namespace uarm_lib {
    namespace typelist {
        template <template <typename> typename Checker, typename... Ts>
        auto concept_filter() {
            // We use a fold expression (...) over the comma operator
            // to build one giant tuple from many small ones.
            return std::tuple_cat([]() {
                if constexpr (Checker<Ts>::value) {
                    return std::tuple<Ts> {};
                } else {
                    return std::tuple<> {};
                }
            }()...);
        }

        template <template <typename> typename ConceptChecker, typename T,
                  typename... Rest>
        using concept_filter_t =
            decltype(concept_filter<ConceptChecker, T, Rest...>());

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