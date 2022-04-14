// from https://github.com/hebaishi/easy-cpp-print
// https://github.com/hebaishi/easy-cpp-print/blob/0e24a7f493dbfae40045c4bb642c4e3f9e76b9a3/include/easyprint.hpp
#pragma once

#include <iostream>
#include <sstream>
#include <tuple>

namespace easyprint {

template <typename ...Targs>
struct default_delimiter {
  static constexpr const char* start_delimiter = "{";
  static constexpr const char* element_delimiter = ", ";
  static constexpr const char* end_delimiter = "}";
};

template <size_t I> struct tuple_delimiter {
  static constexpr const char* value = default_delimiter<int>::element_delimiter;
};

template <> struct tuple_delimiter<0> { static constexpr const char* value = default_delimiter<int>::start_delimiter; };

template <typename T> struct is_tuple { static const bool value = false; };

template <typename... Targs> struct is_tuple<std::tuple<Targs...>> {
  static const bool value = true;
};

// Helper struct to determine
// whether a type has a const_iterator typedef
template <typename T> struct is_const_iterable {

  template <typename C> static std::true_type f(typename C::const_iterator *);

  template <typename C> static std::false_type f(...);

  typedef decltype(f<T>(0)) type;
};

template <typename T>
using is_const_iterable_v = typename is_const_iterable<T>::type;

template <typename T>
inline void print_iterator_helper(std::false_type, std::ostream &os, const T &cont) {
  os << cont;
}

inline void print_iterator_helper(std::false_type, std::ostream &os, const char *cont) {
  os << "\"" << cont << "\"";
}

template <typename TChar>
inline void print_iterator_helper(std::true_type, std::ostream &os,
                           const std::basic_string<TChar> &cont) {
  os << "\"" << cont << "\"";
}

// Functions to recursively print tuples
template <size_t I, typename T>
typename std::enable_if<(!is_tuple<T>::value), void>::type
inline print_tuple(std::ostream &os, const T &cont) {
  print_iterator_helper(is_const_iterable_v<T>(), os, cont);
}

template <size_t I, typename... Targs>
typename std::enable_if<(I == sizeof...(Targs)), void>::type
inline print_tuple(std::ostream &os, const std::tuple<Targs...> &tup) {
  os << default_delimiter<std::tuple<Targs...>>::end_delimiter;
}

template <size_t I, typename... Targs>
typename std::enable_if<(I < sizeof...(Targs)), void>::type
inline print_tuple(std::ostream &os, const std::tuple<Targs...> &tup) {
  os << tuple_delimiter<I>::value;
  auto val = std::get<I>(tup);
  print_tuple<0>(os, val);
  print_tuple<I + 1>(os, tup);
}

template <typename T>
inline void print_iterator_helper(std::true_type, std::ostream &os, const T &cont);

// Pair specialisation
template <typename T1, typename T2>
inline void print_iterator_helper(std::false_type, std::ostream &os,
                           const std::pair<T1, T2> &cont) {
  os << default_delimiter<decltype(cont)>::start_delimiter;
  print_iterator_helper(is_const_iterable_v<T1>(), os, cont.first);
  os << default_delimiter<decltype(cont)>::element_delimiter;
  print_iterator_helper(is_const_iterable_v<T2>(), os, cont.second);
  os << default_delimiter<decltype(cont)>::end_delimiter;
}

// Specialisation for tuples
// Passes control to tuple printing
// functions
template <typename... Targs>
inline void print_iterator_helper(std::false_type, std::ostream &os,
                           const std::tuple<Targs...> &cont) {
  print_tuple<0>(os, cont);
}

// Recursive function to print iterators
template <typename T>
inline void print_iterator_helper(std::true_type, std::ostream &os, const T &cont) {
  os << default_delimiter<decltype(cont)>::start_delimiter;
  if (!cont.empty()) {
    auto it = cont.begin();
    print_iterator_helper(is_const_iterable_v<typename T::value_type>{}, os, *it);
    it++;
    for (; it != cont.cend(); it++) {
      os << default_delimiter<decltype(cont)>::element_delimiter;
      print_iterator_helper(is_const_iterable_v<typename T::value_type>{}, os, *it);
    }
  }
  os << default_delimiter<decltype(cont)>::end_delimiter;
}

// User-facing functions
template <typename T>
inline std::string stringify(const T &container) {
  std::stringstream ss;
  print_iterator_helper(is_const_iterable_v<T>(), ss, container);
  return ss.str();
}

template <typename T>
inline void print(const T &container) {
  std::cout << Stringify(container);
}

}  // namespace easyprint
