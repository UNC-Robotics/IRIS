#ifndef _KINEMATICS_INDEXSEQUENCE_HPP_
#define _KINEMATICS_INDEXSEQUENCE_HPP_

namespace CustomTools {

template <std::size_t...> struct index_sequence {};

namespace detail {
  template <std::size_t N, std::size_t... Is>
  struct index_sequence_maker :
    index_sequence_maker<N-1, N-1, Is...>
  {};
  template <std::size_t... Is>
  struct index_sequence_maker<0, Is...>
  {
    using type = index_sequence<Is...>;
  };
}

template <std::size_t N>
using make_index_sequence = typename detail::index_sequence_maker<N>::type;

}

//index_sequence_maker<3>::type = index_sequence_maker<2, 2>::type = index_sequence_maker<1, 1, 2>::type = index_sequence_maker<0, 0, 1, 2>::type = index_sequence<0, 1, 2>;


#endif
