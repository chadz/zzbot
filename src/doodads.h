#pragma once

#include <functional>

template <class T>
class optional {

  private:
    std::pair<T, bool> obj_;

  public:
    optional() { obj_ = {{}, false}; }
    optional(const T &obj) { obj_ = {obj, true}; }

    operator bool() const { return obj_.second; }

    T &value() { return obj_.first; }
    bool has_value() const { return obj_.second; }
};

template <class T, class P>
T &filter(T &obj, P fn) {
    //still cant figure out how to negate the predicate here... no combination of bind/fun_ptr/negate/etc. works
    obj.erase(std::remove_if(std::begin(obj), std::end(obj), fn) , std::end(obj));
    return obj;
}

// template <class T, class F>
// auto sum_by(T &obj, F fn)  {
//     return std::accumulate(std::begin(obj), std::end(obj), 0, fn);
// }
