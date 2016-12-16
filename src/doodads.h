#pragma once

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

template <class T, class F>
T &filter(T &obj, F fn) {
    obj.erase(std::remove_if(std::begin(obj), std::end(obj), fn), std::end(obj));
    return obj;
}

// template <class T, class F>
// auto sum_by(T &obj, F fn)  {
//     return std::accumulate(std::begin(obj), std::end(obj), 0, fn);
// }
