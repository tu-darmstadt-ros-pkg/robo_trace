#pragma once
// Std
#include <memory>
// Boost
#include <boost/shared_ptr.hpp>

/**
 * 
 * This conversion helper is based on the following stack overflow thread:
 *   -> https://stackoverflow.com/questions/6326757/conversion-from-boostshared-ptr-to-stdshared-ptr
 * 
 * Shout outs!
 * 
 */

namespace {

    template<class SharedPointer> 
    struct Holder {

        SharedPointer p;

        Holder(const SharedPointer &p) : p(p) {
            //
        }

        Holder(const Holder &other) : p(other.p) {
            //
        }

        Holder(Holder &&other) : p(std::move(other.p)) {
            //
        }

        void operator() (...) { 
            p.reset(); 
        }
    };
}

template<class T> 
std::shared_ptr<T> to_std_ptr(const boost::shared_ptr<T> &p) {

    typedef Holder<std::shared_ptr<T>> H;

    if(H *h = boost::get_deleter<H>(p)) {
        return h->p;
    } else {
        return std::shared_ptr<T>(p.get(), Holder<boost::shared_ptr<T>>(p));
    }

}

template<class T> 
boost::shared_ptr<T> to_boost_ptr(const std::shared_ptr<T> &p) {

    typedef Holder<boost::shared_ptr<T>> H;

    if(H * h = std::get_deleter<H>(p)) {
        return h->p;
    } else {
        return boost::shared_ptr<T>(p.get(), Holder<std::shared_ptr<T>>(p));
    }

}