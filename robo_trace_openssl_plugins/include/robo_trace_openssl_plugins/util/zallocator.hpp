#include <openssl/evp.h>
#include <openssl/rand.h>

static const unsigned int KEY_SIZE = 32;
static const unsigned int BLOCK_SIZE = 16;

template <typename T>
struct zallocator {

public:

    typedef T value_type;
    typedef value_type* pointer;
    typedef const value_type* const_pointer;
    typedef value_type& reference;
    typedef const value_type& const_reference;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;

    T* address(T& v) const {
        return &v;
    }

    const T* address(const T& v) const {
        return &v;
    }

    T* allocate(std::size_t n, const void* hint = 0) {

        if (n > std::numeric_limits<std::size_t>::max() / sizeof(T)) {
            throw std::bad_alloc();
        }

        return static_cast<T*>(::operator new(n * sizeof(T)));

    }

    void deallocate(T* p, std::size_t n) {
        OPENSSL_cleanse(p, n*sizeof(T));
        ::operator delete(p); 
    }
    
    std::size_t max_size() const {
        return std::numeric_limits<std::size_t>::max() / sizeof (T);
    }
    
    template<typename U>
    struct rebind {

        typedef zallocator<U> other;

    };

    void construct (T* ptr, const T& val) {
        new (static_cast<T*>(ptr) ) T (val);
    }

    void destroy(T* ptr) {
        static_cast<T*>(ptr)->~T();
    }

#if __cpluplus >= 201103L

    template<typename U, typename... Args>
    void construct(U* ptr, Args&&  ... args) {
        ::new (static_cast<void*> (ptr)) U (std::forward<Args> (args)...);
    }

    template<typename U>
    void destroy(U* ptr) {
        ptr->~U();
    }

#endif
};
