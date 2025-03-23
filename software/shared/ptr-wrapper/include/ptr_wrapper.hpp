#pragma once

#include <functional>
#include <stdexcept>

/// @brief A wrapper around a pointer which will automatically deallocate the resource when it goes out of scope
/// @details Intended for use in situations where @ref shared_ptr can't be used, such as when the resource is not allocated with `new`
template <typename T>
class PtrWrapper
{
public:
    PtrWrapper() = default;
    PtrWrapper(T* ptr, std::function<void(T*)> deallocator)
        : PtrWrapper([ptr](void)
                     { return ptr; }, deallocator)
    {
    }
    PtrWrapper(std::function<T*(void)> allocator, std::function<void(T*)> deallocator)
        : _deallocator(deallocator)
    {
        if (!allocator)
            throw std::invalid_argument("Allocator must be provided");
        if (!deallocator)
            throw std::invalid_argument("Deallocator must be provided");
        _ptr = allocator();
    }

    ~PtrWrapper()
    {
        try
        {
            if (_ptr)
                _deallocator(_ptr);
        }
        catch (...)
        {
            // ignore - since we're in a destructor, we can't throw
        }
    }

    // we do NOT want to allow copying, that doesn't make sense.
    // Moving is OK though
    PtrWrapper(const PtrWrapper&) = delete;
    PtrWrapper(PtrWrapper&& other) noexcept
    {
        swap(*this, other);
    }
    PtrWrapper& operator=(PtrWrapper) = delete;
    PtrWrapper& operator=(PtrWrapper&& other) noexcept
    {
        swap(*this, other);
        return *this;
    }

    PtrWrapper& operator=(T* ptr)
    {
        if (_ptr)
            _deallocator(_ptr);
        _ptr = ptr;
        return *this;
    }

    T** operator&()
    {
        return &_ptr;
    }

    friend void swap(PtrWrapper& first, PtrWrapper& second)
    {
        using std::swap;
        swap(first._ptr, second._ptr);
        swap(first._deallocator, second._deallocator);
    }

    // allow all the common methods of getting the file descriptor/"dereferencing"
    virtual inline explicit operator T*() const { return get(); }
    virtual inline T* operator*() const { return get(); }
    virtual inline T* operator->() const { return get(); }
    virtual inline T* get() const { return _ptr; }

private:
    T* _ptr = nullptr;

    std::function<void(T*)> _deallocator = nullptr;
};