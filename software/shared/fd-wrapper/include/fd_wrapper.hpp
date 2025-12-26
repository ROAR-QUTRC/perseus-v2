#pragma once

#include <functional>

class FdWrapper final
{
public:
    FdWrapper() = default;
    FdWrapper(std::function<int(void)> open, std::function<void(int)> configure = nullptr, std::function<void(int)> close = nullptr);
    ~FdWrapper();

    // we do NOT want to allow copying, that doesn't make sense for file descriptors
    // moving is OK though
    // copy constructor
    FdWrapper(const FdWrapper&) = delete;
    // copy assignment - note that it needs to be a reference, otherwise assignments match for both move and copy
    FdWrapper& operator=(const FdWrapper&) = delete;
    // move constructor
    FdWrapper(FdWrapper&& other) noexcept
    {
        swap(*this, other);
    }
    // move assignment
    FdWrapper& operator=(FdWrapper&& other) noexcept
    {
        swap(*this, other);
        return *this;
    }

    friend void swap(FdWrapper& first, FdWrapper& second) noexcept;

    // allow all the common methods of getting the file descriptor/"dereferencing"
    inline explicit operator int() const { return get(); }
    inline int operator*() const { return get(); }
    inline int get() const { return _fd; }

private:
    void _handle_close();
    int _fd = -1;

    // non-const to allow swapping
    std::function<void(int)> _close = nullptr;
};