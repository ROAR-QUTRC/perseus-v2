#include "fd_wrapper.hpp"

#include <unistd.h>

#include <stdexcept>

using std::function;

FdWrapper::FdWrapper(function<int(void)> open, function<void(int)> configure, function<void(int)> userClose)
    : _close(userClose)
{
    if (!open)
        throw std::invalid_argument("open function must be provided");
    try
    {
        _fd = open();
        // note: open() should throw an exception if it fails, but if it doesn't, throw one here
        if (_fd < 0)
            throw std::runtime_error("Failed to open file descriptor");
        if (configure)
            configure(_fd);
    }
    catch (...)
    {
        _handleClose();
        throw;
    }
}

FdWrapper::~FdWrapper()
{
    try
    {
        _handleClose();
    }
    catch (...)
    {
        // ignore - since we're in a destructor, we can't throw
    }
}

void FdWrapper::_handleClose()
{
    if (_fd != -1)
    {
        if (_close)
            _close(_fd);
        else
            close(_fd);
        _fd = -1;
    }
}

void swap(FdWrapper& first, FdWrapper& second) noexcept
{
    using std::swap;
    swap(first._fd, second._fd);
    swap(first._close, second._close);
}