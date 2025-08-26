#include "canlib_types.hpp"

#include "canlib.hpp"

using std::string;

CanlibParameterGroup::~CanlibParameterGroup()
{
    for (const auto& [name, address] : _savedParams)
        canlibRemoveParameter(address);
}

canlib_error CanlibParameterGroup::getParameterAddress(const string name, canlib_address& out)
{
    if (!_savedParams.contains(name))
        return CANLIB_ERROR_NO_PARAMETER;
    out = _savedParams[name];
    return CANLIB_OK;
}

canlib_error CanlibParameterGroup::addParameter(const string name, const canlib_parameter_description description,
                                                const canlib_data initialData, const canlib_data dataSafeState)
{
    _savedParams[name] = description.address;
    return canlibAddParameter(description, initialData, dataSafeState);
}