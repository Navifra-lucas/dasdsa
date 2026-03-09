#include "Poco/String.h"
#include "nc_brain_agent/nc_brain_agent.h"

#include <nc_brain_agent/web/common/mediaTypeMapper.h>

#include <cctype>

namespace NaviFra {
MediaTypeMapper::MediaTypeMapper()
    : _default("application/binary")
{
}

MediaTypeMapper::~MediaTypeMapper()
{
}

MediaTypeMapper::ConstIterator MediaTypeMapper::find(const std::string& suffix) const
{
    return _map.find(Poco::toLower(suffix));
}

void MediaTypeMapper::add(const std::string& suffix, const std::string& mediaType)
{
    _map[Poco::toLower(suffix)] = mediaType;
}

const std::string& MediaTypeMapper::map(const std::string& suffix) const
{
    ConstIterator it = find(Poco::toLower(suffix));
    if (it != end())
        return it->second;
    else
        return _default;
}

void MediaTypeMapper::setDefault(const std::string& mediaType)
{
    _default = mediaType;
}

}  // namespace NaviFra