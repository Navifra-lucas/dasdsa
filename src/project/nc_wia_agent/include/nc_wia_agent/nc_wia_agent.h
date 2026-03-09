#ifndef NC_WIA_AGNET_H
#define NC_WIA_AGNET_H

#include "core/util/logger.hpp"
#include "core_agent/data/memory_repository.h"
#include "core_agent/data/robot_info.h"
#include "core_agent/data/types.h"
#include "core_agent/util/config.h"

#include <Poco/File.h>
#include <Poco/FileStream.h>
#include <Poco/JSON/Array.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <Poco/Net/HTTPClientSession.h>
#include <Poco/Net/HTTPRequest.h>
#include <Poco/Net/HTTPResponse.h>
#include <Poco/Net/HTTPSession.h>
#include <Poco/Path.h>
#include <Poco/Runnable.h>
#include <Poco/SignalHandler.h>
#include <Poco/StreamCopier.h>
#include <Poco/StreamTokenizer.h>
#include <Poco/StringTokenizer.h>
#include <boost/any.hpp>
#include <boost/variant.hpp>
#include <core_agent/message/message_broker.h>
#include <core_agent/message/message_publisher.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <signal.h>

#include <future>
#include <map>
#include <queue>
#include <string>
#include <vector>

#endif  // NC_WIA_AGNET_H