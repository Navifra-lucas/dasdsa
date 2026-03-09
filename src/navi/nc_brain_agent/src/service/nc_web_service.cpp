#include "nc_brain_agent/nc_brain_agent.h"

#include <nc_brain_agent/service/nc_web_service.h>
#include <nc_brain_agent/utils/nc_agent_config.h>
#include <nc_brain_agent/web/common/mediaTypeMapper.h>
#include <nc_brain_agent/web/common/webServerDispatcher.h>
#include <nc_brain_agent/web/common/webServerRequestHandlerFactory.h>
#include <nc_brain_agent/web/router/logTransportHandler.h>

using namespace NaviFra;

NcWEBService::NcWEBService()
{
}

NcWEBService::~NcWEBService()
{
    _httpServerParams.reset();
    _webServerDispatcher.reset();

    if (_httpServer != nullptr)
        _httpServer->stop();

    delete _httpServer;
}

void NcWEBService::initialize()
{
    _httpServerParams = new Poco::Net::HTTPServerParams();

    _httpServerParams->setMaxQueued(Config::instance().getInt("web.server.MaxQueued", 250));
    _httpServerParams->setMaxThreads(Config::instance().getInt("web.server.MaxThreads", 1));

    WebServerDispatcher::Config dispconfig;

    dispconfig.corsAllowedOrigin =
        Config::instance().getString("web.server.host", "http://localhost") + ":" + Config::instance().getString("web.server.port", "5555");
    Poco::AutoPtr<MediaTypeMapper> mime = new MediaTypeMapper();
    mime->add("html", "text/html");
    mime->add("ico", "image/x-icon");
    mime->add("css", "text/css");
    mime->add("jpeg", "image/jpeg");
    mime->add("jpg", "image/jpeg");
    mime->add("png", "image/png");
    mime->add("json", "application/json");
    mime->add("js", "application/javascript");
    mime->add("svg", "image/svg+xml");

    dispconfig.pMediaTypeMapper = std::move(mime);

    _webServerDispatcher = new WebServerDispatcher(dispconfig);
    _webServerDispatcher->threadPool().addCapacity(50);

    WebServerDispatcher::VirtualPath vPath("/", Config::instance().getString("web.server.Public", "/usr/local/brain/dist/"));
    _webServerDispatcher->addVirtualPath(vPath);

    WebServerDispatcher::VirtualPath apilog;
    apilog.path = "/api/log";
    apilog.cors.enable = true;
    apilog.cors.allowOrigin = "*";
    apilog.pFactory = new LogTransportHandlerFactory();
    _webServerDispatcher->addVirtualPath(apilog);

    try {
        _httpServer = new Poco::Net::HTTPServer(
            new WebServerRequestHandlerFactory(*_webServerDispatcher, false), _webServerDispatcher->threadPool(),
            Poco::Net::ServerSocket(Poco::UInt16(Config::instance().getInt("web.server.port", 5555))), _httpServerParams);

        _httpServer->start();
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }

    NLOG(info) << "Startup complete. Agent WebService";
}