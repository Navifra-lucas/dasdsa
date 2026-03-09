#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <core_agent/data/robot_info.h>
#include <nc_brain_agent/data/nc_brain_map.h>
#include <nc_brain_agent/nc_robot_agent.h>
#include <nc_brain_agent/utils/nc_agent_utils.h>
#include "nc_brain_agent/net/nc_rest_api_utils.h"
#include <tf/tf.h>
#include "core_agent/data/robot_status.h"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <Poco/JSON/Parser.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Array.h>
#include <Poco/Dynamic/Var.h>
#include <unordered_map>
#include <unordered_set>

namespace fs = std::filesystem;   // alias 선언
using Poco::DynamicStruct;
using Poco::File;
using Poco::Path;
using Poco::JSON::Parser;

using namespace Poco::JSON;
using namespace NaviFra;

const std::string NcBrainMap::KEY = "NcBrainMap";

NcBrainMap::NcBrainMap()
{
    main_revision_ = "";
    local_revision_ = "";

    /// 아래 경로 상대 경로하면 에러 생김 절대 경로로
    dirpath_ = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/configs/map/latest/";
    filepath_ = dirpath_ + "map.json";
    teaching_ = "teaching_";

    std::string slam_map_path = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/configs/map/";
    std::string slam_map_file = slam_map_path + "temp.json";

    if (File(slam_map_file).exists()) {
        slam_map_lastmodified_ = Poco::File(slam_map_file).getLastModified();
    }
    else {
        Poco::Timestamp now;
        slam_map_lastmodified_ = now;
    }
}

NcBrainMap::~NcBrainMap()
{
}


// ---- 헬퍼: 들어온 JSON → Array (id,key,acs_map_id만 추출) ----
static Poco::JSON::Array::Ptr ExtractMinimalAreaArray(Poco::JSON::Object::Ptr root) {
    Poco::JSON::Array::Ptr result = new Poco::JSON::Array();

    auto arr = root->getArray("list");
    if (!arr) return result;

    for (size_t i = 0; i < arr->size(); ++i) {
        auto obj = arr->getObject(i);
        if (!obj) continue;

        if (obj->has("id") && obj->has("key") && obj->has("acs_map_id")) {
            Poco::JSON::Object::Ptr minimal = new Poco::JSON::Object();
            minimal->set("id", obj->get("id").convert<std::string>());
            try {
                minimal->set("key", obj->get("key").convert<std::string>());
            } catch (...) {
                minimal->set("key", std::to_string(obj->get("key").convert<int>()));
            }
            minimal->set("acs_map_id", obj->get("acs_map_id").convert<std::string>());
            result->add(minimal);
        }
    }
    return result;
}

// ---- 헬퍼: 파일에서 Array 로드 ----
static Poco::JSON::Array::Ptr LoadAreasArray(const std::string& areas_path) {
    std::ifstream ifs(areas_path);
    if (!ifs.is_open()) return new Poco::JSON::Array();

    std::stringstream buffer; buffer << ifs.rdbuf();
    Poco::JSON::Parser parser;
    auto var = parser.parse(buffer.str());
    auto root = var.extract<Poco::JSON::Object::Ptr>();
    return root->getArray("areas");
}

// ---- 헬퍼: Array 저장 ----
static void SaveAreasArray(const std::string& areas_path, Poco::JSON::Array::Ptr arr) {
    Poco::JSON::Object::Ptr root = new Poco::JSON::Object();
    root->set("areas", arr);

    std::ofstream ofs(areas_path, std::ios::trunc);
    std::ostringstream oss;
    root->stringify(oss, 2);
    ofs << oss.str();
}

// ---- 헬퍼: Array 비교 (집합으로 비교) ----
static bool AreasEqual(Poco::JSON::Array::Ptr A, Poco::JSON::Array::Ptr B) {
    auto toSet = [](Poco::JSON::Array::Ptr arr) {
        std::set<std::tuple<std::string,std::string,std::string>> s;
        if (!arr) return s;
        for (size_t i=0;i<arr->size();++i) {
            auto obj = arr->getObject(i);
            if (!obj) continue;
            s.emplace(
                obj->get("id").convert<std::string>(),
                obj->get("key").convert<std::string>(),
                obj->get("acs_map_id").convert<std::string>()
            );
        }
        return s;
    };
    return toSet(A) == toSet(B);
}

static std::string AreaMapFilePath(const std::string& dirpath, const std::string& area_id) {
    return dirpath + "map_" + area_id + ".json";
}

// area_id, acs_map_id로 파일 upsert
static void RequestMapForArea(const std::string& area_id, const std::string& acs_map_id) {
    std::string dirpath = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/configs/map/latest/";
    fs::create_directories(dirpath);

    const std::string fpath = AreaMapFilePath(dirpath, area_id);
   
    // 파일이 있으면 읽어서 acs_map_id 변경 여부 확인
    NLOG(info) << "[MAP-REQUEST] area_id=" << area_id << ", acs_map_id=" << acs_map_id;

    // 맵 요청
    auto token = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getToken();
    auto res_map = Net::HTTP::get("/scan-maps/"+ acs_map_id, token);

    if (std::get<1>(res_map) == Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) 
    {
        NLOG(info) << "[MAP-REQUEST] Map for area_id=" << area_id << " fetched successfully.";
        Poco::JSON::Object::Ptr map = JSONParse(std::get<0>(res_map));
        InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->localMapUpdateDir(map->getObject("item"), fpath); // save map json
    }
    else
    {
        NLOG(error) << "[MAP-REQUEST] Failed to fetch map for area_id=" << area_id
                    << ", status=" << (int)std::get<1>(res_map)
                    << ", reason=" << std::get<2>(res_map);
    }
}

static std::string GetCurrentAreaID()
{
    const std::string dirpath = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/configs/map/latest/";
    const std::string mapPath = dirpath + "map.json";

    // 1) map.json 읽어서 현재 map_id 추출
    std::ifstream mifs(mapPath);
    if (!mifs.is_open()) {
        NLOG(error) << "[MAP] Cannot open " << mapPath;
        return "";
    }
    std::stringstream mbuf; mbuf << mifs.rdbuf();

    Poco::JSON::Parser parser;
    Poco::Dynamic::Var mvar;
    try {
        mvar = parser.parse(mbuf.str());
    } catch (...) {
        NLOG(error) << "[MAP] Failed to parse map.json";
        return "";
    }
    auto mroot = mvar.extract<Poco::JSON::Object::Ptr>();
    if (!mroot || !mroot->has("id")) {
        NLOG(error) << "[MAP] map.json has no 'id'";
        return "";
    }
    const std::string current_map_id = mroot->get("id").convert<std::string>();
    NLOG(info) << "[MAP] current map id=" << current_map_id;

    // 2) map_*.json 순회
    if (!fs::exists(dirpath)) {
        NLOG(warning) << "[MAP] Directory not found: " << dirpath;
        return "";
    }
    
    for (const auto& entry : fs::directory_iterator(dirpath)) {
        if (!entry.is_regular_file()) continue;

        const std::string fname = entry.path().filename().string();
        if (fname == "map.json") continue;
        if (fname.rfind("map_", 0) != 0) continue;               // "map_"로 시작
        if (entry.path().extension() != ".json") continue;        // .json 확장자

        try {
            std::ifstream ifs(entry.path());
            std::stringstream buf; buf << ifs.rdbuf();

            Poco::Dynamic::Var var = parser.parse(buf.str());
            auto root = var.extract<Poco::JSON::Object::Ptr>();
            if (!root) continue;

            // 내부 맵 id가 현재 map_id와 같으면 area_id 반환
            if (root->has("id")) {
                const std::string file_map_id = root->get("id").convert<std::string>();
                if (file_map_id == current_map_id) {
                    // 파일명에서 추출: map_<area_id>.json
                    const std::string prefix = "map_";
                    const std::string suffix = ".json";
                    std::string nameNoExt = entry.path().filename().string();
                    if (nameNoExt.size() > prefix.size() + suffix.size() &&
                        nameNoExt.rfind(suffix) == nameNoExt.size() - suffix.size()) {
                        // strip prefix/suffix
                        std::string area_id = nameNoExt.substr(prefix.size(),
                                                               nameNoExt.size() - prefix.size() - suffix.size());
                        NLOG(info) << "[MAP] area_id(from filename)=" << area_id
                                   << " matched for map id=" << current_map_id;
                        return area_id;
                    }

                    // 둘 다 실패하면 경고
                    NLOG(warning) << "[MAP] Matched file " << fname
                               << " has no 'area_id' field and filename parse failed.";
                }
            }
        } catch (...) {
            NLOG(warning) << "[MAP] Failed to parse " << fname;
        }
    }

    NLOG(warning) << "[MAP] No area_id found for current map id=" << current_map_id;
    return "";
}

static bool ReadMapFileId(const std::string& fpath, std::string& out_map_id) {
    try {
        std::ifstream ifs(fpath);
        if (!ifs.is_open()) return false;
        std::stringstream buf; buf << ifs.rdbuf();
        Poco::JSON::Parser parser;
        auto var  = parser.parse(buf.str());
        auto root = var.extract<Poco::JSON::Object::Ptr>();
        if (!root || !root->has("id")) return false;
        out_map_id = root->get("id").convert<std::string>();
        return true;
    } catch (...) {
        return false;
    }
}

// area 목록을 바탕으로 map_(area_id).json들과의 정합성 점검 및 정리
static void ReconcileAreasAndMapFiles(const std::string& dirpath, Poco::JSON::Array::Ptr areasArr)
{
    // 1) areas.json 배열로부터 area_id -> acs_map_id 맵 만들기, 그리고 area_id 집합
    std::unordered_map<std::string, std::string> area_to_acs;
    std::unordered_set<std::string> valid_area_ids;

    if (areasArr) {
        for (size_t i = 0; i < areasArr->size(); ++i) {
            auto obj = areasArr->getObject(i);
            if (!obj) continue;
            const std::string area_id = obj->get("id").convert<std::string>();
            const std::string acs_id  = obj->get("acs_map_id").convert<std::string>();
            if (!area_id.empty() && !acs_id.empty()) {
                area_to_acs[area_id] = acs_id;
                valid_area_ids.insert(area_id);
            }
        }
    }

    // 2) 각 area에 대해 map_(area_id).json 존재/내용 확인
    for (const auto& kv : area_to_acs) {
        const std::string& area_id = kv.first;
        const std::string& acs_id  = kv.second;
        const std::string  fpath   = AreaMapFilePath(dirpath, area_id);

        if (!fs::exists(fpath)) {
            // 파일이 없으면 새로 요청(생성/업데이트 책임은 RequestMapForArea가 가짐)
            NLOG(info) << "[RECONCILE] missing map file for area_id=" << area_id
                       << " -> request map (acs_map_id=" << acs_id << ")";
            RequestMapForArea(area_id, acs_id);
            continue;
        }

        // 파일이 있으면 내부 "id"가 acs_map_id와 같은지 확인
        std::string file_map_id;
        if (!ReadMapFileId(fpath, file_map_id)) {
            NLOG(warning) << "[RECONCILE] unreadable or invalid map file: " << fpath
                       << " -> rewrite by request (acs_map_id=" << acs_id << ")";
            RequestMapForArea(area_id, acs_id);
            continue;
        }

        if (file_map_id != acs_id) {
            NLOG(warning) << "[RECONCILE] map file id mismatch for " << fpath
                       << " (file id=" << file_map_id
                       << ", expected=" << acs_id << ") -> refresh";
            RequestMapForArea(area_id, acs_id);
        } else {
            NLOG(info) << "[RECONCILE] OK: " << fpath << " (id=" << file_map_id << ")";
        }
    }

    // 3) 고아 파일 정리: areas.json에 없는 map_*.json 삭제
    if (fs::exists(dirpath)) {
        for (const auto& entry : fs::directory_iterator(dirpath)) {
            if (!entry.is_regular_file()) continue;
            const std::string fname = entry.path().filename().string();
            if (fname == "map.json") continue;
            if (entry.path().extension() != ".json") continue;
            if (fname.rfind("map_", 0) != 0) continue; // "map_"으로 시작하는 파일만

            // 파일명에서 area_id 추출: map_<area_id>.json
            const std::string prefix = "map_";
            const std::string suffix = ".json";
            if (fname.size() <= prefix.size() + suffix.size()) continue;

            std::string area_id_from_name = fname.substr(
                prefix.size(),
                fname.size() - prefix.size() - suffix.size()
            );

            if (valid_area_ids.find(area_id_from_name) == valid_area_ids.end()) {
                // areas.json에 없는 map 파일 -> 삭제
                std::error_code ec;
                if (fs::remove(entry.path(), ec)) {
                    NLOG(info) << "[RECONCILE] removed orphan map file: " << fname;
                } else {
                    if (ec) NLOG(warning) << "[RECONCILE] failed to remove " << fname << " : " << ec.message();
                }
            }
        }
    }
}

static void SavePortalsToFile(const std::string& portals_path, Poco::JSON::Array::Ptr portalsArr) {
    Poco::JSON::Object::Ptr root = new Poco::JSON::Object();
    root->set("portals", portalsArr);

    std::ofstream ofs(portals_path, std::ios::trunc);
    std::ostringstream oss;
    root->stringify(oss, 2);   // 2 → pretty print
    ofs << oss.str();

    NLOG(info) << "[PORTALS] Saved " << portalsArr->size() << " portals to " << portals_path;
}

static Poco::JSON::Array::Ptr ExtractPortalsArray(Poco::JSON::Object::Ptr root) {
    Poco::JSON::Array::Ptr result = new Poco::JSON::Array();

    auto arr = root->getArray("list");
    if (!arr) return result;

    for (size_t i = 0; i < arr->size(); ++i) {
        auto portalObj = arr->getObject(i);
        if (!portalObj) continue;

        Poco::JSON::Object::Ptr minimal = new Poco::JSON::Object();
        minimal->set("id",        portalObj->get("id").convert<std::string>());
        minimal->set("title",     portalObj->get("title").convert<std::string>());
        minimal->set("created_at",portalObj->get("created_at").convert<std::string>());
        minimal->set("updated_at",portalObj->get("updated_at").convert<std::string>());

        // deleted_at은 null일 수 있으므로 string으로 안전하게 처리
        if (portalObj->isNull("deleted_at"))
            minimal->set("deleted_at", Poco::Dynamic::Var());
        else
            minimal->set("deleted_at", portalObj->get("deleted_at").convert<std::string>());

        // nodes 배열 추출
        Poco::JSON::Array::Ptr nodesArr = new Poco::JSON::Array();
        if (portalObj->has("nodes")) {
            auto nArr = portalObj->getArray("nodes");
            if (nArr) {
                for (size_t j = 0; j < nArr->size(); ++j) {
                    auto nodeObj = nArr->getObject(j);
                    if (!nodeObj) continue;

                    Poco::JSON::Object::Ptr n = new Poco::JSON::Object();
                    n->set("area_id", nodeObj->get("area_id").convert<std::string>());
                    n->set("node_id", nodeObj->get("node_id").convert<std::string>());
                    nodesArr->add(n);
                }
            }
        }
        minimal->set("nodes", nodesArr);

        result->add(minimal);
    }

    return result;
}


bool NcBrainMap::initialize()
{
    try {
        LOG_INFO("initialize Brain Map data");

        File path(dirpath_);
        if (path.exists() != true) {
            LOG_INFO("Map Store Directory is Not exits. create Directory");
            path.createDirectories();
            // LOG_INFO("%s Created Directory.", dirpath_.c_str());
            return false;
        }

        File f(filepath_);
        if (f.exists() != true) {
            LOG_ERROR("Map File is Not exits.");
            return false;
        }
        else {
            return loadLocalMap();
        }
    }
    catch (std::exception ex) {
        LOG_ERROR("%s", ex.what());
    }

    return true;
}

bool NcBrainMap::loadLocalMap()
{
    try {
        map_data_ = loadJSON(filepath_);
        if (map_data_.get() == nullptr)
            return false;
        DynamicStruct ds = *map_data_;

        if (ds["revision"].isEmpty()) {
            LOG_ERROR("Map File is Not exits revision.");
        }
        else {
            local_revision_ = ds["revision"].toString();
        }
    }
    catch (std::exception ex) {
        LOG_ERROR("%s", ex.what());
        return false;
    }
    return true;
}

void NcBrainMap::localMapUpdate(Object::Ptr map)
{
    try {
        NLOG(info) << "dirpath_ " << dirpath_;
        if (!File(dirpath_).exists()) {
            File(dirpath_).createDirectories();
        }
        NLOG(info) << "filepath_ " << filepath_;
        saveJSON(filepath_, map);
    }
    catch (std::exception ex) {
        LOG_ERROR("%s", ex.what());
    }
}

static std::string AreaMapFilePath(const MapPaths& p, const std::string& area_id) {
    return p.dir + "map_" + area_id + ".json";
}

// --- file helpers ---
static bool ReadAll(const std::string& path, std::string& out) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;
    std::ostringstream oss; oss << ifs.rdbuf(); out = oss.str();
    return true;
}
static bool WriteAll(const std::string& path, const std::string& content) {
    fs::create_directories(fs::path(path).parent_path());
    std::ofstream ofs(path, std::ios::trunc);
    if (!ofs.is_open()) return false;
    ofs << content;
    return true;
}

// current_area.txt 읽기/쓰기
static std::optional<std::string> ReadCurrentAreaId(const MapPaths& p) {
    std::string s;
    if (!ReadAll(p.cur_txt, s)) return std::nullopt;
    // 공백/개행 제거
    while(!s.empty() && (s.back()=='\n' || s.back()=='\r' || s.back()==' ' || s.back()=='\t')) s.pop_back();
    return s.empty() ? std::optional<std::string>{} : std::optional<std::string>{s};
}
static bool WriteCurrentAreaId(const MapPaths& p, const std::string& area_id) {
    return WriteAll(p.cur_txt, area_id + "\n");
}

// map_(area_id).json → "id"(=map_id) 읽기
static std::optional<std::string> ReadMapIdFromAreaMapFile(const MapPaths& p, const std::string& area_id) {
    const std::string fpath = AreaMapFilePath(p, area_id);
    std::string body;
    if (!ReadAll(fpath, body)) return std::nullopt;
    try {
        Poco::JSON::Parser parser;
        auto var  = parser.parse(body);
        auto root = var.extract<Poco::JSON::Object::Ptr>();
        if (!root || !root->has("id")) return std::nullopt;
        return root->get("id").convert<std::string>();
    } catch (...) {
        return std::nullopt;
    }
}

// map.json → "id"(=map_id) 읽기
static std::optional<std::string> ReadMapJsonId(const MapPaths& p) {
    std::string body;
    if (!ReadAll(p.map_json, body)) return std::nullopt;
    try {
        Poco::JSON::Parser parser;
        auto var  = parser.parse(body);
        auto root = var.extract<Poco::JSON::Object::Ptr>();
        if (!root || !root->has("id")) return std::nullopt;
        return root->get("id").convert<std::string>();
    } catch (...) {
        return std::nullopt;
    }
}

// 디렉터리에서 첫 번째 area 후보 찾기 (map_*.json)
static std::optional<std::string> FindFirstAreaId(const MapPaths& p) {
    if (!fs::exists(p.dir)) return std::nullopt;
    for (const auto& e : fs::directory_iterator(p.dir)) {
        if (!e.is_regular_file()) continue;
        const auto fn = e.path().filename().string();
        if (fn.rfind("map_", 0) != 0 || e.path().extension() != ".json") continue;
        // map_<area_id>.json → area_id 추출
        const std::string prefix = "map_";
        const std::string suffix = ".json";
        if (fn.size() <= prefix.size()+suffix.size()) continue;
        std::string area_id = fn.substr(prefix.size(), fn.size()-prefix.size()-suffix.size());
        return area_id;
    }
    return std::nullopt;
}

// map_(area_id).json을 map.json으로 교체(복사)
static bool ReplaceMapJsonWithArea(const MapPaths& p, const std::string& area_id) {
    const std::string src = AreaMapFilePath(p, area_id);
    if (!fs::exists(src)) return false;
    // 파일 내용 그대로 복사(메타는 무시)
    std::error_code ec;
    fs::copy_file(src, p.map_json, fs::copy_options::overwrite_existing, ec);
    if (ec) {
        // fallback: 수동 복사
        std::string body;
        if (!ReadAll(src, body)) return false;
        bool b_result = WriteAll(p.map_json, body);
    }

    // map update topic 발행
    auto haveMapId = ReadMapJsonId(p);
    NLOG(info) << "[MAP] Replaced map.json with mapid " << *haveMapId;

    updateBrainMap();

    return true;
}

// current area 기준으로 map.json이 최신인지 확인하고 필요시 교체
static bool EnsureMapJsonMatchesCurrentArea(const MapPaths& p) {
    auto curAreaOpt = ReadCurrentAreaId(p);
    if (!curAreaOpt || curAreaOpt->empty()) {
        // 초기화 시도
        return false;
    }
    const std::string& curArea = *curAreaOpt;

    auto wantedMapId = ReadMapIdFromAreaMapFile(p, curArea);
    if (!wantedMapId) {
        // 해당 area의 map 파일이 없다 → 맵 요청 트리거가 필요하다면 여기서 호출
        // RequestMapForArea(curArea, ...);
        return false;
    }

    auto haveMapId = ReadMapJsonId(p);
    if (!haveMapId || *haveMapId != *wantedMapId) {
        // 불일치 → 최신 area 맵 파일로 교체
        return ReplaceMapJsonWithArea(p, curArea);
    }
    return true; // 이미 최신
}

// MQTT/UI 이벤트로 current area가 바뀔 때 호출
static bool SwitchCurrentArea(const MapPaths& p, const std::string& new_area_id) {
    if (new_area_id.empty()) return false;
    if (!fs::exists(AreaMapFilePath(p, new_area_id))) {
        // 필요 시 맵 요청
        // RequestMapForArea(new_area_id, ...);
        return false;
    }
    if (!WriteCurrentAreaId(p, new_area_id)) return false;
    return ReplaceMapJsonWithArea(p, new_area_id);
}

static std::optional<std::string> ReadFirstAreaIdFromAreasJson(const MapPaths& p) {
    std::string body;
    if (!ReadAll(p.areas_path, body)) return std::nullopt;
    try {
        Poco::JSON::Parser parser;
        auto var  = parser.parse(body);
        auto root = var.extract<Poco::JSON::Object::Ptr>();
        if (!root || !root->has("areas")) return std::nullopt;

        auto arr = root->getArray("areas");
        if (!arr || arr->size() == 0) return std::nullopt;

        auto first = arr->getObject(0);
        if (!first || !first->has("id")) return std::nullopt;

        return first->get("id").convert<std::string>();
    } catch (...) {
        return std::nullopt;
    }
}

// current_area.txt가 없으면 areas.json의 첫 area로 설정하고 map.json 동기화
static bool EnsureCurrentAreaInitializedFromAreas(const MapPaths& p) {
    auto cur = ReadCurrentAreaId(p);
    if (cur && !cur->empty()) return true; // 이미 있음

    auto firstAreaId = ReadFirstAreaIdFromAreasJson(p);
    if (!firstAreaId) {
        NLOG(warning) << "[AREA] areas.json is missing or empty; cannot initialize current area.";
        return false;
    }

    // 선택된 area의 map 파일 존재 확인 (없으면 필요한 경우 요청 훅 연결 가능)
    const std::string src = AreaMapFilePath(p, *firstAreaId);
    if (!fs::exists(src)) {
        NLOG(warning) << "[AREA] map file not found for first area: " << src
                   << " (consider RequestMapForArea)";
        return false;
    }

    if (!WriteCurrentAreaId(p, *firstAreaId)) return false;
    return ReplaceMapJsonWithArea(p, *firstAreaId);
}

void NcBrainMap::ChageCurrentArea(const std::string& area_id)
{
    NLOG(info) << "Changing current area to: " << area_id;
    std::string s_path_ = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/configs/map/latest/";
    std::string srcFile = s_path_+"map_" + area_id + ".json";
    std::string dstFile = s_path_+"map.json";

    try {
        NLOG(info) << "Changing current area to: " << area_id;

        if (!fs::exists(srcFile)) {
            NLOG(error) << "Source file does not exist: " << srcFile;
            return;
        }

        fs::copy_file(srcFile, dstFile, fs::copy_options::overwrite_existing);
        NLOG(info) << "Copied " << srcFile << " -> " << dstFile;
        MapPaths paths;
        auto haveMapId = ReadMapJsonId(paths);
        NLOG(info) << "[MAP] Replaced map.json with mapid " << *haveMapId;
    
        updateBrainMap();
        std::string current_area_id = GetCurrentAreaID();

        if( current_area_id.empty()) {
            NLOG(error) << "[MAP] No current area_id found. update map";
        }
        // robot info 업데이트
        std::string areas_path = s_path_ + "areas.json";

        auto storedArr = LoadAreasArray(areas_path);

        std::vector<std::string> vec_area_uuid;
        std::vector<std::string> vec_map_uuid;
        
        if (storedArr) {
            for (size_t i = 0; i < storedArr->size(); ++i) {
                auto obj = storedArr->getObject(i);
                if (!obj) continue;
        
                if (obj->has("id")) {
                    vec_area_uuid.push_back(obj->get("id").convert<std::string>());
                }
                if (obj->has("acs_map_id")) {
                    vec_map_uuid.push_back(obj->get("acs_map_id").convert<std::string>());
                }
            }
        }       
        InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY)->setArea(current_area_id, vec_area_uuid, vec_map_uuid);
    
    }
    catch (const fs::filesystem_error& e) {
        NLOG(error) << "Filesystem error: " << e.what();
    }

}

void NcBrainMap::updateAreaPortal()
{
    NLOG(info) << "Updating area portal information";
    std::string s_path_ = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/configs/map/latest/";
    std::string areas_path = s_path_ + "areas.json";
    fs::create_directories(s_path_);

    auto token = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getToken();
    auto res_areas = Net::HTTP::get("/scan-maps/areas", token);

    // area 정보 처리
    auto areasData = JSONParse(std::get<0>(res_areas));
    NLOG(info) << "Area data received: " << std::get<0>(res_areas);
    
    // 들어온 JSON에서 필요한 3개 필드만 뽑기
    Poco::JSON::Array::Ptr incomingArr = ExtractMinimalAreaArray(areasData);

    if (!fs::exists(areas_path)) 
    {
        // 요구사항 2: 파일이 없다면 저장하고 전체 맵 요청
        NLOG(info) << "[AREAS] areas.json not found. Creating and requesting all maps.";
        SaveAreasArray(areas_path, incomingArr);

        for (size_t i=0;i<incomingArr->size();++i) {
            auto obj = incomingArr->getObject(i);
            RequestMapForArea(obj->get("id").convert<std::string>(), obj->get("acs_map_id").convert<std::string>());
        }
    } 
    else {
        // 요구사항 1: 파일이 있다면 비교
        auto storedArr = LoadAreasArray(areas_path);
        auto incomingArr = ExtractMinimalAreaArray(areasData);
        
        if (!AreasEqual(storedArr, incomingArr)) {
            NLOG(warning) << "[AREAS] Incoming differs from stored.";

            // 집합 구성 (id, key, acs_map_id)
            auto toSet = [](Poco::JSON::Array::Ptr arr) {
                std::set<std::tuple<std::string,std::string,std::string>> s;
                if (!arr) return s;
                for (size_t i=0;i<arr->size();++i) {
                    auto obj = arr->getObject(i);
                    s.emplace(
                        obj->get("id").convert<std::string>(),
                        obj->get("key").convert<std::string>(),
                        obj->get("acs_map_id").convert<std::string>()
                    );
                }
                return s;
            };
            auto S = toSet(storedArr);
            auto I = toSet(incomingArr);
            NLOG(info) << "[AREAS] Stored size: " << S.size() << ", Incoming size: " << I.size();

            // 빠른 lookup을 위해 area_id -> acs_map_id 맵 구성
            auto toMap = [](Poco::JSON::Array::Ptr arr) {
                std::unordered_map<std::string, std::string> m; // area_id -> acs_map_id
                if (!arr) return m;
                for (size_t i=0;i<arr->size();++i) {
                    auto obj = arr->getObject(i);
                    auto id  = obj->get("id").convert<std::string>();
                    auto acs = obj->get("acs_map_id").convert<std::string>();
                    m[id] = acs;
                }
                return m;
            };
            auto storedMap   = toMap(storedArr);
            auto incomingMap = toMap(incomingArr);

            std::string dirpath = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/configs/map/latest/";
            fs::create_directories(dirpath);

            // 1) 신규/변경된 항목 → 파일 upsert + 맵 요청
            for (const auto& t : I) {
                if (!S.count(t)) {
                    const std::string& id  = std::get<0>(t);
                    const std::string& key = std::get<1>(t);
                    const std::string& acs = std::get<2>(t);

                    // 변경 유형: 신규 추가이거나, 기존 id의 acs_map_id 변경
                    bool isChangedAcs = (storedMap.count(id) && storedMap[id] != acs);
                    NLOG(info) << "[AREAS] + New/Changed: id=" << id << ", key=" << key << ", acs_map_id=" << acs
                            << (isChangedAcs ? " (acs_map_id changed)" : "");

                    // 요구사항 2: acs_map_id 가 달라졌다면 새 정보로 파일 채우기(Upsert가 처리)
                    RequestMapForArea(id, acs);
                }
            }

            // 2) 제거된 항목 → 파일 삭제
            for (const auto& t : S) {
                if (!I.count(t)) {
                    const std::string& id  = std::get<0>(t);
                    const std::string& key = std::get<1>(t);
                    const std::string& acs = std::get<2>(t);

                    NLOG(info) << "[AREAS] - Removed: id=" << id << ", key=" << key << ", acs_map_id=" << acs;

                    // 요구사항 1: 과거에 있던 area가 사라졌다면 map_(area_id).json 삭제
                    const std::string fpath = AreaMapFilePath(dirpath, id);
                    std::error_code ec;
                    if (fs::exists(fpath) && fs::remove(fpath, ec)) {
                        NLOG(info) << "[MAP-FILE] removed " << fpath;
                    } else {
                        if (ec) NLOG(warning) << "[MAP-FILE] failed to remove " << fpath << " : " << ec.message();
                    }
                }
            }

            // 3) areas.json을 새 상태로 반영
            NLOG(info) << "[AREAS] areas.json updated after requesting maps.";
        }
        else 
        {
            NLOG(info) << "[AREAS] areas.json is up-to-date.";
        }
        SaveAreasArray(areas_path, incomingArr);
    }

    ReconcileAreasAndMapFiles(s_path_, incomingArr);
    auto storedArr = LoadAreasArray(areas_path);

    std::vector<std::string> vec_area_uuid;
    std::vector<std::string> vec_map_uuid;
    
    if (storedArr) {
        for (size_t i = 0; i < storedArr->size(); ++i) {
            auto obj = storedArr->getObject(i);
            if (!obj) continue;
    
            if (obj->has("id")) {
                vec_area_uuid.push_back(obj->get("id").convert<std::string>());
            }
            if (obj->has("acs_map_id")) {
                vec_map_uuid.push_back(obj->get("acs_map_id").convert<std::string>());
            }
        }
    }        

    // current map id가 변경되었는지 확인
    MapPaths paths;
    EnsureCurrentAreaInitializedFromAreas(paths);
    EnsureMapJsonMatchesCurrentArea(paths);

    std::string current_area_id = GetCurrentAreaID();

    if( current_area_id.empty()) {
        NLOG(error) << "[MAP] No current area_id found. update map";
    }

    // robot info 업데이트
    InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY)->setArea(current_area_id, vec_area_uuid, vec_map_uuid);

    // portal 정보 처리
    auto res_portals = Net::HTTP::get("/scan-maps/portals", token);
    auto portalsData = JSONParse(std::get<0>(res_portals));
    NLOG(info) << "Portal data received: " << std::get<0>(res_portals);

    Poco::JSON::Array::Ptr portalsArr = ExtractPortalsArray(portalsData);

    // 저장 경로
    std::string dirpath = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/configs/map/latest/";
    fs::create_directories(dirpath);
    std::string portals_path = dirpath + "portals.json";

    // 파일 저장
    SavePortalsToFile(portals_path, portalsArr);
}

void NcBrainMap::localMapUpdateDir(Object::Ptr map, std::string s_filepath)
{
    try {
        NLOG(info) << "dirpath_ " << dirpath_;
        if (!File(dirpath_).exists()) {
            File(dirpath_).createDirectories();
        }
        NLOG(info) << "s_filepath " << s_filepath;
        saveJSON(s_filepath, map);
    }
    catch (std::exception ex) {
        LOG_ERROR("%s", ex.what());
    }
}

void NcBrainMap::setMainRevision(std::string revision)
{
    // 필요에 따라 mutex 걸어야 될 수도?
    main_revision_ = revision;
}

// 쓸때가 있나?
void NcBrainMap::setLocalRevision(std::string revision)
{
    Poco::JSON::Object::Ptr map_data = loadJSON(filepath_);
    if (map_data.get() == nullptr)
        return;
    local_revision_ = revision;
    map_data->set("revision", revision);

    localMapUpdate(map_data);
}

std::string NcBrainMap::getLocalRevision()
{
    if (!File(filepath_).exists())
        return "";
    if (loadLocalMap())
        return local_revision_;
    else
        return "";
}

Object::Ptr NcBrainMap::getSlamMap()
{
    try {
        std::string slam_map_file = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/configs/map/" + "temp.json";
        File f(slam_map_file);
        if (f.exists()) {
            return loadJSON(slam_map_file);
        }
        else {
            return nullptr;
        }
    }
    catch (std::exception ex) {
        LOG_ERROR("%s", ex.what());
    }
    return nullptr;
}

bool NcBrainMap::isSlamFileModified()
{
    std::string slam_map_path = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/configs/map/";

    if (!File(slam_map_path).exists()) {
        File(slam_map_path).createDirectories();
    }
    std::string slam_map_file = slam_map_path + "temp.json";

    if (!File(slam_map_file).exists()) {
        LOG_ERROR("slam_map_file File Not exists!");
        return false;
    }

    Poco::Timestamp lastmodified = Poco::File(slam_map_file).getLastModified();
    if (slam_map_lastmodified_ != lastmodified) {
        slam_map_lastmodified_ = lastmodified;
        return true;
    }
    return false;
}

size_t NcBrainMap::getKeyNodesSize()
{
    size_t keynode_size = 0;
    try {
        LOG_INFO("getKeyNodesSize");
        if (!File(filepath_).exists())
            return keynode_size;

        if (!loadLocalMap()) {
            return keynode_size;
        }

        if (map_data_->has("nodes")) {
            Poco::JSON::Array::Ptr nodes = map_data_->getArray("nodes");
            for (size_t index = 0; index < nodes->size(); index++) {
                if (nodes->getObject(index)->has("node_type_cd") &&
                    nodes->getObject(index)->get("node_type_cd").convert<std::string>() != "D40009") {
                    keynode_size++;
                }
            }
        }
    }
    catch (std::exception ex) {
        LOG_ERROR("%s", ex.what());
    }

    return keynode_size;
}

void NcBrainMap::teachingJsonInit(std::string revision)
{
    assert(!revision.empty());

    try {
        LOG_INFO("teachingJsonInit");
        std::string teach_file = dirpath_ + "teaching.json";
        std::string teach_file_dir = dirpath_ + "teaching_" + revision + ".json";

        if (File(teach_file_dir).exists()) {
            Poco::JSON::Object::Ptr teach = loadJSON(teach_file_dir);
            DynamicStruct olddata = *teach;
            DynamicStruct current = *map_data_;

            Object data;
            Poco::JSON::Array nodes;

            for (auto node : olddata["nodes"]) {
                for (auto mapNode : current["nodes"]) {
                    if (node["id"] == mapNode["id"]) {
                        nodes.add(node);
                    }
                }
            }

            data.set("nodes", nodes);
            if (!File(dirpath_).exists()) {
                File(dirpath_).createDirectories();
            }

            if (!data.isNull("nodes"))
                saveJSON(teach_file_dir, data);

            LOG_INFO("teaching_%s.json File exists!", revision.c_str());
            File(teach_file_dir).copyTo(teach_file);
        }

        Object data;
        Poco::JSON::Array nodes;

        std::vector<std::string> files, teachingfiles, versions;
        Poco::File upload(dirpath_);
        upload.list(files);

        std::string searchWord = "teach";
        std::copy_if(files.begin(), files.end(), std::back_inserter(teachingfiles), [&searchWord](const std::string& element) {
            return element.find(searchWord) != std::string::npos && element.compare("teaching.json") != 0;
        });

        for (const std::string& file : teachingfiles) {
            // 파일규칙은 teaching_revision.json 이라서 revsion 만 가져 오기 위해 아래 코드를 수행 함
            Poco::StringTokenizer tokenizer(file, "_", Poco::StringTokenizer::TOK_TRIM | Poco::StringTokenizer::TOK_IGNORE_EMPTY);
            std::string version = tokenizer[1];

            version.replace(version.length() - 5, version.length(), "");
            versions.push_back(version);
        }

        // 현재리비전보다 작은 애들 해당하는 요소를 삭제
        versions.erase(
            std::remove_if(
                versions.begin(), versions.end(),
                [&revision](std::string element) {
                    if (std::atoi(element.c_str()) < std::atoi(revision.c_str()))
                        return true;
                    else
                        return false;
                }),
            versions.end());

        std::sort(versions.begin(), versions.end(), [&](std::string left, std::string right) {
            return std::atoi(left.c_str()) < std::atoi(right.c_str());
        });

        std::string strTemp;
        for (const auto& el : versions) {
            strTemp += el + ", ";
        }

        LOG_INFO("file_List : %s", strTemp.c_str());

        if (versions.size() > 0) {
            std::string target_teach_json = dirpath_ + "teaching_" + versions.back() + ".json";

            if (File(target_teach_json).exists()) {
                Poco::JSON::Object::Ptr teach = loadJSON(target_teach_json);
                DynamicStruct olddata = *teach;
                DynamicStruct current = *map_data_;
                Object data;
                Poco::JSON::Array nodes;
                for (auto node : olddata["nodes"]) {
                    for (auto mapNode : current["nodes"]) {
                        if (node["id"] == mapNode["id"])
                            nodes.add(node);
                    }
                }

                if (!File(dirpath_).exists()) {
                    File(dirpath_).createDirectories();
                }
                if (!data.isNull("nodes")) {
                    saveJSON(teach_file_dir, data);
                    LOG_INFO("teaching_%s.json File exists!", revision.c_str());
                    File(teach_file_dir).copyTo(teach_file);
                }
            }
        }
        else {
            Object data;
            Array nodes;
            data.set("nodes", nodes);
            saveJSON(teach_file, data);
        }
    }
    catch (std::exception ex) {
        LOG_ERROR("BRAIN Teaching file init error reason : %s", ex.what());
    }
}

size_t NcBrainMap::getTeachedNodesSize()
{
    LOG_INFO("getTeachedNodesSize");
    std::string teach_file = dirpath_ + "teaching.json";
    if (File(teach_file).exists()) {
        Object::Ptr data = loadJSON(teach_file);
        return data->getArray("nodes")->size();
    }
    return 0;
}

Object::Ptr NcBrainMap::getTeachedData()
{
    InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY)->setTeachedNodes(getTeachedNodesSize());

    std::string teach_file = dirpath_ + "teaching.json";
    if (File(teach_file).exists()) {
        return loadJSON(teach_file);
    }
    else {
        Object data, node;
        Array nodes;
        nodes.add(node);
        data.set("nodes", nodes);
        saveJSON(teach_file, data);
        return loadJSON(teach_file);
    }

    return nullptr;
}

std::string NcBrainMap::teachingUpdate(Object::Ptr teachNodes)
{
    LOG_INFO("teachingUpdate");

    DynamicStruct dsTeachNodes = *teachNodes;
    std::string project_map_revision = std::to_string((int)dsTeachNodes["map_revision"]);

    if (project_map_revision != getLocalRevision())
        return "map fail";

    std::string teach_file = dirpath_ + "teaching.json";
    std::string teach_file_dir = dirpath_ + "teaching_" + project_map_revision + ".json";
    if (File(teach_file_dir).exists()) {
        Object::Ptr oldData = loadJSON(teach_file_dir);

        Poco::JSON::Array::Ptr nodes = oldData->getArray("nodes");
        for (size_t i = 0; i < nodes->size(); i++) {
            if (nodes->getObject(i)->get("id").convert<std::string>() == teachNodes->get("id").convert<std::string>()) {
                nodes->set(i, teachNodes);
                if (!File(dirpath_).exists()) {
                    File(dirpath_).createDirectories();
                }
                saveJSON(teach_file_dir, oldData);
                File(teach_file_dir).copyTo(teach_file);
                return "success";
            }
        }

        if (!File(dirpath_).exists()) {
            File(dirpath_).createDirectories();
        }
        nodes->add(teachNodes);
        saveJSON(teach_file_dir, oldData);
        File(teach_file_dir).copyTo(teach_file);
        return "success";
    }
    else {
        Object data;
        Array nodes;
        nodes.add(teachNodes);
        data.set("nodes", nodes);
        saveJSON(teach_file_dir, data);
        File(teach_file_dir).copyTo(teach_file);
        return "success";
    }

    return "fail";
}

bool NcBrainMap::getNodePositionByNodeid(std::string node_id, float& node_x, float& node_y, float& node_deg)
{
    if (map_data_->size() == 0) {
        LOG_ERROR("map data is empty.");
        return false;
    }

    Poco::JSON::Array::Ptr node_array;

    if (map_data_->has("nodes"))
        node_array = map_data_->getArray("nodes");
    else {
        LOG_ERROR("nodes data is empty.");
        return false;
    }

    try {
        for (const auto& map_nodes_data : *node_array) {
            Poco::JSON::Object::Ptr map_nodes_data_ = map_nodes_data.extract<Object::Ptr>();

            if (node_id == map_nodes_data_->get("id").convert<std::string>()) {
                Poco::JSON::Object::Ptr goal_position = map_nodes_data_->getObject("position");

                node_x = goal_position->get("x");
                node_y = goal_position->get("y");

                Poco::JSON::Object::Ptr goal_orientation = map_nodes_data_->getObject("orientation");
                tf::Quaternion qOrientation(
                    (float)goal_orientation->get("x"), (float)goal_orientation->get("y"), (float)goal_orientation->get("z"),
                    (float)goal_orientation->get("w"));
                tf::Matrix3x3 m(qOrientation);

                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                node_deg = yaw / M_PI * 180;
            }
        }
        return true;
    }
    catch (std::exception ex) {
        LOG_ERROR("%s", ex.what());
        return false;
    }
}

bool NcBrainMap::getNodePositionNearbyRobot(std::string& node_id, float& node_x, float& node_y, float& node_deg)
{
    if (map_data_->size() == 0) {
        LOG_ERROR("map data is empty.");
        return false;
    }

    Poco::JSON::Array::Ptr node_array;

    if (map_data_->has("nodes"))
        node_array = map_data_->getArray("nodes");
    else {
        LOG_ERROR("nodes data is empty.");
        return false;
    }

    try {
        float find_dist = 500;  // 500mm

        // RobotPose robot_position = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY)->getPose();

        // for (const auto& map_nodes_data : *node_array) {
        //     Poco::JSON::Object::Ptr map_nodes_data_ = map_nodes_data.extract<Object::Ptr>();

        //     Poco::JSON::Object::Ptr node_position = map_nodes_data_->getObject("position");

        //     float f_dist = hypot(
        //         node_position->get("x").convert<float>() - robot_position.position.x,
        //         node_position->get("y").convert<float>() - robot_position.position.y);  // 내 위치
        //     if (f_dist < find_dist) {
        //         find_dist = f_dist;

        //         Poco::JSON::Object::Ptr start_position = map_nodes_data_->getObject("position");

        //         node_id = map_nodes_data_->get("id").convert<std::string>();
        //         node_x = start_position->get("x");
        //         node_y = start_position->get("y");

        //         Poco::JSON::Object::Ptr start_orientation = map_nodes_data_->getObject("orientation");

        //         tf::Quaternion qOrientation(
        //             (float)start_orientation->get("x"), (float)start_orientation->get("y"), (float)start_orientation->get("z"),
        //             (float)start_orientation->get("w"));
        //         tf::Matrix3x3 m(qOrientation);

        //         double roll, pitch, yaw;
        //         m.getRPY(roll, pitch, yaw);

        //         node_deg = yaw / M_PI * 180;
        //     }
        // }
        return true;
    }
    catch (std::exception ex) {
        LOG_ERROR("%s", ex.what());
        return false;
    }
}

bool NcBrainMap::getLinkDriveData(std::string start_node_id, std::string goal_node_id, float& linear_speed, int& drive_type)
{
    if (map_data_->size() == 0) {
        LOG_ERROR("map data is empty.");
        return false;
    }

    Poco::JSON::Array::Ptr link_array;

    if (map_data_->has("links")) {
        link_array = map_data_->getArray("links");
    }
    else {
        LOG_ERROR("links data is empty.");
        return false;
    }

    // 시작노드 후생성 노드 ex) 노드링크 : 0001_0004 시작노드 : 0004  (linear_speed,drive_type) :
    // (to_driving_linear_speed,to_driving_direction) 사용 시작노드 선생성 노드 ex) 노드링크 : 0001_0004 시작노드 : 0001
    // (linear_speed,drive_type) : (from_driving_linear_speed,from_driving_direction) 사용
    try {
        for (const auto& map_links_data : *link_array) {
            Poco::JSON::Object::Ptr map_links_data_ = map_links_data.extract<Object::Ptr>();
            Poco::JSON::Object::Ptr map_link_connected = map_links_data_->getObject("connected");
            // 시작노드 선생성 노드
            if ((map_link_connected->get("from").convert<std::string>() == start_node_id) &&
                (map_link_connected->get("to").convert<std::string>() == goal_node_id)) {
                Poco::JSON::Object::Ptr from_link_driving = map_links_data_->getObject("from_link_drive");
                if (from_link_driving->get("direction").convert<std::string>() == "front")
                    drive_type = 1;
                else  // backward
                    drive_type = 8;
                linear_speed = from_link_driving->get("linear_speed");
                return true;
            }
            // 시작노드 후생성 노드
            if (map_link_connected->get("to").convert<std::string>() == start_node_id &&
                map_link_connected->get("from").convert<std::string>() == goal_node_id) {
                Poco::JSON::Object::Ptr to_link_driving = map_links_data_->getObject("to_link_drive");
                if (to_link_driving->get("direction").convert<std::string>() == "front")
                    drive_type = 1;
                else  // backward
                    drive_type = 8;
                linear_speed = to_link_driving->get("linear_speed");
                return true;
            }
        }
        LOG_ERROR("Connected map link not found.");
        return false;
    }
    catch (std::exception ex) {
        LOG_ERROR("%s", ex.what());
        return false;
    }
}