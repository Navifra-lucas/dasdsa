#ifndef NAVIFRA_NAVICAT_PDO_H
#define NAVIFRA_NAVICAT_PDO_H

#include <string>
#include <cstring>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

namespace NaviFra {
namespace NaviCAT {
namespace PDO {

// PDO 데이터 구조체 (동적으로 생성될 예정)
struct PDOEntry {
    std::string name;
    uint16_t index;
    uint8_t subindex;
    uint8_t bit_length;
    std::string data_type;
    size_t offset;  // 구조체 내에서의 오프셋
};

class PDOMapper {
private:
    std::vector<PDOEntry> tx_pdo_entries_;  // Outputs (Master -> Slave)
    std::vector<PDOEntry> rx_pdo_entries_;  // Inputs (Slave -> Master)
    size_t tx_pdo_size_ = 0;
    size_t rx_pdo_size_ = 0;

    size_t calculateDataSize(const std::string& data_type) {
        static const std::unordered_map<std::string, size_t> type_sizes = {
            {"BOOL", 1}, {"BYTE", 1}, {"SINT", 1}, {"USINT", 1},
            {"INT", 2}, {"UINT", 2}, {"WORD", 2},
            {"DINT", 4}, {"UDINT", 4}, {"DWORD", 4}, {"REAL", 4},
            {"LINT", 8}, {"ULINT", 8}, {"LWORD", 8}, {"LREAL", 8}
        };
        
        auto it = type_sizes.find(data_type);
        return (it != type_sizes.end()) ? it->second : 4;
    }

public:
    bool loadFromYAML(const std::string& yaml_file) {
        try {
            YAML::Node config = YAML::LoadFile(yaml_file);
            
            // TxPDO (Outputs) 로드
            if (config["TxPDO"]) {
                size_t offset = 0;
                for (const auto& entry : config["TxPDO"]) {
                    PDOEntry pdo_entry;
                    pdo_entry.name = entry["name"].as<std::string>();
                    pdo_entry.index = entry["index"].as<uint16_t>();
                    pdo_entry.subindex = entry["subindex"].as<uint8_t>();
                    pdo_entry.bit_length = entry["bit_length"].as<uint8_t>();
                    pdo_entry.data_type = entry["data_type"].as<std::string>();
                    pdo_entry.offset = offset;
                    
                    size_t data_size = calculateDataSize(pdo_entry.data_type);
                    offset += data_size;
                    tx_pdo_entries_.push_back(pdo_entry);
                }
                tx_pdo_size_ = offset;
            }

            // RxPDO (Inputs) 로드
            if (config["RxPDO"]) {
                size_t offset = 0;
                for (const auto& entry : config["RxPDO"]) {
                    PDOEntry pdo_entry;
                    pdo_entry.name = entry["name"].as<std::string>();
                    pdo_entry.index = entry["index"].as<uint16_t>();
                    pdo_entry.subindex = entry["subindex"].as<uint8_t>();
                    pdo_entry.bit_length = entry["bit_length"].as<uint8_t>();
                    pdo_entry.data_type = entry["data_type"].as<std::string>();
                    pdo_entry.offset = offset;
                    
                    size_t data_size = calculateDataSize(pdo_entry.data_type);
                    offset += data_size;
                    rx_pdo_entries_.push_back(pdo_entry);
                }
                rx_pdo_size_ = offset;
            }

            return true;
        } catch (const YAML::Exception& e) {
            std::cerr << "YAML 파싱 오류: " << e.what() << std::endl;
            return false;
        }
    }

    const std::vector<PDOEntry>& getTxPDOEntries() const { return tx_pdo_entries_; }
    const std::vector<PDOEntry>& getRxPDOEntries() const { return rx_pdo_entries_; }
    size_t getTxPDOSize() const { return tx_pdo_size_; }
    size_t getRxPDOSize() const { return rx_pdo_size_; }
    
    void printMapping() const {
        std::cout << "=== TxPDO (Outputs) Mapping ===" << std::endl;
        for (const auto& entry : tx_pdo_entries_) {
            std::printf("%-20s 0x%04X:%02X %s (offset: %zu)\n", 
                       entry.name.c_str(), entry.index, entry.subindex, 
                       entry.data_type.c_str(), entry.offset);
        }
        
        std::cout << "\n=== RxPDO (Inputs) Mapping ===" << std::endl;
        for (const auto& entry : rx_pdo_entries_) {
            std::printf("%-20s 0x%04X:%02X %s (offset: %zu)\n", 
                       entry.name.c_str(), entry.index, entry.subindex, 
                       entry.data_type.c_str(), entry.offset);
        }
        std::cout << std::endl;
    }
};

class PDOData {
private:
    std::unique_ptr<uint8_t[]> data_;
    size_t size_;
    const std::vector<PDOEntry>& entries_;

public:
    PDOData(size_t size, const std::vector<PDOEntry>& entries) 
        : size_(size), entries_(entries) {
        data_ = std::make_unique<uint8_t[]>(size);
        std::memset(data_.get(), 0, size);
    }

    template<typename T>
    void setValue(const std::string& name, T value) {
        for (const auto& entry : entries_) {
            if (entry.name == name) {
                std::memcpy(data_.get() + entry.offset, &value, sizeof(T));
                return;
            }
        }
        std::cerr << "PDO 엔트리를 찾을 수 없음: " << name << std::endl;
    }

    template<typename T>
    T getValue(const std::string& name) const {
        for (const auto& entry : entries_) {
            if (entry.name == name) {
                T value;
                std::memcpy(&value, data_.get() + entry.offset, sizeof(T));
                return value;
            }
        }
        std::cerr << "PDO 엔트리를 찾을 수 없음: " << name << std::endl;
        return T{};
    }

    uint8_t* getData() { return data_.get(); }
    const uint8_t* getData() const { return data_.get(); }
    size_t getSize() const { return size_; }
};

}
}
}

#endif