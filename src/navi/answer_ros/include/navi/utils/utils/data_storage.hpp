/*
 * @file	: data_storage.hpp
 * @date	: Feb 3, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: data storage pattern
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef DATA_STORAGE_HPP_
#define DATA_STORAGE_HPP_

#include <iostream>
#include <map>
#include <utility>

#include "utils/singleton_generator.hpp"
#include "utils/data_manager.hpp"

namespace NVFR {

template <typename T>
class DataStorage : public NaviFra::SingletonGenerator< DataStorage<T> >
{
public:
  DataStorage()
  {
    std::cout << "Construct DataStorage\n";
  }
  virtual ~DataStorage()
  {
    map_data_.clear();
    std::cout << "Destruct DataStorage\n";
  }

  bool FindKey(int n_key) const
  {
    if (map_data_.find(n_key) != map_data_.end()) {
      return true;
    }
    return false;
  }

  bool IsOver(int n_key) const
  {
    const auto& it = map_data_.find(n_key);
    if (it != map_data_.end()) {
      return it->second.IsOver();
    }
    return true;
  }

  T Get(int n_key) const
  {
    const auto& it = map_data_.find(n_key);
    if (it != map_data_.end()) {
      return it->second.Get();
    }
    return T();
  }

  void Set(int n_key, const T& data)
  {
    map_data_[n_key].Set(data);
  }

  void Move(int n_key, T& data)
  {
    map_data_[n_key].Move(data);
  }

private:
  std::map< int, DataManager<T> > map_data_;

};

} // namespace NVFR

#endif
