/*
 * @file	: blaze_vector_stream.hpp
 * @date	: Jan 8, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: override ostream operator of blaze vector
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef BLAZE_VECTOR_STREAM_HPP_
#define BLAZE_VECTOR_STREAM_HPP_

#include <iostream>
#include <blaze/Math.h>

inline std::ostream& operator<<( std::ostream& os, const blaze::DynamicVector<double>& v )
{
  if( v.size() == 0UL ) {
    os << "( )\n";
  }
  else {
    os << "( " << v[0];
    size_t un_vec_size = v.size();
    for( size_t i=1UL; i<un_vec_size; ++i )
      os << " " << v[i];
    os << " )\n";
  }

  return os;
}

inline std::ostream& operator<<( std::ostream& os, const blaze::DVecDVecAddExpr<blaze::DynamicVector<double>, blaze::DynamicVector<double>, false> v )
{
  if( v.size() == 0UL ) {
    os << "( )\n";
  }
  else {
    os << "( " << v[0];
    size_t un_vec_size = v.size();
    for( size_t i=1UL; i<un_vec_size; ++i )
      os << v[i] << " ";
    os << ")\n";
  }

  return os;
}

inline std::ostream& operator<<( std::ostream& os, const blaze::DVecDVecSubExpr<blaze::DynamicVector<double>, blaze::DynamicVector<double>, false> v )
{
  if( v.size() == 0UL ) {
    os << "( )\n";
  }
  else {
    os << "( " << v[0];
    size_t un_vec_size = v.size();
    for( size_t i=1UL; i<un_vec_size; ++i )
      os << v[i] << " ";
    os << ")\n";
  }

  return os;
}

inline std::ostream& operator<<( std::ostream& os, const blaze::DVecDVecDivExpr<blaze::DynamicVector<double>, blaze::DynamicVector<double>, false> v )
{
  if( v.size() == 0UL ) {
    os << "( )\n";
  }
  else {
    os << "( " << v[0];
    size_t un_vec_size = v.size();
    for( size_t i=1UL; i<un_vec_size; ++i )
      os << v[i] << " ";
    os << ")\n";
  }

  return os;
}

inline std::ostream& operator<<( std::ostream& os, const blaze::DVecDVecMultExpr<blaze::DynamicVector<double>, blaze::DynamicVector<double>, false> v )
{
  if( v.size() == 0UL ) {
    os << "( )\n";
  }
  else {
    os << "( " << v[0];
    size_t un_vec_size = v.size();
    for( size_t i=1UL; i<un_vec_size; ++i )
      os << v[i] << " ";
    os << ")\n";
  }

  return os;
}

#endif
