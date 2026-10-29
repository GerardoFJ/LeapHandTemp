#pragma once

#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>

namespace harc_ifbox
{

/** A trivial Matrix implementation backed by std::vector */
struct DataMatrix
{
  /** A single point in the matrix */
  struct Coord
  {
    size_t x;
    size_t y;
  };

  /** Mapping between two DataMatrix coordinates */
  struct Mapping
  {
    Coord dst;
    Coord src;
  };

  using Mappings = std::vector<Mapping>;

  inline DataMatrix() : DataMatrix(0, 0) {}

  inline DataMatrix(size_t rows, size_t cols) { resize(rows, cols); }

  inline void resize(size_t row, size_t col) noexcept
  {
    rows_ = row;
    cols_ = col;
    data_.resize(size());
  }

  inline const uint8_t & operator()(size_t row, size_t col) const noexcept
  {
    assert(row < rows_);
    assert(col < cols_);
    return data_[col + row * cols_];
  }

  inline const uint8_t & operator()(const Coord & coord) const noexcept { return (*this)(coord.x, coord.y); }

  inline uint8_t & operator()(size_t row, size_t col) noexcept
  {
    return const_cast<uint8_t &>(const_cast<const DataMatrix &>(*this)(row, col));
  }

  inline uint8_t & operator()(const Coord & coord) noexcept { return (*this)(coord.x, coord.y); }

  inline size_t rows() const noexcept { return rows_; }

  inline size_t cols() const noexcept { return cols_; }

  inline size_t size() const noexcept { return rows_ * cols_; }

  inline const uint8_t * data() const noexcept { return data_.data(); }

  /** Copy data from rhs into this
   *
   * For every mapping m in \ref mappings (*this)(m.dst) = rhs(m.src)
   */
  inline void update(const DataMatrix & rhs, const Mappings & mappings) noexcept
  {
    auto & self = *this;
    for(const auto & m : mappings) { self(m.dst) = rhs(m.src); }
  }

  /** Directly copy data from rhs into this */
  inline void copy(const DataMatrix & rhs) noexcept
  {
    assert(rows() == rhs.rows() && cols() == rhs.cols());
    data_ = rhs.data_;
  }

  /** Directly copy from a data buffer */
  inline void copy(const uint8_t * src) noexcept { memcpy(data_.data(), src, sizeof(uint8_t) * rows_ * cols_); }

private:
  size_t rows_;
  size_t cols_;
  std::vector<uint8_t> data_;
};

} // namespace harc_ifbox
