#pragma once

#include <HARC_IFBOX/DataMatrix.h>

#include <optional>
#include <string>
#include <utility>

namespace harc_ifbox
{

/** An HARC_IFBOX device provides a (IFBOX_ROWSxIFBOX_COLS) data matrix
 *
 * A patch is a (PATCH_ROWSxPATCH_COLS) matrix which is obtained from the IFBOX matrix
 *
 * This class encapsulate this data as well as mapping from the IFBOX matrix to the PATCH matrix
 *
 * Not all cells in the patch may be active at a given time
 */
struct Patch
{
  /** Create a patch
   *
   * \param name Name of the patch
   *
   * \param rows Number of rows in the patch
   *
   * \param cols Number of cols in the patch
   *
   * \param mappings Mappings between the IFBOX and the patch, if nullopt the data will be directly copied from the
   * IFBOX matrix
   */
  inline Patch(const std::string & name,
               size_t rows,
               size_t cols,
               std::optional<DataMatrix::Mappings> mappings = std::nullopt)
  : name_(name), data_(rows, cols), mappings_(std::move(mappings))
  {
  }

  inline void update(const DataMatrix & ifbox_data) noexcept
  {
    if(mappings_) { data_.update(ifbox_data, *mappings_); }
    else { data_.copy(ifbox_data); }
  }

  inline const std::string & name() const noexcept { return name_; }

  inline const DataMatrix & data() const noexcept { return data_; }

private:
  std::string name_;
  DataMatrix data_;
  std::optional<DataMatrix::Mappings> mappings_;
};

using Patches = std::vector<Patch>;

} // namespace harc_ifbox
