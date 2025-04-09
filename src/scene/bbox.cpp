#include "bbox.h"

#include <algorithm>
#include <iostream>

namespace CGL {

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
