#include "vector2D.h"

std::ostream& operator<<( std::ostream& os, const Vector2D& v ) {
  os << "(" << v.x << "," << v.y << ")";
  return os;
}

