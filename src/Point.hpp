#ifndef POINT_HPP_
#define POINT_HPP_

#include "Config.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"

#include <wx/gdicmn.h>

#pragma GCC diagnostic pop

#include <iostream>

inline std::ostream& operator<<(std::ostream& os, const wxPoint& aPoint)
{
	return os << "(" << aPoint.x << "," << aPoint.y << ")";
}

#endif // POINT_HPP_
