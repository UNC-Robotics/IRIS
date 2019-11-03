#ifndef IO_UTILS_H
#define IO_UTILS_H

#include "global_common.h"

namespace io {

using Vec2s = std::vector<Vec2>;
using Vec3s = std::vector<Vec3>;
using Indices = std::vector<Idx>;

void LoadObjModel(const String& file_name, Vec3s& pos, Vec2s& texture, Vec3s& normal, Indices& v_i, Indices& t_i, Indices& n_i);
void WriteJSPtCloud(const String& file_name, const Vec3s& ptc, const RealNum r, const IdxPoint color);

}


#endif // IO_UTILS_H