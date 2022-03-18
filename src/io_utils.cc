// BSD 3-Clause License

// Copyright (c) 2019, The University of North Carolina at Chapel Hill
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//! @author Mengyu Fu

#include <iostream>
#include <fstream>

#include "io_utils.h"

namespace io {

void LoadObjModel(const String& file_name, Vec3s& pos, Vec2s& texture, Vec3s& normal, Indices& v_i,
                  Indices& t_i, Indices& n_i) {
    std::ifstream fin(file_name, std::ios::in);

    if (!fin) {
        std::cerr << "Cannot open " << file_name << std::endl;
        exit(1);
    }

    pos.clear();
    texture.clear();
    normal.clear();
    t_i.clear();
    t_i.clear();
    n_i.clear();

    String line;
    RealNum x, y, z;
    Idx U, V;
    Idx idx, n;

    while (std::getline(fin, line)) {
        auto key = line.substr(0, 2);

        if (key == "v ") {
            // Check v for vertices.
            std::istringstream v(line.substr(2));
            v >> x >> y >> z;
            pos.emplace_back(x, y, z);
        }
        else if (key == "vt") {
            // Check for texture co-ordinate
            std::istringstream v(line.substr(3));
            v >> U >> V;
            texture.emplace_back(U, V);
        }
        else if (key == "vn") {
            std::istringstream v(line.substr(3));
            v >> x >> y >> z;
            normal.emplace_back(x, y, z);
        }
        else if(key == "f ") {
            std::istringstream v(line.substr(2));
            String field0, field1;
            std::vector<String> fields;

            while (std::getline(v, field0, ' ')) {
                std::istringstream u(field0);
                n = 0;

                while (std::getline(u, field1, '/')) {
                    n++;

                    if (field1.empty()) {
                        continue;
                    }

                    std::istringstream w(field1);
                    w >> idx;

                    if (n == 1) {
                        v_i.push_back(--idx);
                    }
                    else if (n == 2) {
                        t_i.push_back(--idx);
                    }
                    else {
                        n_i.push_back(--idx);
                    }
                }
            }
        }
    }

    std::cout << "Vertices in model: " << pos.size() << std::endl;
    std::cout << "Faces in model: " << v_i.size()/3 << std::endl;
}

void WriteJSPtCloud(const String& file_name, const Vec3s& ptc, const RealNum r,
                    const IdxPoint color) {
    std::ofstream fout(file_name);

    if(!fout.is_open()) {
        std::cerr << "Could not open " << file_name << std::endl;
        exit(1);
    }

    fout << "var ptCloud = [";

    Idx d = 3;

    for (auto i = 0; i < ptc.size() - 1; ++i) {
        auto n = ptc.at(i);
        fout << "[";

        for (auto j = 0; j < d; ++j) {
            fout << std::to_string(n[j]) << ", ";
        }

        fout << r << ", " << color[0] << ", " << color[1] << ", " << color[2] << "]," << std::endl;
    }

    auto n = ptc.at(ptc.size()-1);
    fout << "[";

    for (unsigned int j = 0; j < d; ++j) {
        fout << std::to_string(n[j]) << ", ";
    }

    fout << r << ", " << color[0] << ", " << color[1] << ", " << color[2] << "]," << std::endl;
    fout << "];" << std::endl;

    fout.close();

    std::cout << "File " << file_name << " written." << std::endl;
}





}