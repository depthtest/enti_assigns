/* 
 * File:   PLYManager.h
 * Author: M.Àngels Cerveró Abelló
 * Created on 22 / juliol / 2010, 10:50
 *
 * Copyright (C) 22 / juliol / 2010  M.Àngels Cerveró Abelló
 * Copyright (C) 6 / juny / 2011  Jesús Ojeda
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _PLYMANAGER_H
#define	_PLYMANAGER_H

#include <string>
#include "rply.h"

struct VertsData {
    float *verts;
    float *normals;
    
    float *min, *max;

    long numVerts;
};

struct FacesData {
    unsigned int *faces;

    long numFaces;
};

class PLYManager {
    private:
        struct VertsData vData;
        struct FacesData fData;

        std::string filename;

        bool deleteData;

        static int vertex_callback(p_ply_argument argument);
        static int normal_callback(p_ply_argument argument);
        static int face_callback(p_ply_argument argument);
        void computePerVertexNormals();
    public:
        PLYManager();
        ~PLYManager();
        
        bool readPLY(std::string filename);
        void compactPLYData();
        void writePLY(std::string filename = "");

        void getData(float *&verts, float *&normals, unsigned int *&faces, float *&min, float *&max, unsigned int &numVerts, unsigned int &numFaces);
};

#endif	/* _PLYMANAGER_H */

