/*
 * File:   PLYManager.cpp
 * Author: M.Àngels Cerveró Abelló
 * Created on 22 / juliol / 2010, 10:55
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

#include <cmath>
#include <cstring>
#include <cfloat>
#include <locale.h>
#include "PLYManager.h"

PLYManager::PLYManager() {
    this->vData.verts = NULL;
    this->vData.normals = NULL;
    this->fData.faces = NULL;

    this->vData.min = new float[3];
    this->vData.min[0] = FLT_MAX;
    this->vData.min[1] = FLT_MAX;
    this->vData.min[2] = FLT_MAX;
    this->vData.max = new float[3];
    this->vData.max[0] = -FLT_MAX;
    this->vData.max[1] = -FLT_MAX;
    this->vData.max[2] = -FLT_MAX;

    this->vData.numVerts = 0;
    this->fData.numFaces = 0;

    this->deleteData = true;
}

PLYManager::~PLYManager() {
    if(deleteData) {
        delete [] this->vData.verts;
        delete [] this->vData.normals;
        delete [] this->fData.faces;

        delete [] this->vData.min;
        delete [] this->vData.max;
    }
}

int PLYManager::vertex_callback(p_ply_argument argument) {
    long vertIndx, component;
    VertsData *data = NULL;
    
    ply_get_argument_user_data(argument, (void**) &data, &component);
    ply_get_argument_element(argument, NULL, &vertIndx);
    data->verts[vertIndx*3+component] = ply_get_argument_value(argument);

    if(data->verts[vertIndx*3+component] > data->max[component]) data->max[component] = data->verts[vertIndx*3+component];

    if(data->verts[vertIndx*3+component] < data->min[component]) data->min[component] = data->verts[vertIndx*3+component];

    return 1;
}

int PLYManager::normal_callback(p_ply_argument argument) {
    long vertIndx, component;
    VertsData *data = NULL;

    ply_get_argument_user_data(argument, (void**) &data, &component);
    ply_get_argument_element(argument, NULL, &vertIndx);
    data->normals[vertIndx*3+component] = ply_get_argument_value(argument);

    return 1;
}

int PLYManager::face_callback(p_ply_argument argument) {
    long faceIndx;
    FacesData *data = NULL;

    ply_get_argument_user_data(argument, (void**) &data, NULL);
    ply_get_argument_element(argument, NULL, &faceIndx); //Get face index
    long numVerts, vertIndx;
    ply_get_argument_property(argument, NULL, &numVerts, &vertIndx);
    if(numVerts==3 && vertIndx!=-1) {
        data->faces[faceIndx*3 + vertIndx] = (int)ply_get_argument_value(argument);
    }
    
    return 1;
}

void PLYManager::computePerVertexNormals() {
    unsigned int i;
    double v1[3], v2[3], faceNorm[3];
    double norm;

    for(i=0; i<this->vData.numVerts; i++) {
        this->vData.normals[i*3] = 0.0;
        this->vData.normals[i*3+1] = 0.0;
        this->vData.normals[i*3+2] = 0.0;
    }

    for(i=0; i<this->fData.numFaces; i++) {
        v1[0] = this->vData.verts[this->fData.faces[i*3+1]*3] - this->vData.verts[this->fData.faces[i*3]*3];
        v1[1] = this->vData.verts[this->fData.faces[i*3+1]*3+1] - this->vData.verts[this->fData.faces[i*3]*3+1];
        v1[2] = this->vData.verts[this->fData.faces[i*3+1]*3+2] - this->vData.verts[this->fData.faces[i*3]*3+2];

        v2[0] = this->vData.verts[this->fData.faces[i*3+2]*3] - this->vData.verts[this->fData.faces[i*3]*3];
        v2[1] = this->vData.verts[this->fData.faces[i*3+2]*3+1] - this->vData.verts[this->fData.faces[i*3]*3+1];
        v2[2] = this->vData.verts[this->fData.faces[i*3+2]*3+2] - this->vData.verts[this->fData.faces[i*3]*3+2];

        faceNorm[0] = v1[1]*v2[2]-v1[2]*v2[1];
        faceNorm[1] = v1[2]*v2[0]-v1[0]*v2[2];
        faceNorm[2] = v1[0]*v2[1]-v1[1]*v2[0];

        this->vData.normals[this->fData.faces[i*3]*3] += faceNorm[0];
        this->vData.normals[this->fData.faces[i*3]*3+1] += faceNorm[1];
        this->vData.normals[this->fData.faces[i*3]*3+2] += faceNorm[2];

        this->vData.normals[this->fData.faces[i*3+1]*3] += faceNorm[0];
        this->vData.normals[this->fData.faces[i*3+1]*3+1] += faceNorm[1];
        this->vData.normals[this->fData.faces[i*3+1]*3+2] += faceNorm[2];

        this->vData.normals[this->fData.faces[i*3+2]*3] += faceNorm[0];
        this->vData.normals[this->fData.faces[i*3+2]*3+1] += faceNorm[1];
        this->vData.normals[this->fData.faces[i*3+2]*3+2] += faceNorm[2];
    }

    for(i=0; i<this->vData.numVerts; i++) {
        norm = sqrt(pow(this->vData.normals[i*3], 2) + pow(this->vData.normals[i*3+1], 2) + pow(this->vData.normals[i*3+2], 2));
        this->vData.normals[i*3] /= norm;
        this->vData.normals[i*3+1] /= norm;
        this->vData.normals[i*3+2] /= norm;
    }
}

bool PLYManager::readPLY(std::string filename) {
    bool computeNormals = false;
    
    /* Save application locale */
    const char *old_locale = setlocale(LC_NUMERIC, NULL);
    /* Change to PLY standard */
    setlocale(LC_NUMERIC, "C");
    /* Use the RPly library */
    
    p_ply ply = ply_open(filename.c_str(), NULL, 0, NULL);
    if(!ply){
        return false;
    }
    if(!ply_read_header(ply)){
        return false;
    }
    this->filename = filename;

    this->vData.numVerts = ply_set_read_cb(ply, "vertex", "x", vertex_callback, &this->vData, 0);
    ply_set_read_cb(ply, "vertex", "y", vertex_callback, &this->vData, 1);
    ply_set_read_cb(ply, "vertex", "z", vertex_callback, &this->vData, 2);

    if(ply_set_read_cb(ply, "vertex", "nx", normal_callback, &this->vData, 0) != 0) {
        ply_set_read_cb(ply, "vertex", "ny", normal_callback, &this->vData, 1);
        ply_set_read_cb(ply, "vertex", "nz", normal_callback, &this->vData, 2);
    } else {
        computeNormals = true;
    }

    this->fData.numFaces = ply_set_read_cb(ply, "face", "vertex_indices", face_callback, &this->fData, 0);
    
    this->vData.verts = new float[this->vData.numVerts*3];
    this->vData.normals = new float[this->vData.numVerts*3];
    this->fData.faces = new unsigned int[this->fData.numFaces*3];
    
    if(!ply_read(ply)) {
        return 0;
    }
    ply_close(ply);
    
    /* Restore application locale when done */
    setlocale(LC_NUMERIC, old_locale);

    if(computeNormals) {
        computePerVertexNormals();
    }
    return 1;
}
#include <vector>
void PLYManager::compactPLYData() {
	std::vector<unsigned int> equalVerts(this->vData.numVerts);
	unsigned int vertComp, uniqueComp, i, j, k;
    long numDiffVerts = 1, numEqualVerts = 0;
    float *diffVerts;
    double norm;
    bool found;

    equalVerts[0] = 0;
    i = numDiffVerts;
    while(i<this->vData.numVerts) {
        found = false;
        vertComp = i*3;
        for(j=0; j<i && !found; j++) {
            uniqueComp = j*3;
            if(fabs(this->vData.verts[vertComp]-this->vData.verts[uniqueComp])<1e-9 &&
                    fabs(this->vData.verts[vertComp+1]-this->vData.verts[uniqueComp+1])<1e-9 &&
                    fabs(this->vData.verts[vertComp+2]-this->vData.verts[uniqueComp+2])<1e-9) {
                equalVerts[i] = equalVerts[j];
                numEqualVerts++;

                this->vData.normals[j*3] += this->vData.normals[i*3];
                this->vData.normals[j*3+1] += this->vData.normals[i*3+1];
                this->vData.normals[j*3+2] += this->vData.normals[i*3+2];

                found = true;
            }
        }
        if(!found) {
            equalVerts[i] = i-numEqualVerts;
            numDiffVerts++;
        }
        i++;
    }

    diffVerts = new float[numDiffVerts*3];
    k = 0;
    for(i=0; i<this->vData.numVerts; i++) {
        if(k <= equalVerts[i]) {
            diffVerts[k*3] = this->vData.verts[i*3];
            diffVerts[k*3+1] = this->vData.verts[i*3+1];
            diffVerts[k*3+2] = this->vData.verts[i*3+2];
            k++;
        }
    }
    this->vData.numVerts = numDiffVerts;
    delete [] this->vData.verts;
    this->vData.verts = diffVerts;

    for(i=0; i<this->fData.numFaces; i++) {
        this->fData.faces[i*3] = equalVerts[this->fData.faces[i*3]];
        this->fData.faces[i*3+1] = equalVerts[this->fData.faces[i*3+1]];
        this->fData.faces[i*3+2] = equalVerts[this->fData.faces[i*3+2]];
    }

    for(i=0; i<this->vData.numVerts; i++) {
        norm = sqrt(pow(this->vData.normals[i*3], 2) + pow(this->vData.normals[i*3+1], 2) + pow(this->vData.normals[i*3+2], 2));
        this->vData.normals[i*3] /= norm;
        this->vData.normals[i*3+1] /= norm;
        this->vData.normals[i*3+2] /= norm;
    }
}

void PLYManager::writePLY(std::string filename) {
    p_ply_error_cb error = 0;
    unsigned int i;

    if(filename.compare("") != 0) {
        this->filename = filename;
    } else {
        this->filename = this->filename.substr(0, this->filename.find_last_of("/")+1) + "new_" + this->filename.substr(this->filename.find_last_of("/")+1, filename.size()-1);
    }
    
    /* Save application locale */
    const char *old_locale = setlocale(LC_NUMERIC, NULL);
    /* Change to PLY standard */
    setlocale(LC_NUMERIC, "C");
    /* Use the RPly library */
    
    p_ply newPly = ply_create(this->filename.c_str(), PLY_ASCII, error, 0, NULL);

    ply_add_element(newPly, "vertex", this->vData.numVerts);
    ply_add_property(newPly, "x", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
    ply_add_property(newPly, "y", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
    ply_add_property(newPly, "z", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);

    ply_add_property(newPly, "nx", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
    ply_add_property(newPly, "ny", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
    ply_add_property(newPly, "nz", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);

    ply_add_element(newPly, "face", this->fData.numFaces);
    ply_add_property(newPly, "vertex_indices", PLY_LIST, PLY_UCHAR, PLY_UINT);

    ply_write_header(newPly);

    for(i=0; i<this->vData.numVerts; i++) {
        ply_write(newPly, this->vData.verts[i*3]);
        ply_write(newPly, this->vData.verts[i*3+1]);
        ply_write(newPly, this->vData.verts[i*3+2]);

        ply_write(newPly, this->vData.normals[i*3]);
        ply_write(newPly, this->vData.normals[i*3+1]);
        ply_write(newPly, this->vData.normals[i*3+2]);
    }

    for(i=0; i<this->fData.numFaces; i++) {
        ply_write(newPly, 3);

        ply_write(newPly, this->fData.faces[i*3]);
        ply_write(newPly, this->fData.faces[i*3+1]);
        ply_write(newPly, this->fData.faces[i*3+2]);
    }

    ply_close(newPly);
    
    /* Restore application locale when done */
    setlocale(LC_NUMERIC, old_locale);
}

void PLYManager::getData(float *&verts, float *&normals, unsigned int *&faces, float *&min, float *&max, unsigned int &numVerts, unsigned int &numFaces) {
    numVerts = this->vData.numVerts;
    numFaces = this->fData.numFaces;
    
    verts = this->vData.verts;
    normals = this->vData.normals;
    faces = this->fData.faces;

    min = this->vData.min;
    max = this->vData.max;
}
