/*
*	Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
*
*	This file is part of Mapple: software for processing and rendering
*   meshes and point clouds.
*
*	Mapple is free software; you can redistribute it and/or modify
*	it under the terms of the GNU General Public License Version 3
*	as published by the Free Software Foundation.
*
*	Mapple is distributed in the hope that it will be useful,
*	but WITHOUT ANY WARRANTY; without even the implied warranty of
*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*	GNU General Public License for more details.
*
*	You should have received a copy of the GNU General Public License
*	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef EASY3D_ALGORITHM_SURFACE_MESH_MANHATTAN_H
#define EASY3D_ALGORITHM_SURFACE_MESH_MANHATTAN_H


namespace easy3d {

    class SurfaceMesh;

    /// makes a noisy Manhattan-World mesh model (almost) perfect.
    /// Note: triangle meshes cannot be processed
    class Manhattan {
    public:
        static void apply(
                SurfaceMesh *mesh,
                double ortho_thresh = 0.2,		 // the abs(dot_product) of two subsequent two edge vectors (normalized)
                double w_orig_pos = 1.0,         // do not deviate too much from initial geometry
                double w_ortho = 500.0,          // improve orthogonality
                double w_facet_planar = 500.0    // enforce faces to be planar
        );
    };
}

#endif  // EASY3D_ALGORITHM_SURFACE_MESH_MANHATTAN_H