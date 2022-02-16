/********************************************************************
 * Copyright (C) 2015 Liangliang Nan <liangliang.nan@gmail.com>
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++ library
 *      for processing and rendering 3D data.
 *      Journal of Open Source Software, 6(64), 3255, 2021.
 * ------------------------------------------------------------------
 *
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 ********************************************************************/

#ifndef EASY3D_SURFACE_MESH_MANHATTAN_H
#define EASY3D_SURFACE_MESH_MANHATTAN_H


namespace easy3d {

    class SurfaceMesh;

    /// Manhattan makes a noisy Manhattan-world surface mesh (near) perfect, i.e.,
    ///     - near orthogonal consecutive edges orthogonal
    ///     - polygonal faces planar
    /// Note: triangle meshes cannot be processed.
    class Manhattan {
    public:
        static void apply(
                SurfaceMesh *mesh,
                double ortho_thresh = 0.2,	  // threshold for two subsequent edges to be orthogonal. It is abs(dot product) of two edge vectors (normalized)
                double w_orig_pos = 1.0,      // weight for preventing vertices from deviating from their original positions
                double w_ortho = 500.0,       // weight for the orthogonality energy term
                double w_facet_planar = 500.0 // weight for the face planarity energy term
        );
    };
}

#endif  // EASY3D_SURFACE_MESH_MANHATTAN_H