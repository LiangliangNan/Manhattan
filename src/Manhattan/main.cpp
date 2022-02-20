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

#include <easy3d/viewer/comp_viewer.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/util/logging.h>

#include "manhattan.h"

using namespace easy3d;


int main(int argc, char** argv) {
    // Initialize logging.
    logging::initialize();

    CompViewer viewer(2, 2, "Manhattan");

    // load the model
    const std::string model_file = std::string(DATA_DIR) + "/model1.obj";
    auto model = viewer.add_model(model_file);
    if (!model) {
        LOG(ERROR) << "failed to load model from file: " << model_file;
        return EXIT_FAILURE;
    }

    SurfaceMesh* input_mesh = dynamic_cast<SurfaceMesh*>(model);
    if (!input_mesh) {
        LOG(ERROR) << "the loaded model is not a surface mesh";
        return EXIT_FAILURE;
    }

    // ---------------------------------------------------------------------------
    // now we Manhattanize the input mesh
    auto output_mesh = new SurfaceMesh(*input_mesh); // make a copy of the input mesh
    Manhattan::apply(output_mesh);  // Manhattanize the mesh
    output_mesh->set_name("result");
    viewer.add_model(output_mesh);  // add the mesh to the viewer, so input and output are shown next to each other

    // ---------------------------------------------------------------------------
    // setup content for view(0, 0): the surface of the input mesh (this is the default behavior)
    viewer.assign(0, 0, input_mesh);

    // ---------------------------------------------------------------------------
    // setup content for view(1, 0): the wireframe of the input mesh
    auto input_wireframe = input_mesh->renderer()->get_lines_drawable("edges");
    input_wireframe->set_impostor_type(LinesDrawable::CYLINDER);
    input_wireframe->set_line_width(2);
    input_wireframe->set_uniform_coloring(vec4(0.7f, 0.7f, 1.0f, 1.0f));
    input_wireframe->set_visible(true); // by default wireframe is hidden
    viewer.assign(1, 0, input_wireframe);

    // ---------------------------------------------------------------------------
    // setup content for view(0, 1): the surface of the output mesh (this is the default behavior)
    viewer.assign(0, 1, output_mesh);

    // ---------------------------------------------------------------------------
    // setup content for view(1, 1): the wireframe of the output mesh
    auto output_wireframe = output_mesh->renderer()->get_lines_drawable("edges");
    output_wireframe->set_impostor_type(LinesDrawable::CYLINDER);
    output_wireframe->set_line_width(2);
    output_wireframe->set_uniform_coloring(vec4(0.7f, 0.7f, 1.0f, 1.0f));
    output_wireframe->set_visible(true); // by default wireframe is hidden
    viewer.assign(1, 1, output_wireframe);

    // ---------------------------------------------------------------------------

    return viewer.run();
}
