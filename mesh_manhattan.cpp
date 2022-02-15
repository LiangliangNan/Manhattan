#include "mesh_manhattan.h"
#include <optimizer/optimizer_lm.h>

#include <easy3d/util/logging.h>
#include <easy3d/util/stop_watch.h>
#include <easy3d/core/surface_mesh.h>


namespace easy3d {

    namespace Details {
        struct Ortho {
            std::size_t v_id0;
            std::size_t v_id1;
            std::size_t v_id2;
        };

        struct Planar {
            std::size_t v_id0;
            std::size_t v_id1;
            std::size_t v_id2;
            std::size_t v_id3;
        };


        struct Data {
            std::vector<double> orig_pos;
            std::vector<Ortho> ortho_constraints;
            std::vector<Planar> planar_constraints;

            double w_orig_pos;
            double w_ortho;
            double w_facet_planar;
        };


        //////////////////////////////////////////////////////////////////////////

        inline double ortho_cost(const Ortho &ortho, const double X[]) {
            int id0 = ortho.v_id0 * 3;
            int id1 = ortho.v_id1 * 3;
            int id2 = ortho.v_id2 * 3;

            vec3 p0(X[id0], X[id0 + 1], X[id0 + 2]);
            vec3 p1(X[id1], X[id1 + 1], X[id1 + 2]);
            vec3 p2(X[id2], X[id2 + 1], X[id2 + 2]);

            vec3 v10 = p0 - p1;
            v10 = normalize(v10);
            vec3 v12 = p2 - p1;
            v12 = normalize(v12);
            //return std::abs(dot(v10, v12)); // the function value will be squared
            return dot(v10, v12);
        }


        inline double facet_planar_cost(const Planar &planar, const double X[]) {
            int id0 = planar.v_id0 * 3;
            int id1 = planar.v_id1 * 3;
            int id2 = planar.v_id2 * 3;
            int id3 = planar.v_id3 * 3;

            vec3 p0(X[id0], X[id0 + 1], X[id0 + 2]);
            vec3 p1(X[id1], X[id1 + 1], X[id1 + 2]);
            vec3 p2(X[id2], X[id2 + 1], X[id2 + 2]);
            vec3 p3(X[id3], X[id3 + 1], X[id3 + 2]);

            vec3 v01 = p1 - p0;
            vec3 v03 = p3 - p0;
            vec3 v02 = p2 - p0;
            v02 = normalize(v02);
            vec3 normal = cross(v01, v03);
            normal = normalize(normal);
            //return std::abs(dot(normal, v02)); // the function value will be squared
            return dot(normal, v02);
        }


        void collect_constraints(SurfaceMesh *mesh, Data *data) {
            for (auto f : mesh->faces()) {
                SurfaceMesh::HalfedgeAroundFaceCirculator cir = mesh->halfedges(f);
                SurfaceMesh::HalfedgeAroundFaceCirculator end = cir;
                do {
                    Ortho ortho;

                    SurfaceMesh::Halfedge h = *cir;
                    ortho.v_id0 = mesh->target(h).idx();

                    SurfaceMesh::Halfedge h_next = mesh->next(h);
                    ortho.v_id1 = mesh->target(h_next).idx();

                    SurfaceMesh::Halfedge h_next_next = mesh->next(h_next);
                    ortho.v_id2 = mesh->target(h_next_next).idx();

                    data->ortho_constraints.push_back(ortho);
                    ++cir;
                } while (cir != end);
            }

            for (auto f : mesh->faces()) {
                SurfaceMesh::HalfedgeAroundFaceCirculator cir = mesh->halfedges(f);
                SurfaceMesh::HalfedgeAroundFaceCirculator end = cir;
                do {
                    Planar planar;

                    SurfaceMesh::Halfedge h0 = *cir;
                    planar.v_id0 = mesh->target(h0).idx();

                    SurfaceMesh::Halfedge h1 = mesh->next(h0);
                    planar.v_id1 = mesh->target(h1).idx();

                    SurfaceMesh::Halfedge h2 = mesh->next(h1);
                    planar.v_id2 = mesh->target(h2).idx();

                    SurfaceMesh::Halfedge h3 = mesh->next(h2);
                    planar.v_id3 = mesh->target(h3).idx();

                    data->planar_constraints.push_back(planar);
                    ++cir;
                } while (cir != end);
            }
        }
    }

    using namespace Details;

    static unsigned int itr_count = 0;

    class Objective : public Objective_LM {
    public:
        Objective(int num_func, int num_var, void *data) : Objective_LM(num_func, num_var, data) {}
        int evaluate(const double *x, double *fvec) {
            Data *all_data = reinterpret_cast<Data *>(data_);
            const std::vector<double> &X_orig = all_data->orig_pos;
            const std::vector<Ortho> &ortho_constraints = all_data->ortho_constraints;
            const std::vector<Planar> &planar_constraints = all_data->planar_constraints;

            // orig pos
            for (int i = 0; i < num_var_; ++i)
                fvec[i] = (x[i] - X_orig[i]) * all_data->w_orig_pos;

            //////////////////////////////////////////////////////////////////////////

            // ortho_constraints
            std::size_t offset = num_var_;
            for (std::size_t i = 0; i < ortho_constraints.size(); ++i)
                fvec[offset + i] = ortho_cost(ortho_constraints[i], x) * all_data->w_ortho;

            //////////////////////////////////////////////////////////////////////////

            // facet_planar_constraints
            offset += ortho_constraints.size();
            for (std::size_t i = 0; i < planar_constraints.size(); ++i)
                fvec[offset + i] = facet_planar_cost(planar_constraints[i], x) * all_data->w_facet_planar;

            ++itr_count;
            return 0;
        }
    };


    void MeshManhattan::apply(
            SurfaceMesh *mesh,
            double w_orig_pos,
            double w_ortho,
            double w_facet_planar
    ) {
        if (!mesh) {
            LOG(WARNING) << "no mesh exist";
            return;
        }

        if (!mesh->is_quad_mesh()) {
            LOG(WARNING) << "currently implementation works on quad meshes only";
//            return;
        }

        if (!mesh->is_closed())
            LOG(WARNING) << "may not work because mesh has seams/boundaries";

        LOG(INFO) << "enhancing Manhattan...";
        StopWatch t;
        t.start();

        itr_count = 0;

        //////////////////////////////////////////////////////////////////////////

        Data data;
        data.w_orig_pos = std::sqrt(w_orig_pos);
        data.w_ortho = std::sqrt(w_ortho);
        data.w_facet_planar = std::sqrt(w_facet_planar);

        std::vector<double> X_orig(mesh->n_vertices() * 3);
        auto points = mesh->get_vertex_property<vec3>("v:point");
        for (auto v : mesh->vertices()) {
            const auto& p = points[v];
            X_orig[v.idx() * 3] = p.x;
            X_orig[v.idx() * 3 + 1] = p.y;
            X_orig[v.idx() * 3 + 2] = p.z;
        }

        data.orig_pos = X_orig;
        collect_constraints(mesh, &data);

        //////////////////////////////////////////////////////////////////////////

        std::size_t n = mesh->n_vertices() * 3;
        std::size_t m = data.orig_pos.size()
                        + data.ortho_constraints.size()
                        + data.planar_constraints.size();

        Objective obj(m, n, &data);
        Optimizer_LM lm;
        std::vector<double> x = X_orig; // provide the initial guess
        bool flag = lm.optimize(&obj, x);
        if (flag) {
            for (auto v : mesh->vertices()) {
                points[v] = vec3(
                        x[v.idx() * 3],
                        x[v.idx() * 3 + 1],
                        x[v.idx() * 3 + 2]
                        );
            }
            LOG(INFO) << "done. time: " << t.elapsed_seconds() << " seconds. " << itr_count << " evaluations";
        } else
            LOG(ERROR) << "optimization failed. Error code: " << flag;
    }
}