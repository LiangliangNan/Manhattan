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

#include <optimizer_lbfgs.h>
#include <include/lbfgs.h>


Objective_LBFGS::Objective_LBFGS(const int n, void* data)
    : num_var_(n)
    , data_(data)
{}


Objective_LBFGS::~Objective_LBFGS()
{}


double Objective_LBFGS::evaluate(const double *x, double *g)
{
    double fx = 0.0;
    for (int i = 0; i < num_var_; i += 2) {
        double t1 = 1.0 - x[i];
        double t2 = 10.0 * (x[i+1] - x[i] * x[i]);
        g[i+1] = 20.0 * t2;
        g[i] = -2.0 * (x[i] * g[i+1] + t1);
        fx += t1 * t1 + t2 * t2;
    }
    return fx;
}




Optimizer_LBFGS::Optimizer_LBFGS()
    : obj_value_(std::numeric_limits<double>::max())
    , iteration_(0)
    , func_(nullptr)
{
}


Optimizer_LBFGS::~Optimizer_LBFGS()
{
}


bool Optimizer_LBFGS::run(Objective_LBFGS* func, int N, double* x)
{
    func_ = func;
    obj_value_ = std::numeric_limits<double>::max();
    iteration_ = 0;

    auto _evaluate = [](void *instance, const double *x, double *g, const int n, const double step) -> double {
        return reinterpret_cast<Optimizer_LBFGS*>(instance)->func_->evaluate(x, g);
    };

    auto _progress = [](void *instance, const double *x, const double *g, const double fx,
            const double xnorm, const double gnorm, const double step, int n, int k, int ls) -> int {
        return reinterpret_cast<Optimizer_LBFGS*>(instance)->progress(x, g, fx, xnorm, gnorm, step, n, k, ls);
    };

    /*
        Start the L-BFGS optimization; this will invoke the callback functions
        evaluate() and progress() when necessary.
     */
    obj_value_ = 0.0;
    int ret = lbfgs(N, x, &obj_value_, _evaluate, _progress, this, nullptr);

    /* Report the result. */
    if (ret == 0) {
        printf("L-BFGS optimization terminated with status code = %d (successful). num_iter = %d\n", ret, iteration_);
        return true;
    }
    else {
        printf("L-BFGS optimization terminated with status code = %d (failed). num_iter = %d\n", ret, iteration_);
        return false;
    }
}


bool Optimizer_LBFGS::run(Objective_LBFGS* func, int N, std::vector<double>& x) {
    return run(func, N, x.data());
}


double Optimizer_LBFGS::objective() const {
    return obj_value_;
}


int Optimizer_LBFGS::progress(
    const double *x,
    const double *g,
    const double fx,
    const double xnorm,
    const double gnorm,
    const double step,
    int n,
    int k,
    int ls
    )
{
    iteration_ = k;

#if 0
    printf("Iteration %d:\n", k);
    printf("  fx = %f, x[0] = %f, x[1] = %f, g[0] = %f, g[1] = %f\n", fx, x[0], x[1], g[0], g[1]);
    printf("  xnorm = %f, gnorm = %f, step = %f\n", xnorm, gnorm, step);
    printf("\n");
#else
    printf("Iteration %d: fx = %f, xnorm = %f, gnorm = %f\n", k, fx, xnorm, gnorm);
#endif

    return 0;
}
