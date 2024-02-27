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

#include <optimizer_lm.h>
#include <cminpack.h>

#include <cstdio>


Objective_LM::Objective_LM(int num_func, int num_var, void* data)
    : num_func_(num_func)
    , num_var_(num_var)
    , data_(data)
{}


Objective_LM::~Objective_LM()
{}


int Objective_LM::evaluate(const double *x, double *fvec)
{
    fvec[0] = x[0] - 1.0;
    fvec[1] = x[1] - 1.0;
    return 0;
}



Optimizer_LM::Optimizer_LM() {
}


Optimizer_LM::~Optimizer_LM() {

}


Optimizer_LM::Parameters::Parameters() {
    maxcall = 10000;
    epsilon = 1.e-14;
    stepbound = 100.;
    ftol = 1.e-14;
    xtol = 1.e-14;
    gtol = 1.e-14;
    nprint = 0;
}


bool Optimizer_LM::run(Objective_LM *func, double *x, Parameters *param)
{
	// use default parameter if ctrl == 0
    if (!param)
        param = &default_control_;

    func_ = func;

	// *** allocate work space.

	double *fvec, *diag, *fjac, *qtf, *wa1, *wa2, *wa3, *wa4;
	int *ipvt;

    int m = func->num_function();
    int n = func->num_variables();

    if (!(fvec = new double[m])   || !(diag = new double[n]) || !(qtf = new double[n]) ||
        !(fjac = new double[n*m]) || !(wa1 = new double[n])  || !(wa2 = new double[n]) ||
        !(wa3 = new double[n])    || !(wa4 = new double[m])  || !(ipvt = new int[n]))
	{
        if (fvec) delete[](fvec);
        if (diag) delete[](diag);
        if (qtf)  delete[](qtf);
        if (fjac) delete[](fjac);
        if (wa1)  delete[](wa1);
        if (wa2)  delete[](wa2);
        if (wa3)  delete[](wa3);
        if (wa4)  delete[](wa4);
        if (ipvt) delete[](ipvt);

        param->info = 9;
        return false;
	}

	// *** perform fit.

    param->info = 0;
    param->nfev = 0;

    auto evaluate_func = [](void *instance, int num_fun, int num_var, const double* var, double* fvec, int iflag) ->int {
        return reinterpret_cast<Optimizer_LM*>(instance)->func_->evaluate(var, fvec);
    };

	// this goes through the modified legacy interface:
    param->info =
		lmdif(
        evaluate_func,
        this,
		m,
		n,
        x,
		fvec,
        param->ftol,
        param->xtol,
        param->gtol,
        param->maxcall*(n + 1),
        param->epsilon,
		diag,
		1,
        param->stepbound,
        param->nprint,
        &(param->nfev),
		fjac,
		m,
		ipvt,
		qtf,
		wa1,
		wa2,
		wa3,
		wa4
		);

    if (param->info >= 8)
        param->info = 4;

	// *** clean up.

	delete[](fvec);
	delete[](diag);
	delete[](qtf);
	delete[](fjac);
	delete[](wa1);
	delete[](wa2);
	delete[](wa3);
	delete[](wa4);
	delete[](ipvt);

    printf("LM optimization terminated with status code = %d. num_iter = %d\n", param->info, param->nfev);

	return true;
}


bool Optimizer_LM::run(Objective_LM *func, std::vector<double>& x, Parameters *param) {
    return run(func, x.data(), param);
}
