/*
Copyright (C) 2017  Liangliang Nan
https://3d.bk.tudelft.nl/liangliang/ - liangliang.nan@gmail.com

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


#include <optimizer/optimizer_lm.h>
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


bool Optimizer_LM::optimize(Objective_LM *func, double *x, Parameters *param)
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


bool Optimizer_LM::optimize(Objective_LM *func, std::vector<double>& x, Parameters *param) {
    return optimize(func, x.data(), param);
}
