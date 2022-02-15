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


#ifndef SOLVER_SUITE_OPTIMIZER_LBFGS_H
#define SOLVER_SUITE_OPTIMIZER_LBFGS_H

#include <vector>

#include "optimizer_common.h"


// objective function, allowing user problem to inherit from it.

class OPTIMIZER_API Objective_LBFGS
{
public:

    //  @param  n      The number of variables.
    //  @param  data   The user data sent to lbfgs() function by the client.
    Objective_LBFGS(const int n, void* data = nullptr);

    virtual ~Objective_LBFGS();

    /**
     *  The lbfgs() function call this function to obtain the values of objective
     *  function and its gradients when needed, given current values of variables.
     *
     *  @param  x           The current values of variables.
     *  @param  g           The gradient vector. The callback function must compute
     *                      the gradient values for the current variables.
     *  It returns the value of the objective function for the current variables.
     *
     *  NOTE: This function implements the Rosenbrock function. A client problem
     *        must implement this function to evaluate the values of the objective
     *        function and its gradients.
     */
    inline virtual double evaluate(const double *x, double *g);

private:
    int     num_var_;
    void*   data_;
};


// More optimizers are available in ALGLIB

class OPTIMIZER_API Optimizer_LBFGS
{
public:
    Optimizer_LBFGS();
    virtual ~Optimizer_LBFGS();

    // minimize the objective function.
    // @param  func: the objective function.
    // @param  n: number of variables
    // @param  x: the variables whith provide the initial guess and return the results.
    // return ture on success and false on failure.
    bool run(Objective_LBFGS* func, int n, double* x);
    bool run(Objective_LBFGS* func, int n, std::vector<double>& x);

    // on success, allow user to query the final value of the objective function.
    double objective() const;

    // on success, allow user to query the number of iterations of the optimization process
    int iterations() const;

protected:

    /* The interface to receive the progress of the optimization process.
    *  The function returns zero to continue the optimization process.
    *  Returning a non-zero value will cancel the optimization process.
    */
    virtual int progress(
        const double *x,    // The current values of variables.
        const double *g,    // The current gradient values of variables.
        const double fx,    // The current value of the objective function.
        const double xnorm, // The Euclidean norm of the variables.
        const double gnorm, // The Euclidean norm of the gradients.
        const double step,  // The line-search step used for this iteration.
        int n,  // The number of variables.
        int k,  // The iteration count.
        int ls  // The number of evaluations called for this iteration.
        );

protected:
    double obj_value_;
    int    iteration_;
    Objective_LBFGS* func_;

private:
    //copying disabled
    Optimizer_LBFGS(const Optimizer_LBFGS&);
    Optimizer_LBFGS& operator=(const Optimizer_LBFGS&);
};


#endif  // SOLVER_SUITE_OPTIMIZER_LBFGS_H
