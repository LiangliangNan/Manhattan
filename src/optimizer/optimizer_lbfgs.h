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

#ifndef MANHATTAN_OPTIMIZER_LBFGS_H
#define MANHATTAN_OPTIMIZER_LBFGS_H

#include <vector>


// objective function, allowing user problem to inherit from it.
class Objective_LBFGS
{
public:

    /**
     * Constructor.
     * @param n    The number of variables.
     * @param data The user data sent to lbfgs() function by the client.
     */
    Objective_LBFGS(const int n, void* data = nullptr);

    virtual ~Objective_LBFGS();

    /**
     *  Evaluate the values of the objective function and its gradients, given current values of variables.
     *  This function will be called internally by the lbfgs() function when needed.
     *  @param x    The current values of variables.
     *  @param g    The gradient vector.
     *  @return     The value of the objective function for the current variables.
     *  @example The following code implements this function that aims at minimizing the Rosenbrock function:
     *      @code
     *          double fx = 0.0;
     *          for (int i = 0; i < num_var_; i += 2) {
     *              double t1 = 1.0 - x[i];
     *              double t2 = 10.0 * (x[i+1] - x[i] * x[i]);
     *              g[i+1] = 20.0 * t2;
     *              g[i] = -2.0 * (x[i] * g[i+1] + t1);
     *              fx += t1 * t1 + t2 * t2;
     *          }
     *          return fx;
     *      @endcode
     */
    inline virtual double evaluate(const double *x, double *g) = 0;

private:
    int     num_var_;
    void*   data_;
};


// More optimizers are available in ALGLIB

class Optimizer_LBFGS
{
public:
    Optimizer_LBFGS();
    virtual ~Optimizer_LBFGS();

    /**
     * The core function that minimizes the objective function.
     * @param func the objective function.
     * @param n    number of variables
     * @param x    the variables which provide the initial guess and return the results.
     * @return ture on success and false on failure.
     */
    bool run(Objective_LBFGS* func, int n, double* x);
    bool run(Objective_LBFGS* func, int n, std::vector<double>& x);

    /// on success, allow user to query the final value of the objective function.
    double objective() const;

    /// on success, allow user to query the number of iterations of the optimization process
    int iterations() const;

protected:

    /* The interface to receive the progress of the optimization process.
     * The function returns zero to continue the optimization process.
     * Returning a non-zero value will cancel the optimization process.
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


#endif  // MANHATTAN_OPTIMIZER_LBFGS_H
