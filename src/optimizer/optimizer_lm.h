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


#ifndef MANHATTAN_OPTIMIZER_LM_H
#define MANHATTAN_OPTIMIZER_LM_H

#include <vector>

/**
Optimizer_LM for nonlinear least squares problems using Levenberg-Marquardt method.
It wraps the lmdif() part of cminpack (see http://devernay.free.fr/hacks/cminpack/index.html)

min Sum_{i=0}^{M} ( F(x0,..,xN)_i )^2 )
Where: 
	F: R^n->R^m is a user defined function

****** basic examples *******

1) minimization of a function (other values can be provided via "data")

    class Objective : public Objective_LM {
    public:
        Objective(int num_func, int num_var) : Objective_LM(num_func, num_var) {}

        // expected result: -12.3333 -4.96655
        int evaluate(const double *x, double *fvec) {
            for(int i=0 ; i<num_func_; ++i)
                fvec[i] = i * x[0] + (i/2.0) * x[1] * x[1];
        }
    };

    bool test() {
        Objective obj(3, 2);
        Optimizer_LM lm;
        std::vector<double> x = {4.0, -4.0};
        bool status = lm.optimize(&obj, x);
        std::cout << "the solution is: " << x[0] << "  " << x[1] << std::endl;
        return status;
    }


*********************************

2) for fitting problems, you can provide the "user data"

 // fit the function: f(x1, x2) = exp(a*x1 + b*x2)
 // objective function to be minimized: Sum_i[ exp(a*x1 + b*x2) - f(x1, x2) ]
 
    class Objective : public Objective_LM {
    public:
        Objective(int num_func, int num_var, double* data) : Objective_LM(num_func, num_var, data) {}

        // expected result: -0.664837  0.807553
        int evaluate(const double *x, double *fvec) {
            double* data = reinterpret_cast<double*>(data_);
            for(int i=0; i<num_func_; ++i)
                fvec[i] = std::exp(x[0] * data[i] + x[1] * data[i+num_func_]) - data[i+num_func_*2];
        }
    };

     bool test_data_fitting() {
        double par[] = {1.0, 1.0};
        double data[] = {
            1.0, 2.0, 3.0,    // t0
            2.0, 3.0, 4.0,    // t1
            2.0, 4.0, 3.0     // y
        };

        Objective obj(3, 2, data);
        Optimizer_LM lm;
        bool status = lm.optimize(&obj, par);

        std::cout << "the solution is: " << par[0] << "  " << par[1] << std::endl;
        // the results are: -0.664837  0.807553
        return status;
     }
*/


class Objective_LM
{
public:
    //  @param  num_func    The number of functions
    //  @param  num_var     The number of variables.
    //  @param  data        User data
    Objective_LM(int num_func, int num_var, void* data = nullptr);

    virtual ~Objective_LM();

    int num_function() const { return num_func_; }
    int num_variables() const { return num_var_; }

    /**
     *  Calculate the values of each function at x and return this vector in f. A client problem must implement this
     *  function to evaluate the values of each function.
     *  @param  x       The current values of variables.
     *  @param  fvec    Return the value vector of all the functions.
     *  @param  data    The user data sent to the optimizer by the client.
     *  @return         0 on success or a negative value to terminate.
     *  @example Below is an example that aims at minimizing the objective function E = (x0 - 1.0)^2 + (x1 - 1.0)^2:
     *      @code
     *          fvec[0] = x[0] - 1.0;
     *          fvec[1] = x[1] - 1.0;
     *          return 0;
     *      @endcode
     */
    inline virtual int evaluate(const double *x, double *fvec) = 0;

protected:
    int     num_func_;
    int     num_var_;
    void*   data_;
};



// More optimizers are available in ALGLIB
class Optimizer_LM
{
public:
	Optimizer_LM();
    virtual ~Optimizer_LM();

	// parameters for calling the high-level interface functions
    struct Parameters {
        Parameters();
		double ftol; 		// relative error desired in the sum of squares.
		double xtol; 		// relative error between last two approximations.
		double gtol; 		// orthogonality desired between fvec and its derivs.
        double epsilon; 	// step used to calculate the Jacobian.
		double stepbound; 	// initial bound to steps in the outer loop.
		double fnorm; 		// norm of the residue vector fvec.
		int maxcall; 		// maximum number of iterations.
        int nfev; 			// actual number of iterations.
        int nprint; 		// desired frequency of print outs.
		int info; 			// status of minimization.
	} ;

public:

    //  func:   your evaluate function (no need to provide Jacobian)
    //  x:      the variable vector (should be initialized with guess), which also returns the result.
    //  param:  parameter for the optimizer (use default parameters if para is null).
    bool optimize(Objective_LM* func, double* x, Parameters* para = nullptr);
    bool optimize(Objective_LM* func, std::vector<double>& x, Parameters* para = nullptr);

private:
    Parameters default_control_;		// control of this object
    Objective_LM *func_;

private:
    //copying disabled
    Optimizer_LM(const Optimizer_LM&);
    Optimizer_LM& operator=(const Optimizer_LM&);
};


#endif  // MANHATTAN_OPTIMIZER_LM_H
