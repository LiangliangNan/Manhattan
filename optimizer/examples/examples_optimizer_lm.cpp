#include <optimizer/optimizer_lm.h>

#include <iostream>
#include <cmath>


namespace data_fitting {
     // for fitting problems, you can provide the "user data"

     // fit the function: f(x0, x1) = exp(a*x0 + b*x1)
     // objective function to be minimized: Sum_i[ exp(a*x0 + b*x1) - f(x0, x1) ]^2

    class Objective : public Objective_LM {
    public:
        Objective(int num_func, int num_var, double* data) : Objective_LM(num_func, num_var, data) {}

        int evaluate(const double *x, double *fvec) {
            double* data = reinterpret_cast<double*>(data_);
            for(int i=0; i<num_func_; ++i)
                fvec[i] = std::exp(x[0] * data[i] + x[1] * data[i+num_func_]) - data[i+num_func_*2];
            return 0;
        }
    };

     bool test() {
        std::cout << "---- data fitting using Optimier_LM ---\n";
        double par[] = {1.0, 1.0};
        double data[] = {
            1.0, 2.0, 3.0,    // x0
            2.0, 3.0, 4.0,    // x1
            2.0, 4.0, 3.0     // y
        };

        Objective obj(3, 2, data);
        Optimizer_LM lm;
        bool status = lm.optimize(&obj, par);

        std::cout << "the solution is: " << par[0] << "  " << par[1] << std::endl;
        std::cout << "expected result: -0.664837  0.807553" << std::endl << std::endl;
        return status;
     }
}


namespace minimization {

    // minimization of a function

    class Objective : public Objective_LM {
    public:
        Objective(int num_func, int num_var) : Objective_LM(num_func, num_var) {}
        int evaluate(const double *x, double *fvec) {
            for(int i=0 ; i<num_func_; ++i)
                fvec[i] = i * x[0] + (i/2.0) * x[1] * x[1];
            return 0;
        }
    };

    bool test() {
        std::cout << "---- function minimization using Optimier_LM ---\n";

        Objective obj(3, 2);
        Optimizer_LM lm;
        std::vector<double> x = {4.0, -4.0};
        bool status = lm.optimize(&obj, x);

        std::cout << "the solution is: " << x[0] << "  " << x[1] << std::endl;
        std::cout << "expected result: -12.3333  -4.96655" << std::endl << std::endl;
        return status;
    }

}


bool test_optimizer_lm() {
    data_fitting::test();
    minimization::test();
    return true;
}
