#include <optimizer/optimizer_lbfgs.h>

#include <iostream>
#include <vector>
#include <cmath>


#if 0

bool test_optimizer_lbfgs() {
    std::cout << "---- function minimization using Optimier_LBFGS ---\n";

    const int N = 10000;
    std::vector<double> x(N, 0); // initial values are all zeros

    Objective_LBFGS obj(N);

    Optimizer_LBFGS opt;
    if (opt.run(&obj, N, x.data())) {
        /* Report the result. */
        printf("  fx = %f, x[0] = %f, x[1] = %f\n\n", opt.objective(), x[0], x[1]);
        return true;
    }
    else
        return false;
}

#else

    bool test_optimizer_lbfgs() {
        std::cout << "---- function minimization using Optimier_LBFGS ---\n";

        std::vector<double> x(2, 0); // initial values are all zeros

        class Objective : public Objective_LBFGS
        {
        public:
            Objective (int n) : Objective_LBFGS(n) {}
            double evaluate(const double *x, double *g) {
                double t1 = x[0] - 0.5;
                double t2 = x[1] - 1.0;
                g[0] = 2 * t1;
                g[1] = 2 * t2;
                return t1 * t1 + t2 * t2;
            }
        };

        Objective obj(2);

        Optimizer_LBFGS opt;
        if (opt.run(&obj, 2, x.data())) {
            /* Report the result. */
            printf("  fx = %f, x[0] = %f, x[1] = %f\n\n", opt.objective(), x[0], x[1]);
            return true;
        }
        else
            return false;
    }
#endif
