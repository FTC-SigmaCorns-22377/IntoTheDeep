#include <iostream>
#include <fstream>
using namespace std;

#include <cmath>
#include <vector>

#include "integrator.h"
//#include "RK_2.h"
//#include "RK_4A.h"
//#include "RK_4B.h"
#include "RK_45.h"
//#include "RK_5.h"
//#include "RK_10.h"


/******************************************************************************
 *                           Utility Functions                                *
 ******************************************************************************/

/* Prints the current time and state to the log file */
void printState(std::ofstream& file, double t, double z[], int nDim) {
    file << t ;
    for (int j = 0; j < nDim; j++) {
        file << ", " << z[j];
    }
    file << "\n";
}


/******************************************************************************
 *                     Hard-Coded Low-Order Methods                           *
 ******************************************************************************/

/* Takes a simple euler step for the system */
void eulerStep(DynFun dynFun, double t0, double t1, double z0[], double z1[], int nDim) {

    double dt = t1 - t0;
    double *dz;
    dz = new double[nDim];

    dynFun(t0, z0, dz);

    for (int i = 0; i < nDim; i++) {
        z1[i] = z0[i] + dt * dz[i];
    }

    delete [] dz;

}


/* Time step using the mid-point method */
void midPointStep(DynFun dynFun, double tLow, double tUpp, double zLow[], double zUpp[], int nDim) {

    double dt = tUpp - tLow;

    /// Stage 0
    double t0 = tLow;
    double *z0 = zLow;
    double *f0; f0 = new double[nDim];
    dynFun(t0, z0, f0);

    /// Stage 1
    double t1 = t0 + 0.5 * dt;
    double *z1; z1 = new double[nDim];
    double *f1; f1 = new double[nDim];
    for (int i = 0; i < nDim; i++) {
        z1[i] = z0[i] + 0.5 * dt * f0[i];
    }
    dynFun(t1, z1, f1);

    /// Collect Stages:
    for (int i = 0; i < nDim; i++) {
        zUpp[i] = zLow[i] + dt * f1[i];
    }

    delete [] f0;
    delete [] f1;
    delete [] z1;

}


/* Time step using 4th-order "Classical" Runge Kutta */
void rungeKuttaStep(DynFun dynFun, double tLow, double tUpp, double zLow[], double zUpp[], int nDim) {

    double dt = tUpp - tLow;

    /// Stage 0
    double t0 = tLow;
    double *z0 = zLow;
    double *f0; f0 = new double[nDim];
    dynFun(t0, z0, f0);

    /// Stage 1
    double t1 = t0 + 0.5 * dt;
    double *z1; z1 = new double[nDim];
    double *f1; f1 = new double[nDim];
    for (int i = 0; i < nDim; i++) {
        z1[i] = zLow[i] + 0.5 * dt * f0[i];
    }
    dynFun(t1, z1, f1);

    /// Stage 2
    double t2 = t1;
    double *z2; z2 = new double[nDim];
    double *f2; f2 = new double[nDim];
    for (int i = 0; i < nDim; i++) {
        z2[i] = zLow[i] + 0.5 * dt * f1[i];
    }
    dynFun(t2, z2, f2);

    /// Stage 3
    double t3 = tLow + dt;
    double *z3; z3 = new double[nDim];
    double *f3; f3 = new double[nDim];
    for (int i = 0; i < nDim; i++) {
        z3[i] = zLow[i] + dt * f2[i];
    }
    dynFun(t3, z3, f3);

    /// Collect Stages:
    for (int i = 0; i < nDim; i++) {
        zUpp[i] = zLow[i] + (dt / 6) * (f0[i] + 2.0 * f1[i] + 2.0 * f2[i] + f3[i]);
    }

    delete [] f0;
    delete [] f1;
    delete [] z1;
    delete [] f2;
    delete [] z2;
    delete [] f3;
    delete [] z3;

}



/******************************************************************************
 *                      General-Form Runge-Kutta Step                         *
 ******************************************************************************/


/* General-Purpose Runge--Kutta integration step, using a Butcher table.
 * tLow = time at beginning of the step
 * tUpp = time at the end of the step
 * zLow = state at the beginning of the step
 * zUpp = state at the end of the step (unknown  --  Computed by this function)
 * nDim = dimension of the state space
  * A[] gives the time coefficients for the method
 * B[] gives the state propagation coefficients (assume lower triangular matrix)
 * C[] gives the solution coefficients for the method
 * nStage = number of stages in the Runge--Kutta method
 * Look at the example code to understand formatting for these inputs.
 */
void RK_STEP(DynFun dynFun,
             double tLow, double tUpp, double zLow[], double zUpp[], int nDim,
             double A[], double B[], double C[], int nStage)
{
    /// Allocate memory using vectors instead of VLAs:
    std::vector<double> t(nStage);
    std::vector<std::vector<double>> z(nStage, std::vector<double>(nDim));
    std::vector<std::vector<double>> f(nStage, std::vector<double>(nDim));

    /// Populate time grid:
    double dt = tUpp - tLow;
    for (int iStage = 0; iStage < nStage; iStage++) {
        t[iStage] = tLow + dt * A[iStage];
    }

    /// Initial State:
    for (int iDim = 0; iDim < nDim; iDim++) {
        z[0][iDim] = zLow[iDim];
    }

    /// Dynamics at initial point:
    dynFun(t[0], z[0].data(), f[0].data());

    /// March through each stage:
    double sum;
    int idx;
    for (int iStage = 1; iStage < nStage; iStage++) {
        for (int iDim = 0; iDim < nDim; iDim++) {
            sum = 0.0;
            for (int j = 0; j < iStage; j++) {
                idx = iStage * (iStage - 1) / 2 + j;   // Triangle numbers
                sum += B[idx] * f[iStage - 1][iDim];
            }
            z[iStage][iDim] = zLow[iDim] + dt * sum;
        }
        dynFun(t[iStage], z[iStage].data(), f[iStage].data());
    }

    /// Compute the final estimate:
    for (int iDim = 0; iDim < nDim; iDim++) {
        sum = 0.0;
        for (int iStage = 0; iStage < nStage; iStage++) {
            sum += C[iStage] * f[iStage][iDim];
        }
        zUpp[iDim] = zLow[iDim] + dt * sum;
    }
}
/******************************************************************************
 *                      Simulation Wrapper Function                           *
 ******************************************************************************/

/* Runs several time steps using euler integration */
void simulate(DynFun dynFun, double t0, double t1, double z0[], double z1[],
              int nDim, int nStep, IntegrationMethod method)
{
    double dt, tLow, tUpp;
    double *zLow;
    double *zUpp;

    /// Allocate memory:
    zLow = new double[nDim];
    zUpp = new double[nDim];

    /// File IO stuff:
    ofstream logFile;
    logFile.open("logFile.csv");

    /// Initial conditions
    tLow = t0;
    for (int i = 0; i < nDim; i++) {
        zLow[i] = z0[i];
    }

    /// March forward in time:
    dt = (t1 - t0) / ((double) nStep);
    for (int i = 0; i < nStep; i++) {
        tUpp = tLow + dt;
        switch (method) {
            case Euler:
                eulerStep(dynFun, tLow, tUpp, zLow, zUpp, nDim); break;
            case MidPoint:
                midPointStep(dynFun, tLow, tUpp, zLow, zUpp, nDim); break;
            case RungeKutta:
                rungeKuttaStep(dynFun, tLow, tUpp, zLow, zUpp, nDim); break;
//            case RK_2:
//                rk2step(dynFun, tLow, tUpp, zLow, zUpp, nDim); break;
//            case RK_4A:
//                rk4Astep(dynFun, tLow, tUpp, zLow, zUpp, nDim); break;
//            case RK_4B:
//                rk4Bstep(dynFun, tLow, tUpp, zLow, zUpp, nDim); break;
            case RK_45:
                rk45step(dynFun, tLow, tUpp, zLow, zUpp, nDim); break;
//            case RK_5:
//                rk5step(dynFun, tLow, tUpp, zLow, zUpp, nDim); break;
//            case RK_10:
//                rk10step(dynFun, tLow, tUpp, zLow, zUpp, nDim); break;
        }

        /// Print the state of the simulation:
        printState(logFile, tLow, zLow, nDim);

        /// Advance temp variables:
        tLow = tUpp;
        for (int j = 0; j < nDim; j++) {
            zLow[j] = zUpp[j];
        }
    }
    printState(logFile, tLow, zLow, nDim);

    delete [] zLow;
    delete [] zUpp;

    logFile.close();

}