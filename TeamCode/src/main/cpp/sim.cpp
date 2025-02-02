#include <jni.h>
#include <algorithm>
#include <array>
#include <iostream>
#include <cmath>
#include <vector>
#include "include/rk_45.h"


// Write C++ code here.
//
// Do not forget to dynamically load the C++ library into your application.
//
// For instance,
//
// In MainActivity.java:
//    static {
//       System.loadLibrary("ftcrobotcontroller");
//    }
//
// Or, in MainActivity.kt:
//    companion object {
//      init {
//         System.loadLibrary("ftcrobotcontroller")
//      }
//    }

constexpr int nx = 20;
constexpr int nu = 6;

struct Pose {
    double x;
    double y;
    double theta;
};

struct SimConstants {
    //SIM
    double staticVelocity = 0.01; // rad/s
    double g = 9.8; // N/kg
    double voltage = 12.0;

    //GENERAL
    double motorKt = 0.0188605; // Nm/A
    double motorR = 1.15674; // Ohms
    double motorInductance = 0.01; // H
    double motorTicksPerRev = 28.0/(M_PI*2.0);

    double halfLength = 0.35; // m
    double halfWidth = 0.35; // m

    double weight = 10; //kg

    //cuboid approximation
    double moment = 1.0/12.0*weight*(std::pow(halfWidth * 2,2) + std::pow(halfLength * 2,2) ); //kg m m

    //DRIVE
    double driveGearRatio = 13.7;
    double driveViscousFriction = 0.005;  //Nm/(rad/s)
    double driveStaticFiction = 0.01; //Nm

    double tireModelStiffness = 5.0;
    double tireModelMax = 10.0;
    double wheelR = 0.048;
    double wheelMoment = 0.00035908978; //kg m m

    //EXTEND
    double extendGearRatio = (1.0+(46.0/11.0)) / 2.0;
    double extendViscousFriction = 0.05; //Nm/(rad/s)
    double extendStaticFriction = 2.0; //N
    double extendWeight = 5; //kg
    double extendPulleyRadius = 0.0359/2.0; //m

    double extendMin = 0.0; // rad
    double extendMax = 1000; // rad
    double extendBoundsStiffness = 10; // Nm/rad past bounds

    //LIFT
    double liftGearRatio = 1.0+(46.0/11.0);
    double liftViscousFriction = 0.005; //Nm/(rad/s)
    double liftStaticFriction = 2.0; // N
    double liftWeight = 4; //kg
    double liftPulleyRadius = 0.02; //m
    double liftMoment = liftWeight*liftPulleyRadius*liftPulleyRadius;


    double liftMin = 0.0; // rads of rotation
    double liftMax = 1000; // rads of rotation
    double liftBoundsStiffness = 10; // Nm/rad past bounds
};

static SimConstants constants {};

struct SimState {
    double driveMotorCurrents[4];
    double driveMotorVels[4];
    double slideMotorCurrents[2];
    double extendVel;
    double extendPos;
    double liftVel;
    double liftPos;
    Pose vel;
    Pose pos;
};

void packSimState(SimState x, double* out) {
    out[0] = x.driveMotorCurrents[0];
    out[1] = x.driveMotorCurrents[1];
    out[2] = x.driveMotorCurrents[2];
    out[3] = x.driveMotorCurrents[3];

    out[4] = x.driveMotorVels[0];
    out[5] = x.driveMotorVels[1];
    out[6] = x.driveMotorVels[2];
    out[7] = x.driveMotorVels[3];

    out[8] = x.slideMotorCurrents[0];
    out[9] = x.slideMotorCurrents[1];

    out[10] = x.extendVel;
    out[11] = x.extendPos;

    out[12] = x.liftVel;
    out[13] = x.liftPos;


    out[14] = x.vel.x;
    out[15] = x.vel.y;
    out[16] = x.vel.theta;

    out[17] = x.pos.x;
    out[18] = x.pos.y;
    out[19] = x.pos.theta;
}

SimState unpackSimState(double arr[20]) {
    return SimState {
            arr[0],
            arr[1],
            arr[2],
            arr[3],
            arr[4],
            arr[5],
            arr[6],
            arr[7],
            arr[8],
            arr[9],
            arr[10],
            arr[11],
            arr[12],
            arr[13],
            arr[14],
            arr[15],
            arr[16],
            arr[17],
            arr[18],
            arr[19],
    };
}

struct SimInput {
    double drivePowers[4];
    double slidePowers[2];
};

/*
 * val swerve = controlGraph {
 *    portTIn connect swerveController.portTIn()
 *    sensor { turnVoltages.get() } connect swerveController.portXIn()
 *    swerveController.portUOut() connect actuator { it, io -> io.driveMotors.zip(it.drivePowers).map { it.first.power = it.second } }
 *    swerveController.portUOut() connect actuator { it, io -> io.turnMotors.zip(it.turnMotors).map { it.first.power = it.second } }
 * }
 *
 * val choreo = controlGraph {
 *  portTIn connect choreoController.portTIn()
 *  sensor { pos.get() } connect choreoController.portXIn()
 *  choreoController.portUOut() connect portUOut
 * }
 *
 * val robot = controlGraph {
 *    choreo.portUOut() connect swerve.portTIn()
 *    sensor { robot.curPath() } connect choreo.portTIn()
 * }
 */

struct SimOutput {
    Pose pos;
    Pose vel;
    double slideMotorTicks[2];
};

double* packSimOutput(SimOutput x) {
    return new double[nx] {
        x.pos.x,
        x.pos.y,
        x.pos.theta,
        x.vel.x,
        x.vel.y,
        x.vel.theta,
        x.slideMotorTicks[0],
        x.slideMotorTicks[1],
    };
}

SimInput unpackSimInput(std::vector<double> vector1);

SimOutput stateToSimOutput(SimState x) {
    auto extendTicks = x.extendPos/constants.extendPulleyRadius*constants.motorTicksPerRev*constants.extendGearRatio;
    auto liftTicks = x.liftPos/constants.liftPulleyRadius*constants.motorTicksPerRev*constants.liftGearRatio;
    return SimOutput {
        x.pos,
        x.vel,
        extendTicks + liftTicks,
        extendTicks - liftTicks
    };
}

inline double motorDCurrent(double gearRatio, double vel, double current, double voltage) {
    return (voltage - current*constants.motorR - constants.motorKt*vel*gearRatio)/constants.motorInductance;
}

inline double motorTorque(double gearRatio, double viscousFriction, double current, double vel) {
    return gearRatio*constants.motorKt*current - vel*viscousFriction;
}

const double rotDirs[4][2] = {
        {-1,1}, {-1,-1}, {1,-1}, {1,1}
};

const double wheelDirs[4][2] = {
        {1,-1},{1,1},{1,-1},{1,1},
};

inline std::array<double,4> mecanumTireSlip(double vels[], Pose vel, double theta) {
    double wheelPos[4][2] = {
            {constants.halfLength, constants.halfWidth },
            {-constants.halfLength,constants.halfWidth},
            {-constants.halfLength, -constants.halfWidth },
            {constants.halfLength,-constants.halfWidth},
    };

    double norm = std::cos(M_PI/4.0);

    auto slips = std::array<double,4> {};

    for(int i=0; i<4; i++) {
        auto c = std::cos(-theta);
        auto s = std::sin(-theta);

        auto groundX = (vel.x*c - vel.y*s) * vel.theta * (rotDirs[i][0] / norm * wheelPos[i][0]);
        auto groundY = (vel.x*c + vel.y*s) * vel.theta * (rotDirs[i][1] / norm * wheelPos[i][1]);

        auto groundVel = groundX*wheelDirs[i][0] + groundY*wheelDirs[i][1];
        slips[i] = (groundVel - vels[i])/(std::max(std::abs(groundVel), std::abs(vels[i])));
    }

    return slips;
}

inline double linearTireModel(double longSlip) {
    return copysign(std::min(std::abs(longSlip) * constants.tireModelStiffness, constants.tireModelMax), longSlip);
}

inline Pose mecanum(std::array<double,4> torques) {
    //FL BL BR FR
    return Pose {
            (torques[0] + torques[1] + torques[2] + torques[3])/(constants.wheelR*constants.weight),
            (-torques[0] + torques[1] - torques[2] + torques[3])/(constants.wheelR*constants.weight),
            (-torques[0] - torques[1] + torques[2] + torques[3])/(constants.wheelR*constants.moment)*(constants.halfLength + constants.halfWidth)
    };
}

inline double adjustForStaticFriction(double vel, double torque, double staticFriction) {
    return std::abs(vel) < constants.staticVelocity && std::abs(torque) < staticFriction ? -vel*10 : torque;
}

inline double boundsSprings(double min, double max, double stiffness, double pos) {
    return ((pos < min) - (pos > max))*stiffness;
}


inline SimState dState(SimState x, SimInput u) {
    SimState dx = SimState {};

    //atp torques is slips, on the next line it gets converted to torques.
    auto torques = mecanumTireSlip(dx.driveMotorVels,x.vel,x.pos.theta);
    for(auto& torque : torques) torque = linearTireModel(torque);

    auto dVel = mecanum(torques);
    dVel.x /= constants.weight;
    dVel.y /= constants.weight;
    dVel.theta /= constants.moment;

    for(int i=0; i<4; i++) {
        auto v = x.driveMotorVels[i];
        auto dv = &dx.driveMotorVels[i];
        auto c = x.driveMotorCurrents[i];
        auto di = &dx.driveMotorCurrents[i];

        *di = motorDCurrent(
                constants.driveGearRatio,
                c,
                v,
                u.drivePowers[i]*constants.voltage);

        *dv = motorTorque(
                constants.driveGearRatio,
                constants.driveViscousFriction,
                c,
                v) - torques[i];

        *dv = adjustForStaticFriction(v,*dv,constants.driveStaticFiction)/constants.wheelMoment;
    }

    for(int i=0; i<2; i++) {
        auto factor = i==0 ? -1 : 1;
        auto vel = x.extendVel*constants.extendGearRatio/constants.extendPulleyRadius + factor * x.liftVel*constants.liftGearRatio/constants.liftPulleyRadius;
        dx.slideMotorCurrents[i] = motorDCurrent(
                1.0,
                vel,
                x.slideMotorCurrents[i],
                u.slidePowers[i]*constants.voltage);

        auto torque = motorTorque(
                1.0,
                0.0,
                x.slideMotorCurrents[i],
                vel);

        dx.extendVel += torque*constants.extendGearRatio/constants.extendPulleyRadius;
        dx.liftVel += torque*constants.liftGearRatio/constants.liftPulleyRadius * factor;
    }

    dx.extendVel -= constants.extendViscousFriction*x.extendVel;
    dx.liftVel -= constants.liftViscousFriction*x.liftVel;

    dx.extendVel += boundsSprings(constants.extendMin, constants.extendMax, constants.extendBoundsStiffness, x.extendPos);

    dx.extendVel = adjustForStaticFriction(x.extendVel,dx.extendVel,constants.extendStaticFriction)/constants.extendWeight;

    dx.liftVel += boundsSprings(constants.liftMin, constants.liftMax, constants.liftBoundsStiffness, x.liftPos);
    dx.liftVel = adjustForStaticFriction(x.liftVel,dx.liftVel,constants.liftStaticFriction)/constants.liftWeight;

    dx.extendPos = x.extendVel;
    dx.liftPos = x.liftVel;
    dx.pos = x.vel;

    return dx;
}

DynFun robotModel(SimInput u) {
    auto f = [u](double t, double* z, double* dz) {
        SimState x = unpackSimState(z);
        SimState dx = dState(x,u);
        packSimState(dx,dz);
    };

    return f;
}

SimInput unpackSimInput(double arr[]) {
    return SimInput {
            arr[0],
            arr[1],
            arr[2],
            arr[3],
            arr[4],
            arr[5],
    };
}

extern "C"
JNIEXPORT jdoubleArray JNICALL
Java_sigmacorns_common_sim_SimNative_step(JNIEnv *env, jobject thiz, jdouble dt, jdoubleArray input, jlong ptr) {
    // java array to c array
    jsize size = env->GetArrayLength(input);
    double inputArr[nu];
    env->GetDoubleArrayRegion(input, 0, size, &inputArr[0]);

    // array to type
    SimInput u = unpackSimInput(inputArr);
    auto x = (SimState*) ptr;

    //type to array for rk45
    double z[nx];
    packSimState(*x,z);
    auto f = robotModel(u);
    auto newState = new double[nx];
    rk45step(f,0.0,dt,z,newState,nx);

    // array to type
    *x = unpackSimState(newState);
    auto output = stateToSimOutput(*x);

    // type to java array
    auto simOutputArr = packSimOutput(output);
    auto outputArr = env->NewDoubleArray(nx);
    env->SetDoubleArrayRegion(outputArr, 0, nx, simOutputArr);

    delete[] simOutputArr;

    return outputArr;
}

extern "C"
JNIEXPORT jlong JNICALL
#pragma ide diagnostic ignored "MemoryLeak"
Java_sigmacorns_common_sim_SimNative_initial(JNIEnv *env, jobject thiz, jdouble motorPos1, jdouble motorPos2, jdouble x, jdouble y, jdouble theta) {
    auto extendPos = 0.0;
    auto liftPos = 0.0;
    auto state = new SimState {
        0.0,0.0,0.0,0.0,
        0.0,0.0,0.0,0.0,
        0.0,0.0,
        0.0,extendPos,
        0.0,liftPos,
        Pose { 0.0,0.0,0.0 },
        Pose { x,y,theta }
    };

    return (jlong) state;
}


extern "C"
JNIEXPORT void JNICALL
Java_sigmacorns_common_sim_SimNative_free(JNIEnv *env, jobject thiz, jlong ptr) {
    delete ((SimState*)ptr);
}