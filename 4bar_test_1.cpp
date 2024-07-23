#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

Eigen::VectorXd calcMotorAngle(Eigen::VectorXd& jointAngle_);

int main(int argc, char* argv[]) {
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);

    raisim::World world;
    // auto laikago = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\laikago\\laikago.urdf");
    // auto ball = world.addSphere(0.1, 1);

    auto ground = world.addGround(0);
    auto DHAL = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\DHAL_4bar\\DHAL_4bar.urdf");
    const double dt = 0.002;
    world.setTimeStep(dt);

    /// launch raisim server for visualization. Can be visualized using raisimUnity
    raisim::RaisimServer server(&world);
    server.launchServer();

    double R2D = 180/M_PI;
    double D2R = M_PI/180;

    int gcInitDim = DHAL->getGeneralizedCoordinateDim();
    int gvInitDim = DHAL->getGeneralizedVelocityDim();
    std::cout << "gcInitDim: " << gcInitDim << std::endl;
    std::cout << "gvInitDim: " << gvInitDim << std::endl;

    Eigen::VectorXd gcInit(gcInitDim);
    Eigen::VectorXd gvInit(gvInitDim);
    Eigen::VectorXd jointAngle(12), motorAngle(12);

    gcInit.setZero();
    gvInit.setZero();
    gcInit.segment(0,7) << 0.0, 0.0, 0.9, 1.0, 0.0, 0.0, 0.0;
    gcInit.segment(7,6) << 0.0, 0.0, -5*D2R, 30*D2R, -25*D2R, 0.0;
    gcInit.segment(21,6) << 0.0, 0.0, -5*D2R, 30*D2R, -25*D2R, 0.0;
    gcInit(35) = 0.0;

    jointAngle << gcInit.segment(7,6), gcInit.segment(21,6);
    motorAngle = calcMotorAngle(jointAngle);

    std::cout << jointAngle << std::endl;
    std::cout << "-----------" << std::endl;
    std::cout << motorAngle << std::endl;
    std::cout << "===========" << std::endl;

    gcInit(7) = motorAngle(0);
    gcInit(8) = motorAngle(1);
    gcInit(9) = motorAngle(2);
    gcInit(19) = motorAngle(3);
    gcInit(17) = motorAngle(4);
    gcInit(18) = motorAngle(5);
    gcInit(21) = motorAngle(6);
    gcInit(22) = motorAngle(7);
    gcInit(23) = motorAngle(8);
    gcInit(33) = motorAngle(9);
    gcInit(31) = motorAngle(10);
    gcInit(32) = motorAngle(11);

    double jointPGainVal = 200.0;
    Eigen::VectorXd jointPgain(DHAL->getDOF());
    Eigen::VectorXd jointDgain(DHAL->getDOF());
    jointPgain << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  jointPGainVal, jointPGainVal, jointPGainVal, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  jointPGainVal, jointPGainVal, jointPGainVal,
                  0.0,
                  jointPGainVal, jointPGainVal, jointPGainVal, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  jointPGainVal, jointPGainVal, jointPGainVal,
                  0.0,
                  jointPGainVal;
    jointDgain << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  50.0, 50.0, 50.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  50.0, 50.0, 50.0,
                  0.0,
                  50.0, 50.0, 50.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  50.0, 50.0, 50.0,
                  0.0,
                  50.0;

    // DHAL->setGeneralizedCoordinate(gcInit);
    // DHAL->setGeneralizedVelocity(gvInit);

    for (int i = 0; i < 1000000; i++) {

        // // DHAL->setGeneralizedCoordinate(gcInit);
        // // DHAL->setGeneralizedVelocity(gvInit);
        Eigen::VectorXd gcCurrent = DHAL->getGeneralizedCoordinate().e();
        Eigen::VectorXd gvCurrent = DHAL->getGeneralizedVelocity().e();
        gcCurrent.segment(0,7) << 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0;
        gvCurrent.segment(0,6).setZero();

        // if (i < 1000) {
        DHAL->setGeneralizedCoordinate(gcCurrent);
        DHAL->setGeneralizedVelocity(gvCurrent);
        // }

        Eigen::VectorXd gcInput(gcInitDim);
        Eigen::VectorXd gvInput(gvInitDim);
        jointAngle << gcInit.segment(7, 6), gcInit.segment(21, 6);

        double tOffset = 5000;
        if (i > tOffset) {
            double maxAngle = 0.0*D2R; // [rad]
            double T = 3.0;
            int targetJointNum1 = 4;
            int targetJointNum2 = 10;
            double angle1 = maxAngle * sin(2*M_PI*((i-tOffset)/(T/dt)));
            double angle2 = maxAngle * sin(2*M_PI*((i-tOffset)/(T/dt)));

            jointAngle(targetJointNum1) += angle1;
            jointAngle(targetJointNum2) += angle2;
        }
        motorAngle = calcMotorAngle(jointAngle);

        gcInput(7) = motorAngle(0);
        gcInput(8) = motorAngle(1);
        gcInput(9) = motorAngle(2);
        gcInput(19) = motorAngle(3);
        gcInput(17) = motorAngle(4);
        gcInput(18) = motorAngle(5);
        gcInput(21) = motorAngle(6);
        gcInput(22) = motorAngle(7);
        gcInput(23) = motorAngle(8);
        gcInput(33) = motorAngle(9);
        gcInput(31) = motorAngle(10);
        gcInput(32) = motorAngle(11);

        Eigen::VectorXd targetPosition(gcInitDim);
        Eigen::VectorXd targetVelocity(gvInitDim);
        targetPosition = gcInput;
        targetVelocity.setZero();

        DHAL->setPdGains(jointPgain,jointDgain);
        DHAL->setPdTarget(targetPosition,targetVelocity);

        // if (i % 100 == 0 && i != 0) {
        //     std::cout << "Angle: " << gcInit(7+targetJointNum) + angle << "    :   " << "Model: " << gcCurrent(7+targetJointNum) << std::endl;
        // }

        RS_TIMED_LOOP(world.getTimeStep()*1e6);
        server.integrateWorldThreadSafe();
    }

    server.killServer();
}

Eigen::VectorXd calcMotorAngle(Eigen::VectorXd& jointAngle_) {
    double R2D = 180/M_PI;
    double D2R = M_PI/180;

    Eigen::VectorXd motorAngle(12);

    double leftHipYaw = jointAngle_(0);
    double leftHipRoll = jointAngle_(1);
    double leftHipPitch = jointAngle_(2);
    double leftKnee = jointAngle_(3);
    double leftAnklePitch = -jointAngle_(4);
    double leftAnkleRoll = -jointAngle_(5);
    double rightHipYaw = jointAngle_(6);
    double rightHipRoll = jointAngle_(7);
    double rightHipPitch = jointAngle_(8);
    double rightKnee = jointAngle_(9);
    double rightAnklePitch = -jointAngle_(10);
    double rightAnkleRoll = -jointAngle_(11);

    // Hip Motor Angles
    double leftHipYawMotorAngle = leftHipYaw;
    double leftHipRollMotorAngle = leftHipRoll;
    double leftHipPitchMotorAngle = leftHipPitch;
    double rightHipYawMotorAngle = rightHipYaw;
    double rightHipRollMotorAngle = rightHipRoll;
    double rightHipPitchMotorAngle = rightHipPitch;

    // Knee Motor Angles
    double leftKneeMotorAngle = leftKnee - 70*D2R;
    double rightKneeMotorAngle = rightKnee - 70*D2R;

    // Ankle Motor Angles
    double A11, B11, C11, A12, B12, C12;
    double A21, B21, C21, A22, B22, C22;

    const double h = 0.41;
    const double s = 0.065;
    const double l_short = 0.195;
    const double l_long = 0.315;
    const double b = 0.008;
    const double w = 0.04;
    const double p11 = 0.20464;
    const double p12 = 0.08964;
    const double p21 = 0.08964;
    const double p22 = 0.20464;
    const double cx = 0.06401;
    const double cy = 0.04;
    const double cz = 0.01129;

    // Ankle Motor Angles - Left
    A11 = pow(b,2) - 2*cos(leftAnklePitch)*b*cx + 2*sin(leftAnkleRoll)*sin(leftAnklePitch)*b*cy - 2*cos(leftAnkleRoll)*sin(leftAnklePitch)*b*cz + pow(cx,2) + 2*sin(leftAnklePitch)*cx*h - 2*sin(leftAnklePitch)*cx*p11 + pow(cy,2) + 2*cos(leftAnklePitch)*sin(leftAnkleRoll)*cy*h - 2*cos(leftAnklePitch)*sin(leftAnkleRoll)*cy*p11 - 2*cos(leftAnkleRoll)*cy*w + pow(cz,2) - 2*cos(leftAnkleRoll)*cos(leftAnklePitch)*cz*h + 2*cos(leftAnkleRoll)*cos(leftAnklePitch)*cz*p11 - 2*sin(leftAnkleRoll)*cz*w + pow(h,2) - 2*h*p11 - pow(l_short,2) + pow(p11,2) + pow(s,2) + pow(w,2);
    B11 = 2*h*s - 2*p11*s + 2*cx*s*sin(leftAnklePitch) - 2*cz*s*cos(leftAnkleRoll)*cos(leftAnklePitch) + 2*cy*s*cos(leftAnklePitch)*sin(leftAnkleRoll);
    C11 = 2*cx*s*cos(leftAnklePitch) - 2*b*s + 2*cz*s*cos(leftAnkleRoll)*sin(leftAnklePitch) - 2*cy*s*sin(leftAnkleRoll)*sin(leftAnklePitch);
    A12 = pow(b,2) - 2*cos(leftAnklePitch)*b*cx - 2*sin(leftAnkleRoll)*sin(leftAnklePitch)*b*cy - 2*cos(leftAnkleRoll)*sin(leftAnklePitch)*b*cz + pow(cx,2) + 2*sin(leftAnklePitch)*cx*h - 2*sin(leftAnklePitch)*cx*p12 + pow(cy,2) - 2*cos(leftAnklePitch)*sin(leftAnkleRoll)*cy*h + 2*cos(leftAnklePitch)*sin(leftAnkleRoll)*cy*p12 - 2*cos(leftAnkleRoll)*cy*w + pow(cz,2) - 2*cos(leftAnkleRoll)*cos(leftAnklePitch)*cz*h + 2*cos(leftAnkleRoll)*cos(leftAnklePitch)*cz*p12 + 2*sin(leftAnkleRoll)*cz*w + pow(h,2) - 2*h*p12 - pow(l_long,2) + pow(p12,2) + pow(s,2) + pow(w,2);
    B12 = 2*h*s - 2*p12*s + 2*cx*s*sin(leftAnklePitch) - 2*cz*s*cos(leftAnkleRoll)*cos(leftAnklePitch) - 2*cy*s*cos(leftAnklePitch)*sin(leftAnkleRoll);
    C12 = 2*cx*s*cos(leftAnklePitch) - 2*b*s + 2*cz*s*cos(leftAnkleRoll)*sin(leftAnklePitch) + 2*cy*s*sin(leftAnkleRoll)*sin(leftAnklePitch);

    double leftAnkleLeftMotorAngleAcos = acos((-A11)/(sqrt(pow(B11,2)+pow(C11,2))));
    double leftAnkleLeftMotorAngleAtan = atan(C11/B11);
    double leftAnkleRightMotorAngleAcos = acos((-A12)/(sqrt(pow(B12,2)+pow(C12,2))));
    double leftAnkleRightMotorAngleAtan = atan(C12/B12);

    if (leftAnkleLeftMotorAngleAtan < 0*D2R) {
        leftAnkleLeftMotorAngleAtan = 90*D2R - leftAnkleLeftMotorAngleAtan;
    }
    if (leftAnkleRightMotorAngleAtan < 0*D2R) {
        leftAnkleRightMotorAngleAtan = 90*D2R - leftAnkleRightMotorAngleAtan;
    }
    double leftAnkleLeftMotorAngle = leftAnkleLeftMotorAngleAcos + leftAnkleLeftMotorAngleAtan - 90*D2R;
    double leftAnkleRightMotorAngle = leftAnkleRightMotorAngleAcos + leftAnkleRightMotorAngleAtan - 90*D2R;

    // Ankle Motor Angles - Right
    // A21 = pow(b,2) - 2*cos(rightAnklePitch)*b*cx + 2*sin(rightAnkleRoll)*sin(rightAnklePitch)*b*cy - 2*cos(rightAnkleRoll)*sin(rightAnklePitch)*b*cz + pow(cx,2) + 2*sin(rightAnklePitch)*cx*h - 2*sin(rightAnklePitch)*cx*p21 + pow(cy,2) + 2*cos(rightAnklePitch)*sin(rightAnkleRoll)*cy*h - 2*cos(rightAnklePitch)*sin(rightAnkleRoll)*cy*p21 - 2*cos(rightAnkleRoll)*cy*w + pow(cz,2) - 2*cos(rightAnkleRoll)*cos(rightAnklePitch)*cz*h + 2*cos(rightAnkleRoll)*cos(rightAnklePitch)*cz*p21 - 2*sin(rightAnkleRoll)*cz*w + pow(h,2) - 2*h*p21 - pow(l_long,2) + pow(p21,2) + pow(s,2) + pow(w,2);
    // B21 = 2*h*s - 2*p21*s + 2*cx*s*sin(rightAnklePitch) - 2*cz*s*cos(rightAnkleRoll)*cos(rightAnklePitch) + 2*cy*s*cos(rightAnklePitch)*sin(rightAnkleRoll);
    // C21 = 2*cx*s*cos(rightAnklePitch) - 2*b*s + 2*cz*s*cos(rightAnkleRoll)*sin(rightAnklePitch) - 2*cy*s*sin(rightAnkleRoll)*sin(rightAnklePitch);
    // A22 = pow(b,2) - 2*cos(rightAnklePitch)*b*cx - 2*sin(rightAnkleRoll)*sin(rightAnklePitch)*b*cy - 2*cos(rightAnkleRoll)*sin(rightAnklePitch)*b*cz + pow(cx,2) + 2*sin(rightAnklePitch)*cx*h - 2*sin(rightAnklePitch)*cx*p22 + pow(cy,2) - 2*cos(rightAnklePitch)*sin(rightAnkleRoll)*cy*h + 2*cos(rightAnklePitch)*sin(rightAnkleRoll)*cy*p22 - 2*cos(rightAnkleRoll)*cy*w + pow(cz,2) - 2*cos(rightAnkleRoll)*cos(rightAnklePitch)*cz*h + 2*cos(rightAnkleRoll)*cos(rightAnklePitch)*cz*p22 + 2*sin(rightAnkleRoll)*cz*w + pow(h,2) - 2*h*p22 - pow(l_short,2) + pow(p22,2) + pow(s,2) + pow(w,2);
    // B22 = 2*h*s - 2*p22*s + 2*cx*s*sin(rightAnklePitch) - 2*cz*s*cos(rightAnkleRoll)*cos(rightAnklePitch) - 2*cy*s*cos(rightAnklePitch)*sin(rightAnkleRoll);
    // C22 = 2*cx*s*cos(rightAnklePitch) - 2*b*s + 2*cz*s*cos(rightAnkleRoll)*sin(rightAnklePitch) + 2*cy*s*sin(rightAnkleRoll)*sin(rightAnklePitch);

    A21 = pow(b,2) - 2*cos(rightAnklePitch)*b*cx + 2*sin(rightAnkleRoll)*sin(rightAnklePitch)*b*cy - 2*cos(rightAnkleRoll)*sin(rightAnklePitch)*b*cz + pow(cx,2) + 2*sin(rightAnklePitch)*cx*h - 2*sin(rightAnklePitch)*cx*p21 + pow(cy,2) + 2*cos(rightAnklePitch)*sin(rightAnkleRoll)*cy*h - 2*cos(rightAnklePitch)*sin(rightAnkleRoll)*cy*p21 - 2*cos(rightAnkleRoll)*cy*w + pow(cz,2) - 2*cos(rightAnkleRoll)*cos(rightAnklePitch)*cz*h + 2*cos(rightAnkleRoll)*cos(rightAnklePitch)*cz*p21 - 2*sin(rightAnkleRoll)*cz*w + pow(h,2) - 2*h*p21 - pow(l_long,2) + pow(p21,2) + pow(s,2) + pow(w,2);
    B21 = 2*h*s - 2*p21*s + 2*cx*s*sin(rightAnklePitch) - 2*cz*s*cos(rightAnkleRoll)*cos(rightAnklePitch) + 2*cy*s*cos(rightAnklePitch)*sin(rightAnkleRoll);
    C21 = 2*cx*s*cos(rightAnklePitch) - 2*b*s + 2*cz*s*cos(rightAnkleRoll)*sin(rightAnklePitch) - 2*cy*s*sin(rightAnkleRoll)*sin(rightAnklePitch);
    A22 = pow(b,2) - 2*cos(rightAnklePitch)*b*cx - 2*sin(rightAnkleRoll)*sin(rightAnklePitch)*b*cy - 2*cos(rightAnkleRoll)*sin(rightAnklePitch)*b*cz + pow(cx,2) + 2*sin(rightAnklePitch)*cx*h - 2*sin(rightAnklePitch)*cx*p22 + pow(cy,2) - 2*cos(rightAnklePitch)*sin(rightAnkleRoll)*cy*h + 2*cos(rightAnklePitch)*sin(rightAnkleRoll)*cy*p22 - 2*cos(rightAnkleRoll)*cy*w + pow(cz,2) - 2*cos(rightAnkleRoll)*cos(rightAnklePitch)*cz*h + 2*cos(rightAnkleRoll)*cos(rightAnklePitch)*cz*p22 + 2*sin(rightAnkleRoll)*cz*w + pow(h,2) - 2*h*p22 - pow(l_short,2) + pow(p22,2) + pow(s,2) + pow(w,2);
    B22 = 2*h*s - 2*p22*s + 2*cx*s*sin(rightAnklePitch) - 2*cz*s*cos(rightAnkleRoll)*cos(rightAnklePitch) - 2*cy*s*cos(rightAnklePitch)*sin(rightAnkleRoll);
    C22 = 2*cx*s*cos(rightAnklePitch) - 2*b*s + 2*cz*s*cos(rightAnkleRoll)*sin(rightAnklePitch) + 2*cy*s*sin(rightAnkleRoll)*sin(rightAnklePitch);


    double rightAnkleLeftMotorAngleAcos = acos((-A21)/(sqrt(pow(B21,2)+pow(C21,2))));
    double rightAnkleLeftMotorAngleAtan = atan(C21/B21);
    double rightAnkleRightMotorAngleAcos = acos((-A22)/(sqrt(pow(B22,2)+pow(C22,2))));
    double rightAnkleRightMotorAngleAtan = atan(C22/B22);

    if (rightAnkleLeftMotorAngleAtan < 0*D2R) {
        rightAnkleLeftMotorAngleAtan = 90*D2R - rightAnkleLeftMotorAngleAtan;
    }
    if (rightAnkleRightMotorAngleAtan < 0*D2R) {
        rightAnkleRightMotorAngleAtan = 90*D2R - rightAnkleRightMotorAngleAtan;
    }
    double rightAnkleLeftMotorAngle = rightAnkleLeftMotorAngleAcos + rightAnkleLeftMotorAngleAtan - 90*D2R;
    double rightAnkleRightMotorAngle = rightAnkleRightMotorAngleAcos + rightAnkleRightMotorAngleAtan - 90*D2R;

    motorAngle << leftHipYawMotorAngle, leftHipRollMotorAngle, leftHipPitchMotorAngle,
                    leftKneeMotorAngle, leftAnkleLeftMotorAngle, leftAnkleRightMotorAngle,
                    rightHipYawMotorAngle, rightHipRollMotorAngle, rightHipPitchMotorAngle,
                    rightKneeMotorAngle, rightAnkleLeftMotorAngle, rightAnkleRightMotorAngle;

    return motorAngle;
}
