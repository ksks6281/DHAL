// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#if WIN32
#include <timeapi.h>
#endif

int main(int argc, char* argv[]) {
    int cnt_loop = 0;
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");

    raisim::World world;
    auto checkerBoard = world.addGround();
    auto humanoid_ = world.addArticulatedSystem(binaryPath.getDirectory() + "/rsc/DRCD_Humanoid/drcd_humanoid.urdf");
    raisim::RaisimServer server(&world);

    int gc_dim_ = humanoid_->getGeneralizedCoordinateDim();
    int gv_dim_ = humanoid_->getGeneralizedVelocityDim();
    Eigen::Matrix<double,32,1> gcInit_;
    Eigen::Matrix<double,31,1> gvInit_;



    gcInit_.setZero();
    gvInit_.setZero();
    gcInit_.segment(0, 7) << 0.0, 0.0, 1.2, 1.0, 0.0, 0.0, 0.0; // Base
    gcInit_.segment(7, 12) << 0.0, 0.0, -0.174533, 0.523599, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.349066, 0.0; // Left leg
    gcInit_.segment(19, 12) << 0.0, 0.0, -0.174533, 0.523599, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.349066, 0.0; // Right leg
    std::cout << " gc dimension... " << gc_dim_<<std::endl;
    std::cout << " gc dimension... " << gv_dim_<<std::endl;
    std::cout << " action dim... " << humanoid_->getDOF() <<std::endl;


    Eigen::VectorXd jointPgain(humanoid_->getDOF()), jointDgain(humanoid_->getDOF());
    /// we set the gains of the unactuated joints to zero
    jointPgain << 0,0,0, 0,0,0,
            120,120,120,120,
            120,0,0,
            120,0,0,
            0,0,
            120,120,120,120,
            120,0,0,
            120,0,0,
            0,0,0;
    jointDgain << 0,0,0, 0,0,0,
            50.0,50.0,50.0,50.0,
            50.0,0,0,
            50.0,0,0,
            0,0,
            50.0,50.0,50.0,50.0,
            50.0,0,0,
            50.0,0,0,
            0,0,0;

//    jointPgain.setConstant(100);
//    jointDgain.setConstant(30);

    jointPgain.segment(0,6).setZero();
    jointDgain.segment(0,6).setZero();

    humanoid_->setGeneralizedCoordinate(gcInit_);
    server.launchServer();
    const int stepsPerJoint = 2000; // 2 seconds / 0.001 seconds per step
    const double maxAngle = 10.0 * M_PI / 180.0; // converting degrees to radians

    const int numJoints = 24;

    for (int i=0; i<10000000; i++) {

        Eigen::Matrix<double,32,1> gc_cur = humanoid_->getGeneralizedCoordinate().e();
        Eigen::Matrix<double,31,1> gv_cur = humanoid_->getGeneralizedVelocity().e();
//        std::cout <<" gc cur... " << gc_cur.transpose()<<std::endl;
        gc_cur.block(0,0,3,1).setZero();
        gc_cur.block(3,0,4,1) <<1.0, 0,0,0;
        gc_cur(2) = 1.5;
        gv_cur.block(0,0,6,1).setZero();

//        std::cout <<" gc revised... " << gc_cur.transpose()<<std::endl;
        humanoid_->setGeneralizedCoordinate(gc_cur);
        humanoid_->setGeneralizedVelocity(gv_cur);

        int currentJoint = (i / stepsPerJoint) % numJoints; // 현재 조인트를 결정
        double t = (i % stepsPerJoint) / static_cast<double>(stepsPerJoint); // 0에서 1까지의 시간 비율
//        double angle = maxAngle * sin(M_PI * t); // -20도에서 20도까지의 Sin 파형
        double angle;
        if (t < 0.33) {
            // First half of the period (0 to -20 degrees)
            angle = -maxAngle * sin(2 * M_PI * t/0.66);
        } else if(t>0.33 and t<0.66){

            angle = maxAngle * sin(2 * M_PI * (t-0.33)/0.33);
        }
        else {
            // Second half of the period (20 degrees to 0)
            angle = 0;
        }
        Eigen::Matrix<double,7+numJoints+1,1> target_position;
        Eigen::Matrix<double,6+numJoints+1,1> target_velocity;

        target_position = gcInit_;
        target_velocity.setZero();

        target_position(7+currentJoint,0) += angle;

        humanoid_->setPdGains(jointPgain, jointDgain);
        humanoid_->setPdTarget(target_position, target_velocity);
cnt_loop++;
        std::cout << " time ... "<< cnt_loop*world.getTimeStep() << std::endl;
        std::cout << " target position... "<< target_position.block(7,0,24,1).transpose()<<std::endl;
        RS_TIMED_LOOP(1e6*world.getTimeStep())
        server.integrateWorldThreadSafe();



        raisim::MSLEEP(1);
    }

    server.stopRecordingVideo();
    server.killServer();
}