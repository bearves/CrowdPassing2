#ifndef MOVE_GAIT_H
#define MOVE_GAIT_H

#include <thread>
#include <functional>
#include <cstdint>
#include <map>

#include <aris.h>
#include <aris_control_pipe.h>
#include <Robot_Base.h>
#include <Robot_Gait.h>
#include "CrowdPassingPlanner.h"
#include "LowpassFilter.h"

using namespace std;
using namespace aris::control;

//static const double PI = 3.141592653589793;

struct MoveRotateParam final :public aris::server::GaitParamBase
{
    double targetBodyPE213[6]{0};
    std::int32_t totalCount;
};


namespace ForceTask
{
    void parseContinueMoveBegin(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);
    void parseContinueMoveJudge(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);
    void parseOpenDoorBegin(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);
    void parseOpenDoorJudge(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);
    int continueMove(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);
    int openDoor(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);
    void StartRecordData();
    void inv3(double * matrix,double * invmatrix);
    void crossMultiply(double * vector_in1, double *vector_in2, double * vector_out);
    double dotMultiply(double *vector_in1, double *vector_in2);
    double norm(double * vector_in);

    class ATest
    { 
        public:
            static int TestFunc(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
            {
                return 0;
            }
    };

    struct ContinueMoveParam final :public aris::server::GaitParamBase
    {
        std::int32_t move_direction;
    };

    enum MoveState
    {
        None,
        PointLocate1,
        PointLocate2,
        LocateAjust,
        Forward,
        Backward,
        Rightward,
        Leftward,
        Follow,
        Downward,
        Upward,
        Pullhandle,
        Pushhandle,
        PrePush,
        Push,
    };

    enum PushState
    {
        now2Start,
        leftWalk,
        forwardWalk,
    };

    struct CM_RecordParam
    {
        double bodyPE_last[6];
        double bodyVel_last[6];
        double pEE_last[18];

        double forceSum[6]{0,0,0,0,0,0};
        double forceAvg[6]{0,0,0,0,0,0};
        double force[6];
    };

    struct OpenDoorParam :public CM_RecordParam
    {
        MoveState moveState;
        PushState pushState;
        int ret{0};
        std::int32_t count;
        std::int32_t countIter{0};
        Robots::WalkParam walkParam;

        const double toolInR[3]{0,0.08,-0.385};
        double toolInG[3];
        double toolVel[3]{0,0,0};

        //MoveState: PointLocation
        double pointLocation1[6];
        double pointLocation2[6];
        double pointLocation3[6];
        double location[3][3];

        //Door Location
        double planeYPR[3]{0,0,0};
        double handleLocation[3]{0,0,0};
        //startPE
        double beginPE[6];
        double vector0[3];
        double vector1[3];
        double vector2[3];

        //now2Start used twice
        double nowPE[6]; //used in Follow again
        double nowPee[18];
        double startPE[6];
        const int now2StartCount{2000};

        //MoveState: Follow
        double startPeeInB[18];
        double endPeeInB[18];
        const int followCount{2000};

        //MoveState: Downward
        bool downwardFlag;
        int downwardCount;

        //PushState
        double handlePE[6];
        double nowPm[4][4];
        double xNowInG[3];
        double yNowInG[3];
        double now2startDistance[3];
        double now2startDistanceInB[6]{0,0,0,0,0,0};
        double now2startDistanceInG[6]{0,0,0,0,0,0};
        double handle2startDistance[3];

        //pause
        MoveState moveState_last;
        int pauseCount{0};
        bool pauseFlag;
    };
};

extern Pipe<ForceTask::OpenDoorParam> openDoorPipe;
static std::thread openDoorThread;

void parseMoveWithRotate(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);
int moveWithRotate(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);

namespace CrowdPassing
{
    enum class GAIT_CMD
    {
        NOCMD = 0,
        INIT  = 1,
        START = 2,
        STOP  = 3,
        CLEAR_FORCE = 4
    };

    struct CrowdPassingGaitParam final : public aris::server::GaitParamBase
    {
    };

    class CrowdPassingGaitWrapper
    {
        public:
            static const double FORCE_DEADZONE[6];
            static const double FORCE_LIMIT[6];
            static const int ZERO_SAMPLE_COUNT = 500;
            static int  zeroingCount;
            static double rawForce[6];
            static double forceSum[6];
            static double forceBase[6];
            static double filteredForce[6];
            static double mappedForce[6];
            static double feetPosition[18];
            static double initialBodyPosition[6];
            static CrowdPassingPlanner crowdPassingPlanner;
            static LowpassFilter<6> lpf;
            static GAIT_CMD command;
            
            static void ParseCmds(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);
            static int GaitFunction(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);
        private:
            static void RequireZeroing();
            static void Zeroing(const double* forceData);
            static void ForceMapping(const double* srcForceData, double* destForceData);
    };

    static CrowdPassingGaitWrapper wrapper;
}

#endif // MOVE_GAIT_H
