#ifndef VISION_WRAPPER_H
#define VISION_WRAPPER_H

#include <thread>
#include <chrono>
#include "aris.h"
#include "DataExchange.h"
#include "Aris_Vision/Aris_Vision.h"
#include "Aris_Vision/Vision_ObstacleDetection.h"
#include "Aris_Vision/Vision_Terrain0.h"

namespace Vision
{
    using namespace aris::core;

    aris::sensor::KINECT kinect1;

    TerrainAnalysis terrainAnalysisResult;
    ObstacleDetection obstacleDetectionResult;
    std::thread visionThread;

    void StartVision()
    {
        using namespace VisionForceExhange;
        kinect1.start();
        cout << "Begin vision thread\n";
        visionThread = std::thread([&]()
        {
            while(true)
            {
                //terrainAnalysisResult.TerrainAnalyze(visiondata.get().gridMap, visiondata.get().pointCloud);

                if(isWriteData == true)
                {
                    auto visiondata = kinect1.getSensorData();
                    obstacleDetectionResult.ObstacleDetecting(visiondata.get().obstacleMap);

                    terrainAnalysisResult.TerrainAnalyze(visiondata.get().gridMap, visiondata.get().pointCloud);
                    cout<<"Begin Write Data-----"<<endl;
                    if(obstacleDetectionResult.obsPoses.size() > 0)
                    {
                        nextPositionGCS[0] =  obstacleDetectionResult.nextPosition[0].Y;
                        nextPositionGCS[1] = -obstacleDetectionResult.nextPosition[0].X;
                    }
                    else
                    {
                        nextPositionGCS[0] = -200;
                        nextPositionGCS[1] = -200;
                    }
                    cout<<"NextPosition X:"<<nextPositionGCS[0]<<" Y:"<<nextPositionGCS[1]<<endl;
                    cout<<obstacleDetectionResult.frames_num<<endl;
                    cout<<"Finish Write Data-----"<<endl;
                    isWriteData = false;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1800));
            }
        });
        cout << "Vision thread started" << endl;
    }
}
#endif
