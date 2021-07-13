#include "utility.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include "elevation_msgs/OccupancyElevation.h"

const float RESOLUTION = 0.1;
const int LENGTH = 800;
const int WIDTH = 540;

class zlpPRM {
private:
    // ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInObstacles;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInCost;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPath;

    // string positionFilePath;
    string filePath;
    pcl::PointXYZI originPosition;

    vector<float> costMap;

    state_t *robotState;
    state_t *goalState;
    kdtree_t *kdtree;
    float map_min[3];
    float map_max[3];
    vector<state_t *> nodeList;
    vector<state_t *> pathList;

    pcl::PointXYZRGB pointXYZRGBtemp;

    bool costUpdateFlag[NUM_COSTS];

public:
    // zlpPRM():nh("~"){
    zlpPRM() {
        cloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudInObstacles.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudInCost.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloudPath.reset(new pcl::PointCloud<pcl::PointXYZI>());
        costMap.resize(LENGTH * WIDTH, 0);
        string filePath = "/home/zlp/catkin_ws/src/PRM/occupancy_elevation/550.txt";
        kdtree = kd_create(3);
        robotState = new state_t;
        goalState = new state_t;
        for (int i = 0; i < NUM_COSTS; ++i)
            costUpdateFlag[i] = false;
        for (int i = 0; i < costHierarchy.size(); ++i)
            costUpdateFlag[costHierarchy[i]] = true;
    }

    bool run() {
        // 得到局部地图的原点位置（map坐标系下）
        originPosition.x = 0;
        originPosition.y = 0;
        originPosition.z = 0;

        // 获得局部地图的点云表示形式（z表示高程、i表示占用）
        readTxt();
       // if (cloudIn->points.size() != LENGTH * WIDTH) return false;

        // get boundry

        map_min[0] = originPosition.x;
        map_min[1] = originPosition.y;
        map_min[2] = originPosition.z;
        map_max[0] = originPosition.x + RESOLUTION * LENGTH;
        map_max[1] = originPosition.y + RESOLUTION * WIDTH;
        map_max[2] = originPosition.z;
        ROS_INFO("2. get boundry[%f,%f,%f,%f", map_min[0], map_min[1], map_max[0], map_max[1]);

        pcl::console::TicToc time;
        time.tic();

        ROS_INFO("3. updateCostMap");
        updateCostMap();

        ROS_INFO("4. begin Sample");
        double sampling_start_time = ros::WallTime::now().toSec();
        int Sample_count = 0;
        while (ros::WallTime::now().toSec() - sampling_start_time < 0.05) {
            // ROS_INFO("4.x. get in while");
            state_t *newState = new state_t;
            if (sampleState(newState)) {
                // 1.1 Too close discard
                if (nodeList.size() != 0 && stateTooClose(newState) == true) {
                    // ROS_INFO("4.x too close");
                    delete newState;
                    continue;
                }
                // 1.2 Save new state and insert to KD-tree
                newState->stateId = nodeList.size(); // mark state ID
                nodeList.push_back(newState);
                // insertIntoKdtree(newState);
                kd_insert(kdtree, newState->x, newState);
                Sample_count++;
            } else
                delete newState;
        }
        ROS_INFO("4.1. finish Sample, get %d points", nodeList.size());

        ROS_INFO("4.2 show sample points in pcl");
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSamplePoint(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (int i = 0; i < nodeList.size(); i++) {
            pointXYZRGBtemp.x = nodeList[i]->x[0];
            pointXYZRGBtemp.y = nodeList[i]->x[1];
            pointXYZRGBtemp.z = nodeList[i]->x[2];
            pointXYZRGBtemp.r = 0;
            pointXYZRGBtemp.g = 255;
            pointXYZRGBtemp.b = 0;
            cloudSamplePoint->push_back(pointXYZRGBtemp);
        }
        boost::shared_ptr <pcl::visualization::PCLVisualizer> viewer(
                new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(1.0);

        pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGB> rgb(cloudSamplePoint);
        viewer->addPointCloud<pcl::PointXYZRGB>(cloudSamplePoint, "cloud1");
          viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud1");
        viewer->addPointCloud<pcl::PointXYZI>(cloudIn, "cloudall");

        ROS_INFO("4.3 show obstacle points in pcl");
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudObstaclePoint(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (int i = 0; i < cloudInObstacles->points.size(); i++) {
            pointXYZRGBtemp.x = cloudInObstacles->points[i].x;
            pointXYZRGBtemp.y = cloudInObstacles->points[i].y;
            pointXYZRGBtemp.z = cloudInObstacles->points[i].z;
            pointXYZRGBtemp.r = 255;
            pointXYZRGBtemp.g = 0;
            pointXYZRGBtemp.b = 0;
            cloudObstaclePoint->push_back(pointXYZRGBtemp);
        }

        viewer->addPointCloud<pcl::PointXYZRGB>(cloudObstaclePoint, "cloudObstacle");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloudObstacle");

        ROS_INFO("5.0 begin add edges");

        // updateStatesAndEdges();

        // 1. get Xinit
        robotState->x[0] = originPosition.x + 100 * RESOLUTION;
        robotState->x[1] = originPosition.y + 140 * RESOLUTION;
        robotState->x[2] = cloudIn->points[100 * WIDTH + 140].z;
        // double roll, pitch, yaw;
        // tf::Matrix3x3 m(transform.getRotation());
        // m.getRPY(roll, pitch, yaw);
        // robotState->theta = 0 + M_PI; // change from -PI~PI to 0~1*PI
        robotState->theta = M_PI / 2;
        // 2. add edges for the nodes that are within a certain radius of mapCenter
        vector < state_t * > nearStates;

        getNearStates(robotState, nearStates, 80);
        ROS_INFO("5.1 get init's neighbors %d", nearStates.size());
        if (nearStates.size() == 0) {
            ROS_INFO("5.x nearStates's size is zeros, return");
            return false;
        }

        ROS_INFO("5.2 get in loop");
        float edgeCosts[NUM_COSTS];
        neighbor_t thisNeighbor;
        for (int i = 0; i < nearStates.size(); ++i) {
            for (int j = i + 1; j < nearStates.size(); ++j) {
                // 4.1 height difference larger than threshold, too steep to connect
                if (abs(nearStates[i]->x[2] - nearStates[j]->x[2]) > 1) {
                    deleteEdge(nearStates[i], nearStates[j]);
                    continue;
                }
                // 4.2 distance larger than x, too far to connect
                float distanceBetween = distance(nearStates[i]->x, nearStates[j]->x);
                if (distanceBetween > 2 || distanceBetween < 0.3) {
                    deleteEdge(nearStates[i], nearStates[j]);
                    continue;
                }
                // 4.3 this edge is connectable
                if (edgePropagation(nearStates[i], nearStates[j], edgeCosts) == true) {
                    // even if edge already exists, we still need to update costs (casuse elevation may change)
                    deleteEdge(nearStates[i], nearStates[j]);
                    for (int k = 0; k < NUM_COSTS; ++k)
                        thisNeighbor.edgeCosts[k] = edgeCosts[k];
                    // ROS_INFO("5.x add a edge");
                    thisNeighbor.neighbor = nearStates[j];
                    nearStates[i]->neighborList.push_back(thisNeighbor);
                    thisNeighbor.neighbor = nearStates[i];
                    nearStates[j]->neighborList.push_back(thisNeighbor);

                    string lineName = "line" + to_string(i) + to_string(j);
                    viewer->addLine<pcl::PointXYZ>(
                            pcl::PointXYZ(nearStates[i]->x[0], nearStates[i]->x[1], nearStates[i]->x[2]),
                            pcl::PointXYZ(nearStates[j]->x[0], nearStates[j]->x[1], nearStates[j]->x[2]),
                            lineName);

                } else { // edge is not connectable, delete old edge if it exists
                    deleteEdge(nearStates[i], nearStates[j]);
                }
            }
        }

        ROS_INFO("5.x show the edges");

        ROS_INFO("6.0 Begin BFS ");

        pathList.clear();
        // globalPath.poses.clear();

        // 1. reset costs and parents
        for (int i = 0; i < nodeList.size(); ++i) {
            for (int j = 0; j < NUM_COSTS; ++j)
                nodeList[i]->costsToRoot[j] = FLT_MAX;
            nodeList[i]->parentState = NULL;
        }
        // 2. find the state that is the closest to the robot
        state_t *startState = NULL;

        vector < state_t * > nearRobotStates;

        getNearStates(robotState, nearRobotStates, 2);
        if (nearRobotStates.size() == 0) {
            ROS_INFO(" 6.x the neighbors of init is zeros, return false");
            return false;
        }
        ROS_INFO("6.1 get the neighbors of init: %d", nearRobotStates.size());

        ROS_INFO("6.2 find the nearest state");
        float nearRobotDist = FLT_MAX;
        for (int i = 0; i < nearRobotStates.size(); ++i) {
            float dist = distance(nearRobotStates[i]->x, robotState->x);
            if (dist < nearRobotDist && nearRobotStates[i]->neighborList.size() != 0) {
                nearRobotDist = dist;
                startState = nearRobotStates[i];
            }
        }

        if (startState == NULL || startState->neighborList.size() == 0) {
            ROS_INFO("6.x NO nearest state");
            return false;
        }

        ROS_INFO("6.3 begin BFS loop");
        for (int i = 0; i < NUM_COSTS; ++i)
            startState->costsToRoot[i] = 0;
        float thisCost;
        vector < state_t * > Queue;
        Queue.push_back(startState);

        while (Queue.size() > 0) {
            // ROS_INFO("6.x new loop and Openlist size:%d",Queue.size());
            // find the state that can offer lowest cost in this depth and remove it from Queue
            state_t *fromState = minCostStateInQueue(Queue);
            Queue.erase(remove(Queue.begin(), Queue.end(), fromState), Queue.end());
            // stop searching if minimum cost path to goal is found
            // if (fromState == nearestGoalState)
            //     break;
            // lopp through all neighbors of this state
            for (int i = 0; i < fromState->neighborList.size(); ++i) {
                state_t *toState = fromState->neighborList[i].neighbor;
                // Loop through cost hierarchy
                for (vector<int>::const_iterator iter = costHierarchy.begin(); iter != costHierarchy.end(); iter++) {
                    int costIndex = *iter;

                    thisCost = fromState->costsToRoot[costIndex] + fromState->neighborList[i].edgeCosts[costIndex];
                    // If cost can be improved, update this ndoe with new costs
                    if (thisCost < toState->costsToRoot[costIndex]) {
                        updateCosts(fromState, toState, i); // update toState's costToRoot
                        toState->parentState = fromState; // change parent for toState
                        Queue.push_back(toState);
                    }
                        // If cost is same, go to compare secondary cost
                    else if (thisCost == toState->costsToRoot[costIndex]) {
                        continue;
                    }
                        // If cost becomes higher, abort this propagation
                    else
                        break;
                }
            }
        }

        ROS_INFO("6.4 get goal state");
        goalState->x[0] = originPosition.x + 700 * RESOLUTION;
        goalState->x[1] = originPosition.y + 300 * RESOLUTION;
        goalState->x[2] = cloudIn->points[700 * WIDTH + 300].z;
        ROS_INFO("6.5 get the nearest state of goal");
        state_t *nearestGoalState = getNearestState(goalState);
        // find near states to nearestGoalState

        ROS_INFO("6.6 get the nearest state that has parent");
        vector < state_t * > nearGoalStates;
        getNearStates(nearestGoalState, nearGoalStates, 5);
        if (nearGoalStates.size() > 0) {
            float nearRobotDist = FLT_MAX;
            for (int i = 0; i < nearGoalStates.size(); ++i) {
                float dist = distance(nearGoalStates[i]->x, goalState->x);
                if (dist < nearRobotDist && nearGoalStates[i]->parentState != NULL) {
                    nearRobotDist = dist;
                    nearestGoalState = nearGoalStates[i];
                }
            }
        }
        // the nearest goal state is invalid
        if (nearestGoalState == NULL ||
            nearestGoalState->parentState == NULL) // no path to the nearestGoalState is found
            return false;

        // 4. Extract path
        ROS_INFO("6.7 Extract Path");
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pathCloudView(new pcl::PointCloud<pcl::PointXYZRGB>());

        state_t *thisState = nearestGoalState;
        while (thisState->parentState != NULL) {

            pathList.insert(pathList.begin(), thisState);
            addPointXYZRGB(pathCloudView, thisState->x[0], thisState->x[1], thisState->x[2], 0, 0, 255);

            string linePathName = "linePath" + to_string(pathList.size());
            viewer->addLine<pcl::PointXYZ>(
                    pcl::PointXYZ(thisState->x[0], thisState->x[1], thisState->x[2]),
                    pcl::PointXYZ(thisState->parentState->x[0], thisState->parentState->x[1],
                                  thisState->parentState->x[2]),
                    0, 0, 255,
                    linePathName);

            thisState = thisState->parentState;
        }
        pathList.insert(pathList.begin(), robotState); // add current robot state
        pathList.push_back(goalState); // add goal state

        ROS_INFO("6.8 get Path %d", pathList.size());
        viewer->addPointCloud<pcl::PointXYZRGB>(pathCloudView, "cloudPath");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "cloudPath");

        ROS_INFO("pclView:: view the start and goal");
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudEndpoints(new pcl::PointCloud<pcl::PointXYZRGB>());
        addPointXYZRGB(cloudEndpoints, robotState->x[0], robotState->x[1], robotState->x[2], 0, 0, 255);
        addPointXYZRGB(cloudEndpoints, goalState->x[0], goalState->x[1], goalState->x[2], 0, 0, 255);
        viewer->addPointCloud<pcl::PointXYZRGB>(cloudEndpoints, "cloudEndpoints");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "cloudEndpoints");


        if (pathList.size() <= 1)
            return false;
        ROS_INFO("7.0 Cubic Spline");
        nav_msgs::Path originPath;
        nav_msgs::Path splinePath;
        originPath.header.frame_id = "map";
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";

        originPath.poses.clear();
        for (int i = 0; i < pathList.size(); i++) {
            pose.pose.position.x = pathList[i]->x[0];
            pose.pose.position.y = pathList[i]->x[1];
            pose.pose.position.z = pathList[i]->x[2];
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
            originPath.poses.push_back(pose);
        }
        path_smoothing::CubicSplineInterpolator csi("smooth");
        csi.interpolatePath(originPath, splinePath);
        ROS_INFO("7.1 finish cubic spline %d,%d", originPath.poses.size(), splinePath.poses.size());

        ROS_INFO("\033[1;32m---->\033[0m Apply PRM in %f s", time.toc() / 1000.0);

        // globalPath = splinePath;
        // displayGlobalPath = globalPath; // displayGlobalPath is only changed during planning
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCubicSpline(new pcl::PointCloud <pcl::PointXYZRGB>);
        for (int i = 0; i < splinePath.poses.size(); i++) {
            addPointXYZRGB(
                    cloudCubicSpline,
                    splinePath.poses[i].pose.position.x, splinePath.poses[i].pose.position.y,
                    splinePath.poses[i].pose.position.z,
                    0, 0, 255
            );
        }
        viewer->addPointCloud<pcl::PointXYZRGB>(cloudCubicSpline, "cloudCubicSpline");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloudCubicSpline");

        // sum of cost
        float costSum = 0.0;
        for (int i = 0; i < splinePath.poses.size(); i++) {
            int rounded_x = (int) ((splinePath.poses[i].pose.position.x - map_min[0]) / RESOLUTION);
            int rounded_y = (int) ((splinePath.poses[i].pose.position.y - map_min[1]) / RESOLUTION);
            costSum += cloudInCost->points[rounded_y + rounded_x * WIDTH].intensity;
        }

        ROS_INFO("7.2 path cost %f", costSum);

//         string filePath = "/home/zlp/catkin_ws/src/PRM/poses.txt";
//
//           	ofstream outfile;
//         outfile.open(filePath);
//         for( int i=0;i<originPath.poses.size();i++){
//         	outfile<<originPath.poses[i].pose.position.x<<","
//         	<<originPath.poses[i].pose.position.y<<","
//         	<<originPath.poses[i].pose.position.z<<'\n';
//         }
//         outfile.close();


        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }

    }

    void
    addPointXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn, double x, double y, double z, int r, int g, int b) {
        pcl::PointXYZRGB point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.r = r;
        point.g = g;
        point.b = b;
        cloudIn->push_back(point);
    }

    // 3. Planning
    // bfsSearch();
    // 4. Visualize Roadmap
    state_t *minCostStateInQueue(vector<state_t *> Queue) {
        // Loop through cost hierarchy
        for (vector<int>::const_iterator iter1 = costHierarchy.begin(); iter1 != costHierarchy.end(); ++iter1) {
            int costIndex = *iter1;
            vector < state_t * > tempQueue;
            float minCost = FLT_MAX;
            // loop through nodes saved in Queue
            for (vector<state_t *>::const_iterator iter2 = Queue.begin(); iter2 != Queue.end(); ++iter2) {
                state_t *thisState = *iter2;
                // if cost is lower, we put it in tempQueue in case other nodes can offer same cost
                if (thisState->costsToRoot[costIndex] < minCost) {
                    minCost = thisState->costsToRoot[costIndex];
                    tempQueue.clear();
                    tempQueue.push_back(thisState);
                }
                    // same cost can be offered by other nodes, we save them to tempQueue for next loop (secondary cost)
                else if (thisState->costsToRoot[costIndex] == minCost)
                    tempQueue.push_back(thisState);
            }
            // Queue is used again for next loop
            Queue.clear();
            Queue = tempQueue;
        }
        // If cost hierarchy is selected correctly, there will be only one element left in Queue (no other ties)
        return Queue[0];
    }

    void updateCosts(state_t *fromState, state_t *toState, int neighborInd) {
        for (int i = 0; i < NUM_COSTS; ++i)
            toState->costsToRoot[i] = fromState->costsToRoot[i] + fromState->neighborList[neighborInd].edgeCosts[i];
    }

    double getStateHeight(state_t *stateIn) {
        int rounded_x = (int) ((stateIn->x[0] - map_min[0]) / RESOLUTION);
        int rounded_y = (int) ((stateIn->x[1] - map_min[1]) / RESOLUTION);
        return cloudIn->points[rounded_y + rounded_x * WIDTH].z;
    }

    float getStateCost(state_t *stateIn) {
        int rounded_x = (int) ((stateIn->x[0] - map_min[0]) / RESOLUTION);
        int rounded_y = (int) ((stateIn->x[1] - map_min[1]) / RESOLUTION);
        return cloudInCost->points[rounded_y + rounded_x * WIDTH].intensity;
    }

    void updateCostMap() {
        int inflationSize = 1;
        for (int i = 0; i < LENGTH * WIDTH; i++) {
            int idX = int(i / WIDTH);
            int idY = int(i % WIDTH);
            if (cloudIn->points[i].intensity > 0) {
                for (int m = -inflationSize; m <= inflationSize; m++) {
                    for (int n = -inflationSize; n <= inflationSize; n++) {
                        int newIdX = idX + m;
                        int newIdY = idY + n;
                        if (newIdX < 0 || newIdX >= LENGTH || newIdY < 0 || newIdY >= WIDTH)
                            continue;
                        int index = newIdY + newIdX * WIDTH;
                        costMap[index] = std::max(costMap[index], std::sqrt(float(m * m + n * n)));
                    }
                }
            }
        }
    }

    bool sampleState(state_t *stateCurr) {
        // random x and y
        for (int i = 0; i < 2; ++i)
            stateCurr->x[i] =
                    (double) rand() / (RAND_MAX + 1.0) * (map_max[i] - map_min[i]) - (map_max[i] - map_min[i]) / 2.0 +
                    (map_max[i] + map_min[i]) / 2.0;
        // random heading
        stateCurr->theta = (double) rand() / (RAND_MAX + 1.0) * 2 * M_PI - M_PI;
        // collision checking before getting height info
        if (isIncollision(stateCurr)) {
            return false;
        }
        // z is not random since the robot is constrained to move on the ground
        stateCurr->x[2] = getStateHeight(stateCurr);
        if (stateCurr->x[2] == -1) {
            return false;
        }

        // cost is too heigh
        if (getStateCost(stateCurr) > 0.2) {
            return false;
        }

        return true;
    }

    bool isIncollision(state_t *stateIn) {
        // if the state is outside the map, discard this state
        if (stateIn->x[0] <= map_min[0] || stateIn->x[0] >= map_max[0]
            || stateIn->x[1] <= map_min[1] || stateIn->x[1] >= map_max[1])
            return true;
        // if the distance to the nearest obstacle is less than xxx, in collision
        int rounded_x = (int) ((stateIn->x[0] - map_min[0]) / RESOLUTION);
        int rounded_y = (int) ((stateIn->x[1] - map_min[1]) / RESOLUTION);
        int index = rounded_y + rounded_x * WIDTH;

        // close to obstacles within ... m
        if (costMap[index] > 0)
            return true;
        return false;
    }

    bool isIncollision(int rounded_x, int rounded_y, int index) {
        if (rounded_x < 0 || rounded_x >= localMapArrayLength ||
            rounded_y < 0 || rounded_y >= localMapArrayWidth)
            return true;

        // close to obstacles within ... m
        if (costMap[index] > 0)
            return true;

        // stateIn->x is on an unknown grid
        if (cloudIn->points[index].z == -1)
            return true;


        return false;
    }

    bool stateTooClose(state_t *stateIn) {
        // restrict the nearest sample that can be inserted to the roadmap
        state_t *nearestState = getNearestState(stateIn);
        if (nearestState == NULL) {
            ROS_INFO("nearestState = NULL");
            return false;
        }
        if (distance(stateIn->x, nearestState->x) > 0.5 && fabs(stateIn->x[2] - nearestState->x[2]) <= 1) {

            return false;
        }

        return true;
    }

    float distance(double state_from[3], double state_to[3]) {
        return sqrt((state_to[0] - state_from[0]) * (state_to[0] - state_from[0]) +
                    (state_to[1] - state_from[1]) * (state_to[1] - state_from[1]) +
                    (state_to[2] - state_from[2]) * (state_to[2] - state_from[2]));
    }

    bool readTxt() {

        // 打开存储着点云高程/占用信息/cost的文件
        ifstream file;
        filePath = "/home/zlp/catkin_ws/src/PRM/occupancy_elevation/550.txt";
        file.open(filePath.data());   //将文件流对象与文件连接起来
        assert(file.is_open());   //若失败,则输出错误消息,并终止程序运行
        ROS_INFO("1: file open success ");
        // 循环读取每个点的高程/占用信息/cost
        string s, item;
        float height;
        int occupancy;
        float cost;
        pcl::PointXYZI point;
        pcl::PointXYZI pointCost;
        int index = 0, index_x, index_y;
        ROS_INFO("2: begin read file ");
        while (getline(file, s)) {
            // get height and occupancy
            stringstream ss;
            ss.str(s);
            getline(ss, item, ',');
            height = stod(item);
            getline(ss, item, ',');
            occupancy = stoi(item);
            getline(ss, item, ',');
            cost = stod(item);

            index_x = int(index / WIDTH);
            index_y = int(index % WIDTH);
            point.x = originPosition.x + index_x * RESOLUTION;
            point.y = originPosition.y + index_y * RESOLUTION;
            point.z = height;
            point.intensity = occupancy;

//            if (height == -1){
//                index++;
//                continue;
//            }

            cloudIn->push_back(point);

            if (point.intensity == 100) {
                cloudInObstacles->push_back(point);
            }

            pointCost.x = originPosition.x + index_x * RESOLUTION;
            pointCost.y = originPosition.x + index_y * RESOLUTION;
            pointCost.z = height;
            pointCost.intensity = cost;

            cloudInCost->push_back(pointCost);
            index++;

            // if(index %100  == 0) cout<<index<<'\t';
        }
        file.close();             //关闭文件输入流
        ROS_INFO("2.1: finish file reading ,get points %d", index);
        return true;
    }

    state_t *getNearestState(state_t *stateIn) {
        kdres_t *kdres = kd_nearest(kdtree, stateIn->x);
        if (kd_res_end(kdres)) {
            kd_res_free(kdres);
            return NULL;
        }
        state_t *nearestState = (state_t *) kd_res_item_data(kdres);
        kd_res_free(kdres);
        return nearestState;
    }

    void getNearStates(state_t *stateIn, vector<state_t *> &vectorNearStatesOut, double radius) {
        kdres_t *kdres = kd_nearest_range(kdtree, stateIn->x, radius);
        vectorNearStatesOut.clear();
        // Create the vector data structure for storing the results
        int numNearVertices = kd_res_size(kdres);
        if (numNearVertices == 0) {
            kd_res_free(kdres);
            return;
        }
        kd_res_rewind(kdres);
        while (!kd_res_end(kdres)) {
            state_t *stateCurr = (state_t *) kd_res_item_data(kdres);
            vectorNearStatesOut.push_back(stateCurr);
            kd_res_next(kdres);
        }
        kd_res_free(kdres);
    }

    void deleteEdge(state_t *stateA, state_t *stateB) {
        // "remove" compacts the elements that differ from the value to be removed (state_in) in the beginning of the vector 
        // and returns the iterator to the first element after that range. Then "erase" removes these elements (who's value is unspecified).
        compareState = stateB;
        stateA->neighborList.erase(
                std::remove_if(stateA->neighborList.begin(), stateA->neighborList.end(), isStateExsiting),
                stateA->neighborList.end());
        compareState = stateA;
        stateB->neighborList.erase(
                std::remove_if(stateB->neighborList.begin(), stateB->neighborList.end(), isStateExsiting),
                stateB->neighborList.end());
    }

    bool edgePropagation(state_t *state_from, state_t *state_to, float edgeCosts[NUM_COSTS]) {
        // 0. initialize edgeCosts
        for (int i = 0; i < NUM_COSTS; ++i)
            edgeCosts[i] = 0;
        // 1. segment the edge for collision checking
        int steps = floor(distance(state_from->x, state_to->x) / (RESOLUTION));
        float stepX = (state_to->x[0] - state_from->x[0]) / steps;
        float stepY = (state_to->x[1] - state_from->x[1]) / steps;
        float stepZ = (state_to->x[2] - state_from->x[2]) / steps;
        // 2. allocate memory for a state, this state must be deleted after collision checking
        state_t *stateCurr = new state_t;
        stateCurr->x[0] = state_from->x[0];
        stateCurr->x[1] = state_from->x[1];
        stateCurr->x[2] = state_from->x[2];

        int rounded_x, rounded_y, indexInLocalMap;

        // 3. collision checking loop
        for (int stepCount = 0; stepCount < steps; ++stepCount) {
            stateCurr->x[0] += stepX;
            stateCurr->x[1] += stepY;
            stateCurr->x[2] += stepZ;

            rounded_x = (int) ((stateCurr->x[0] - map_min[0]) / RESOLUTION);
            rounded_y = (int) ((stateCurr->x[1] - map_min[1]) / RESOLUTION);
            indexInLocalMap = rounded_y + rounded_x * WIDTH;

            if (isIncollision(rounded_x, rounded_y, indexInLocalMap)) {
                delete stateCurr;
                return false;
            }

            // costs propagation
            if (costUpdateFlag[2])
                edgeCosts[2] = edgeCosts[2] + mapResolution; // distance cost
        }

        rounded_x = (int) ((state_to->x[0] - map_min[0]) / RESOLUTION);
        rounded_y = (int) ((state_to->x[1] - map_min[1]) / RESOLUTION);
        indexInLocalMap = rounded_y + rounded_x * WIDTH;

        if (costUpdateFlag[1])
            edgeCosts[1] = cloudInCost->points[indexInLocalMap].intensity;

        delete stateCurr;
        return true;
    }

};


int main(int argc, char **argv) {

    // ros::init(argc, argv, "traversability_mapping");

    // TraversabilityPRM TPRM;

    // ROS_INFO("\033[1;32m---->\033[0m Traversability Planner Started.");

    zlpPRM PRM;
    PRM.run();

    // pcl::visualization::CloudViewer viewer ("test");
    // viewer.showCloud(pointCloud);
    // while (!viewer.wasStopped()){ };


    // ros::spin();
    return 0;
}
