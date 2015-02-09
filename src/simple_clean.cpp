#include <ros/ros.h>
#include <nav2d_navigator/MoveToPosition2DAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Point.h>
#include <nav2d_navigator/SendCommand.h>
#include <nav2d_msgs/RobotPose.h>
#include <costmap_2d/costmap_2d.h>
#include <stdlib.h> 
#include <iostream> 
#include <string> 
#include <sstream> 
#include <boost/thread.hpp>
#include <occupancy_grid_utils/coordinate_conversions.h>


const static unsigned char CELL_CLEANED = 'C';
const static float TARGET_DISTANCE = 0.25;
const static float TARGET_ANGLE = 0.10000000149;
const static unsigned int MAX_HEIGHT = 1000;
const static unsigned int MAX_WIDTH = 1000;

using namespace std;

enum pointStatus { Uncovered, Covered, Invalid };

typedef actionlib::SimpleActionClient<nav2d_navigator::MoveToPosition2DAction> MoveClient;

typedef struct {
	double x;
	double y;
} Point;

bool containsPoint(std::vector<Point> points, Point point) {
	for (std::vector<Point>::iterator it = points.begin() ; it != points.end(); ++it)
		if(abs((*it).x - point.x) < 0.1 && abs((*it).y - point.y) < 0.1)
			return true;
			
	return false;
}

void removePoint(std::vector<Point> points, Point point) {
	for (std::vector<Point>::iterator it = points.begin() ; it != points.end(); ++it)
		if(abs((*it).x - point.x) < 0.1 && abs((*it).y - point.y) < 0.1)
			points.erase(it);
}



class SimpleCleaner
{
private:
	ros::NodeHandle nh;
	ros::Subscriber mapSub;
	ros::Subscriber poseSub;
	ros::ServiceClient mappingClient;
	ros::ServiceClient getMapClient;
	MoveClient moveClient;
	bool moveToPos(float x, float y, float theta);
	boost::shared_mutex* accessMap;
	
	std::vector<Point> coveredPoints;
	std::vector<Point> backtrackingPoints;
	
	double currentX;
	double currentY;
	double theta;
	
	bool cleaning;
	
public:
	SimpleCleaner(): moveClient("MoveTo", true){

		poseSub = nh.subscribe("others", 10, &SimpleCleaner::poseCallback, this);
		mappingClient = nh.serviceClient<nav2d_navigator::SendCommand>("/StartMapping");
		getMapClient = nh.serviceClient<nav_msgs::GetMap>("/static_map");
		accessMap = new boost::shared_mutex();
		coveredPoints = std::vector<Point>();
		backtrackingPoints = std::vector<Point>();
		cleaning = false;
	}
	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void poseCallback(const nav2d_msgs::RobotPose::ConstPtr& msg);
	void startMapping();
	void cleanerThread();
	
	void doneCb(const actionlib::SimpleClientGoalState& state,
            const nav2d_navigator::MoveToPosition2DResult::ConstPtr& result);

	void getRobotNeighbors(const nav_msgs::OccupancyGrid& map, pointStatus neighbors[4] );
	void worldToMap(const nav_msgs::OccupancyGrid& map, double wx, double wy, unsigned int & mx, unsigned int & my);
	void mapToWorld(const nav_msgs::OccupancyGrid& map, unsigned int mx, unsigned int my, double & wx, double & wy);
	pointStatus checkPointFree(const nav_msgs::OccupancyGrid& map, unsigned int mx, unsigned int my);
	void planNextMove(const nav_msgs::OccupancyGrid& map, pointStatus neighbors[4], double & finalX, double & finalY);

	
	boost::shared_mutex* getLock() {
     return accessMap;
   }
   
   double getX() {
	   return currentX;
   }
   double getY() {
	   return currentY;
   }
   double getTheta() {
	   return theta;
   }
  /* int getMapSize() {
		return map->getSizeInCellsX();
   }*/
};

/*
void SimpleCleaner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	int width = msg->info.width;
	int height = msg->info.height;
	double resolution = msg->info.resolution;
	double origin_x = msg->info.origin.position.x;
	double origin_y = msg->info.origin.position.y;

	//boost::unique_lock < boost::shared_mutex > lock(*accessMap);
/*
	if(width>MAX_WIDTH || height > MAX_HEIGHT) {
		ROS_ERROR_STREAM("MAP IS TOO BIG: " << width << ":" << height);
		return;
	}

	if(map->getOriginX()!=origin_x || map->getOriginY()!=origin_y)
		map->updateOrigin(origin_x,origin_y);
	//costmap_2d::Costmap2D temp = costmap_2d::Costmap2D(width, height, resolution, origin_x, origin_y, 100);
	ROS_INFO_STREAM("A");
	int maxIndex = map->getIndex(width,height);
	unsigned int x;
	unsigned int y;
	ROS_INFO_STREAM("B");
	for(int i=0;i<maxIndex;i++) {
		map->indexToCells(i,x,y)	;
		double cost = msg->data[i];
		//if(map->getCost(x,y) != CELL_CLEANED)
			//map->setCost(x,y,cost);
	}
	*/	
/*
	costmap_2d::Costmap2D temp = costmap_2d::Costmap2D(width, height, resolution, origin_x, origin_y, 0);
	unsigned int x;
	unsigned int y;
	int maxIndex = temp.getIndex(width,height);
	for(int i=0;i<maxIndex;i++) {
		temp.indexToCells(i,x,y);
		double cost = msg->data[i];
		temp.setCost(x,y,cost);
	}
	//ROS_INFO_STREAM("Map update " << width << ":" << height);
}*/

void SimpleCleaner::poseCallback(const nav2d_msgs::RobotPose::ConstPtr& msg) {
	
	currentX = msg->pose.x;
	currentY = msg->pose.y;
	theta = msg->pose.theta;
	//ROS_INFO_STREAM("Pose:" << x << ":" << y << " " << theta);
	
	/*if(moveClient.isServerConnected())
		if(moveClient.getState() == actionlib::SimpleClientGoalState::LOST)
			ROS_INFO_STREAM("FREE");
		else 
			ROS_INFO_STREAM("TRACKING GOAL");
	*/		
	if(!cleaning) {
		cleaning = true;
		moveToPos(currentX,currentY,0);
		//moveToPos(3,1,0);
		//moveToPos(2,1,0);
		//moveToPos(2,0,0);
		
	}
	//ROS_INFO_STREAM("Current Pos: " << currentX << ":" << currentY);
	
}
	
void SimpleCleaner::worldToMap(const nav_msgs::OccupancyGrid& map, double wx, double wy, unsigned int & mx, unsigned int & my) {
	int width = map.info.width;
	int height = map.info.height;
	double resolution = map.info.resolution;
	double origin_x = map.info.origin.position.x;
	double origin_y = map.info.origin.position.y;		

	
	costmap_2d::Costmap2D temp = costmap_2d::Costmap2D(width, height, resolution, origin_x, origin_y, 0);
	//ROS_INFO("%f:%f",currentX,currentY);
	
	double minDistance = width*height;
	
	for(int i = 0; i<map.data.size();i++) {
		unsigned int grid_x;
		unsigned int grid_y;
		temp.indexToCells(i,grid_x,grid_y);
		
		double currentx;
		double currenty;
		temp.mapToWorld(grid_x,grid_y, currentx, currenty);
		
		double distance = abs(currentx - wx) + abs(currenty - wy);
		if(distance < minDistance) {
			minDistance = distance;
			mx = grid_x;
			my = grid_y;
		}
	}
}

void SimpleCleaner::mapToWorld(const nav_msgs::OccupancyGrid& map, unsigned int mx, unsigned int my, double & wx, double & wy){
	wx = map.info.origin.position.x + (mx + 0.5) * map.info.resolution;
	wy = map.info.origin.position.y + (my + 0.5) * map.info.resolution;

}


pointStatus SimpleCleaner::checkPointFree(const nav_msgs::OccupancyGrid& map, unsigned int mx, unsigned int my){
	int width = map.info.width;
	int height = map.info.height;
	double resolution = map.info.resolution;
	double origin_x = map.info.origin.position.x;
	double origin_y = map.info.origin.position.y;	
	
	costmap_2d::Costmap2D temp = costmap_2d::Costmap2D(width, height, resolution, origin_x, origin_y, 0);
	double wx;
	double wy;
	temp.mapToWorld(mx, my, wx, wy);
	Point point;
	point.x = wx;
	point.y = wy;
	
	if(!containsPoint(coveredPoints, point))
		if(temp.getCost(mx,my) == 0)
			return Uncovered;
		else
			return Invalid;
			
	return Covered;
}

void SimpleCleaner::getRobotNeighbors(const nav_msgs::OccupancyGrid& map, pointStatus neighbors[4] ) {
	
	
	unsigned int robotX;
	unsigned int robotY;
	
	worldToMap(map, currentX, currentY, robotX, robotY);
	ROS_INFO("Current: %f:%f",currentX,currentY);
	
	
	neighbors[0] = checkPointFree(map, robotX, robotY+3); //check North
	neighbors[1] = checkPointFree(map, robotX, robotY-3); //check South
	neighbors[2] = checkPointFree(map, robotX+3, robotY); //check East
	neighbors[3] = checkPointFree(map, robotX-3, robotY); //check Weast
	
}

void SimpleCleaner::planNextMove(const nav_msgs::OccupancyGrid& map, pointStatus neighbors[4], double &finalX, double &finalY){
	bool decided = false;
	unsigned int robotX;
	unsigned int robotY;
	worldToMap(map, currentX, currentY, robotX, robotY);

	if(neighbors[0] == Uncovered)
		if(!decided){
			mapToWorld(map, robotX, robotY+3, finalX, finalY);
			decided = true;
		}
	if(neighbors[1] == Uncovered)
		if(!decided){
			mapToWorld(map, robotX, robotY-3, finalX, finalY);
			decided = true;
		} else {
			double nx, ny;
			mapToWorld(map, robotX, robotY-3, nx, ny);
			Point point;
			point.x = nx;
			point.y = ny;
			if(!containsPoint(backtrackingPoints, point))
				backtrackingPoints.push_back(point);
		}	
	if(neighbors[2] == Uncovered)
		if(!decided){
			mapToWorld(map, robotX+3, robotY, finalX, finalY);
			decided = true;
		} else {
			double nx, ny;
			mapToWorld(map, robotX+3, robotY, nx, ny);
			Point point;
			point.x = nx;
			point.y = ny;
			if(!containsPoint(backtrackingPoints, point))
				backtrackingPoints.push_back(point);
		}
	if(neighbors[3] == Uncovered)
		if(!decided){
			mapToWorld(map, robotX-3, robotY, finalX, finalY);
			decided = true;
		} else {
			double nx, ny;
			mapToWorld(map, robotX-3, robotY, nx, ny);
			Point point;
			point.x = nx;
			point.y = ny;
			if(!containsPoint(backtrackingPoints, point))
				backtrackingPoints.push_back(point);
		}	
}

void SimpleCleaner::startMapping() {
	nav2d_navigator::SendCommand srv;
	srv.request.command = 3;
	if(!mappingClient.call(srv))
		ROS_ERROR("Failed to send GETMAP_COMMAND to GetMap-Client.");
}

void SimpleCleaner::doneCb(const actionlib::SimpleClientGoalState& state,
            const nav2d_navigator::MoveToPosition2DResult::ConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
	nav_msgs::GetMap srv_map;
	if(!getMapClient.call(srv_map)) {
		ROS_ERROR("Failed to send GETMAP_COMMAND to GetMap-Client.");
	} else {
		Point point;
		point.x = currentX;
		point.y = currentY;
		coveredPoints.push_back(point);
		removePoint(backtrackingPoints, point);
		const nav_msgs::OccupancyGrid& map (srv_map.response.map);
		pointStatus neighbors[4];
		getRobotNeighbors(map, neighbors);
		double nextX, nextY;
		planNextMove(map, neighbors, nextX, nextY);
		moveToPos(nextX, nextY, theta); //TODO theta
	}

}

void activeCb()
{
  ROS_INFO_STREAM("Goal just went active");
}

void feedbackCb(const nav2d_navigator::MoveToPosition2DFeedback::ConstPtr& feedback)
{
  ROS_INFO_STREAM("Distance to goal " << feedback->distance);
}

bool SimpleCleaner::moveToPos(float x, float y, float theta) {
	
	 // Define the goal
    float goal_x = x;
    float goal_y = y;
    float goal_theta = theta;

	moveClient.waitForServer();
	ROS_INFO_STREAM("Sending goal: " << x << ":" << y);

	// Send Goal 
	nav2d_navigator::MoveToPosition2DGoal goal;

	goal.target_pose.x = x;
	goal.target_pose.y = y;
	goal.target_pose.theta = theta;
	goal.target_distance = 0.15;
	goal.target_angle = 0.10000000149;

	/*goal.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = goal_x;
    goal.target_pose.pose.position.y = goal_y;

	// Convert the Euler angle to quaternion
    double radians = goal_theta * (M_PI/180);
    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians);

    geometry_msgs::Quaternion qMsg;
    tf::quaternionTFToMsg(quaternion, qMsg);
    goal.target_pose.pose.orientation = qMsg;

    ROS_INFO("Sending goal to robot no. %s: x = %f, y = %f, theta = %f", robot_id, goal_x, goal_y, goal_theta);*/

	moveClient.sendGoal(goal,boost::bind(&SimpleCleaner::doneCb, this, _1, _2), &activeCb, &feedbackCb);
    // Wait for the action to return
    /*ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO_STREAM("You have reached the goal! " << x << ":" << y);
		return true;
	}  else {
        ROS_INFO_STREAM("The base failed for some reason");
		return false;
	}*/

}

/* void drawMap() {
	int width = map.info.width;
		int height = map.info.height;
		double resolution = map.info.resolution;
		double origin_x = map.info.origin.position.x;
		double origin_y = map.info.origin.position.y;		

       std::string mapdatafile = "generatedMap.pgm";
       FILE* out = fopen(mapdatafile.c_str(), "w");
       if (!out)
       {
         ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
         return;
       }
 
       fprintf(out, "P5\n# CREATOR: simple_clean.cpp %.3f m/pix\n%d %d\n255\n",
               resolution, width, height);	

	   int m_sizeX2 = width>>1;
	   int m_sizeY2 = height>>1;
	   double mapX = width/2 + ((origin_x-m_sizeX2)/resolution + 0.5*width/resolution);
	   double mapY = height/2 + ((origin_y-m_sizeX2)/resolution + 0.5*width/resolution);
	    
	   std::vector<signed char> a(map.data);
	   
	   costmap_2d::Costmap2D temp = costmap_2d::Costmap2D(width, height, resolution, origin_x, origin_y, 0);
		ROS_INFO("%f:%f",currentX,currentY);
		for(int i = 0; i<a.size();i++) {
			unsigned int grid_x;
			unsigned int grid_y;
			temp.indexToCells(i,grid_x,grid_y);
			
			double wx;
			double wy;
			temp.mapToWorld(grid_x,grid_y, wx, wy);
			unsigned int worldX = floor(grid_x + origin_x/resolution);
			unsigned int worldY = floor(grid_y + origin_y/resolution);
			//ROS_INFO("%d:%d - %d:%d",worldX,worldY,floor(origin_x),floor(origin_y));
			
			int cost = a.at(i); 
			if(abs(wx - currentX) < 0.1 && abs(wy - currentY) < 0.1) {
				fputc(100, out);
				ROS_INFO_STREAM("ON " << grid_x << ":" << grid_y);
			} else if(cost == 0) { //occ [0,0.1)
				fputc(254, out);
			} else if (cost == +100) { //occ (0.65,1]
				fputc(000, out);
			} else { //occ [0.1,0.65]
				fputc(205, out);
			}
		}
		
       fclose(out);
}*/

int main(int argc, char** argv) {
	

    ros::init(argc, argv, "simple_clean");
	
	SimpleCleaner cleaner;
	cleaner.startMapping();
	ros::Duration(12).sleep();
	ros::AsyncSpinner spinner(3);
	spinner.start();
	ros::waitForShutdown();
}
