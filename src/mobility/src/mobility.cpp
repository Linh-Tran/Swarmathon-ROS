#include <ros/ros.h>

//ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>

//ROS messages
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

//Custom messages
#include <shared_messages/TagsImage.h>

// To handle shutdown signals so the node quits properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

//Other
#include <iostream>
#include <vector>

using namespace std;

//Random number generator
random_numbers::RandomNumberGenerator* rng;	

//Mobility Logic Functions
void setVelocity(double linearVel, double angularVel);

//Numeric Variables
geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D goalLocation;
int currentMode = 0;
float mobilityLoopTimeStep = 0.1; //time between the mobility loop calls
float status_publish_interval = 5;
float killSwitchTimeout = 10;
std_msgs::Int16 targetDetected; //ID of the detected target
bool targetsCollected [256] = {0}; //array of booleans indicating whether each target ID has been found

vector <char> pattern;
char curDir;

// state machine states
#define STATE_MACHINE_TRANSFORM	0
#define STATE_MACHINE_ROTATE	1
#define STATE_MACHINE_TRANSLATE	2
int stateMachineState = STATE_MACHINE_TRANSFORM;

geometry_msgs::Twist velocity;
char host[128];
string publishedName;
char prev_state_machine[128];

//Publishers
ros::Publisher velocityPublish;
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher targetCollectedPublish;
ros::Publisher targetPickUpPublish;
ros::Publisher targetDropOffPublish;

//Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber targetsCollectedSubscriber;

//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer killSwitchTimer;

//Spiral
void setTargetN();
void setTargetE();
void setTargetS();
void setTargetW();
void generatePattern(int N_circuits, int N_robots);
//void printPattern();
int  calcDistanceTravel (int i_robot, int i_circuit, int N_robots, char dir);
void getPattern(string ith_pattern);
void getTarget();
bool targetHit();

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
void joyCmdHandler(const geometry_msgs::Twist::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const shared_messages::TagsImage::ConstPtr& tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void mobilityStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void targetsCollectedHandler(const std_msgs::Int16::ConstPtr& message);
void killSwitchTimerEventHandler(const ros::TimerEvent& event);

int main(int argc, char **argv) {

  generatePattern(3,6);
  // printPattern();
  gethostname(host, sizeof (host));
  string hostname(host);

  rng = new random_numbers::RandomNumberGenerator(); //instantiate random number generator

  goalLocation.theta = rng->uniformReal(0, 2 * M_PI); //set initial random heading
    
  targetDetected.data = -1; //initialize target detected
    
  //select initial search position 50 cm from center (0,0)
  goalLocation.x = 0.5 * cos(goalLocation.theta);
  goalLocation.y = 0.5 * sin(goalLocation.theta);

  if (argc >= 2) {
    publishedName = argv[1];
    cout << "Welcome to the world of tomorrow " << publishedName << "!  Mobility module started." << endl;
  } else {
    publishedName = hostname;
    cout << "No Name Selected. Default is: " << publishedName << endl;
  }

  // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
  ros::init(argc, argv, (publishedName + "_MOBILITY"), ros::init_options::NoSigintHandler);
  ros::NodeHandle mNH;

  signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

  joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
  modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
  targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
  obstacleSubscriber = mNH.subscribe((publishedName + "/obstacle"), 10, obstacleHandler);
  odometrySubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, odometryHandler);
  targetsCollectedSubscriber = mNH.subscribe(("targetsCollected"), 10, targetsCollectedHandler);

  status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
  velocityPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/velocity"), 10);
  stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
  targetCollectedPublish = mNH.advertise<std_msgs::Int16>(("targetsCollected"), 1, true);
  targetPickUpPublish = mNH.advertise<sensor_msgs::Image>((publishedName + "/targetPickUpImage"), 1, true);
  targetDropOffPublish = mNH.advertise<sensor_msgs::Image>((publishedName + "/targetDropOffImage"), 1, true);

  publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
  //killSwitchTimer = mNH.createTimer(ros::Duration(killSwitchTimeout), killSwitchTimerEventHandler);
  stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);
    
  ros::spin();
  return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent&) {
  std_msgs::String stateMachineMsg;
  if (currentMode == 2 || currentMode == 3) { //Robot is in automode

    switch(stateMachineState) {
			
      //Select rotation or translation based on required adjustment
      //If no adjustment needed, select new goal
    case STATE_MACHINE_TRANSFORM: {
      stateMachineMsg.data = "TRANSFORMING";
      //If angle between current and goal is significant
      if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1) {
	stateMachineState = STATE_MACHINE_ROTATE; //rotate
      }
      //If goal has not yet been reached
      else if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
	stateMachineState = STATE_MACHINE_TRANSLATE; //translate
      }
      //If returning with a target
      else if (targetDetected.data != -1) {
	//If goal has not yet been reached
	if (hypot(0.0 - currentLocation.x, 0.0 - currentLocation.y) > 0.5) {
	  //set angle to center as goal heading
	  goalLocation.theta = M_PI + atan2(currentLocation.y, currentLocation.x);
						
	  //set center as goal position
	  goalLocation.x = 0.0;
	  goalLocation.y = 0.0;
	}
	//Otherwise, reset target and select new random uniform heading
	else {
	  targetDetected.data = -1;
	  setTargetN();
	}
      }
      //Otherwise, assign a new goal
      else {
	//setTargetN();					
	getTarget();
	ROS_INFO("LT line 201");
	goalLocation.x = currentLocation.x + (0.5* cos(goalLocation.theta));
	goalLocation.y = currentLocation.y + (0.5* sin(goalLocation.theta));
	// //set new goal location according to pattern.
	// //call setTargets
	// 	//select new heading from Gaussian distribution around current heading
	// 	goalLocation.theta = rng->gaussian(currentLocation.theta, 0.25);
					
	//select new position 50 cm from current location
	// goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
	// goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
      }
				
      //Purposefully fall through to next case without breaking
    }
			
      //Calculate angle between currentLocation.theta and goalLocation.theta
      //Rotate left or right depending on sign of angle
      //Stay in this state until angle is minimized
    case STATE_MACHINE_ROTATE: {
      stateMachineMsg.data = "ROTATING";
      if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) > 0.1) {
	setVelocity(0.0, 0.5); //rotate left
      }
      else if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) < -0.1) {
	setVelocity(0.0, -0.5); //rotate right
      }
      else {
	setVelocity(0.0, 0.0); //stop
	stateMachineState = STATE_MACHINE_TRANSLATE; //move to translate step
      }
      break;
    }
			
      //Calculate angle between currentLocation.x/y and goalLocation.x/y
      //Drive forward
      //Stay in this state until angle is at least PI/2
    case STATE_MACHINE_TRANSLATE: {
      stateMachineMsg.data = "TRANSLATING";
      if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
	setVelocity(0.3, 0.0);
      }
      else {
	setVelocity(0.0, 0.0); //stop
	stateMachineState = STATE_MACHINE_TRANSFORM; //move back to transform step
      }
      break;
    }
		
    default: {
      break;
    }
    }
  }

  else { // mode is NOT auto

    // publish current state for the operator to see
    stateMachineMsg.data = "WAITING";
  }

  // publish state machine string for user, only if it has changed, though
  if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0) {
    stateMachinePublish.publish(stateMachineMsg);
    sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
  }
}

void setTargetN()
{
  ROS_INFO("LT In mobility.cpp:: setTargetN()");
  goalLocation.theta = M_PI*0.5;
 
}

void setTargetE()
{
  ROS_INFO("LT In mobility.cpp:: setTargetE()");
  goalLocation.theta = 0.0;
}

void setTargetS()
{
  ROS_INFO("LT In mobility.cpp:: setTargetS()");
  goalLocation.theta = M_PI*-0.5;
}

void setTargetW()
{
  ROS_INFO("LT In mobility.cpp:: setTargetW()");
  goalLocation.theta = M_PI;	
  // goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
  // goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
}

void generatePattern(int N_circuits, int N_robots)
{
  ROS_INFO("LT In mobility.cpp:: generatePattern()");
  
  vector<string> paths;
  string ith_path;

  for (int i_robot = 1; i_robot <= N_robots; i_robot++)
    {
      for (int i_circuit = 0; i_circuit < N_circuits; i_circuit++)
        {
	  int n_steps_north = calcDistanceTravel(i_robot, i_circuit, N_robots,'N');
	  for (int j = 0; j < n_steps_north; j++)
            {
	      ith_path += 'N';
            }
            
	  int n_steps_east = calcDistanceTravel(i_robot, i_circuit, N_robots,'E');
	  for (int j = 0; j < n_steps_east; j++)
            {
	      ith_path += 'E';
            }

	  int n_steps_south = calcDistanceTravel(i_robot, i_circuit, N_robots,'S');
	  for (int j = 0; j < n_steps_south; j++)
            {
	      ith_path += 'S';
            }

	  int n_steps_west = calcDistanceTravel(i_robot, i_circuit, N_robots,'W');
	  for (int j = 0; j < n_steps_west; j++)
            {
	      ith_path += 'W';
            }

        }
    }	
  paths.push_back(ith_path);
  ith_path.clear();
}

// void printPattern()
// {
//   for (int i = 0; i<pattern.size(); i++)
//     {
//       ROS_INFO("LT path::", pattern[i]);
//     }
// }

int calcDistanceTravel (int i_robot, int i_circuit, int N_robots, char dir)
{
  int i = i_robot;
  int j = i_circuit;
  int N = N_robots;
  int n_steps = 0;
   
  if (dir == 'N' || dir == 'E')
    {
      if (j == 0)
        {
	  n_steps = i;
	  return n_steps;
        }
      else if (j == 1)
        {
	  n_steps = calcDistanceTravel(i, j-1, N, dir) + i + N;
	  return n_steps;
        }
      else 
        {
	  n_steps = calcDistanceTravel(i, j-1, N, dir) + 2*N;
	  return n_steps;
        }
    }

  else if (dir == 'S' || dir == 'W')
    {
      if (j == 0)
        {
	  n_steps = calcDistanceTravel(i, j , N, 'N') + i;
	  return n_steps;
        }

      else if (j > 0)
        {
	  n_steps = calcDistanceTravel(i, j, N, 'N') + N;
	  return n_steps;
        }

      else
        {
	  cout << "Error direction" << dir<< "is invalid" << endl;
        }
    }
  return 0;
}

void getPattern (string ith_pattern)
{
  copy(ith_pattern.begin(), ith_pattern.end(), back_inserter(pattern));
  reverse(pattern.begin(), pattern.end());
}

void getTarget()
{
  // ROS_INFO("LT In mobility.cpp:: getTarget()");
  if (targetHit() == true && pattern.size() > 0)
    {
      curDir = pattern [pattern.size()-1];
      ROS_INFO("LT inside if");	
      switch(curDir)
	{
	case 'N':
	  setTargetN();
	  ROS_INFO("LT N");
	  break;
	case 'S':
	  setTargetS();
	  ROS_INFO("LT S");
	  break;
	case 'E':
	  setTargetE();
	  ROS_INFO("LT E");
	  break;
	case 'W':
	  setTargetW();
	  ROS_INFO("LT W");
	  break;
	}
      pattern.pop_back();
    }
	
  else if (pattern.size() == 0)
    { 
      // ROS_INFO("LT size == 0");
      stateMachineState = STATE_MACHINE_TRANSLATE;
      //setVelocity(0.0, 0.0); //stop
      // stateMachineState = STATE_MACHINE_TRANSFORM; //move back to transform step8*/
    }
}

bool targetHit()
{
  bool hit = false;
  if (hypot(goalLocation.x-currentLocation.x, goalLocation.y - currentLocation.y) < 0.5)
    {
      hit = true;
    }
  return hit;
}
void setVelocity(double linearVel, double angularVel) 
{
  // Stopping and starting the timer causes it to start counting from 0 again.
  // As long as this is called before the kill swith timer reaches killSwitchTimeout seconds
  // the rover's kill switch wont be called.
  killSwitchTimer.stop();
  killSwitchTimer.start();
  
  velocity.linear.x = linearVel * 1.5;
  velocity.angular.z = angularVel * 8; //scaling factor for sim; removed by aBridge node
  velocityPublish.publish(velocity);
}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/

void targetHandler(const shared_messages::TagsImage::ConstPtr& message) {

  //if this is the goal target
  if (message->tags.data[0] == 256) {
    //if we were returning with a target
    if (targetDetected.data != -1) {
      //publish to scoring code
      targetDropOffPublish.publish(message->image);
      targetDetected.data = -1;
    }
  }

  //if target has not previously been detected 
  else if (targetDetected.data == -1) {
        
    //check if target has not yet been collected
    if (!targetsCollected[message->tags.data[0]]) {
      //copy target ID to class variable
      targetDetected.data = message->tags.data[0];
			
      //set angle to center as goal heading
      goalLocation.theta = M_PI + atan2(currentLocation.y, currentLocation.x);
			
      //set center as goal position
      goalLocation.x = 0.0;
      goalLocation.y = 0.0;
			
      //publish detected target
      targetCollectedPublish.publish(targetDetected);

      //publish to scoring code
      targetPickUpPublish.publish(message->image);

      //switch to transform state to trigger return to center
      stateMachineState = STATE_MACHINE_TRANSFORM;
    }
  }
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
  currentMode = message->data;
  setVelocity(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message) {
  if (message->data > 0) {
    //obstacle on right side
    if (message->data == 1) {
      //select new heading 0.2 radians to the left
      goalLocation.theta = currentLocation.theta + 0.2;
    }
		
    //obstacle in front or on left side
    else if (message->data == 2) {
      //select new heading 0.2 radians to the right
      goalLocation.theta = currentLocation.theta - 0.2;
    }
							
    //select new position 50 cm from current location
    goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
    goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
		
    //switch to transform state to trigger collision avoidance
    stateMachineState = STATE_MACHINE_TRANSFORM;
  }
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
  //Get (x,y) location directly from pose
  currentLocation.x = message->pose.pose.position.x;
  currentLocation.y = message->pose.pose.position.y;
	
  //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
  tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  currentLocation.theta = yaw;
}

void joyCmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
  if (currentMode == 0 || currentMode == 1) 
    {
      setVelocity(message->linear.x, message->angular.z);
    } 
}


void publishStatusTimerEventHandler(const ros::TimerEvent&)
{
  std_msgs::String msg;
  msg.data = "online";
  status_publisher.publish(msg);
}

// Safety precaution. No movement commands - might have lost contact with ROS. Stop the rover.
// Also might no longer be receiving manual movement commands so stop the rover.
void killSwitchTimerEventHandler(const ros::TimerEvent& t)
{
  // No movement commands for killSwitchTime seconds so stop the rover 
  setVelocity(0,0);
  double current_time = ros::Time::now().toSec();
  //ROS_INFO("In mobility.cpp:: killSwitchTimerEventHander(): Movement input timeout. Stopping the rover at %6.4f.", current_time);
}

void targetsCollectedHandler(const std_msgs::Int16::ConstPtr& message) {
  targetsCollected[message->data] = 1;
}

void sigintEventHandler(int sig)
{
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}
