#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>

#include <ctime>
#include <list.h>
#include <deque.h>
#include <algorithm>

#include "stage.hh"

using namespace Stg;


enum RobotState
{
    UNKNOWN  = 0,
    CHARGING = 1,
    SEARCHING = 2,
    INGROUP = 3,
    HOMING = 4
};

enum MessageType
{
    SYNC = 0,
    IAMCOMMING = 1,
    IAMLEAVING = 2,
    WELCOME = 3,
    BYE = 4
};

/* Minimal Message Structure */
struct RobotMessage {
    MessageType type;
    unsigned int sender;
    unsigned int receiver;
    double t;
    //double val;
    list<unsigned int> ids;
};

typedef struct
{
  ModelPosition* position;
  ModelRanger* ranger; //Sonar Sensor
  ModelFiducial* fiducial;

  ModelFiducial::Fiducial* closest;
  radians_t closest_bearing;
  meters_t closest_range;
  radians_t closest_heading_error;
  list<RobotMessage> inbox;
  RobotState state;
  //double belief;
  list<unsigned int> teammates;
  deque<double> d2c; //This one is used to check if robot is in group
} robot_t;

// Define const double XXX = YYYY; Here for global access during control

const double FATTMAX = 1.0;
const double FREPMAX = -1.0;
const double FWATT = 0.01;
const double RMIN = 5.0;

static list<RobotMessage> MessageList;

// This is aligned with RobotStates
const Stg::Color StateColors[5] = {Color("white"), Color("grey"), Color("blue"), Color("green"), Color("cyan")};

// Local Functions

double getCurrentTimeStamp()
{
  timeval tt;
  gettimeofday(&tt, NULL);
  double sec = (double) tt.tv_sec;
  double usec = (double) tt.tv_usec;
  return sec + (usec / 1.0e6);
}


void setRobotState(robot_t* r, RobotState s)
{
    r->state = s;
    r->position->SetColor(StateColors[s]);
}

// No Broadcast support yet ...
void sendMessage(MessageType type, int sender, int receiver, list<unsigned int> ids)
{
    RobotMessage msg;
    msg.type = type;
    msg.t = getCurrentTimeStamp();
    msg.sender = sender;
    msg.receiver = receiver;
    //msg.val = val;
    //msg.ids.assign(ids.begin(), ids.end());
    msg.ids = ids;
    
    MessageList.push_back(msg);
}

void updateMyInbox(robot_t* robot)
{
    //robot->inbox.clear();
    list<RobotMessage>::iterator i;
    RobotMessage msg;
    for (i = MessageList.begin(); i != MessageList.end(); ++i)
    {
        msg = (*i);
        if (msg.receiver == robot->position->GetId())
        {
            robot->inbox.push_back(msg);
            MessageList.erase(i);
        }
    }
}

//void updateRobotBelief(robot_t robot, )
double sigmoid(double x)
{
    return 1.0 / (1.0 + exp(-1.0 * x));
}


// forward declare
int RangerUpdate( ModelRanger* mod, robot_t* robot );
int FiducialUpdate( ModelFiducial* fid, robot_t* robot );


// Stage calls this when the model starts up

extern "C" int Init( Model* mod )
{
  robot_t* robot = new robot_t;
  robot->position = (ModelPosition*)mod;

  

  // subscribe to the ranger, which we use for navigating
  robot->ranger = (ModelRanger*)mod->GetUnusedModelOfType( "ranger" );
  assert( robot->ranger );

  // ask Stage to call into our ranger update function
  robot->ranger->AddCallback( Model::CB_UPDATE, (model_callback_t)RangerUpdate, robot );

  robot->fiducial = (ModelFiducial*)mod->GetUnusedModelOfType( "fiducial" ) ;
  robot->position->SetFiducialReturn(robot->position->GetId());
 
  assert( robot->fiducial );
  
  /* Each robot transmits its ID via fiducial */
  
  
  robot->fiducial->AddCallback( Model::CB_UPDATE, (model_callback_t)FiducialUpdate, robot );
  
  
  robot->teammates.push_front(robot->position->GetId());
  setRobotState(robot, SEARCHING);
//  if (robot->position->GetId() == 2)
//  {
//      setRobotState(robot, INGROUP);
//  }
  robot->fiducial->Subscribe();
  robot->ranger->Subscribe();
  robot->position->Subscribe();

  return 0; //ok
}

int RangerUpdate( ModelRanger* rgr, robot_t* robot )
{
    return 0;
}

int FiducialUpdate( ModelFiducial* fid, robot_t* robot)
{
  // find the closest teammate

  double dist = 1e6; // big

  robot->closest = NULL;

  FOR_EACH( it, fid->GetFiducials() )
    {
      ModelFiducial::Fiducial* other = &(*it);

      if( other->range < dist )
        {
          dist = other->range;
          robot->closest = other;
        }
    }

//  if( robot->closest ) // if we saw someone
//    {
//      robot->closest_bearing = robot->closest->bearing;
//      robot->closest_range = robot->closest->range;
//      robot->closest_heading_error = robot->closest->geom.a;
//    }
//  
  //printf("Position of %d : %6.4f %6.4f \n", robot->position->GetId(), robot->position->GetPose().x, robot->position->GetPose().y);

  /* This is some shared knowledge, let's say it's global now */
  
  static Pose attCenter = Pose(0.0, 0.0, 0.0, 0.0);
  static double movementX = 0.0;//1e-4;
  double att_dx = attCenter.x - robot->position->GetPose().x;
  double att_dy = attCenter.y - robot->position->GetPose().y;
  double att_d = sqrt( (att_dx * att_dx) + (att_dy * att_dy) );
  
  robot->d2c.push_front(att_d);
  if (robot->d2c.size() > 100)
  {
      robot->d2c.pop_back();
  }
  
  updateMyInbox(robot);
  
  /* Message Parsing */
  
  /* End Message Parsing */
  
  /* State Based Communication and Transitions */
  
  printf("I am robot %d, I think there are %d bots in the world. [Inbox Size : %4d] \n", robot->position->GetId(), robot->teammates.size(), robot->inbox.size());
  
//  list<RobotMessage>::iterator mit;
//  for (mit = robot->inbox.begin(); mit != robot->inbox.end(); ++mit)
//  {
//      RobotMessage msg = *mit;
//      printf("** From:%2d To:%2d Type:%d Val:%4.2f T:%10.2f\n", msg.sender, msg.receiver, msg.type, msg.val, msg.t);
//  }
//  
  if (robot->state == SEARCHING)
  {
      
      // Check if any WELCOME message received
      list<RobotMessage>::iterator mit;
      list<unsigned int> updated;
      double tMax = 0.0;
      for (mit = robot->inbox.begin(); mit != robot->inbox.end(); ++mit)
      {
          RobotMessage msg = *(mit);
          if (msg.type == WELCOME)
          {
              // Looking for most recent Message
              if (msg.t > tMax)
              {
                  tMax = msg.t;
                  updated = msg.ids;
              }
          }
      
          // I am only looking for welcome messages, erase everything else
          msg.ids.clear();
          robot->inbox.erase(mit);
      }
      
      
      if (updated.size() > 0) // Hurray! Welcome Message Received
      {
          setRobotState(robot, INGROUP);
          robot->teammates = updated;
      }
      else if (robot->d2c.size() > 80) // Check if you are in group based on differences over recent distances to center
      {
          double meand = 0.0;
          deque<double>::iterator qit;
          deque<double>::iterator prev;
          double diff;
          bool first = true;
          for (qit = robot->d2c.begin(); qit != robot->d2c.end(); ++qit)
          {
              if (!first)
              {
                  diff = fabs((*prev) - (*qit));
                  meand += diff;
              }
              else
              {
                  first = false;
              }
              prev = qit;
          }
          meand /= robot->d2c.size();
          if (meand < 0.01)
          {
              //printf("I am robot %d and just promoted!\n", robot->position->GetId());
              setRobotState(robot, INGROUP);
          }
      }
      
      // Still Searching? AKA : No Welcome Message
      if (robot->state == SEARCHING)
      {
          // Send Messages to all other neighbors 
          FOR_EACH( it, fid->GetFiducials() )
          {
              ModelFiducial::Fiducial* other = &(*it);
              sendMessage(IAMCOMMING, robot->position->GetId(), other->id, robot->teammates); // The value is not important
          }
      }
  }
  else if (robot->state == INGROUP)
  {
      
      // Step 1: Look for incomming messages and take appropriate update
      
      list<RobotMessage>::iterator mit;
      
      // first check JOINING and LEAVING bots
      bool joinleave = false;
      list<unsigned int> welcomeacklist;
      list<unsigned int> byeacklist;
      for (mit = robot->inbox.begin(); mit != robot->inbox.end(); ++mit)
      {
          RobotMessage msg = *mit;
          if (msg.type == IAMCOMMING)
          {
              joinleave = true;
              robot->teammates.push_back(msg.sender);
              welcomeacklist.push_back(msg.sender);
              
              msg.ids.clear();
              robot->inbox.erase(mit);
          }
          else if (msg.type == IAMLEAVING)
          {
              joinleave = true;
              robot->teammates.remove(msg.sender);
              byeacklist.push_back(msg.sender);
              //sendMessage(BYE, robot->position->GetId(), msg.sender, robot-); // Reply Back
              
              msg.ids.clear();
              robot->inbox.erase(mit);
          }
      }
      
      list<unsigned int>::iterator iit;
      
      //printf("######## Sending %d welcome back messages \n", welcomeacklist.size());

      for (iit = welcomeacklist.begin(); iit != welcomeacklist.end(); ++iit)
      {
          sendMessage(WELCOME, robot->position->GetId(), *iit, robot->teammates); // Reply Back  
      }
      
      // Update your belief using INGROUP SYNC messages
      
      double tMax = 0.0;
      list<unsigned int> updated;
      for (mit = robot->inbox.begin(); mit != robot->inbox.end(); ++mit)
      {
          RobotMessage msg = *mit;
          if (msg.type == SYNC)
          {
              if (msg.t > tMax)
              {
                  updated = msg.ids;
                  tMax = msg.t;
              }
              
              msg.ids.clear();
              robot->inbox.erase(mit);
          }
      }

      if (updated.size() > 0)
      {
          robot->teammates.sort();
          updated.sort();
          robot->teammates.merge(updated);
          robot->teammates.unique();
      }
     
      
      // Step 2: Send Sync Messages (Don't send messages to people you You said welcome!)
      
      FOR_EACH( it, fid->GetFiducials() )
      {
          ModelFiducial::Fiducial* other = &(*it);
          
          bool found = false;
          for (iit = welcomeacklist.begin(); (iit != welcomeacklist.end()) && (!found); ++iit)
          {
              if (other->id == *iit) found = true;
          }
          
          if (!found) sendMessage(SYNC, robot->position->GetId(), other->id, robot->teammates); 
      }
      
      // Step 3: Check State Trnasitions for itself
  }
  else if (robot->state == HOMING)
  {
      ;
  }
  else if (robot->state == CHARGING)
  {
      ;
  }
  /* Actions Based on State - No State Transition Here */
  if ((robot->state == SEARCHING) || (robot->state == INGROUP))  
  {
      

      attCenter.x += movementX;

      if (fabs(attCenter.x) > 4.0)
      {
          movementX = -1.0 * movementX;
      }

      //printf("Error of %d : %6.4f %6.4f \n", robot->position->GetId(), att_dx, att_dy);
      

      double att_force;
      if (att_d < RMIN)
      {
          att_force = -FATTMAX;
      }
      else
      {
          att_force = FATTMAX * 2.0 * (sigmoid(att_d - RMIN) - 0.5);
      }

      double att_theta = atan2(att_dy, att_dx);

      double att_force_x = att_force * cos(att_theta);
      double att_force_y = att_force * sin(att_theta);

      double rep_force = 0.0;
      double rep_force_x = 0.0;
      double rep_force_y = 0.0;

      double sum_force_x = att_force_x;
      double sum_force_y = att_force_y;

      //printf("I am robot #%2d and my time is %8.4f, my buddies are: ", robot->position->GetId(), getCurrentTimeStamp());
      FOR_EACH( it, fid->GetFiducials() )
        {
          ModelFiducial::Fiducial* other = &(*it);
          double rep_d = other->range;
          double rep_theta = other->bearing;
          rep_force = FREPMAX * (1.0 - (2.0 * (sigmoid(rep_d) - 0.5)) );
          rep_force_x = rep_force * cos(rep_theta);
          rep_force_y = rep_force * sin(rep_theta);

          sum_force_x += rep_force_x;
          sum_force_y += rep_force_y;
          //printf("%d,", other->id);
        }
      //printf("\n");


      // Forward Kinematic Model (Point Robot)

      double vx_global = sum_force_x;
      double vy_global = sum_force_y;

      //printf("Target Speed of %d : %6.4f %6.4f \n", robot->position->GetId(), vx, vy);

      radians_t theta = robot->position->GetPose().a;
      double vx_robot = ( cos(theta) * vx_global) + (sin(theta) * vy_global);
      double vy_robot = (-sin(theta) * vx_global) + (cos(theta) * vy_global);

      /* The Angular Velocity Module Calculation */

      double w_robot = 0.0;
      radians_t r_heading = robot->position->GetPose().a;
      radians_t angle_error = normalize ( normalize(att_theta + 3.1415) - r_heading);
      w_robot = FWATT * 2.0 * (sigmoid(angle_error) - 0.5);

      w_robot = 0.0;
      robot->position->SetSpeed( vx_robot, vy_robot, 0.0);
  }
  
  //robot->position->SetSpeed(0.0, 0.0, 1.0);
  return 0;
}
