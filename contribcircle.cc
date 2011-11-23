#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>

#include <ctime>
#include <list>
#include <deque>
#include <algorithm>

#include "stage.hh"

using namespace Stg;
using namespace std;

#define MAX_MESSAGE_LIST_SIZE 4096
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
    ADDROBOT = 0,
    REMOVEROBOT = 1,
    IAMCOMMING = 2,
    IAMLEAVING = 3,
    WELCOME = 4,
    BYE = 5
};

/* Minimal Message Structure */
struct RobotMessage {
    MessageType type;
    unsigned int sender;
    unsigned int receiver;
    double t;
    //double val;
    
    unsigned int robotid;
    unsigned int relaycount;
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
  
  double energy;
  Stg::Pose charger;
} robot_t;

// Define const double XXX = YYYY; Here for global access during control

const double FATTMAX = 1.0;
const double FREPMAX = -1.0;
const double FWATT = 1.0;
const double RMIN = 2.0;

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
    r->inbox.clear();
    r->d2c.clear();
}

// No Broadcast support yet ...
void sendMessage(MessageType type, int sender, int receiver, unsigned int rid, unsigned int rc)
{
//    if (MessageList.size() > MAX_MESSAGE_LIST_SIZE)
//        return;
    
    RobotMessage msg;
    msg.type = type;
    msg.t = getCurrentTimeStamp();
    msg.sender = sender;
    msg.receiver = receiver;
    //msg.val = val;
    //msg.ids.assign(ids.begin(), ids.end());
    msg.robotid = rid;
    msg.relaycount = rc;
    
    MessageList.push_back(msg);
}

void updateMyInbox(robot_t* robot)
{
    //robot->inbox.clear();
    list<RobotMessage>::iterator i;
    RobotMessage msg;
    
    i = MessageList.begin();
    while (i != MessageList.end())
    {
        msg = (*i);
        if (msg.receiver == robot->position->GetId())
        {
            robot->inbox.push_back(msg);
            MessageList.erase(i++);
        }
        else
        {
            ++i;
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
  srand ( time(NULL) );
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
  robot->energy = 1000.0 + (robot->position->GetId() * 200.0);
//  if (robot->position->GetId() == 2)
//  {
//      setRobotState(robot, INGROUP);
//  }
  robot->fiducial->Subscribe();
  //robot->ranger->Subscribe();
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
  
  
//  list<RobotMessage>::iterator mit;
//  for (mit = robot->inbox.begin(); mit != robot->inbox.end(); ++mit)
//  {
//      RobotMessage msg = *mit;
//      printf("** From:%2d To:%2d Type:%d Val:%4.2f T:%10.2f\n", msg.sender, msg.receiver, msg.type, msg.val, msg.t);
//  }
//  
  
  //printf("I am robot %d, I think there are %d bots in the world. [In: %4d][E: %4.2f] \n", robot->position->GetId(), robot->teammates.size(), robot->inbox.size(), robot->energy);
  
  if (robot->state == SEARCHING)
  {
      
      // Check if any WELCOME message received
      list<RobotMessage>::iterator mit;

      bool updated = false;
      mit = robot->inbox.begin();
      while (mit != robot->inbox.end())
      {
          RobotMessage msg = *(mit);
          //printf("From: %2d To: %2d Says: %2d \n", msg.sender, msg.receiver, msg.robotid);
          if (msg.type == WELCOME)
          {
              updated = true;
              robot->teammates.push_back(msg.robotid);
          }
          robot->inbox.erase(mit++);
      }
      
      
      if (updated) // Hurray! Welcome Message Received
      {
          //printf("Hurray ###################\n");
          setRobotState(robot, INGROUP);
          robot->teammates.sort();
          robot->teammates.unique();
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
              FOR_EACH( it, fid->GetFiducials() )
              {
                  ModelFiducial::Fiducial* other = &(*it);
                  robot->teammates.push_back(other->id);
              }
          }
      }
      
      // Still Searching? AKA : No Welcome Message
      if (robot->state == SEARCHING)
      {
          // Send Messages to all other neighbors 
          FOR_EACH( it, fid->GetFiducials() )
          {
              ModelFiducial::Fiducial* other = &(*it);
              sendMessage(IAMCOMMING, robot->position->GetId(), other->id, 0, 0); // The value is not important
          }
      }
  }
  else if (robot->state == INGROUP)
  {
      

      robot->energy -= 2.5;
      // Step 1: Look for incomming messages and take appropriate update
      
      list<RobotMessage>::iterator mit;
      
      // first check JOINING and LEAVING bots
      bool joinleave = false;
      list<unsigned int> welcomeacklist;
      list<unsigned int> byeacklist;
      
      mit = robot->inbox.begin();
      while (mit != robot->inbox.end())
      {
          RobotMessage msg = *mit;
          if (msg.type == IAMCOMMING)
          {
              joinleave = true;
              robot->teammates.push_back(msg.sender);
              welcomeacklist.push_back(msg.sender);              
              robot->inbox.erase(mit++);
          }
          else if (msg.type == IAMLEAVING)
          {
              joinleave = true;
              robot->teammates.remove(msg.sender);
              byeacklist.push_back(msg.sender);
              sendMessage(BYE, robot->position->GetId(), msg.sender, 0, 0); // Reply Back              
              robot->inbox.erase(mit++);
          }
          else
          {
              ++mit;
          }
      }
      
      list<unsigned int>::iterator iit;
      list<unsigned int>::iterator tit; //Not only iterator names are confusing, this one is also a little bit naughty ;)

      for (iit = welcomeacklist.begin(); iit != welcomeacklist.end(); ++iit)
      {
          for (tit = robot->teammates.begin(); tit != robot->teammates.end(); ++tit)
          {
              // Send all my teammates information to all people who told me they are coming!
              //printf("*** WELCOME MESSAGE to %2d from %2d says %2d\n", robot->position->GetId(), *iit, *tit);
              sendMessage(WELCOME, robot->position->GetId(), *iit, *tit, 0); // Reply Back  
          }
      }
      
      // Create a new list of nearby robots that are not in welcome or bye lists
      
      list<unsigned int> inGroupNeighbors;
      FOR_EACH( it, fid->GetFiducials() )
      {
          ModelFiducial::Fiducial* other = &(*it);
          
          bool found = false;
          for (iit = welcomeacklist.begin(); (iit != welcomeacklist.end()) /* && (!found) */; ++iit)
          {
              if (other->id == *iit) found = true;
          }
          
          for (iit = byeacklist.begin(); (iit != byeacklist.end()) /* && (!found) */; ++iit)
          {
              if (other->id == *iit) found = true;
          }
          
          if (!found) inGroupNeighbors.push_back(other->id); 
      }
      
      // Update your belief using INGROUP SYNC messages
            
      mit = robot->inbox.begin();
      while (mit != robot->inbox.end())
      {
          RobotMessage msg = *mit;
          if ((msg.type == ADDROBOT) || (msg.type == REMOVEROBOT))
          {              
              if (msg.type == ADDROBOT)
              {
                  robot->teammates.push_back(msg.robotid);
              }
              else if (msg.type == REMOVEROBOT)
              {
                  robot->teammates.remove(msg.robotid);
              }
              
              //inGroupNeighbors.remove(msg.sender);
              if (msg.relaycount > 0)
              {
                  list<unsigned int>::iterator nit;
                  for (nit = inGroupNeighbors.begin(); nit != inGroupNeighbors.end(); ++nit)
                  {
                      sendMessage(msg.type, robot->position->GetId(), *nit, msg.robotid, msg.relaycount-1);
                  }
              }
          }
          robot->inbox.erase(mit++);
      }

      robot->teammates.sort();
      robot->teammates.unique();
      
           
      list<unsigned int>::iterator nit;
      for (nit = inGroupNeighbors.begin(); nit != inGroupNeighbors.end(); ++nit)
      {
          for (tit = welcomeacklist.begin(); tit != welcomeacklist.end(); ++tit)
          {
              sendMessage(ADDROBOT, robot->position->GetId(), *nit, *tit, 3);
          }
          
          for (tit = byeacklist.begin(); tit != byeacklist.end(); ++tit)
          {
              sendMessage(REMOVEROBOT, robot->position->GetId(), *nit, *tit, 3);
          }
          
//          for (tit = robot->teammates.begin(); tit != robot->teammates.end(); ++tit)
//          {
//              if (*tit != *nit)
//              {
//                  sendMessage(ADDROBOT, robot->position->GetId(), *nit, *tit, 1);
//              }
//          }
          
      }
      
      // Step 3: Check State Trnasitions for itself
      if (robot->energy < 500.0)
      {
          FOR_EACH( it, fid->GetFiducials() )
          {
              ModelFiducial::Fiducial* other = &(*it);
              //printf("**** I AM LEAVING GUYZ: %d \n", robot->position->GetId());
              sendMessage(IAMLEAVING, robot->position->GetId(), other->id, 0, 0); // The value is not important
          }
          robot->teammates.clear();
          
          robot->charger = Stg::Pose::Random(-10.0, 10.0, -16.0, -10.0);
          robot->position->Stop();
          setRobotState(robot, HOMING);
      }
      
      
  }
  else if (robot->state == HOMING)
  {
      //robot->position->SetPose(Stg::Pose::Random(-12.0, 12.0, -14.0, -10.0));
      
      double h_dx = robot->charger.x - robot->position->GetPose().x;
      double h_dy = robot->charger.y - robot->position->GetPose().y;
      
      double h_d = sqrt( (h_dx * h_dx) + (h_dy * h_dy) );
      
      if (h_d < 0.1)
      {
          setRobotState(robot,CHARGING);
      }
      else
      {
          double h_force = FATTMAX * 2.0 * (sigmoid(h_d) - 0.5);

          double h_theta = atan2(h_dy, h_dx);

          double h_force_x = h_force * cos(h_theta);
          double h_force_y = h_force * sin(h_theta);

          radians_t theta = robot->position->GetPose().a;
          double vx_robot = ( cos(theta) * h_force_x) + (sin(theta) * h_force_y);
          double vy_robot = (-sin(theta) * h_force_x) + (cos(theta) * h_force_y);

          robot->position->SetSpeed(vx_robot, vy_robot, 0.0);
      }
      //setRobotState(robot, CHARGING);
  }
  else if (robot->state == CHARGING)
  {
      if (robot->energy < robot->position->GetId() * 1000.0)
      {
          robot->energy += 10.0;
          robot->position->SetSpeed(0.0, 0.0, 0.0);
      }
      else
      {
          setRobotState(robot, SEARCHING);
      }
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
      double rmin_adaptive = RMIN + ((double) robot->teammates.size() / 4.0);
      if (att_d < rmin_adaptive)
      {
          att_force = -FATTMAX;
      }
      else
      {
          att_force = FATTMAX * 2.0 * (sigmoid(att_d - rmin_adaptive) - 0.5);
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
          double rep_theta = normalize(other->bearing + robot->position->GetPose().a);
          //double rep_theta = other->
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
      //radians_t angle_error = normalize(att_theta + 3.1415) - r_heading;

      w_robot = FWATT * 2.0 * (sigmoid(angle_error) - 0.5);

      robot->position->SetSpeed( vx_robot, vy_robot, w_robot);
  }
  
  //robot->position->SetSpeed(0.0, 0.0, 1.0);
  return 0;
}
