#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>

#include <ctime>
#include <list.h>

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
    double val;
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
  double belief;
} robot_t;

// Define const double XXX = YYYY; Here for global access during control

const double FATTMAX = 1.0;
const double FREPMAX = -1.0;
const double FWATT = 0.01;
const double RMIN = 5.0;

static list<RobotMessage> MessageList;

// Local Functions

double getCurrentTimeStamp()
{
  timeval tt;
  gettimeofday(&tt, NULL);
  double sec = (double) tt.tv_sec;
  double usec = (double) tt.tv_usec;
  return sec + (usec / 1.0e6);
}


// No Broadcast support yet ...
void sendMessage(MessageType type, int sender, int receiver, double val)
{
    RobotMessage msg;
    msg.type = type;
    msg.t = getCurrentTimeStamp();
    msg.sender = sender;
    msg.receiver = receiver;
    msg.val = val;
    
    MessageList.push_back(msg);
}

void updateMyInbox(robot_t* robot)
{
    robot->inbox.clear();
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

list<RobotMessage> myInbox(int id)
{
    
}

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
  
  robot->state = SEARCHING;
  robot->belief = 1.0;
  
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

/*
  // Find the nearest robot code
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

  if( robot->closest ) // if we saw someone
    {
      robot->closest_bearing = robot->closest->bearing;
      robot->closest_range = robot->closest->range;
      robot->closest_heading_error = robot->closest->geom.a;
    }
*/
  //printf("Position of %d : %6.4f %6.4f \n", robot->position->GetId(), robot->position->GetPose().x, robot->position->GetPose().y);

  /* This is some shared knowledge, let's say it's global now */
  
  static Pose attCenter = Pose(0.0, 0.0, 0.0, 0.0);
  static double movementX = 0.0;//1e-4;
  
  updateMyInbox(robot);
  
  /* Message Parsing */
  
  /* End Message Parsing */
  
  /* State Based Communication and Transitions */
  
  if (robot->state == SEARCHING)
  {
      FOR_EACH( it, fid->GetFiducials() )
      {
          ModelFiducial::Fiducial* other = &(*it);
          sendMessage(IAMCOMMING, robot->position->GetId(), other->id, 0.0); // The value is not important
      }
  }
  else if (robot->state == INGROUP)
  {
      ;
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
      double att_dx = attCenter.x - robot->position->GetPose().x;
      double att_dy = attCenter.y - robot->position->GetPose().y;

      attCenter.x += movementX;

      if (fabs(attCenter.x) > 4.0)
      {
          movementX = -1.0 * movementX;
      }

      //printf("Error of %d : %6.4f %6.4f \n", robot->position->GetId(), att_dx, att_dy);
      double att_d = sqrt( (att_dx * att_dx) + (att_dy * att_dy) );

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
