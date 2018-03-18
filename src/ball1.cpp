/****************************************************************
Dropping ball program

I assume some version of Open Dynamics Engine (ODE) is installed.

Changed drawstuff so this is negative and coordinate system has
X positive:
#define LIGHTY (-0.4f)

Note: 
x points to viewer's right.
z is up.
y points away from viewer, so angles are negative.

****************************************************************/

#include <math.h>
#include <string>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "ros/ros.h"

/****************************************************************/

#ifdef _MSC_VER
#pragma warning(disable : 4244 4305) // for VC++, no precision loss complaints
#endif

/****************************************************************/
/****************************************************************/

#define TRUE 1
#define FALSE 0

#define XX 0
#define YY 1
#define ZZ 2

// #define TIME_STEP 0.005
#define TIME_STEP 0.001
#define WAIT_TO_START 0.1

// These should be set up in configuration file
#define BALL_DIAMETER 0.01          // meters
#define WALL_WIDTH 0.5              // meters
#define WALL_THICKNESS 0.01         // meters
#define WALL_HEIGHT 0.5             // meters
#define FUDGE1 (BALL_DIAMETER / 10) // space between ball and wall

// size in pixels of window
#define WINDOW_WIDTH 500
#define WINDOW_HEIGHT 500

// used to allocate space for obstacles
#define MAX_N_GEOMS 100

// object types
#define BALL 0
#define OBSTACLE 1
#define WALL 2
#define UNKNOWN -1

/****************************************************************/

// select correct drawing functions (we are using double version)
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawConvex dsDrawConvexD
#define dsDrawTriangle dsDrawTriangleD
#endif

/****************************************************************/
// Typedefs

typedef struct rect
{
  double dimensions[3];
  double pos[3];
  double angle;
} RECT;

typedef struct setup
{
  // these are static ODE objects: just geometries

  dGeomID the_wall;

  RECT obstacles[MAX_N_GEOMS];
  dGeomID the_obstacles[MAX_N_GEOMS];
  int n_obstacles;
} SETUP;

/****************************************************************/
// Globals

static dReal my_time = 0; // what time is it?

// ODE dynamics and collision objects
static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;

// the ODE ball
static dBodyID ball_body;
static dGeomID ball_geom;

// the current setup
static SETUP current_setup;

static int simulation_running = 1;

/****************************************************************/

// This function is called at the start of the simulation
// to set up the point of view of the camera.
// to initialize the ball position
static void start()
{
  float xyz[3] = {(float)(WALL_WIDTH / 2), -0.35f, (float)(WALL_HEIGHT / 2)};
  float hpr[3] = {90.000f, 0.0000f, 0.0000f};
  dsSetViewpoint(xyz, hpr);
}

/****************************************************************/
// this program figures out what kind of collision it is.

void identify_contact(dGeomID o, int *type, const char **str, SETUP *h)
{
  int i;

  if (o == ball_geom)
  {
    *type = BALL;
    *str = "ball";
    return;
  }
  if (o == h->the_wall)
  {
    *type = WALL;
    *str = "wall";
    return;
  }
  for (i = 0; i < h->n_obstacles; i++)
    if (o == h->the_obstacles[i])
    {
      *type = OBSTACLE;
      *str = "obstacle";
      return;
    }
  *type = UNKNOWN;
  *str = "unknown";
}

/****************************************************************/

/*
When the collision system detects that two objects are colliding, it
calls this routine which determines the points of contact and creates
temporary joints. The surface parameters of the joint (friction,
bounce velocity, CFM, etc) are also set here.
*/
// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  dContact contact;
  int o1_type, o2_type, c_type;
  const char *o1_string, *o2_string, *c_string;

  // Need to identify contact:
  identify_contact(o1, &o1_type, &o1_string, &current_setup);
  identify_contact(o2, &o2_type, &o2_string, &current_setup);
  // printf( "%s %s\n", o1_string, o2_string );
  // give up if unknown object involved
  if (o1_type == UNKNOWN)
    return;
  if (o2_type == UNKNOWN)
    return;
  if (o1_type == BALL)
  {
    c_type = o2_type;
    c_string = o2_string;
  }
  else if (o2_type == BALL)
  {
    c_type = o1_type;
    c_string = o1_string;
  }
  else
    return; // no ball involved, give up.

  if (c_type == WALL)
  {
    contact.surface.mode = dContactBounce | dContactSoftCFM;
    // friction parameter
    contact.surface.mu = dInfinity;
    // bounce is the amount of "bouncyness".
    contact.surface.bounce = 0.0;
    // bounce_vel is the minimum incoming velocity to cause a bounce
    contact.surface.bounce_vel = 1e10;
    // constraint force mixing parameter
    contact.surface.soft_cfm = 0.01; // 0.001 in bounce
  }
  else if (c_type == OBSTACLE)
  {
    contact.surface.mode = dContactBounce | dContactSoftCFM;
    // friction parameter
    contact.surface.mu = 1.0;
    // bounce is the amount of "bouncyness".
    contact.surface.bounce = 0.3; // used to be 0.1
    // bounce_vel is the minimum incoming velocity to cause a bounce
    contact.surface.bounce_vel = 0.01;
    // constraint force mixing parameter
    contact.surface.soft_cfm = 0.01; // 0.001 in bounce
  }
  else
    return; // no obstacle or wall involved, give up

  if (int numc = dCollide(o1, o2, 1, &contact.geom, sizeof(dContact)))
  {
    dJointID c = dJointCreateContact(world, contactgroup, &contact);
    dJointAttach(c, b1, b2);
  }
}

/****************************************************************/
// no controller

void do_user_stuff(SETUP *h)
{
  return;
}

/****************************************************************/

void do_simulation_step()
{
  // find collisions and add contact joints
  dSpaceCollide(space, 0, &nearCallback);

  // step the simulation
  dWorldStep(world, TIME_STEP);

  my_time += TIME_STEP;

  // remove all contact joints
  dJointGroupEmpty(contactgroup);
}

/****************************************************************/

void draw_stuff()
{
  int i;
  const dReal *pos;
  const dReal *R;
  dReal sides[3];

  // redraw sphere at new location
  pos = dGeomGetPosition(ball_geom);
  R = dGeomGetRotation(ball_geom);
  dsSetColor(1, 0, 0);
  dsDrawSphere(pos, R, dGeomSphereGetRadius(ball_geom));

  // redraw wall
  pos = dGeomGetPosition(current_setup.the_wall);
  R = dGeomGetRotation(current_setup.the_wall);
  dGeomBoxGetLengths(current_setup.the_wall, sides);
  dsSetColor(0.0, 0.75, 0.0);
  dsDrawBox(pos, R, sides);

  // redraw obstacles
  for (i = 0; i < current_setup.n_obstacles; i++)
  {
    pos = dGeomGetPosition(current_setup.the_obstacles[i]);
    R = dGeomGetRotation(current_setup.the_obstacles[i]);
    dGeomBoxGetLengths(current_setup.the_obstacles[i], sides);
    dsSetColor(0.75, 0.5, 0.25);
    dsDrawBox(pos, R, sides);
  }
}

/****************************************************************/

/*
This is the main simulation loop that calls the collision detection
function, steps the simulation, resets the temporary contact joint
group, and redraws the objects at their new position.
*/

// simulation loop
static void simLoop(int pause)
{
  const dReal *pos;
  const dReal *pos2;
  static int printed_result = 0;
  dQuaternion q;

  // do user defined stuff
  do_user_stuff(&current_setup);

  if (simulation_running)
    do_simulation_step();

  // hold ball at top
  if (my_time < WAIT_TO_START)
  {
    dBodySetPosition(ball_body, 0.0, 0.0, WALL_HEIGHT);
    q[0] = 1.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;
    dBodySetQuaternion(ball_body, q);
    dBodySetLinearVel(ball_body, 0.0, 0.0, 0.0);
    dBodySetAngularVel(ball_body, 0.0, 0.0, 0.0);
  }

  pos = dGeomGetPosition(ball_geom);
  pos2 = dBodyGetPosition(ball_body);
  printf("%lg %lg %lg\n", my_time, pos[XX], pos[ZZ]);
  if (pos[ZZ] < BALL_DIAMETER / 2)
  {
    simulation_running = FALSE;
    dGeomSetPosition(ball_geom, pos[XX], pos[YY], BALL_DIAMETER / 2 - 0.0001);
    dBodySetPosition(ball_body, pos2[XX], pos2[YY], BALL_DIAMETER / 2 - 0.0001);
    q[0] = 1.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;
    dBodySetQuaternion(ball_body, q);
    dBodySetLinearVel(ball_body, 0.0, 0.0, 0.0);
    dBodySetAngularVel(ball_body, 0.0, 0.0, 0.0);
    if (!printed_result)
    {
      printf("%lg %lg %lg\n", my_time, pos[XX], pos[ZZ]);
      printed_result = 1;
      exit(0);
    }
  }

  draw_stuff();
  ros::spinOnce();
}

/****************************************************************/

int read_setup_file(const char *filename, SETUP *h)
{
  int i;
  FILE *stream;
  char buffer[1000];

  stream = fopen(filename, "r");
  if (stream == NULL)
  {
    fprintf(stderr, "Can't open setup file %s", filename);
    exit(-1);
  }

  h->n_obstacles = 0;

  for (;;)
  {
    // read keyword
    if (fscanf(stream, "%s", buffer) < 1)
      break; // if we didn't read anything we are done

    // handle an obstacle
    if (strcmp(buffer, "o") == 0)
    {
      i = h->n_obstacles;
      if (fscanf(stream, "%lg%lg%lg%lg%lg%lg%lg",
                 &(h->obstacles[i].dimensions[XX]),
                 &(h->obstacles[i].dimensions[YY]),
                 &(h->obstacles[i].dimensions[ZZ]),
                 &(h->obstacles[i].pos[XX]),
                 &(h->obstacles[i].pos[YY]),
                 &(h->obstacles[i].pos[ZZ]),
                 &(h->obstacles[i].angle)) < 7)
      {
        fprintf(stderr, "bad obstacle in setup file %s\n", filename);
        exit(-1);
      }
      printf("obstacle: %g %g %g %g %g %g %g\n",
             h->obstacles[i].dimensions[XX],
             h->obstacles[i].dimensions[YY],
             h->obstacles[i].dimensions[ZZ],
             h->obstacles[i].pos[XX],
             h->obstacles[i].pos[YY],
             h->obstacles[i].pos[ZZ],
             h->obstacles[i].angle);
      h->n_obstacles++;
      continue;
    }

    fprintf(stderr, "bad keyword %s in setup file %s\n", buffer, filename);
    exit(-1);
  }

  fclose(stream);
}

/****************************************************************/

void create_bodies(const char *filename, SETUP *h)
{
  int i;
  dMass m;
  dQuaternion q;

  read_setup_file(filename, h);

  // create ball object
  ball_body = dBodyCreate(world);
  ball_geom = dCreateSphere(space, BALL_DIAMETER / 2);
  dMassSetSphere(&m, 1, BALL_DIAMETER / 2);
  dBodySetMass(ball_body, &m);
  dGeomSetBody(ball_geom, ball_body);
  dBodySetLinearDamping(ball_body, 1e-4);
  dBodySetLinearDampingThreshold(ball_body, 1e-7);
  // set initial position and velocity
  dBodySetPosition(ball_body, 0.0, 0.0, WALL_HEIGHT);
  q[0] = 1.0;
  q[1] = 0.0;
  q[2] = 0.0;
  q[3] = 0.0;
  dBodySetQuaternion(ball_body, q);
  dBodySetLinearVel(ball_body, 0.0, 0.0, 0.0);
  dBodySetAngularVel(ball_body, 0.0, 0.0, 0.0);

  /*
    FROM FAQ:

  How can an immovable body be created?

  In other words, how can you create a body that doesn't move, but
  that interacts with other bodies? The answer is to create a geom
  only, without the corresponding rigid body object. The geom is
  associated with a rigid body ID of zero. Then in the contact
  callback when you detect a collision between two geoms with a
  nonzero body ID and a zero body ID, you can simply pass those two
  IDs to the dJointAttach function as normal. This will create a
  contact between the rigid body and the static environment.
  */

  h->the_wall = dCreateBox(space, WALL_WIDTH, WALL_THICKNESS, WALL_HEIGHT);
  dGeomSetPosition(h->the_wall, WALL_WIDTH / 2 - BALL_DIAMETER,
                   BALL_DIAMETER + FUDGE1, WALL_HEIGHT / 2);

  for (i = 0; i < h->n_obstacles; i++)
  {
    h->the_obstacles[i] =
        dCreateBox(space, h->obstacles[i].dimensions[XX],
                   h->obstacles[i].dimensions[YY],
                   h->obstacles[i].dimensions[ZZ]);
    dGeomSetPosition(h->the_obstacles[i], h->obstacles[i].pos[XX],
                     h->obstacles[i].pos[YY],
                     h->obstacles[i].pos[ZZ]);
    q[0] = cos(h->obstacles[i].angle / 2);
    q[1] = 0.0;
    q[2] = sin(h->obstacles[i].angle / 2);
    q[3] = 0.0;
    dGeomSetQuaternion(h->the_obstacles[i], q);
  }
}

//============================

// called when a key pressed

static void command(int cmd)
{
  // do nothing
}

/****************************************************************/

/*
Main program: When the program starts, the callbacks are set up,
everything is initialized, and then the simulation is started.
*/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "marble_simulator_ode_node");
  ros::NodeHandle n("~");
  std::string obstacle_file_name;
  if (n.getParam("obstacle_file", obstacle_file_name))
  {
      ROS_INFO("Got param: %s", obstacle_file_name.c_str());
  }
  else
  {
      ROS_ERROR("Failed to get param 'obstacle_file'");
  }

  dReal erp, cfm;
  

  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.stop = 0;
  fn.command = &command;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

  dInitODE();
  // create world
  world = dWorldCreate();
  space = dHashSpaceCreate(0);
  dWorldSetGravity(world, 0.0, 0.0, -9.81);
  dWorldSetCFM(world, 1e-5);
  // dWorldSetERP (dWorldID, dReal erp);
  erp = dWorldGetERP(world);
  cfm = dWorldGetCFM(world);
  /*
  printf( "erp: %g, cfm: %g, kp: %g, kd: %g\n",
	  erp, cfm, erp/(cfm*TIME_STEP), (1 - erp)/cfm );
  */

  contactgroup = dJointGroupCreate(0);
  const char* filename = obstacle_file_name.c_str();
  create_bodies(filename, &current_setup);

  // run simulation
  dsSimulationLoop(argc, argv, WINDOW_WIDTH, WINDOW_HEIGHT, &fn);

  // clean up
  dJointGroupDestroy(contactgroup);
  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE();
  return 0;
}

/****************************************************************/
