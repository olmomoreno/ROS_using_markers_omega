
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <stdio.h>
#include "dhdc.h"

#define K   500.0

int main( int argc, char** argv )
{

  int enc[DHD_MAX_DOF];
  int encCount;

  double px, py, pz, zp;
  double fx, fy, fz;
  int posx;
  int posy;
  int posz;
  int orix;
  int oriy;
  int oriz;
  int done=0;

  fx = 0.0;
  fy = 0.0;
  zp = -0.000483438;

  ros::init(argc, argv, "basic_shapes_omega");
  ros::NodeHandle n;
  ros::Rate r(1000);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1000);

  //OMEGA open the first available device
  if (dhdOpen () < 0) {
    //printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
    dhdSleep (2.0);
    return -1;
  }

  //OMEGA identify number of encoders to report based on device type
  switch (dhdGetSystemType ()) {
  case DHD_DEVICE_DELTA3:
  case DHD_DEVICE_OMEGA3:
  case DHD_DEVICE_FALCON:
    encCount = 3;
    break;
  case DHD_DEVICE_DELTA6:
  case DHD_DEVICE_OMEGA33:
  case DHD_DEVICE_OMEGA33_LEFT:
    encCount = 6;
    break;
  case DHD_DEVICE_OMEGA331:
  case DHD_DEVICE_OMEGA331_LEFT:
    encCount = 7;
    break;
  case DHD_DEVICE_CONTROLLER:
  case DHD_DEVICE_CONTROLLER_HR:
  default:
    encCount = 8;
    break;
  }

// enable force
  dhdEnableForce (DHD_ON);

  // Set our initial shape type to be a cube
// %Tag(SHAPE_INIT)%
  uint32_t shape = visualization_msgs::Marker::CUBE;
// %EndTag(SHAPE_INIT)%

// %Tag(MARKER_INIT)%
  while (ros::ok())
  {
    
    //OMEGA read all available encoders
    if (dhdGetEnc (enc) < 0) {
      //printf ("error: cannot read encoders (%s)\n", dhdErrorGetLastStr ());
      //done = 1;
    }   

   // get end-effector position
    dhdGetPosition (&px, &py, &pz);
   
   fz = -K*(pz-zp);

 // apply forces
    dhdSetForce (fx, fy, fz);

// print out encoders according to system type
    ROS_INFO_STREAM("Encoders(" <<  enc[0] << "," << enc[1] << "," 
       << enc[2] << "," << enc[3] << "," << enc[4] << "," << pz << "," <<  fz << ")");


    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();
// %EndTag(MARKER_INIT)%

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
// %Tag(NS_ID)%
    marker.ns = "basic_shapes_omega";
    marker.id = 0;
// %EndTag(NS_ID)%

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
// %Tag(TYPE)%
    marker.type = shape;
// %EndTag(TYPE)%

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
// %Tag(ACTION)%
    marker.action = visualization_msgs::Marker::ADD;
// %EndTag(ACTION)%

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
// %Tag(POSE)%
    marker.pose.position.x = px*100;
    marker.pose.position.y = py*100;
    marker.pose.position.z = pz*100;
    marker.pose.orientation.x = pz*100;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
// %EndTag(POSE)%

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
// %Tag(SCALE)%
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
// %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
// %Tag(COLOR)%
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
// %EndTag(COLOR)%

// %Tag(LIFETIME)%
    marker.lifetime = ros::Duration();
// %EndTag(LIFETIME)%

    // Publish the marker
// %Tag(PUBLISH)%
    /*while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }*/
    marker_pub.publish(marker);
// %EndTag(PUBLISH)%

 // exit if the button is pushed
    done += dhdGetButton (0);
    if(done==1) ros::shutdown();

// %Tag(SLEEP_END)%
    r.sleep();
  }
// %EndTag(SLEEP_END)%
  
//printf ("exiting application\n");
  ROS_INFO_STREAM("exiting application\n");

// disable force
  dhdEnableForce (DHD_OFF);
  //OMEGA close the connection
  dhdClose ();
}
