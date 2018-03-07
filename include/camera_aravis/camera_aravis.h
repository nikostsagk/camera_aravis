// camera_aravis
//
// This is a ROS node that operates GenICam-based cameras via the Aravis library.
// Commonly available camera features are supported through the dynamic_reconfigure user-interface and GUI,
// and for those features not in the GUI but that are specific to a camera, they can be set in the
// camera by setting the appropriate parameter at startup.  This code reads those parameters, and
// if any of them match a camera feature, then the camera is written to.
//
// For example, if a camera has a feature called "IRFormat" that is an integer 0, 1, or 2, you can do
// rosparam set camnode/IRFormat 2
// and this driver will write it to the camera at startup.  Note that the datatype of the parameter
// must be correct for the camera feature (e.g. bool, int, double, string, etc), so for example you should use
// rosparam set camnode/GainAuto true
// and NOT
// rosparam set camnode/GainAuto 1
//

#include <arv.h>

#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <glib.h>

#include <math.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "camera_aravis/CamStats.h"
#include <dynamic_reconfigure/server.h>
#include <tf/transform_listener.h>
#include <camera_aravis/CameraAravisIDSConfig.h>
#include <camera_aravis/CameraAravisMakoConfig.h>
#include <camera_aravis/CameraAravisProsilicaConfig.h>
#include <camera_aravis/CameraAravisPointgreyConfig.h>

#include <boost/thread.hpp>

#include "XmlRpc.h"

//#define TUNING	// Allows tuning the gains for the timestamp controller.  Publishes output on topic /dt, and receives gains on params /kp, /ki, /kd

#define CLIP(x,lo,hi)	MIN(MAX((lo),(x)),(hi))
#define THROW_ERROR(m) throw std::string((m))

#define TRIGGERSOURCE_SOFTWARE	0
#define TRIGGERSOURCE_LINE1		1
#define TRIGGERSOURCE_LINE2		2

#define ARV_PIXEL_FORMAT_BIT_PER_PIXEL(pixel_format)  (((pixel_format) >> 16) & 0xff)
#define ARV_PIXEL_FORMAT_BYTE_PER_PIXEL(pixel_format) ((((pixel_format) >> 16) & 0xff) >> 3)

namespace camera_aravis
{

typedef struct
{
  const char *szName;
  const char *szTag;
  ArvDomNode *pNode;
  ArvDomNode *pNodeSibling;
} NODEEX;

volatile gboolean bCancel;

void set_cancel (int signal)
{
    bCancel = TRUE;
}

const static char * NAME = "aravis";

class CameraNode
{

  typedef camera_aravis::CameraAravisIDSConfig IDSConfig;
  typedef camera_aravis::CameraAravisMakoConfig MakoConfig;
  typedef camera_aravis::CameraAravisProsilicaConfig ProsilicaConfig;
  typedef camera_aravis::CameraAravisPointgreyConfig PointgreyConfig;

  typedef struct
  {
    GMainLoop  *main_loop;
    int         nBuffers;	// Counter for Hz calculation.
    guint64 past_completed_buffers;
    guint64 past_failures;
    guint64 past_underruns;
    guint64 past_missing;
    unsigned int n_secs;
    int annotating_period;
    int specified_frame_rate;
    ros::Publisher                          cam_stats_pub;
    ArvGvStream *pstream_for_periodic_cb;
  } ApplicationData;
  // ------------------------------------

  boost::thread init_thread_;

  // Conversions from integers to Arv types.
  const static char *szBufferStatusFromInt[];

  ArvGvStream *CreateStream(ros::NodeHandle &nh);

  //void RosReconfigure_callback(Config &config, uint32_t level);
  
  static void stream_priority_callback (void *user_data, ArvStreamCallbackType type, ArvBuffer *buffer);
  
  static void NewBuffer_callback(ArvStream *pStream, gpointer *data);

  static void ControlLost_callback(ArvGvDevice *pGvDevice, gpointer* data);

  static gboolean SoftwareTrigger_callback(void *pCamera);

  // PeriodicTask_callback()
  // Check for termination, and spin for ROS.
  static gboolean PeriodicTask_callback (void *data);

  // Get the child and the child's sibling, where <p___> indicates an indirection.
  NODEEX GetGcFirstChild(ArvGc *pGenicam, NODEEX nodeex);

  // Get the sibling and the sibling's sibling, where <p___> indicates an indirection.
  NODEEX GetGcNextSibling(ArvGc *pGenicam, NODEEX nodeex);

  // Walk the DOM tree, i.e. the tree represented by the XML file in the camera, and that contains all the various features, parameters, etc.
  void PrintDOMTree(ArvGc *pGenicam, NODEEX nodeex, int nIndent, bool debug=false);
  
  std::string setCameraFeature(std::string featureName, std::string featureVal);
  int setCameraFeature(std::string featureName, int featureVal);
  double setCameraFeature(std::string featureName, double featureVal);
  bool setCameraFeature(std::string featureName, bool featureVal);
  
  void setFeatureFromParam(ros::NodeHandle &nh, std::string paramName, std::string type);

  const char* GetPixelEncoding(ArvPixelFormat pixel_format);

  //virtual void onInit();

private:
  ApplicationData                         applicationData;

  image_transport::CameraPublisher        publisher;
  camera_info_manager::CameraInfoManager *pCameraInfoManager;

  IDSConfig                               configIDS;
  MakoConfig                              configMako;
  ProsilicaConfig                         configProsilica;
  PointgreyConfig                         configPointgrey;
  
  int                                     idSoftwareTriggerTimer;
  
  std::string                             frame_id;
  
  int                                     xRoi;
  int                                     yRoi;
  int                                     widthRoi;
  int                                     widthRoiMin;
  int                                     widthRoiMax;
  int                                     heightRoi;
  int                                     heightRoiMin;
  int                                     heightRoiMax;

  int                                     widthSensor;
  int                                     heightSensor;

  const char                             *pszPixelformat;
  unsigned                                nBytesPixel;
  ArvCamera                              *pCamera;
  ArvDevice                              *pDevice;
public:
  
  void Start();
};
}
