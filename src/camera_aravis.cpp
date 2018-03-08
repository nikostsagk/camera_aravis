#include <camera_aravis/camera_aravis.h>

namespace camera_aravis
{

// Conversions from integers to Arv types.
const char* CameraNode::szBufferStatusFromInt[] = {
  "ARV_BUFFER_STATUS_SUCCESS",
  "ARV_BUFFER_STATUS_CLEARED",
  "ARV_BUFFER_STATUS_TIMEOUT",
  "ARV_BUFFER_STATUS_MISSING_PACKETS",
  "ARV_BUFFER_STATUS_WRONG_PACKET_ID",
  "ARV_BUFFER_STATUS_SIZE_MISMATCH",
  "ARV_BUFFER_STATUS_FILLING",
  "ARV_BUFFER_STATUS_ABORTED"
};

std::string CameraNode::setCameraFeature(std::string featureName, std::string featureVal){
  arv_device_set_string_feature_value (pDevice, featureName.c_str(), featureVal.c_str());
  std::string valFromCam = arv_device_get_string_feature_value (pDevice, featureName.c_str());
  if(valFromCam == featureVal){
    ROS_INFO_NAMED (NAME, "%s = %s", featureName.c_str(), valFromCam.c_str());
  }
  else{
    ROS_WARN_NAMED (NAME, "%s couldn't be set to %s, instead set to %s", featureName.c_str(), featureVal.c_str(), valFromCam.c_str());
  }
  return valFromCam;
}

int CameraNode::setCameraFeature(std::string featureName, int featureVal){
  arv_device_set_integer_feature_value (pDevice,featureName.c_str(),featureVal);
  int valFromCam = arv_device_get_integer_feature_value (pDevice, featureName.c_str());
  if(valFromCam == featureVal){
    ROS_INFO_NAMED (NAME, "%s = %d", featureName.c_str(), valFromCam);
  }
  else{
    ROS_WARN_NAMED (NAME, "%s couldn't be set to %d, instead set to %d", featureName.c_str(), featureVal, valFromCam);
  }
  return valFromCam;
}

double CameraNode::setCameraFeature(std::string featureName, double featureVal){
  arv_device_set_float_feature_value (pDevice,featureName.c_str(),featureVal);
  double valFromCam = arv_device_get_float_feature_value (pDevice, featureName.c_str());
  if(valFromCam == featureVal){
    ROS_INFO_NAMED (NAME, "%s = %f", featureName.c_str(), valFromCam);
  }
  else{
    ROS_WARN_NAMED (NAME, "%s couldn't be set to %f, instead set to %f", featureName.c_str(), featureVal, valFromCam);
  }
  return valFromCam;
}

bool CameraNode::setCameraFeature(std::string featureName, bool featureVal){
  gboolean gfeatureVal;
  if (featureVal == true){
    gfeatureVal = true;
  }
  else{
    gfeatureVal = false;
  }
  arv_device_set_boolean_feature_value (pDevice,featureName.c_str(),gfeatureVal);
  gboolean valFromCam = arv_device_get_boolean_feature_value (pDevice, featureName.c_str());
  if(valFromCam == gfeatureVal){
    ROS_INFO_NAMED (NAME, "%s = %s", featureName.c_str(), valFromCam?"True":"False");
  }
  else{
    ROS_WARN_NAMED (NAME, "%s couldn't be set to %s, instead set to %s", featureName.c_str(), gfeatureVal?"True":"False", valFromCam?"True":"False");
  }
  return valFromCam;
}
void CameraNode::getCameraFeature(std::string featureName, std::string& featureVal){
  featureVal = arv_device_get_string_feature_value (pDevice, featureName.c_str());
}
void CameraNode::getCameraFeature(std::string featureName, int& featureVal){
  featureVal = arv_device_get_integer_feature_value (pDevice, featureName.c_str());
}
void CameraNode::getCameraFeature(std::string featureName, double& featureVal){
  featureVal = arv_device_get_float_feature_value (pDevice, featureName.c_str());
}
void CameraNode::getCameraFeature(std::string featureName, bool& featureVal){
  gboolean valFromCam = arv_device_get_boolean_feature_value (pDevice, featureName.c_str());
  featureVal = valFromCam?true:false;
}

void CameraNode::setFeatureFromParam(ros::NodeHandle &nh, std::string paramName, std::string type){
  std::string node_name = ros::this_node::getName();
  std::string fullParamName = node_name+"/"+paramName;
  if (nh.hasParam(fullParamName))
  {
    if(type == "str"){
      std::string strFeature;
      nh.getParam(fullParamName, strFeature);
      setCameraFeature(paramName, strFeature);
    }
    else if(type == "int"){
      int intFeature;
      nh.getParam(fullParamName, intFeature);
      setCameraFeature(paramName, intFeature);
    }
    else if(type == "float"){
      double floatFeature;
      nh.getParam(fullParamName, floatFeature);
      setCameraFeature(paramName, floatFeature);
    }
    else if(type == "bool"){
      bool boolFeature;
      nh.getParam(fullParamName, boolFeature);
      setCameraFeature(paramName, boolFeature);
    }
    else{
      ROS_WARN_NAMED (NAME, "Invalid type %s for camera feature %s", type.c_str(), paramName.c_str());
    }
  }
  else{
    ROS_WARN_NAMED (NAME, "Parameter %s doesn't exist", paramName.c_str());
  }
} //setFeatureFromParam()

ArvGvStream* CameraNode::CreateStream(ros::NodeHandle &nh)
{
    gboolean 		bAutoBuffer = FALSE;
    gboolean 		bPacketResend = TRUE;
    unsigned int 	timeoutPacket = 40; // milliseconds
    unsigned int 	timeoutFrameRetention = 200;

    ArvGvStream* pStream = (ArvGvStream *)arv_device_create_stream (pDevice, stream_priority_callback, NULL);
    
    std::string node_name = ros::this_node::getName();
    std::string fullParamName = node_name+"/"+"thread_priority";
    int priority;
    if (nh.hasParam(fullParamName))
    {
      nh.getParam(fullParamName, priority);
      if(priority == 2){
        if (arv_make_thread_realtime (10)){
          ROS_INFO_NAMED (NAME, "Set stream thread to realtime priority");
        }
        else{
          ROS_WARN_NAMED (NAME, "Couldn't set stream thread to realtime priority");
        }
      }
  		else if(priority == 1){
        if(arv_make_thread_high_priority (-10)){
          ROS_INFO_NAMED (NAME, "Set stream thread to high priority");
        }
        else{
          ROS_WARN_NAMED (NAME, "Couldn't set stream thread to high priority");
        }
      }
  		else{
        ROS_INFO_NAMED (NAME, "Set stream thread to normal priority");
      }
    }
    
    if (pStream)
    {
        ArvBuffer	*pBuffer;
        gint 		 nbytesPayload;


        if (!ARV_IS_GV_STREAM (pStream))
            ROS_WARN_NAMED (NAME, "Stream is not a GV_STREAM");

        if (bAutoBuffer)
            g_object_set (pStream,
                          "socket-buffer",
                          ARV_GV_STREAM_SOCKET_BUFFER_AUTO,
                          "socket-buffer-size", 0,
                          NULL);
        if (!bPacketResend)
            g_object_set (pStream,
                          "packet-resend",
                          bPacketResend ? ARV_GV_STREAM_PACKET_RESEND_ALWAYS : ARV_GV_STREAM_PACKET_RESEND_NEVER,
                          NULL);
        g_object_set (pStream,
                      "packet-timeout",
                      (unsigned) timeoutPacket * 1000,
                      "frame-retention", (unsigned) timeoutFrameRetention * 1000,
                      NULL);

        // Load up some buffers.
        nbytesPayload = arv_camera_get_payload (pCamera);
        for (int i=0; i<50; i++)
        {
            pBuffer = arv_buffer_new (nbytesPayload, NULL);
            arv_stream_push_buffer ((ArvStream *)pStream, pBuffer);
        }
    }
    return pStream;
} // CreateStream()

void CameraNode::RosReconfigure_callback_pointgrey(PointgreyConfig &newconfig, uint32_t level)
{    
    int             changedAcquisitionMode;
    int             changedAcquisitionFrameRate;
    int             changedExposureTime;
    int             changedExposureAuto;
    int             changedAutoExposureTimeLowerLimit;
    int             changedAutoExposureTimeUpperLimit;
    int             changedGain;
    int             changedGainAuto;
    int             changedAutoGainLowerLimit;
    int             changedAutoGainUpperLimit;

    // Find what the user changed.
    changedAcquisitionMode    			= (newconfig.AcquisitionMode != configPointgrey.AcquisitionMode);
    changedAcquisitionFrameRate         = (newconfig.AcquisitionFrameRate != configPointgrey.AcquisitionFrameRate);
    
    changedExposureTime = (newconfig.ExposureTime != configPointgrey.ExposureTime);
    changedExposureAuto 		= (newconfig.ExposureAuto != configPointgrey.ExposureAuto);
    changedAutoExposureTimeLowerLimit = (newconfig.AutoExposureTimeLowerLimit != configPointgrey.AutoExposureTimeLowerLimit);
    changedAutoExposureTimeUpperLimit = (newconfig.AutoExposureTimeUpperLimit != configPointgrey.AutoExposureTimeUpperLimit);
    
    changedGain         		= (newconfig.Gain != configPointgrey.Gain);
    changedGainAuto     		= (newconfig.GainAuto != configPointgrey.GainAuto);
    changedAutoGainLowerLimit = (newconfig.AutoGainLowerLimit != configPointgrey.AutoGainLowerLimit);
    changedAutoGainUpperLimit = (newconfig.AutoGainUpperLimit != configPointgrey.AutoGainUpperLimit);
    
    // Adjust other controls dependent on what the user changed.
    if (changedExposureTime || changedGain){
      newconfig.ExposureAuto = "Off";
      newconfig.GainAuto = "Off";
    }
    changedExposureAuto 		= (newconfig.ExposureAuto != configPointgrey.ExposureAuto);
    changedGainAuto     		= (newconfig.GainAuto != configPointgrey.GainAuto);
    
    // Set params into the camera.
    if (changedAcquisitionMode)
    {
      newconfig.AcquisitionMode = setCameraFeature("AcquisitionMode", newconfig.AcquisitionMode);
      ros::Duration(1.0).sleep();
    }
    
    if (changedAcquisitionFrameRate)
    {
      newconfig.AcquisitionFrameRate = setCameraFeature("AcquisitionFrameRate", newconfig.AcquisitionFrameRate);
    }
    
    if (changedExposureTime)
    {
      newconfig.ExposureTime = setCameraFeature("ExposureTime", newconfig.ExposureTime);
      ros::Duration(1.0).sleep();
    }
    
    if (changedExposureAuto)
    {
      newconfig.ExposureAuto = setCameraFeature("ExposureAuto", newconfig.ExposureAuto);
      ros::Duration(1.0).sleep();
      if (newconfig.ExposureAuto=="Once")
      {
          newconfig.ExposureAuto = "Off";
          newconfig.ExposureAuto = setCameraFeature("ExposureAuto", newconfig.ExposureAuto);
          ros::Duration(1.0).sleep();
      }
    }
    
    if (changedAutoExposureTimeLowerLimit)
    {
      newconfig.AutoExposureTimeLowerLimit = setCameraFeature("AutoExposureTimeLowerLimit", newconfig.AutoExposureTimeLowerLimit);
    }
    
    if (changedAutoExposureTimeUpperLimit)
    {
      newconfig.AutoExposureTimeUpperLimit = setCameraFeature("AutoExposureTimeUpperLimit", newconfig.AutoExposureTimeUpperLimit);
    }
    
    if (changedGain)
    {
      newconfig.Gain = setCameraFeature("Gain", newconfig.Gain);
      ros::Duration(1.0).sleep();
    }
    
    if (changedGainAuto)
    {
      newconfig.GainAuto = setCameraFeature("GainAuto", newconfig.GainAuto);
      ros::Duration(1.0).sleep();
      if (newconfig.GainAuto=="Once")
      {
          newconfig.GainAuto = "Off";
          newconfig.GainAuto = setCameraFeature("GainAuto", newconfig.GainAuto);
          ros::Duration(1.0).sleep();
      }
    }
    
    if (changedAutoGainLowerLimit)
    {
      newconfig.AutoGainLowerLimit = setCameraFeature("AutoGainLowerLimit", newconfig.AutoGainLowerLimit);
    }
    
    if (changedAutoGainUpperLimit)
    {
      newconfig.AutoGainUpperLimit = setCameraFeature("AutoGainUpperLimit", newconfig.AutoGainUpperLimit);
    }

    getCameraFeature("AcquisitionMode", newconfig.AcquisitionMode);
    getCameraFeature("AcquisitionFrameRate", newconfig.AcquisitionFrameRate);
    getCameraFeature("ExposureAuto", newconfig.ExposureAuto);
    getCameraFeature("ExposureTime", newconfig.ExposureTime);
    getCameraFeature("AutoExposureTimeLowerLimit", newconfig.AutoExposureTimeLowerLimit);
    getCameraFeature("AutoExposureTimeUpperLimit", newconfig.AutoExposureTimeUpperLimit);
    getCameraFeature("GainAuto", newconfig.GainAuto);
    getCameraFeature("Gain", newconfig.Gain);
    getCameraFeature("AutoGainLowerLimit", newconfig.AutoGainLowerLimit);
    getCameraFeature("AutoGainUpperLimit", newconfig.AutoGainUpperLimit);
    
    configPointgrey = newconfig;
} // RosReconfigure_callback_pointgrey()


void CameraNode::RosReconfigure_callback_IDS(IDSConfig &newconfig, uint32_t level)
{    
    int             changedAcquisitionMode;
    int             changedAcquisitionFrameRate;
    int             changedExposureTime;
    int             changedExposureAuto;
    int             changedBrightnessAutoExposureTimeLimitMode;
    int             changedBrightnessAutoExposureTimeMin;
    int             changedBrightnessAutoExposureTimeMax;
    int             changedGain;
    int             changedGainAuto;
    int             changedBrightnessAutoGainLimitMode;
    int             changedBrightnessAutoGainMin;
    int             changedBrightnessAutoGainMax;
    int             changedGamma;

    // Find what the user changed.
    changedAcquisitionMode    			= (newconfig.AcquisitionMode != configIDS.AcquisitionMode);
    changedAcquisitionFrameRate         = (newconfig.AcquisitionFrameRate != configIDS.AcquisitionFrameRate);
    
    changedExposureTime = (newconfig.ExposureTime != configIDS.ExposureTime);
    changedExposureAuto 		= (newconfig.ExposureAuto != configIDS.ExposureAuto);
    changedBrightnessAutoExposureTimeLimitMode = (newconfig.BrightnessAutoExposureTimeLimitMode != configIDS.BrightnessAutoExposureTimeLimitMode);
    changedBrightnessAutoExposureTimeMin = (newconfig.BrightnessAutoExposureTimeMin != configIDS.BrightnessAutoExposureTimeMin);
    changedBrightnessAutoExposureTimeMax = (newconfig.BrightnessAutoExposureTimeMax != configIDS.BrightnessAutoExposureTimeMax);
    
    changedGain         		= (newconfig.Gain != configIDS.Gain);
    changedGainAuto     		= (newconfig.GainAuto != configIDS.GainAuto);
    changedBrightnessAutoGainLimitMode = (newconfig.BrightnessAutoGainLimitMode != configIDS.BrightnessAutoGainLimitMode);
    changedBrightnessAutoGainMin = (newconfig.BrightnessAutoGainMin != configIDS.BrightnessAutoGainMin);
    changedBrightnessAutoGainMax = (newconfig.BrightnessAutoGainMax != configIDS.BrightnessAutoGainMax);
    
    changedGamma            = (newconfig.Gamma != configIDS.Gamma);

    // Adjust other controls dependent on what the user changed.
    if (changedExposureTime || changedGain){
      newconfig.ExposureAuto = "Off";
      newconfig.GainAuto = "Off";
    }
    changedExposureAuto 		= (newconfig.ExposureAuto != configIDS.ExposureAuto);
    changedGainAuto     		= (newconfig.GainAuto != configIDS.GainAuto);
    
    // Set params into the camera.
    if (changedAcquisitionMode)
    {
      newconfig.AcquisitionMode = setCameraFeature("AcquisitionMode", newconfig.AcquisitionMode);
      ros::Duration(1.0).sleep();
    }
    
    if (changedAcquisitionFrameRate)
    {
      newconfig.AcquisitionFrameRate = setCameraFeature("AcquisitionFrameRate", newconfig.AcquisitionFrameRate);
    }
    
    if (changedExposureTime)
    {
      newconfig.ExposureTime = setCameraFeature("ExposureTime", newconfig.ExposureTime);
      ros::Duration(1.0).sleep();
    }
    
    if (changedExposureAuto)
    {
      newconfig.ExposureAuto = setCameraFeature("ExposureAuto", newconfig.ExposureAuto);
      ros::Duration(1.0).sleep();
      if (newconfig.ExposureAuto=="Once")
      {
          newconfig.ExposureAuto = "Off";
          newconfig.ExposureAuto = setCameraFeature("ExposureAuto", newconfig.ExposureAuto);
          ros::Duration(1.0).sleep();
      }
    }
    if(changedBrightnessAutoExposureTimeLimitMode){
      newconfig.BrightnessAutoExposureTimeLimitMode = setCameraFeature("BrightnessAutoExposureTimeLimitMode", newconfig.BrightnessAutoExposureTimeLimitMode);
      ros::Duration(1.0).sleep();
    }
    
    if (changedBrightnessAutoExposureTimeMin)
    {
      newconfig.BrightnessAutoExposureTimeMin = setCameraFeature("BrightnessAutoExposureTimeMin", newconfig.BrightnessAutoExposureTimeMin);
    }
    
    if (changedBrightnessAutoExposureTimeMax)
    {
      newconfig.BrightnessAutoExposureTimeMax = setCameraFeature("BrightnessAutoExposureTimeMax", newconfig.BrightnessAutoExposureTimeMax);
    }
    
    if (changedGain)
    {
      newconfig.Gain = setCameraFeature("Gain", newconfig.Gain);
      ros::Duration(1.0).sleep();
    }
    
    if (changedGainAuto)
    {
      newconfig.GainAuto = setCameraFeature("GainAuto", newconfig.GainAuto);
      ros::Duration(1.0).sleep();
      if (newconfig.GainAuto=="Once")
      {
          newconfig.GainAuto = "Off";
          newconfig.GainAuto = setCameraFeature("GainAuto", newconfig.GainAuto);
          ros::Duration(1.0).sleep();
      }
    }
    if(changedBrightnessAutoGainLimitMode){
      newconfig.BrightnessAutoGainLimitMode = setCameraFeature("BrightnessAutoGainLimitMode", newconfig.BrightnessAutoGainLimitMode);
      ros::Duration(1.0).sleep();
    }

    if (changedBrightnessAutoGainMin)
    {
      newconfig.BrightnessAutoGainMin = setCameraFeature("BrightnessAutoGainMin", newconfig.BrightnessAutoGainMin);
    }
    
    if (changedBrightnessAutoGainMax)
    {
      newconfig.BrightnessAutoGainMax = setCameraFeature("BrightnessAutoGainMax", newconfig.BrightnessAutoGainMax);
    }
    
    if(changedGamma)
    {
      newconfig.Gamma = setCameraFeature("Gamma", newconfig.Gamma);
    }

    getCameraFeature("AcquisitionMode", newconfig.AcquisitionMode);
    getCameraFeature("AcquisitionFrameRate", newconfig.AcquisitionFrameRate);
    getCameraFeature("ExposureAuto", newconfig.ExposureAuto);
    getCameraFeature("ExposureTime", newconfig.ExposureTime);
    getCameraFeature("BrightnessAutoExposureTimeLimitMode", newconfig.BrightnessAutoExposureTimeLimitMode);
    getCameraFeature("BrightnessAutoExposureTimeMin", newconfig.BrightnessAutoExposureTimeMin);
    getCameraFeature("BrightnessAutoExposureTimeMax", newconfig.BrightnessAutoExposureTimeMax);
    getCameraFeature("GainAuto", newconfig.GainAuto);
    getCameraFeature("Gain", newconfig.Gain);
    getCameraFeature("BrightnessAutoGainLimitMode", newconfig.BrightnessAutoGainLimitMode);
    getCameraFeature("BrightnessAutoGainMin", newconfig.BrightnessAutoGainMin);
    getCameraFeature("BrightnessAutoGainMax", newconfig.BrightnessAutoGainMax);
    
    configIDS = newconfig;

} // RosReconfigure_callback_IDS()*/

void CameraNode::RosReconfigure_callback_mako(MakoConfig &newconfig, uint32_t level)
{    
    int             changedAcquisitionMode;
    int             changedAcquisitionFrameRateAbs;
    int             changedExposureTimeAbs;
    int             changedExposureAuto;
    int             changedExposureAutoAlg;
    int             changedExposureAutoAdjustTol;
    int             changedExposureAutoOutliers;
    int             changedExposureAutoTarget;
    int             changedExposureAutoMin;
    int             changedExposureAutoMax;

    int             changedGain;
    int             changedGainAuto;
    int             changedGainAutoAdjustTol;
    int             changedGainAutoOutliers;
    int             changedGainAutoTarget;
    int             changedGainAutoMin;
    int             changedGainAutoMax;

    int             changedDSPSubregionLeft;
    int             changedDSPSubregionTop;
    int             changedDSPSubregionRight;
    int             changedDSPSubregionBottom;
    
    // Find what the user changed.
    changedAcquisitionMode    			= (newconfig.AcquisitionMode != configMako.AcquisitionMode);
    changedAcquisitionFrameRateAbs         = (newconfig.AcquisitionFrameRateAbs != configMako.AcquisitionFrameRateAbs);
    
    changedExposureTimeAbs = (newconfig.ExposureTimeAbs != configMako.ExposureTimeAbs);
    changedExposureAuto 		= (newconfig.ExposureAuto != configMako.ExposureAuto);
    changedExposureAutoAlg 		= (newconfig.ExposureAutoAlg != configMako.ExposureAutoAlg);
    changedExposureAutoAdjustTol 		= (newconfig.ExposureAutoAdjustTol != configMako.ExposureAutoAdjustTol);
    changedExposureAutoOutliers 		= (newconfig.ExposureAutoOutliers != configMako.ExposureAutoOutliers);
    changedExposureAutoTarget 		= (newconfig.ExposureAutoTarget != configMako.ExposureAutoTarget);
    changedExposureAutoMin = (newconfig.ExposureAutoMin != configMako.ExposureAutoMin);
    changedExposureAutoMax = (newconfig.ExposureAutoMax != configMako.ExposureAutoMax);
    
    changedGain         		= (newconfig.Gain != configMako.Gain);
    changedGainAuto     		= (newconfig.GainAuto != configMako.GainAuto);
    changedGainAutoAdjustTol = (newconfig.GainAutoAdjustTol != configMako.GainAutoAdjustTol);
    changedGainAutoOutliers = (newconfig.GainAutoOutliers != configMako.GainAutoOutliers);
    changedGainAutoTarget = (newconfig.GainAutoTarget != configMako.GainAutoTarget);
    changedGainAutoMin = (newconfig.GainAutoMin != configMako.GainAutoMin);
    changedGainAutoMax = (newconfig.GainAutoMax != configMako.GainAutoMax);
    
    changedDSPSubregionLeft = (newconfig.DSPSubregionLeft != configMako.DSPSubregionLeft);
    changedDSPSubregionTop = (newconfig.DSPSubregionTop != configMako.DSPSubregionTop);
    changedDSPSubregionRight = (newconfig.DSPSubregionRight != configMako.DSPSubregionRight);
    changedDSPSubregionBottom = (newconfig.DSPSubregionBottom != configMako.DSPSubregionBottom);

    // Adjust other controls dependent on what the user changed.
    if (changedExposureTimeAbs || changedGain){
      newconfig.ExposureAuto = "Off";
      newconfig.GainAuto = "Off";
    }
    changedExposureAuto 		= (newconfig.ExposureAuto != configMako.ExposureAuto);
    changedGainAuto     		= (newconfig.GainAuto != configMako.GainAuto);
    
    // Set params into the camera.
    if (changedAcquisitionMode)
    {
      newconfig.AcquisitionMode = setCameraFeature("AcquisitionMode", newconfig.AcquisitionMode);
      ros::Duration(1.0).sleep();
    }
    
    if (changedAcquisitionFrameRateAbs)
    {
      newconfig.AcquisitionFrameRateAbs = setCameraFeature("AcquisitionFrameRateAbs", newconfig.AcquisitionFrameRateAbs);
    }
    
    if (changedExposureTimeAbs)
    {
      newconfig.ExposureTimeAbs = setCameraFeature("ExposureTimeAbs", newconfig.ExposureTimeAbs);
      ros::Duration(1.0).sleep();
    }
    
    if (changedExposureAuto)
    {
      newconfig.ExposureAuto = setCameraFeature("ExposureAuto", newconfig.ExposureAuto);
      ros::Duration(1.0).sleep();
      if (newconfig.ExposureAuto=="Once")
      {
          newconfig.ExposureAuto = "Off";
          newconfig.ExposureAuto = setCameraFeature("ExposureAuto", newconfig.ExposureAuto);
          ros::Duration(1.0).sleep();
      }
    }
    
    if (changedExposureAutoAlg)
    {
      newconfig.ExposureAutoAlg = setCameraFeature("ExposureAutoAlg", newconfig.ExposureAutoAlg);
    }
    
    if (changedExposureAutoAdjustTol)
    {
      newconfig.ExposureAutoAdjustTol = setCameraFeature("ExposureAutoAdjustTol", newconfig.ExposureAutoAdjustTol);
    }
    
    if (changedExposureAutoOutliers)
    {
      newconfig.ExposureAutoOutliers = setCameraFeature("ExposureAutoOutliers", newconfig.ExposureAutoOutliers);
    }
    
    if (changedExposureAutoTarget)
    {
      newconfig.ExposureAutoTarget = setCameraFeature("ExposureAutoTarget", newconfig.ExposureAutoTarget);
    }
    
    if (changedExposureAutoMin)
    {
      newconfig.ExposureAutoMin = setCameraFeature("ExposureAutoMin", newconfig.ExposureAutoMin);
    }
    
    if (changedExposureAutoMax)
    {
      newconfig.ExposureAutoMax = setCameraFeature("ExposureAutoMax", newconfig.ExposureAutoMax);
    }
    
    if (changedGain)
    {
      newconfig.Gain = setCameraFeature("Gain", newconfig.Gain);
      ros::Duration(1.0).sleep();
    }
    
    if (changedGainAuto)
    {
      newconfig.GainAuto = setCameraFeature("GainAuto", newconfig.GainAuto);
      ros::Duration(1.0).sleep();
      if (newconfig.GainAuto=="Once")
      {
          newconfig.GainAuto = "Off";
          newconfig.GainAuto = setCameraFeature("GainAuto", newconfig.GainAuto);
          ros::Duration(1.0).sleep();
      }
    }
    
    if (changedGainAutoAdjustTol)
    {
      newconfig.GainAutoAdjustTol = setCameraFeature("GainAutoAdjustTol", newconfig.GainAutoAdjustTol);
    }
    
    if (changedGainAutoOutliers)
    {
      newconfig.GainAutoOutliers = setCameraFeature("GainAutoOutliers", newconfig.GainAutoOutliers);
    }
    
    if (changedGainAutoTarget)
    {
      newconfig.GainAutoTarget = setCameraFeature("GainAutoTarget", newconfig.GainAutoTarget);
    }
    
    if (changedGainAutoMin)
    {
      newconfig.GainAutoMin = setCameraFeature("GainAutoMin", newconfig.GainAutoMin);
    }
    
    if (changedGainAutoMax)
    {
      newconfig.GainAutoMax = setCameraFeature("GainAutoMax", newconfig.GainAutoMax);
    }
    
    if (changedDSPSubregionLeft)
    {
      newconfig.DSPSubregionLeft = setCameraFeature("DSPSubregionLeft", newconfig.DSPSubregionLeft);
    }
    
    if (changedDSPSubregionTop)
    {
      newconfig.DSPSubregionTop = setCameraFeature("DSPSubregionTop", newconfig.DSPSubregionTop);
    }
    
    if (changedDSPSubregionRight)
    {
      newconfig.DSPSubregionRight = setCameraFeature("DSPSubregionRight", newconfig.DSPSubregionRight);
    }
    
    if (changedDSPSubregionBottom)
    {
      newconfig.DSPSubregionBottom = setCameraFeature("DSPSubregionBottom", newconfig.DSPSubregionBottom);
    }

    getCameraFeature("AcquisitionMode", newconfig.AcquisitionMode);
    getCameraFeature("AcquisitionFrameRateAbs", newconfig.AcquisitionFrameRateAbs);
    getCameraFeature("ExposureAuto", newconfig.ExposureAuto);
    getCameraFeature("ExposureTimeAbs", newconfig.ExposureTimeAbs);
    getCameraFeature("ExposureAutoAlg", newconfig.ExposureAutoAlg);
    getCameraFeature("ExposureAutoAdjustTol", newconfig.ExposureAutoAdjustTol);
    getCameraFeature("ExposureAutoOutliers", newconfig.ExposureAutoOutliers);
    getCameraFeature("ExposureAutoTarget", newconfig.ExposureAutoTarget);
    getCameraFeature("ExposureAutoMin", newconfig.ExposureAutoMin);
    getCameraFeature("ExposureAutoMax", newconfig.ExposureAutoMax);
    
    getCameraFeature("GainAuto", newconfig.GainAuto);
    getCameraFeature("Gain", newconfig.Gain);
    getCameraFeature("GainAutoAdjustTol", newconfig.GainAutoAdjustTol);
    getCameraFeature("GainAutoOutliers", newconfig.GainAutoOutliers);
    getCameraFeature("GainAutoTarget", newconfig.GainAutoTarget);
    getCameraFeature("GainAutoMin", newconfig.GainAutoMin);
    getCameraFeature("GainAutoMax", newconfig.GainAutoMax);
    
    getCameraFeature("DSPSubregionLeft", newconfig.DSPSubregionLeft);
    getCameraFeature("DSPSubregionTop", newconfig.DSPSubregionTop);
    getCameraFeature("DSPSubregionRight", newconfig.DSPSubregionRight);
    getCameraFeature("DSPSubregionBottom", newconfig.DSPSubregionBottom);

    configMako = newconfig;
} // RosReconfigure_callback_mako()


void CameraNode::RosReconfigure_callback_prosilica(ProsilicaConfig &newconfig, uint32_t level)
{    
    int             changedAcquisitionMode;
    int             changedAcquisitionFrameRateAbs;
    int             changedExposureTimeAbs;
    int             changedExposureAuto;
    int             changedExposureAutoAlg;
    int             changedExposureAutoAdjustTol;
    int             changedExposureAutoOutliers;
    int             changedExposureAutoTarget;
    int             changedExposureAutoMin;
    int             changedExposureAutoMax;

    int             changedGain;
    int             changedGainAuto;
    int             changedGainAutoAdjustTol;
    int             changedGainAutoOutliers;
    int             changedGainAutoTarget;
    int             changedGainAutoMin;
    int             changedGainAutoMax;

    int             changedDSPSubregionLeft;
    int             changedDSPSubregionTop;
    int             changedDSPSubregionRight;
    int             changedDSPSubregionBottom;

    // Find what the user changed.
    changedAcquisitionMode    			= (newconfig.AcquisitionMode != configProsilica.AcquisitionMode);
    changedAcquisitionFrameRateAbs         = (newconfig.AcquisitionFrameRateAbs != configProsilica.AcquisitionFrameRateAbs);
    
    changedExposureTimeAbs = (newconfig.ExposureTimeAbs != configProsilica.ExposureTimeAbs);
    changedExposureAuto 		= (newconfig.ExposureAuto != configProsilica.ExposureAuto);
    changedExposureAutoAlg 		= (newconfig.ExposureAutoAlg != configProsilica.ExposureAutoAlg);
    changedExposureAutoAdjustTol 		= (newconfig.ExposureAutoAdjustTol != configProsilica.ExposureAutoAdjustTol);
    changedExposureAutoOutliers 		= (newconfig.ExposureAutoOutliers != configProsilica.ExposureAutoOutliers);
    changedExposureAutoTarget 		= (newconfig.ExposureAutoTarget != configProsilica.ExposureAutoTarget);
    changedExposureAutoMin = (newconfig.ExposureAutoMin != configProsilica.ExposureAutoMin);
    changedExposureAutoMax = (newconfig.ExposureAutoMax != configProsilica.ExposureAutoMax);
    
    changedGain         		= (newconfig.Gain != configProsilica.Gain);
    changedGainAuto     		= (newconfig.GainAuto != configProsilica.GainAuto);
    changedGainAutoAdjustTol = (newconfig.GainAutoAdjustTol != configProsilica.GainAutoAdjustTol);
    changedGainAutoOutliers = (newconfig.GainAutoOutliers != configProsilica.GainAutoOutliers);
    changedGainAutoTarget = (newconfig.GainAutoTarget != configProsilica.GainAutoTarget);
    changedGainAutoMin = (newconfig.GainAutoMin != configProsilica.GainAutoMin);
    changedGainAutoMax = (newconfig.GainAutoMax != configProsilica.GainAutoMax);
    
    changedDSPSubregionLeft = (newconfig.DSPSubregionLeft != configProsilica.DSPSubregionLeft);
    changedDSPSubregionTop = (newconfig.DSPSubregionTop != configProsilica.DSPSubregionTop);
    changedDSPSubregionRight = (newconfig.DSPSubregionRight != configProsilica.DSPSubregionRight);
    changedDSPSubregionBottom = (newconfig.DSPSubregionBottom != configProsilica.DSPSubregionBottom);
    
    // Adjust other controls dependent on what the user changed.
    if (changedExposureTimeAbs || changedGain){
      newconfig.ExposureAuto = "Off";
      newconfig.GainAuto = "Off";
    }
    changedExposureAuto 		= (newconfig.ExposureAuto != configProsilica.ExposureAuto);
    changedGainAuto     		= (newconfig.GainAuto != configProsilica.GainAuto);
    
    // Set params into the camera.
    if (changedAcquisitionMode)
    {
      newconfig.AcquisitionMode = setCameraFeature("AcquisitionMode", newconfig.AcquisitionMode);
      ros::Duration(1.0).sleep();
    }
    
    if (changedAcquisitionFrameRateAbs)
    {
      newconfig.AcquisitionFrameRateAbs = setCameraFeature("AcquisitionFrameRateAbs", newconfig.AcquisitionFrameRateAbs);
    }
    
    if (changedExposureTimeAbs)
    {
      newconfig.ExposureTimeAbs = setCameraFeature("ExposureTimeAbs", newconfig.ExposureTimeAbs);
      ros::Duration(1.0).sleep();
    }
    
    if (changedExposureAuto)
    {
      newconfig.ExposureAuto = setCameraFeature("ExposureAuto", newconfig.ExposureAuto);
      ros::Duration(1.0).sleep();
      if (newconfig.ExposureAuto=="Once")
      {
          newconfig.ExposureAuto = "Off";
          newconfig.ExposureAuto = setCameraFeature("ExposureAuto", newconfig.ExposureAuto);
          ros::Duration(1.0).sleep();
      }
    }
    
    if (changedExposureAutoAlg)
    {
      newconfig.ExposureAutoAlg = setCameraFeature("ExposureAutoAlg", newconfig.ExposureAutoAlg);
    }
    
    if (changedExposureAutoAdjustTol)
    {
      newconfig.ExposureAutoAdjustTol = setCameraFeature("ExposureAutoAdjustTol", newconfig.ExposureAutoAdjustTol);
    }
    
    if (changedExposureAutoOutliers)
    {
      newconfig.ExposureAutoOutliers = setCameraFeature("ExposureAutoOutliers", newconfig.ExposureAutoOutliers);
    }
    
    if (changedExposureAutoTarget)
    {
      newconfig.ExposureAutoTarget = setCameraFeature("ExposureAutoTarget", newconfig.ExposureAutoTarget);
    }
    
    if (changedExposureAutoMin)
    {
      newconfig.ExposureAutoMin = setCameraFeature("ExposureAutoMin", newconfig.ExposureAutoMin);
    }
    
    if (changedExposureAutoMax)
    {
      newconfig.ExposureAutoMax = setCameraFeature("ExposureAutoMax", newconfig.ExposureAutoMax);
    }
    
    if (changedGain)
    {
      newconfig.Gain = setCameraFeature("Gain", newconfig.Gain);
      ros::Duration(1.0).sleep();
    }
    
    if (changedGainAuto)
    {
      newconfig.GainAuto = setCameraFeature("GainAuto", newconfig.GainAuto);
      ros::Duration(1.0).sleep();
      if (newconfig.GainAuto=="Once")
      {
          newconfig.GainAuto = "Off";
          newconfig.GainAuto = setCameraFeature("GainAuto", newconfig.GainAuto);
          ros::Duration(1.0).sleep();
      }
    }
    
    if (changedGainAutoAdjustTol)
    {
      newconfig.GainAutoAdjustTol = setCameraFeature("GainAutoAdjustTol", newconfig.GainAutoAdjustTol);
    }
    
    if (changedGainAutoOutliers)
    {
      newconfig.GainAutoOutliers = setCameraFeature("GainAutoOutliers", newconfig.GainAutoOutliers);
    }
    
    if (changedGainAutoTarget)
    {
      newconfig.GainAutoTarget = setCameraFeature("GainAutoTarget", newconfig.GainAutoTarget);
    }
    
    if (changedGainAutoMin)
    {
      newconfig.GainAutoMin = setCameraFeature("GainAutoMin", newconfig.GainAutoMin);
    }
    
    if (changedGainAutoMax)
    {
      newconfig.GainAutoMax = setCameraFeature("GainAutoMax", newconfig.GainAutoMax);
    }
    
    if (changedDSPSubregionLeft)
    {
      newconfig.DSPSubregionLeft = setCameraFeature("DSPSubregionLeft", newconfig.DSPSubregionLeft);
    }
    
    if (changedDSPSubregionTop)
    {
      newconfig.DSPSubregionTop = setCameraFeature("DSPSubregionTop", newconfig.DSPSubregionTop);
    }
    
    if (changedDSPSubregionRight)
    {
      newconfig.DSPSubregionRight = setCameraFeature("DSPSubregionRight", newconfig.DSPSubregionRight);
    }
    
    if (changedDSPSubregionBottom)
    {
      newconfig.DSPSubregionBottom = setCameraFeature("DSPSubregionBottom", newconfig.DSPSubregionBottom);
    }
    
    getCameraFeature("AcquisitionMode", newconfig.AcquisitionMode);
    getCameraFeature("AcquisitionFrameRateAbs", newconfig.AcquisitionFrameRateAbs);
    getCameraFeature("ExposureAuto", newconfig.ExposureAuto);
    getCameraFeature("ExposureTimeAbs", newconfig.ExposureTimeAbs);
    getCameraFeature("ExposureAutoAlg", newconfig.ExposureAutoAlg);
    getCameraFeature("ExposureAutoAdjustTol", newconfig.ExposureAutoAdjustTol);
    getCameraFeature("ExposureAutoOutliers", newconfig.ExposureAutoOutliers);
    getCameraFeature("ExposureAutoTarget", newconfig.ExposureAutoTarget);
    getCameraFeature("ExposureAutoMin", newconfig.ExposureAutoMin);
    getCameraFeature("ExposureAutoMax", newconfig.ExposureAutoMax);
    
    getCameraFeature("GainAuto", newconfig.GainAuto);
    getCameraFeature("Gain", newconfig.Gain);
    getCameraFeature("GainAutoAdjustTol", newconfig.GainAutoAdjustTol);
    getCameraFeature("GainAutoOutliers", newconfig.GainAutoOutliers);
    getCameraFeature("GainAutoTarget", newconfig.GainAutoTarget);
    getCameraFeature("GainAutoMin", newconfig.GainAutoMin);
    getCameraFeature("GainAutoMax", newconfig.GainAutoMax);
    
    getCameraFeature("DSPSubregionLeft", newconfig.DSPSubregionLeft);
    getCameraFeature("DSPSubregionTop", newconfig.DSPSubregionTop);
    getCameraFeature("DSPSubregionRight", newconfig.DSPSubregionRight);
    getCameraFeature("DSPSubregionBottom", newconfig.DSPSubregionBottom);
    
    configProsilica = newconfig;
} // RosReconfigure_callback_prosilica()

void CameraNode::stream_priority_callback (void *user_data, ArvStreamCallbackType type, ArvBuffer *buffer)
{
  return;
}

void CameraNode::NewBuffer_callback (ArvStream *pStream, gpointer* data)
{
    CameraNode* This =reinterpret_cast<CameraNode*>(data);

    static uint64_t  cm = 0L;	// Camera time prev
    uint64_t  		 cn = 0L;	// Camera time now

#ifdef TUNING
    static uint64_t  rm = 0L;	// ROS time prev
#endif
    uint64_t  		 rn = 0L;	// ROS time now

    static uint64_t	 tm = 0L;	// Calculated image time prev
    uint64_t		 tn = 0L;	// Calculated image time now

    static int64_t   em = 0L;	// Error prev.
    int64_t  		 en = 0L;	// Error now between calculated image time and ROS time.
    int64_t  		 de = 0L;	// derivative.
    int64_t  		 ie = 0L;	// integral.
    int64_t			 u = 0L;	// Output of controller.

    int64_t			 kp1 = 0L;		// Fractional gains in integer form.
    int64_t			 kp2 = 1024L;
    int64_t			 kd1 = 0L;
    int64_t			 kd2 = 1024L;
    int64_t			 ki1 = -1L;		// A gentle pull toward zero.
    int64_t			 ki2 = 1024L;

    static uint32_t	 iFrame = 0;	// Frame counter.

    ArvBuffer		*pBuffer;

    pBuffer = arv_stream_try_pop_buffer (pStream);
    if (pBuffer != NULL)
    {
        if (arv_buffer_get_status (pBuffer) == ARV_BUFFER_STATUS_SUCCESS)
        {
            sensor_msgs::Image msg;

            size_t buffer_size;
            char *buffer_data = (char *) arv_buffer_get_data (pBuffer, &buffer_size);

            This->applicationData.nBuffers++;
            std::vector<uint8_t> this_data(buffer_size);
            memcpy(&this_data[0], buffer_data, buffer_size);


            // Camera/ROS Timestamp coordination.
            cn				= (uint64_t)arv_buffer_get_timestamp (pBuffer);	// Camera now
            rn	 			= ros::Time::now().toNSec();					// ROS now

            if (iFrame < 10)
            {
                cm = cn;
                tm  = rn;
            }

            // Control the error between the computed image timestamp and the ROS timestamp.
            en = (int64_t)tm + (int64_t)cn - (int64_t)cm - (int64_t)rn; // i.e. tn-rn, but calced from prior values.
            de = en-em;
            ie += en;
            u = kp1*(en/kp2) + ki1*(ie/ki2) + kd1*(de/kd2);  // kp<0, ki<0, kd>0

            // Compute the new timestamp.
            tn = (uint64_t)((int64_t)tm + (int64_t)cn-(int64_t)cm + u);

#ifdef TUNING
            ROS_WARN_NAMED (NAME, "en=%16ld, ie=%16ld, de=%16ld, u=%16ld + %16ld + %16ld = %16ld", en, ie, de, kp1*(en/kp2), ki1*(ie/ki2), kd1*(de/kd2), u);
            ROS_WARN_NAMED (NAME, "cn=%16lu, rn=%16lu, cn-cm=%8ld, rn-rm=%8ld, tn-tm=%8ld, tn-rn=%ld", cn, rn, cn-cm, rn-rm, (int64_t)tn-(int64_t)tm, tn-rn);
            msgInt64.data = tn-rn; //cn-cm+tn-tm; //
            ppubInt64->publish(msgInt64);
            rm = rn;
#endif

            // Save prior values.
            cm = cn;
            tm = tn;
            em = en;

            // Construct the image message.
            msg.header.stamp.fromNSec(tn);
            msg.header.seq = arv_buffer_get_frame_id (pBuffer);
            msg.header.frame_id = This->frame_id;
            msg.width = This->widthRoi;
            msg.height = This->heightRoi;
            msg.encoding = This->pszPixelformat;
            msg.step = msg.width * This->nBytesPixel;
            msg.data = this_data;

            // get current CameraInfo data
            sensor_msgs::CameraInfo camerainfo = This->pCameraInfoManager->getCameraInfo();
            camerainfo.header.stamp = msg.header.stamp;
            camerainfo.header.seq = msg.header.seq;
            camerainfo.header.frame_id = msg.header.frame_id;
            camerainfo.width = This->widthRoi;
            camerainfo.height = This->heightRoi;

            This->publisher.publish(msg, camerainfo);

        }
        else
            ROS_WARN_THROTTLE (5, "Frame error: %s", szBufferStatusFromInt[arv_buffer_get_status (pBuffer)]);

        arv_stream_push_buffer (pStream, pBuffer);
        iFrame++;
    }
} // NewBuffer_callback()


void CameraNode::ControlLost_callback (ArvGvDevice *pGvDevice, gpointer* data)
{
    ROS_ERROR_NAMED (NAME, "Control lost.");
    bCancel = TRUE;
}

gboolean CameraNode::SoftwareTrigger_callback (void *device)
{
    arv_device_execute_command ((ArvDevice *)device, "TriggerSoftware");
    return TRUE;
}


// PeriodicTask_callback()
// Check for termination, and spin for ROS.
gboolean CameraNode::PeriodicTask_callback (void *data)
{
    CameraNode* This =reinterpret_cast<CameraNode*>(data);
    ApplicationData *pData = &This->applicationData;
    guint64 n_completed_buffers;
    guint64 n_failures;
    guint64 n_underruns;
    guint64 n_resent;
    guint64 n_missing;
    
    CamStats cam_stats_msg;
    //ROS_INFO_NAMED (NAME, "Frame rate = %d Hz", pData->nBuffers);
    arv_stream_get_statistics ((ArvStream *)pData->pstream_for_periodic_cb, &n_completed_buffers, &n_failures, &n_underruns);
    arv_gv_stream_get_statistics (pData->pstream_for_periodic_cb, &n_resent, &n_missing);
    
    pData->n_secs += 1;  
    bool annotatingEnabled = ((pData->annotating_period > 0) 
                                && (pData->annotating_period < 1000) 
                                && (pData->specified_frame_rate > 0)
                                && (pData->specified_frame_rate < 50));  
    if(annotatingEnabled && (pData->n_secs >= pData->annotating_period)){
      gint64 expectedNumFrames = gint64(floor(float(pData->specified_frame_rate * pData->annotating_period)*0.99));
      if(gint64(n_completed_buffers - pData->past_completed_buffers) < expectedNumFrames){
        cam_stats_msg.annotate_frames_drop = true;
        ROS_WARN_NAMED(NAME, "Frame rate dropped by more than 1%% in the last %d seconds", pData->annotating_period);
      }
      else{
        cam_stats_msg.annotate_frames_drop = false;
      }
      if((n_failures != pData->past_failures) || (n_underruns != pData->past_underruns) || (n_missing != pData->past_missing)){
        cam_stats_msg.annotate_packets_drop = true;
        ROS_WARN_NAMED(NAME, "Noticed network-related packet drop in the last %d seconds", pData->annotating_period);
      }
      else{
        cam_stats_msg.annotate_packets_drop = false;
      }
      pData->n_secs = 0;
      pData->past_completed_buffers = n_completed_buffers;
      pData->past_failures = n_failures;
      pData->past_underruns = n_underruns;
      pData->past_missing = n_missing;
    }
    else{
      cam_stats_msg.annotate_frames_drop = false;
      cam_stats_msg.annotate_packets_drop = false;
    }
    cam_stats_msg.curr_frames_received = pData->nBuffers;
    cam_stats_msg.total_completed_buffers = n_completed_buffers;
    cam_stats_msg.total_failures = n_failures;
    cam_stats_msg.total_underruns = n_underruns;
    cam_stats_msg.total_resent_packets = n_resent;
    cam_stats_msg.total_missing_packets = n_missing;
    
    cam_stats_msg.header.stamp = ros::Time::now();
    pData->cam_stats_pub.publish(cam_stats_msg);
    
    pData->nBuffers = 0;
    if (bCancel)
    {
        g_main_loop_quit (pData->main_loop);
        return FALSE;
    }

    ros::spinOnce();

    return TRUE;
} // PeriodicTask_callback()


// Get the child and the child's sibling, where <p___> indicates an indirection.
NODEEX CameraNode::GetGcFirstChild(ArvGc *pGenicam, NODEEX nodeex)
{
    const char *szName=0;

    if (nodeex.pNode)
    {
        nodeex.pNode = arv_dom_node_get_first_child(nodeex.pNode);
        if (nodeex.pNode)
        {
            nodeex.szName = arv_dom_node_get_node_name(nodeex.pNode);
            nodeex.pNodeSibling = arv_dom_node_get_next_sibling(nodeex.pNode);

            // Do the indirection.
            if (nodeex.szName[0]=='p' && strcmp("pInvalidator", nodeex.szName))
            {
                szName = arv_dom_node_get_node_value(arv_dom_node_get_first_child(nodeex.pNode));
                nodeex.pNode  = (ArvDomNode *)arv_gc_get_node(pGenicam, szName);
                if (nodeex.pNode)
                {
                    nodeex.szTag = arv_dom_node_get_node_name(nodeex.pNode);
                }
            }
            else
            {
                nodeex.szTag = nodeex.szName;
            }
        }
        else
            nodeex.pNodeSibling = NULL;
    }
    else
    {
        nodeex.szName = NULL;
        nodeex.szTag = NULL;
        nodeex.pNodeSibling = NULL;
    }

    ROS_DEBUG_NAMED(NAME, "GFC name=%s, node=%p, sib=%p", szName, nodeex.pNode, nodeex.pNodeSibling);


    return nodeex;
} // GetGcFirstChild()


// Get the sibling and the sibling's sibling, where <p___> indicates an indirection.
NODEEX CameraNode::GetGcNextSibling(ArvGc *pGenicam, NODEEX nodeex)
{
    const char *szName=0;

    // Go to the sibling.
    nodeex.pNode = nodeex.pNodeSibling;
    if (nodeex.pNode)
    {
        nodeex.szName = arv_dom_node_get_node_name(nodeex.pNode);
        nodeex.pNodeSibling = arv_dom_node_get_next_sibling(nodeex.pNode);

        // Do the indirection.
        if (nodeex.szName[0]=='p' && strcmp("pInvalidator", nodeex.szName))
        {
            szName = arv_dom_node_get_node_value(arv_dom_node_get_first_child(nodeex.pNode));
            nodeex.pNode = (ArvDomNode *)arv_gc_get_node(pGenicam, szName);
            if (nodeex.pNode)
            {
                nodeex.szTag = arv_dom_node_get_node_name(nodeex.pNode);
            }
        }
        else
        {
            nodeex.szTag = nodeex.szName;
        }
    }
    else
    {
        nodeex.szName = NULL;
        nodeex.szTag = NULL;
        nodeex.pNodeSibling = NULL;
    }

    ROS_DEBUG_NAMED(NAME, "GNS name=%s, node=%p, sib=%p", nodeex.szName, nodeex.pNode, nodeex.pNodeSibling);


    return nodeex;
} // GetGcNextSibling()


// Walk the DOM tree, i.e. the tree represented by the XML file in the camera, and that contains all the various features, parameters, etc.
void CameraNode::PrintDOMTree(ArvGc *pGenicam, NODEEX nodeex, int nIndent, bool debug)
{
    char		*szIndent=0;
    const char *szFeature=0;
    const char *szDomName=0;
    const char *szFeatureValue=0;

    szIndent = new char[nIndent+1];
    memset(szIndent,' ',nIndent);
    szIndent[nIndent]=0;

    nodeex = GetGcFirstChild(pGenicam, nodeex);
    if (nodeex.pNode)
    {
        do
        {
            if (ARV_IS_GC_FEATURE_NODE((ArvGcFeatureNode *)nodeex.pNode))
            {
                szDomName = arv_dom_node_get_node_name(nodeex.pNode);
                szFeature = arv_gc_feature_node_get_name((ArvGcFeatureNode *)nodeex.pNode);
                szFeatureValue = arv_gc_feature_node_get_value_as_string((ArvGcFeatureNode *)nodeex.pNode, NULL);
                if (szFeature && szFeatureValue && szFeatureValue[0])
                {
                    if (debug)
                    {
		        ROS_DEBUG_NAMED (NAME, "FeatureName: %s%s, %s=%s", szIndent, szDomName, szFeature, szFeatureValue);
                    }
                    else
                    {
		        ROS_INFO_NAMED (NAME, "FeatureName: %s%s, %s=%s", szIndent, szDomName, szFeature, szFeatureValue);
                    }
                }
            }
            PrintDOMTree(pGenicam, nodeex, nIndent+4, debug);

            // Go to the next sibling.
            nodeex = GetGcNextSibling(pGenicam, nodeex);
        } while (nodeex.pNode && nodeex.pNodeSibling);
    }
} //PrintDOMTree()

const char* CameraNode::GetPixelEncoding(ArvPixelFormat pixel_format)
{
  // TODO: this is a table, it should not be implemented as structured code. Const array/vector lookup?
    static std::string none;
    switch(pixel_format)
    {
    using namespace sensor_msgs::image_encodings;

    // supported grayscale encodings
    case ARV_PIXEL_FORMAT_MONO_8:         return MONO8.c_str();
    case ARV_PIXEL_FORMAT_MONO_8_SIGNED:  return TYPE_8SC1.c_str(); // OpenCV type
    case ARV_PIXEL_FORMAT_MONO_16:        return MONO16.c_str();

    // supported color encodings
    case ARV_PIXEL_FORMAT_RGB_8_PACKED:   return RGB8.c_str();
    case ARV_PIXEL_FORMAT_BGR_8_PACKED:   return BGR8.c_str();
    case ARV_PIXEL_FORMAT_RGBA_8_PACKED:  return RGBA8.c_str();
    case ARV_PIXEL_FORMAT_BGRA_8_PACKED:  return BGRA8.c_str();
    case ARV_PIXEL_FORMAT_YUV_422_PACKED: return YUV422.c_str();

    // supported bayer encodings
    case ARV_PIXEL_FORMAT_BAYER_GR_8:     return BAYER_GRBG8.c_str();
    case ARV_PIXEL_FORMAT_BAYER_RG_8:     return BAYER_RGGB8.c_str();
    case ARV_PIXEL_FORMAT_BAYER_GB_8:     return BAYER_GBRG8.c_str();
    case ARV_PIXEL_FORMAT_BAYER_BG_8:     return BAYER_BGGR8.c_str();
    case ARV_PIXEL_FORMAT_BAYER_GR_16:    return BAYER_GRBG8.c_str();
    case ARV_PIXEL_FORMAT_BAYER_RG_16:    return BAYER_RGGB16.c_str();
    case ARV_PIXEL_FORMAT_BAYER_GB_16:    return BAYER_GBRG16.c_str();
    case ARV_PIXEL_FORMAT_BAYER_BG_16:    return BAYER_BGGR16.c_str();

// unsupported encodings
//  case ARV_PIXEL_FORMAT_BAYER_GR_10:
//  case ARV_PIXEL_FORMAT_BAYER_RG_10:
//  case ARV_PIXEL_FORMAT_BAYER_GB_10:
//  case ARV_PIXEL_FORMAT_BAYER_BG_10:
//  case ARV_PIXEL_FORMAT_BAYER_GR_12:
//  case ARV_PIXEL_FORMAT_BAYER_RG_12:
//  case ARV_PIXEL_FORMAT_BAYER_GB_12:
//  case ARV_PIXEL_FORMAT_BAYER_BG_12:
//  case ARV_PIXEL_FORMAT_BAYER_GR_12_PACKED:
//  case ARV_PIXEL_FORMAT_BAYER_RG_12_PACKED:
//  case ARV_PIXEL_FORMAT_BAYER_GB_12_PACKED:
//  case ARV_PIXEL_FORMAT_BAYER_BG_12_PACKED:
//  case ARV_PIXEL_FORMAT_RGB_10_PACKED:
//  case ARV_PIXEL_FORMAT_BGR_10_PACKED:
//  case ARV_PIXEL_FORMAT_RGB_12_PACKED:
//  case ARV_PIXEL_FORMAT_BGR_12_PACKED:
//  case ARV_PIXEL_FORMAT_YUV_411_PACKED:
//  case ARV_PIXEL_FORMAT_YUV_444_PACKED:
//  case ARV_PIXEL_FORMAT_RGB_8_PLANAR:
//  case ARV_PIXEL_FORMAT_RGB_10_PLANAR:
//  case ARV_PIXEL_FORMAT_RGB_12_PLANAR:
//  case ARV_PIXEL_FORMAT_RGB_16_PLANAR:
//  case ARV_PIXEL_FORMAT_YUV_422_YUYV_PACKED:
//  case ARV_PIXEL_FORMAT_CUSTOM_BAYER_GR_12_PACKED:
//  case ARV_PIXEL_FORMAT_CUSTOM_BAYER_RG_12_PACKED:
//  case ARV_PIXEL_FORMAT_CUSTOM_BAYER_GB_12_PACKED:
//  case ARV_PIXEL_FORMAT_CUSTOM_BAYER_BG_12_PACKED:
//  case ARV_PIXEL_FORMAT_CUSTOM_YUV_422_YUYV_PACKED:
//  case ARV_PIXEL_FORMAT_CUSTOM_BAYER_GR_16:
//  case ARV_PIXEL_FORMAT_CUSTOM_BAYER_RG_16:
//  case ARV_PIXEL_FORMAT_CUSTOM_BAYER_GB_16:
//  case ARV_PIXEL_FORMAT_CUSTOM_BAYER_BG_16:
    }

    return 0;
} // GetPixelEncoding()

/*
void CameraNode::onInit()
{
    NODELET_DEBUG("Starting Camera Nodelet");
    //! We will be retrying to open camera until it is open, which may block the
    //! thread. Nodelet::onInit() should not block, hence spawning a new thread
    //! to do initialization.
    init_thread_ = boost::thread(boost::bind(&CameraNode::onInitImpl, this));
}
*/

void CameraNode::Start()
{

//    ros::NodeHandle& nh = getNodeHandle(); // unused??
    ros::NodeHandle nh;

    char   		*pszGuid = NULL;
    char    	 szGuid[512];
    int			 nInterfaces = 0;
    int			 nDevices = 0;
    int 		 i = 0;
    ArvGcNode	*pGcNode;
    GError		*error=NULL;

    applicationData.nBuffers = 0;
    applicationData.main_loop = 0;
    applicationData.past_completed_buffers = 0;
    applicationData.past_failures = 0;
    applicationData.past_underruns = 0;
    applicationData.past_missing = 0;
    applicationData.n_secs = 0;
    applicationData.annotating_period = 2000;
    applicationData.specified_frame_rate = 100;
    std::string annotation_param = ros::this_node::getName()+"/period_for_stats_annotation";
    if(nh.hasParam(annotation_param)){
      nh.getParam(annotation_param, applicationData.annotating_period);
      if((applicationData.annotating_period > 0) && (applicationData.annotating_period < 1000)){
        ROS_INFO_NAMED (NAME, "Camera stats annotation period set to %d seconds", applicationData.annotating_period);
      }
      else{
        ROS_WARN_NAMED (NAME, "Camera stats annotation disabled");
      }
    }
    else{
      ROS_WARN_NAMED (NAME, "Camera stats annotation disabled");
    }
    
    std::string frame_rate_id = ros::this_node::getName()+"/frame_rate";
    if (nh.hasParam(frame_rate_id))
    {
      nh.getParam(frame_rate_id, applicationData.specified_frame_rate);
      if((applicationData.specified_frame_rate > 0) && (applicationData.specified_frame_rate < 50)){
        ROS_INFO_NAMED (NAME, "Camera stats frame rate set to %d Hz", applicationData.specified_frame_rate);
      }
      else{
        ROS_WARN_NAMED (NAME, "Camera stats annotation disabled. Frame rate out of range");
      }
    }
    else{
      ROS_WARN_NAMED (NAME, "Camera stats annotation disabled. Frame rate is unknown");
    }
    
    bCancel = FALSE;

    idSoftwareTriggerTimer = 0;

    // Print out some useful info.
    ROS_INFO_NAMED (NAME, "Attached cameras:");
    arv_update_device_list();
    nInterfaces = arv_get_n_interfaces();
    ROS_INFO_NAMED (NAME, "# Interfaces: %d", nInterfaces);

    nDevices = arv_get_n_devices();
    ROS_INFO_NAMED (NAME, "# Devices: %d", nDevices);
    for (i=0; i<nDevices; i++)
        ROS_INFO_NAMED (NAME, "Device%d: %s", i, arv_get_device_id(i));

    // TODO: how did this work with multiple devices?
    if (nDevices>0)
    {
        std::string cam_id = ros::this_node::getName()+"/guid";
        if (nh.hasParam(cam_id))
        {
            std::string		stGuid;

            nh.getParam(cam_id, stGuid);
            strcpy (szGuid, stGuid.c_str());
            pszGuid = szGuid;
        }
        else
            pszGuid = NULL;


        // Open the camera, and set it up.
        ROS_INFO_NAMED (NAME, "Opening: %s", pszGuid ? pszGuid : "(any)");
        while (TRUE)
        {
            pCamera = arv_camera_new(pszGuid);
            if (pCamera)
                break;
            else
            {
                ROS_WARN_NAMED (NAME, "Could not open camera %s.  Retrying...", pszGuid);
                ros::Duration(1.0).sleep();
                ros::spinOnce();
            }
        }

        pDevice = arv_camera_get_device(pCamera);
        ROS_INFO_NAMED (NAME, "Opened: %s-%s", arv_device_get_string_feature_value (pDevice, "DeviceVendorName"), arv_device_get_string_feature_value (pDevice, "DeviceID"));

        std::string node_name = ros::this_node::getName();
        std::string fullParamName = node_name+"/frame_id";
        if (nh.hasParam(fullParamName))
        {
          nh.getParam(fullParamName, frame_id);
        }
        xRoi=0; yRoi=0; widthRoi=0; heightRoi=0;
        arv_camera_get_sensor_size			(pCamera, &widthSensor, &heightSensor);
        arv_camera_get_width_bounds			(pCamera, &widthRoiMin, &widthRoiMax);
        arv_camera_get_height_bounds		(pCamera, &heightRoiMin, &heightRoiMax);
        arv_camera_get_region (pCamera, &xRoi, &yRoi, &widthRoi, &heightRoi);
        pszPixelformat   		= GetPixelEncoding(arv_camera_get_pixel_format(pCamera));
        nBytesPixel      		= ARV_PIXEL_FORMAT_BYTE_PER_PIXEL(arv_device_get_integer_feature_value(pDevice, "PixelFormat"));
        
        // Print information.
        ROS_INFO_NAMED (NAME, "    Using Camera Configuration:");
        ROS_INFO_NAMED (NAME, "    ---------------------------");
        ROS_INFO_NAMED (NAME, "    Vendor name          = %s", arv_device_get_string_feature_value (pDevice, "DeviceVendorName"));
        ROS_INFO_NAMED (NAME, "    Model name           = %s", arv_device_get_string_feature_value (pDevice, "DeviceModelName"));
        ROS_INFO_NAMED (NAME, "    Device id            = %s", arv_device_get_string_feature_value (pDevice, "DeviceID"));
        ROS_INFO_NAMED (NAME, "    Sensor width         = %d", widthSensor);
        ROS_INFO_NAMED (NAME, "    Sensor height        = %d", heightSensor);
        ROS_INFO_NAMED (NAME, "    ROI x,y,w,h          = %d, %d, %d, %d", xRoi, yRoi, widthRoi, heightRoi);
        ROS_INFO_NAMED (NAME, "    Pixel format         = %s", pszPixelformat);
        if(!pszPixelformat)
        {
            pszPixelformat = g_string_ascii_down(g_string_new(arv_device_get_string_feature_value(pDevice, "PixelFormat")))->str;
            ROS_WARN_NAMED (NAME, "Pixelformat %s unsupported", pszPixelformat);
        }
        ROS_INFO_NAMED (NAME, "    BytesPerPixel        = %d", nBytesPixel);
        
        if(node_name.find("pointgrey", 0) != std::string::npos)
        {
          ROS_INFO_NAMED (NAME, " Setting Parameters for pointgrey camera");
          ROS_INFO_NAMED (NAME, "    ---------------------------");
          setFeatureFromParam(nh, "AcquisitionMode", "str");
          setFeatureFromParam(nh, "AcquisitionFrameRate", "float");
          setFeatureFromParam(nh, "AcquisitionFrameRateAuto", "str");
          setFeatureFromParam(nh, "ExposureAuto", "str");
          setFeatureFromParam(nh, "ExposureMode", "str");
          setFeatureFromParam(nh, "ExposureTime", "float");
          setFeatureFromParam(nh, "AutoExposureTimeLowerLimit", "float");
          setFeatureFromParam(nh, "AutoExposureTimeUpperLimit", "float");
          setFeatureFromParam(nh, "GainAuto", "str");
          setFeatureFromParam(nh, "Gain", "float");
          setFeatureFromParam(nh, "AutoGainLowerLimit", "float");
          setFeatureFromParam(nh, "AutoGainUpperLimit", "float");
          setFeatureFromParam(nh, "GevSCPSPacketSize", "int");
          setFeatureFromParam(nh, "TriggerMode", "str");
          arv_camera_set_region (pCamera, xRoi, yRoi, widthRoiMax, heightRoiMax);
          
          // Start the dynamic_reconfigure server. Don't set the callback yet so that we can override the default configuration
          //boost::recursive_mutex config_mutex;
          dynamic_reconfigure::Server<PointgreyConfig>          reconfigureServerPointgrey;
          dynamic_reconfigure::Server<PointgreyConfig>::CallbackType      reconfigureCallbackPointgrey;
  	      reconfigureCallbackPointgrey = boost::bind(&CameraNode::RosReconfigure_callback_pointgrey, this,  _1, _2);
          ros::Duration(1.0).sleep();
          
          getCameraFeature("AcquisitionMode", configPointgrey.AcquisitionMode);
          getCameraFeature("AcquisitionFrameRate", configPointgrey.AcquisitionFrameRate);
          getCameraFeature("ExposureAuto", configPointgrey.ExposureAuto);
          getCameraFeature("ExposureTime", configPointgrey.ExposureTime);
          getCameraFeature("AutoExposureTimeLowerLimit", configPointgrey.AutoExposureTimeLowerLimit);
          getCameraFeature("AutoExposureTimeUpperLimit", configPointgrey.AutoExposureTimeUpperLimit);
          getCameraFeature("GainAuto", configPointgrey.GainAuto);
          getCameraFeature("Gain", configPointgrey.Gain);
          getCameraFeature("AutoGainLowerLimit", configPointgrey.AutoGainLowerLimit);
          getCameraFeature("AutoGainUpperLimit", configPointgrey.AutoGainUpperLimit);

          reconfigureServerPointgrey.updateConfig(configPointgrey); // sync up with dynamic reconfig so everyone has the same config
          reconfigureServerPointgrey.setCallback(reconfigureCallbackPointgrey);
        }
        else  if(node_name.find("avt_mako", 0) != std::string::npos)
        {
          ROS_INFO_NAMED (NAME, " Setting Parameters for AVT Mako camera");
          ROS_INFO_NAMED (NAME, "    ---------------------------");
          setFeatureFromParam(nh, "AcquisitionMode", "str");
          setFeatureFromParam(nh, "AcquisitionFrameRateAbs", "float");
          setFeatureFromParam(nh, "ExposureAuto", "str");
          setFeatureFromParam(nh, "ExposureMode", "str");
          setFeatureFromParam(nh, "ExposureTimeAbs", "float");
          setFeatureFromParam(nh, "ExposureAutoAlg", "str");
          setFeatureFromParam(nh, "ExposureAutoAdjustTol", "int");
          setFeatureFromParam(nh, "ExposureAutoOutliers", "int");
          setFeatureFromParam(nh, "ExposureAutoTarget", "int");
          setFeatureFromParam(nh, "ExposureAutoMin", "int");
          setFeatureFromParam(nh, "ExposureAutoMax", "int");
          setFeatureFromParam(nh, "GainAuto", "str");
          setFeatureFromParam(nh, "Gain", "float");
          setFeatureFromParam(nh, "GainAutoAdjustTol", "int");
          setFeatureFromParam(nh, "GainAutoOutliers", "int");
          setFeatureFromParam(nh, "GainAutoTarget", "int");
          setFeatureFromParam(nh, "GainAutoMin", "float");
          setFeatureFromParam(nh, "GainAutoMax", "float");
          
          /*config.dsp_subregion_bottom  = std::min(config.dsp_subregion_bottom,  (int)heightRoiMax);
          config.dsp_subregion_left    = std::max(config.dsp_subregion_left, (int)0);
          config.dsp_subregion_right   = std::min(config.dsp_subregion_right, (int)widthRoiMax);
          config.dsp_subregion_top     = std::max(config.dsp_subregion_top, (int)0);

          // If bottom is smaller than top, swap them
          if(config.dsp_subregion_bottom < config.dsp_subregion_top){
              int temp_buf = config.dsp_subregion_bottom;
              config.dsp_subregion_bottom  = config.dsp_subregion_top;
              config.dsp_subregion_top = temp_buf;
          }
          // If right is smaller than left, swap them
          if(config.dsp_subregion_right < config.dsp_subregion_left){
              int temp_buf = config.dsp_subregion_right;
              config.dsp_subregion_right  = config.dsp_subregion_left;
              config.dsp_subregion_left = temp_buf;
          }*/
          
          setFeatureFromParam(nh, "DSPSubregionLeft", "int");
          setFeatureFromParam(nh, "DSPSubregionTop", "int");
          setFeatureFromParam(nh, "DSPSubregionRight", "int");
          setFeatureFromParam(nh, "DSPSubregionBottom", "int");
          setFeatureFromParam(nh, "GevSCPSPacketSize", "int");
          setFeatureFromParam(nh, "TriggerMode", "str");
          arv_camera_set_region (pCamera, xRoi, yRoi, widthRoiMax, heightRoiMax);
          double valFromCam = arv_device_get_float_feature_value (pDevice, "AcquisitionFrameRateLimit");
          ROS_WARN_NAMED (NAME, "The maximum frame rate possible for the current exposure duration pixel format is %f", valFromCam);
          
          // Start the dynamic_reconfigure server. Don't set the callback yet so that we can override the default configuration
          //boost::recursive_mutex config_mutex;
          dynamic_reconfigure::Server<MakoConfig>          reconfigureServerMako;
          dynamic_reconfigure::Server<MakoConfig>::CallbackType      reconfigureCallbackMako;
  	      reconfigureCallbackMako = boost::bind(&CameraNode::RosReconfigure_callback_mako, this,  _1, _2);
          ros::Duration(1.0).sleep();
          
          getCameraFeature("AcquisitionMode", configMako.AcquisitionMode);
          getCameraFeature("AcquisitionFrameRateAbs", configMako.AcquisitionFrameRateAbs);
          getCameraFeature("ExposureAuto", configMako.ExposureAuto);
          getCameraFeature("ExposureTimeAbs", configMako.ExposureTimeAbs);
          getCameraFeature("ExposureAutoAlg", configMako.ExposureAutoAlg);
          getCameraFeature("ExposureAutoAdjustTol", configMako.ExposureAutoAdjustTol);
          getCameraFeature("ExposureAutoOutliers", configMako.ExposureAutoOutliers);
          getCameraFeature("ExposureAutoTarget", configMako.ExposureAutoTarget);
          getCameraFeature("ExposureAutoMin", configMako.ExposureAutoMin);
          getCameraFeature("ExposureAutoMax", configMako.ExposureAutoMax);
          
          getCameraFeature("GainAuto", configMako.GainAuto);
          getCameraFeature("Gain", configMako.Gain);
          getCameraFeature("GainAutoAdjustTol", configMako.GainAutoAdjustTol);
          getCameraFeature("GainAutoOutliers", configMako.GainAutoOutliers);
          getCameraFeature("GainAutoTarget", configMako.GainAutoTarget);
          getCameraFeature("GainAutoMin", configMako.GainAutoMin);
          getCameraFeature("GainAutoMax", configMako.GainAutoMax);
          
          getCameraFeature("DSPSubregionLeft", configMako.DSPSubregionLeft);
          getCameraFeature("DSPSubregionTop", configMako.DSPSubregionTop);
          getCameraFeature("DSPSubregionRight", configMako.DSPSubregionRight);
          getCameraFeature("DSPSubregionBottom", configMako.DSPSubregionBottom);
          
          reconfigureServerMako.updateConfig(configMako); // sync up with dynamic reconfig so everyone has the same config
          reconfigureServerMako.setCallback(reconfigureCallbackMako);
        }
        else if(node_name.find("avt_prosilica", 0) != std::string::npos)
        {
          ROS_INFO_NAMED (NAME, " Setting Parameters for AVT Prosilica camera");
          ROS_INFO_NAMED (NAME, "    ---------------------------");
          setFeatureFromParam(nh, "AcquisitionMode", "str");
          setFeatureFromParam(nh, "AcquisitionFrameRateAbs", "float");
          setFeatureFromParam(nh, "ExposureAuto", "str");
          setFeatureFromParam(nh, "ExposureMode", "str");
          setFeatureFromParam(nh, "ExposureTimeAbs", "float");
          setFeatureFromParam(nh, "ExposureAutoAlg", "str");
          setFeatureFromParam(nh, "ExposureAutoAdjustTol", "int");
          setFeatureFromParam(nh, "ExposureAutoOutliers", "int");
          setFeatureFromParam(nh, "ExposureAutoTarget", "int");
          setFeatureFromParam(nh, "ExposureAutoMin", "int");
          setFeatureFromParam(nh, "ExposureAutoMax", "int");
          setFeatureFromParam(nh, "GainAuto", "str");
          setFeatureFromParam(nh, "Gain", "float");
          setFeatureFromParam(nh, "GainAutoAdjustTol", "int");
          setFeatureFromParam(nh, "GainAutoOutliers", "int");
          setFeatureFromParam(nh, "GainAutoTarget", "int");
          setFeatureFromParam(nh, "GainAutoMin", "float");
          setFeatureFromParam(nh, "GainAutoMax", "float");
          setFeatureFromParam(nh, "DSPSubregionLeft", "int");
          setFeatureFromParam(nh, "DSPSubregionTop", "int");
          setFeatureFromParam(nh, "DSPSubregionRight", "int");
          setFeatureFromParam(nh, "DSPSubregionBottom", "int");
          setFeatureFromParam(nh, "GevSCPSPacketSize", "int");
          setFeatureFromParam(nh, "TriggerMode", "str");
          arv_camera_set_region (pCamera, xRoi, yRoi, widthRoiMax, heightRoiMax);
          double valFromCam = arv_device_get_float_feature_value (pDevice, "AcquisitionFrameRateLimit");
          ROS_WARN_NAMED (NAME, "The maximum frame rate possible for the current exposure duration pixel format is %f", valFromCam);
          
          // Start the dynamic_reconfigure server. Don't set the callback yet so that we can override the default configuration
          //boost::recursive_mutex config_mutex;
          dynamic_reconfigure::Server<ProsilicaConfig>          reconfigureServerProsilica;
          dynamic_reconfigure::Server<ProsilicaConfig>::CallbackType      reconfigureCallbackProsilica;
  	      reconfigureCallbackProsilica = boost::bind(&CameraNode::RosReconfigure_callback_prosilica, this,  _1, _2);
          ros::Duration(1.0).sleep();
          
          getCameraFeature("AcquisitionMode", configProsilica.AcquisitionMode);
          getCameraFeature("AcquisitionFrameRateAbs", configProsilica.AcquisitionFrameRateAbs);
          getCameraFeature("ExposureAuto", configProsilica.ExposureAuto);
          getCameraFeature("ExposureTimeAbs", configProsilica.ExposureTimeAbs);
          getCameraFeature("ExposureAutoAlg", configProsilica.ExposureAutoAlg);
          getCameraFeature("ExposureAutoAdjustTol", configProsilica.ExposureAutoAdjustTol);
          getCameraFeature("ExposureAutoOutliers", configProsilica.ExposureAutoOutliers);
          getCameraFeature("ExposureAutoTarget", configProsilica.ExposureAutoTarget);
          getCameraFeature("ExposureAutoMin", configProsilica.ExposureAutoMin);
          getCameraFeature("ExposureAutoMax", configProsilica.ExposureAutoMax);
          
          getCameraFeature("GainAuto", configProsilica.GainAuto);
          getCameraFeature("Gain", configProsilica.Gain);
          getCameraFeature("GainAutoAdjustTol", configProsilica.GainAutoAdjustTol);
          getCameraFeature("GainAutoOutliers", configProsilica.GainAutoOutliers);
          getCameraFeature("GainAutoTarget", configProsilica.GainAutoTarget);
          getCameraFeature("GainAutoMin", configProsilica.GainAutoMin);
          getCameraFeature("GainAutoMax", configProsilica.GainAutoMax);
          
          getCameraFeature("DSPSubregionLeft", configProsilica.DSPSubregionLeft);
          getCameraFeature("DSPSubregionTop", configProsilica.DSPSubregionTop);
          getCameraFeature("DSPSubregionRight", configProsilica.DSPSubregionRight);
          getCameraFeature("DSPSubregionBottom", configProsilica.DSPSubregionBottom);
          
          reconfigureServerProsilica.updateConfig(configProsilica); // sync up with dynamic reconfig so everyone has the same config
          reconfigureServerProsilica.setCallback(reconfigureCallbackProsilica);
        }
        else if(node_name.find("ids_gv", 0) != std::string::npos)
        {  
          ROS_INFO_NAMED (NAME, " Setting Parameters for IDS camera");
          ROS_INFO_NAMED (NAME, "    ---------------------------");
          setFeatureFromParam(nh, "AcquisitionMode", "str");
          setFeatureFromParam(nh, "AcquisitionFrameRate", "float");
          setFeatureFromParam(nh, "ExposureAuto", "str");
          setFeatureFromParam(nh, "ExposureMode", "str");
          setFeatureFromParam(nh, "ExposureTime", "float");
          setFeatureFromParam(nh, "BrightnessAutoExposureTimeLimitMode", "str");
          setFeatureFromParam(nh, "BrightnessAutoExposureTimeMin", "float");
          setFeatureFromParam(nh, "BrightnessAutoExposureTimeMax", "float");
          setFeatureFromParam(nh, "GainAuto", "str");
          setFeatureFromParam(nh, "Gain", "float");
          setFeatureFromParam(nh, "BrightnessAutoGainLimitMode", "str");
          setFeatureFromParam(nh, "BrightnessAutoGainMin", "float");
          setFeatureFromParam(nh, "BrightnessAutoGainMax", "float");
          setFeatureFromParam(nh, "Gamma", "float");
          setFeatureFromParam(nh, "GevSCPSPacketSize", "int");
          setFeatureFromParam(nh, "TriggerMode", "str");
          arv_camera_set_region (pCamera, xRoi, yRoi, widthRoiMax, heightRoiMax);
          
          // Start the dynamic_reconfigure server. Don't set the callback yet so that we can override the default configuration
          //boost::recursive_mutex config_mutex;
          dynamic_reconfigure::Server<IDSConfig>          reconfigureServerIDS;
          dynamic_reconfigure::Server<IDSConfig>::CallbackType      reconfigureCallbackIDS;
  	      reconfigureCallbackIDS = boost::bind(&CameraNode::RosReconfigure_callback_IDS, this,  _1, _2);
          ros::Duration(1.0).sleep();
          
          getCameraFeature("AcquisitionMode", configIDS.AcquisitionMode);
          getCameraFeature("AcquisitionFrameRate", configIDS.AcquisitionFrameRate);
          getCameraFeature("ExposureAuto", configIDS.ExposureAuto);
          getCameraFeature("ExposureTime", configIDS.ExposureTime);
          getCameraFeature("BrightnessAutoExposureTimeLimitMode", configIDS.BrightnessAutoExposureTimeLimitMode);
          getCameraFeature("BrightnessAutoExposureTimeMin", configIDS.BrightnessAutoExposureTimeMin);
          getCameraFeature("BrightnessAutoExposureTimeMax", configIDS.BrightnessAutoExposureTimeMax);
          getCameraFeature("GainAuto", configIDS.GainAuto);
          getCameraFeature("Gain", configIDS.Gain);
          getCameraFeature("BrightnessAutoGainLimitMode", configIDS.BrightnessAutoGainLimitMode);
          getCameraFeature("BrightnessAutoGainMin", configIDS.BrightnessAutoGainMin);
          getCameraFeature("BrightnessAutoGainMax", configIDS.BrightnessAutoGainMax);
          getCameraFeature("Gamma", configIDS.Gamma);
          
          reconfigureServerIDS.updateConfig(configIDS); // sync up with dynamic reconfig so everyone has the same config
          reconfigureServerIDS.setCallback(reconfigureCallbackIDS);
        }
        else{
          ROS_INFO_NAMED (NAME, "Default camera parameters set");
        }
/*
        	isImplementedBinning = arv_camera_is_binning_available(pCamera);
        // Get parameter bounds.
        arv_camera_get_exposure_time_bounds	(pCamera, &configMin.ExposureTimeAbs, &configMax.ExposureTimeAbs);
        arv_camera_get_gain_bounds			(pCamera, &configMin.Gain, &configMax.Gain);
*/
	// Print the tree of camera features, with their values.
	ROS_DEBUG_NAMED (NAME, "    ----------------------------------------------------------------------------------");
	NODEEX		 nodeex;
	ArvGc	*pGenicam=0;
	pGenicam = arv_device_get_genicam(pDevice);

	nodeex.szName = "Root";
	nodeex.pNode = (ArvDomNode	*)arv_gc_get_node(pGenicam, nodeex.szName);
	nodeex.pNodeSibling = NULL;
	const bool USE_ROS_DEBUG = true;
	PrintDOMTree(pGenicam, nodeex, 0, USE_ROS_DEBUG); // use ROS_DEBUG
	ROS_DEBUG_NAMED (NAME, "    ----------------------------------------------------------------------------------");

        // Start the camerainfo manager.
        pCameraInfoManager = new camera_info_manager::CameraInfoManager(nh, arv_device_get_string_feature_value (pDevice, "DeviceID"));

        ArvGvStream *pStream = NULL;
        while (TRUE)
        {
            pStream = CreateStream(nh);
            if (pStream)
                break;
            else
            {
                ROS_WARN_NAMED (NAME, "Could not create image stream for %s.  Retrying...", pszGuid);
                ros::Duration(1.0).sleep();
                ros::spinOnce();
            }
        }
        
        guint64 n_completed_buffers;
        guint64 n_failures;
        guint64 n_underruns;
        guint64 n_resent;
        guint64 n_missing;
        arv_stream_get_statistics ((ArvStream *)pStream, &n_completed_buffers, &n_failures, &n_underruns);
        arv_gv_stream_get_statistics (pStream, &n_resent, &n_missing);
        applicationData.past_completed_buffers = n_completed_buffers;
        applicationData.past_failures = n_failures;
        applicationData.past_underruns = n_underruns;
        applicationData.past_missing = n_missing;
        
        // Set up image_raw.
        image_transport::ImageTransport		*pTransport = new image_transport::ImageTransport(nh);
        publisher = pTransport->advertiseCamera("image_raw", 1);
        
        applicationData.cam_stats_pub = nh.advertise<CamStats>("statistics", 10);
        applicationData.pstream_for_periodic_cb = pStream;
        // Connect signals with callbacks.
        g_signal_connect (pStream, "new-buffer",   G_CALLBACK (NewBuffer_callback),   this);
        g_signal_connect (pDevice, "control-lost", G_CALLBACK (ControlLost_callback), this);
        g_timeout_add_seconds (1, PeriodicTask_callback, (void *)this);
        arv_stream_set_emit_signals ((ArvStream *)pStream, TRUE);


        void (*pSigintHandlerOld)(int);
        pSigintHandlerOld = signal (SIGINT, set_cancel);

        arv_device_execute_command (pDevice, "AcquisitionStart");

        applicationData.main_loop = g_main_loop_new (NULL, FALSE);
        g_main_loop_run (applicationData.main_loop);
        
        if (idSoftwareTriggerTimer)
        {
            g_source_remove(idSoftwareTriggerTimer);
            idSoftwareTriggerTimer = 0;
        }
        
        signal (SIGINT, pSigintHandlerOld);

        g_main_loop_unref (applicationData.main_loop);

        arv_stream_get_statistics ((ArvStream *)pStream, &n_completed_buffers, &n_failures, &n_underruns);
        ROS_INFO_NAMED (NAME, "Completed buffers = %Lu", (unsigned long long) n_completed_buffers);
        ROS_INFO_NAMED (NAME, "Failures          = %Lu", (unsigned long long) n_failures);
        ROS_INFO_NAMED (NAME, "Underruns         = %Lu", (unsigned long long) n_underruns);
        arv_gv_stream_get_statistics (pStream, &n_resent, &n_missing);
        ROS_INFO_NAMED (NAME, "Resent buffers    = %Lu", (unsigned long long) n_resent);
        ROS_INFO_NAMED (NAME, "Missing           = %Lu", (unsigned long long) n_missing);

        arv_device_execute_command (pDevice, "AcquisitionStop");

        arv_stream_set_emit_signals ((ArvStream *)pStream, FALSE);
        g_object_unref (pStream);

    }
    else
        ROS_ERROR_NAMED (NAME, "No cameras detected.");

    ros::shutdown();

    return;
}

}
