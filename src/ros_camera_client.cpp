//*****************************************************************************
//
// File Name	: 'ros_camera_client.cpp'
// Author	: Steve NGUYEN
// Contact      : steve.nguyen.000@gmail.com
// Created	: mercredi, mai  1 2019
// Revised	:
// Version	:
// Target MCU	:
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//
// Notes:	Based on the pretty messy info from Seedstudio and https://github.com/ropod-project/hps_camera
//
//*****************************************************************************



#include "../include/api.h"      //api interface
#include "ros/ros.h"             //ros
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>


#include <iostream>
#include <signal.h>
#include <sstream>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

HPS3D_HandleTypeDef handle;
AsyncIObserver_t My_Observer;
ObstacleConfigTypedef ObstacleConf;
sensor_msgs::PointCloud2 outputPointCloud;
uint64_t seq = 0;

ros::Publisher camera_pub; // Global variable, because the observer callback
                           // function needs to be used



// void convertPointCloud(HPS3D_HandleTypeDef *handle, sensor_msgs::PointCloud2 *outputPointCloud)
void convertPointCloud(AsyncIObserver_t *handle, sensor_msgs::PointCloud2 *outputPointCloud)
{
  outputPointCloud->header.stamp = ros::Time::now();

  int numberOfPoints = RES_HEIGHT * RES_WIDTH;
  int numberOfFields = outputPointCloud->fields.size();

  for (int i = 0; i < RES_HEIGHT; i++)
  {
    for (int j = 0; j < RES_WIDTH; j++)
    {

      float32_t xCoordinate = (float32_t)(-handle->MeasureData.point_cloud_data[0].point_data[i * RES_WIDTH + j].x) / 1000.0;
      uint8_t* xCoordinateBytes = (uint8_t *)&xCoordinate;
      float32_t yCoordinate = (float32_t)(handle->MeasureData.point_cloud_data[0].point_data[i * RES_WIDTH + j].y) / 1000.0;
      uint8_t* yCoordinateBytes = (uint8_t *)&yCoordinate;
      float32_t zCoordinate = (float32_t)(handle->MeasureData.point_cloud_data[0].point_data[i * RES_WIDTH + j].z) / 1000.0;
      // uint8_t* zCoordinateBytes = reinterpret_cast<uint8_t*>(&zCoordinate);
      uint8_t* zCoordinateBytes = (uint8_t *)&zCoordinate;

      // printf("(%f %f %f | %d %d %d) ",xCoordinate,yCoordinate,zCoordinate,xCoordinateBytes,yCoordinateBytes,zCoordinateBytes);
      // printf("(%f %f %f) ",xCoordinate,yCoordinate,zCoordinate);

      int arrayPosition = (i * RES_WIDTH + j) * (numberOfFields * 4);
      outputPointCloud->data[arrayPosition] = xCoordinateBytes[0];
      outputPointCloud->data[arrayPosition + 1] = xCoordinateBytes[1];
      outputPointCloud->data[arrayPosition + 2] = xCoordinateBytes[2];
      outputPointCloud->data[arrayPosition + 3] = xCoordinateBytes[3];

      outputPointCloud->data[arrayPosition + 4] = yCoordinateBytes[0];
      outputPointCloud->data[arrayPosition + 5] = yCoordinateBytes[1];
      outputPointCloud->data[arrayPosition + 6] = yCoordinateBytes[2];
      outputPointCloud->data[arrayPosition + 7] = yCoordinateBytes[3];

      outputPointCloud->data[arrayPosition + 8] = zCoordinateBytes[0];
      outputPointCloud->data[arrayPosition + 9] = zCoordinateBytes[1];
      outputPointCloud->data[arrayPosition + 10] = zCoordinateBytes[2];
      outputPointCloud->data[arrayPosition + 11] = zCoordinateBytes[3];
    }
  }
  // printf("\n");
}


void init_ptcloud()
{

  outputPointCloud.header.seq = seq;
  outputPointCloud.header.frame_id = "hps_camera";
  outputPointCloud.height = RES_HEIGHT;
  outputPointCloud.width = RES_WIDTH;
  outputPointCloud.point_step = 12;
  outputPointCloud.row_step = RES_WIDTH * outputPointCloud.point_step;
  outputPointCloud.is_dense = true;

  sensor_msgs::PointField pointFieldX;
  pointFieldX.name = "x";
  pointFieldX.offset = 0;
  pointFieldX.datatype = sensor_msgs::PointField::FLOAT32;
  pointFieldX.count = 1;

  sensor_msgs::PointField pointFieldY;
  pointFieldY.name = "y";
  pointFieldY.offset = 4;
  pointFieldY.datatype = sensor_msgs::PointField::FLOAT32;
  pointFieldY.count = 1;

  sensor_msgs::PointField pointFieldZ;
  pointFieldZ.name = "z";
  pointFieldZ.offset = 8;
  pointFieldZ.datatype = sensor_msgs::PointField::FLOAT32;
  pointFieldZ.count = 1;

  outputPointCloud.fields.push_back(pointFieldX);
  outputPointCloud.fields.push_back(pointFieldY);
  outputPointCloud.fields.push_back(pointFieldZ);

  outputPointCloud.data.resize(RES_HEIGHT * RES_WIDTH * outputPointCloud.fields.size() * 4);


}


// The observer callback function
void *User_Func(HPS3D_HandleTypeDef *handle, AsyncIObserver_t *event) {

  // printf("event\n");
  if (event->AsyncEvent == ISubject_Event_DataRecvd) {
    // printf("TYPE: %d\n",event->RetPacketType);
    switch (event->RetPacketType) {
      case FULL_DEPTH_PACKET:

        seq += 1;

        outputPointCloud.header.seq = seq;
        convertPointCloud(event, &outputPointCloud);
        camera_pub.publish(outputPointCloud);
        printf("PTCLOUD %ld\n",seq);
/*



  printf("FULL_DEPTH distance = %d  event->RetPacketType = %d\n",
  event->MeasureData.full_depth_data->distance_average,
  event->RetPacketType);
  printf("height = %d\n",event->MeasureData.point_cloud_data[0].height);
  printf("width = %d\n",event->MeasureData.point_cloud_data[0].width);
  printf("points = %d\n",event->MeasureData.point_cloud_data[0].points);
  for(int i = 0; i < 60; i++)
  {
  for(int j = 0; j < 160; j++)
  {
  printf("(%d, %d, %d),",(int16_t)event->MeasureData.point_cloud_data[0].point_data[j + i * 160].x, (int16_t)event->MeasureData.point_cloud_data[0].point_data[j + i * 160].y, (uint16_t)event->MeasureData.point_cloud_data[0].point_data[j + i * 160].z);
  }
  printf("\n");
  }
*/
        break;


      case SIMPLE_ROI_PACKET:
        break;
      case FULL_ROI_PACKET:
        break;

      case SIMPLE_DEPTH_PACKET:
        break;
      case NULL_PACKET:
        // The return packet type is empty
        printf("NULL??\n");
        break;
      default:
        printf("system error!\n");
        break;
    }
  }
  return 0;
}

// check ctrl+c signal
void signal_handler(int signo) {
  handle.RunMode = RUN_IDLE;
  HPS3D_SetRunMode(&handle);

  HPS3D_RemoveObserver(&My_Observer);
  if (HPS3D_RemoveDevice(&handle) != RET_OK) {
    printf("HPS3D_RemoveDevice faild\n");
  } else {
    printf("HPS3D_RemoveDevice succeed\n");
  }


  exit(0);
}

// printf log callback function
void my_printf(uint8_t *str) { std::cout << str; }

int main(int argc, char **argv) {

  ros::init(argc, argv, "ros_camera_client"); // ros init
  ros::NodeHandle n;                          // Create a node
  camera_pub = n.advertise<sensor_msgs::PointCloud2>("hps_camera/depth/points", 1);
  // printf("H: %d W: %d\n", RES_HEIGHT, RES_WIDTH);
  uint32_t a = 0;
  uint8_t fileName[10][20];
  uint32_t dev_cnt = 0;
  RET_StatusTypeDef ret = RET_OK;

  OpticalParamConfTypeDef opticalParamConf;

  std::stringstream sclient_name;

  init_ptcloud();

  // Install the signal
  if (signal(SIGINT, signal_handler) == SIG_ERR) {
    printf("sigint error");
  }
  if (signal(SIGTSTP, signal_handler) == SIG_ERR) {
    printf("sigint error");
  }


  // set debug enable and install printf log callback function
  HPS3D_SetDebugEnable(true);
  HPS3D_SetDebugFunc(&my_printf);

  handle.RetPacketType = FULL_DEPTH_PACKET;
  HPS3D_SetPacketType(&handle);

  // Observer callback function and initialization
  My_Observer.AsyncEvent = ISubject_Event_DataRecvd;
  My_Observer.NotifyEnable = true;
  // My_Observer.ObserverID = 2;
  My_Observer.ObserverID = 0;//???
  // My_Observer.RetPacketType = NULL_PACKET;

  // Lists the optional devices
  dev_cnt =
    HPS3D_GetDeviceList((uint8_t *)"/dev/", (uint8_t *)"ttyACM", fileName);
  printf("Current connectable device：\n");
  for (uint32_t i = 0; i < dev_cnt; i++) {
    printf("%d: %s\n", i, fileName[i]);
  }

  /*
    printf("Please enter the corresponding serial number：\n");
    scanf("%d", &a);
    handle.DeviceName = fileName[a];
  */

  //Connect to the first

  handle.DeviceName = fileName[0];
  do {
    // Device Connection
    ret = HPS3D_Connect(&handle);
    if (ret != RET_OK) {
      printf("Device open failed！ret = %d\n", ret);
      break;
    }



    //activate ptcloud before setting running mode
    ret=HPS3D_SetOpticalEnable(&handle, true); //???
    if (ret != RET_OK) {
      printf("Optical enable failed！ret = %d\n", ret);
      break;
    }

    /*
      //ERROR??
      HPS3D_GetOpticalParamConf(&handle, &opticalParamConf);
      if (ret != RET_OK) {
      printf("Read Optical conf failed！ret = %d\n", ret);
      break;
      }
      else
      printf("Optical param configuration.\n Enabled: %d\n Horizontal viewing angle: %d\n Vertical viewing angle: %d\n Horizontal illumination angle: %d\n Vertical illumination angle: %d\n\n", opticalParamConf.enable, opticalParamConf.viewing_angle_horiz, opticalParamConf.viewing_angle_vertical, opticalParamConf.illum_angle_horiz, opticalParamConf.illum_angle_vertical);
    */


    HPS3D_SetPointCloudEn(true);
/*
//??
    ObstacleConf.enable = false;
    ObstacleConf.frame_head = 0xEB81;
    ObstacleConf.invaild_value = 15000;
    ObstacleConf.number = 3;
    ObstacleConf.vaild_max_dist = 5000;
    ObstacleConf.vaild_min_dist = 200;
    HPS3D_ObstacleConfigInit(&ObstacleConf);
*/

    // Device init
    ret = HPS3D_ConfigInit(&handle);
    if (RET_OK != ret) {
      printf("Initialization failed:%d\n", ret);
      break;
    }
    printf("Initialization succeed\n");

    /*
    // The actual invocation of the service
    if (client.call(srv)) {
    while (ros::ok()) {
    printf("rev cmd = %s\n", srv.response.control_cmd.c_str());
    if (strcmp(srv.response.control_cmd.c_str(), "start") == 0) {
    break;
    }
    }
    } else {
    break;
    }
    printf("login succeed!\n");
    */

    // Add observer one
    HPS3D_AddObserver(&User_Func, &handle, &My_Observer);

    // //Set running mode
    handle.RunMode = RUN_CONTINUOUS;
    HPS3D_SetRunMode(&handle);


    printf("Set ptcloud ok\n");
    ros::Duration(1.0).sleep();
  } while (0);

  if (ret != RET_OK) {
    // Remove device and disconnect
    handle.RunMode = RUN_IDLE;
    HPS3D_SetRunMode(&handle);

    HPS3D_RemoveObserver(&My_Observer);
    HPS3D_RemoveDevice(&handle);
    printf("Initialization failed, Remove device\n");
    return 0;
  }
  ros::Rate r(50);
  while (ros::ok()) {
/*
  printf("Start single measurement.\n");
  try{
  HPS3D_SingleMeasurement(&handle);
  }
  catch(...)
  {
  printf("??\n");
  continue;
  }
  printf("End single measurement.\n");
  switch (handle.RetPacketType) {
  case SIMPLE_ROI_PACKET:
  msg.distance_average =
  handle.MeasureData.simple_roi_data[0].distance_average;
  printf("SIMPLE ROI distance = %d\n", msg.distance_average);
  // Publish MSG messages
  camera_pub.publish(msg);
  break;
  case FULL_ROI_PACKET:
  // Assign the average distance to the MSG message
  msg.distance_average =
  handle.MeasureData.full_roi_data[0].distance_average;
  printf("FULL ROI distance = %d\n", msg.distance_average);
  // Publish MSG messages
  camera_pub.publish(msg);
  break;
  case FULL_DEPTH_PACKET:
  msg.distance_average =
  handle.MeasureData.full_depth_data->distance_average;
  printf("FULL DEPTH distance = %d\n", msg.distance_average);
  // Publish MSG messages
  camera_pub.publish(msg);
  break;
  case SIMPLE_DEPTH_PACKET:
  msg.distance_average =
  handle.MeasureData.simple_depth_data->distance_average;
  printf("SIMPLE DEPTH distance = %d\n", msg.distance_average);
  // Publish MSG messages
  camera_pub.publish(msg);
  break;
  case NULL_PACKET:
  // The return packet type is empty
  break;
  default:
  printf("system error!\n");
  break;
  }


*/
    // Waiting to receive
    ros::spinOnce();
    r.sleep();
  }

  handle.RunMode = RUN_IDLE;
  HPS3D_SetRunMode(&handle);

  HPS3D_RemoveObserver(&My_Observer);
  HPS3D_RemoveDevice(&handle);
  return 0;
}
