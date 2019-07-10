#ifdef SendMessage
#undef SendMessage
#endif


#include <RobotRaconteur.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>


#include "robotraconteur_generated.h"

#include <Windows.h>
#include <Kinect.h>
#include <iostream>
#include <boost/enable_shared_from_this.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <pcl/io/boost.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#pragma once

class Kinect2_impl : public sensors::kinect2::Kinect, public boost::enable_shared_from_this < Kinect2_impl >
{
public:

	Kinect2_impl();
	~Kinect2_impl();

	HRESULT StartupKinect();
	HRESULT ShutdownKinect();
	

	virtual uint8_t EnableSensors(sensors::kinect2::KinectMultiSourcePtr s);
	virtual uint8_t DisableSensors();
	virtual sensors::kinect2::KinectMultiSourcePtr SensorsEnabled();

	virtual sensors::kinect2::ImageHeaderPtr getColorImageHeader();
	virtual sensors::kinect2::ImageHeaderPtr getDepthImageHeader();

	virtual sensors::kinect2::ImagePtr getCurrentColorImage();
	virtual sensors::kinect2::DepthImagePtr getCurrentDepthImage();
	virtual sensors::kinect2::DepthImagePtr getCurrentInfraredImage();
	virtual sensors::kinect2::ImagePtr getCurrentBodyIndexImage();
	virtual sensors::kinect2::DepthImagePtr getCurrentLongExposureInfraredImage();
	virtual RR_INTRUSIVE_PTR<sensors::kinect2::PointCloud> getPointCloud();
	virtual RR_INTRUSIVE_PTR<RobotRaconteur::RRArray<uint64_t > > getTrackedBodyIDs();
	virtual RR_INTRUSIVE_PTR<sensors::kinect2::KinectBody > getDetectedBody(int32_t index);


private:
	uint64_t tracked_body_ids[6];
	IBody *kinect_bodies[BODY_COUNT];
	RGBQUAD *color_image_data;
	uint8_t *bodyindex_image_data;
	uint16_t *depth_image_data;
	uint16_t *infrared_image_data;
	uint16_t *longexposure_infrared_image_data;
	DWORD enabledSources;

	int color_image_width, color_image_height;
	int depth_image_width, depth_image_height;

	IKinectSensor *kinect;
	IMultiSourceFrameReader *multi_reader;
	ICoordinateMapper *coordinate_mapper;

	WAITABLE_HANDLE h_event;
	boost::mutex mtx_;
	boost::thread t1;


	void MultiSourceFrameArrived(IMultiSourceFrameArrivedEventArgs* pArgs);

	void backgroundPollingThread();

	template<class Interface> inline void SafeRelease(Interface *& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL)
		{
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}
};