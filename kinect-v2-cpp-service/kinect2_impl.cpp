
#include "kinect2_impl.h"

namespace RR = RobotRaconteur;

Kinect2_impl::Kinect2_impl() : sensors::kinect2::Kinect()
{ 

	this->multi_reader = NULL;
	this->enabledSources = FrameSourceTypes_None;
	HRESULT hr = StartupKinect();
	if (FAILED(hr))
	{
		std::cout << "Failed to Startup Kinect: error code " << HRESULT_CODE(hr) << std::endl;
		return;
	}
	// Allocate memory for the different image streams (consider moving this to the enable_streams section?)
	this->bodyindex_image_data = new uint8_t[this->depth_image_width * this->depth_image_height];
	this->depth_image_data = new uint16_t[this->depth_image_width * this->depth_image_height];
	this->infrared_image_data = new uint16_t[this->depth_image_width * this->depth_image_height];
	this->longexposure_infrared_image_data = new uint16_t[this->depth_image_width * this->depth_image_height];
	this->color_image_data = new RGBQUAD[this->color_image_width * this->color_image_height];
	
	for (int i = 0; i < BODY_COUNT; i++)
		this->kinect_bodies[i] = { 0 };

	t1 = boost::thread(boost::bind(&Kinect2_impl::backgroundPollingThread, this));
	
}

Kinect2_impl::~Kinect2_impl()
{
	ShutdownKinect();
	delete bodyindex_image_data;
	delete depth_image_data;
	delete infrared_image_data;
	delete longexposure_infrared_image_data;
	delete color_image_data;
}

HRESULT Kinect2_impl::StartupKinect()
{
	HRESULT hr = NULL;

	// Attempt access to default Kinect-2 sensor
	std::cout << "Looking for Default Kinect Sensor" << std::endl;
	hr = GetDefaultKinectSensor(&this->kinect);
	if (FAILED(hr))
		return hr;

	// If Kinect is found, initialize
	if (this->kinect)
	{
		std::cout << "Found Kinect, Initializing..." << std::endl;

		hr = this->kinect->Open();
		if FAILED(hr) { return hr; }
		
		// Get Color Frame Information
		std::cout << "Getting Color Frame Info...";
		IFrameDescription *color_frame_description = NULL;
		IColorFrameSource *color_frame_source = NULL;
		hr = this->kinect->get_ColorFrameSource(&color_frame_source);
		if FAILED(hr) { return hr; }
		color_frame_source->get_FrameDescription(&color_frame_description);
		if FAILED(hr) { return hr; }
		color_frame_description->get_Width(&this->color_image_width);
		color_frame_description->get_Height(&this->color_image_height);
		SafeRelease(color_frame_source);
		SafeRelease(color_frame_description);
		std::cout << "Success" << std::endl;

		// Get Depth Frame Information
		std::cout << "Getting Depth Frame Info...";
		IFrameDescription *depth_frame_description = NULL;
		IDepthFrameSource *depth_frame_source = NULL;
		hr = this->kinect->get_DepthFrameSource(&depth_frame_source);
		if FAILED(hr) { return hr; }
		hr = depth_frame_source->get_FrameDescription(&depth_frame_description);
		if FAILED(hr) { return hr; }
		depth_frame_description->get_Width(&this->depth_image_width);
		depth_frame_description->get_Height(&this->depth_image_height);
		SafeRelease(depth_frame_source);
		SafeRelease(depth_frame_description);
		std::cout << "Success" << std::endl;


		// added by me Read Joint Data
		//std::cout << "Getting Body Frame Info...";
		//IFrameDescription *body_frame_description = NULL;
		//IBodyFrameReader *bodyFrameReader = NULL;
		//IBodyFrameSource *bodyFrameSource = NULL;
		//hr = this->kinect->get_BodyFrameSource(&bodyFrameSource);
		//if FAILED(hr) { return hr; }
		//hr = bodyFrameSource->OpenReader(&bodyFrameReader);
		//if FAILED(hr) { return hr; }
		//body_frame_description->get_Width(&this->depth_image_width);
		//body_frame_description->get_Height(&this->depth_image_height);
		//SafeRelease(bodyFrameSource);
		//SafeRelease(body_frame_description);
		//std::cout << "Success" << std::endl;



		ICoordinateMapper *coordinate_mapper;
		hr = this->kinect->get_CoordinateMapper(&coordinate_mapper);
		
		CameraIntrinsics depth_camera_intrinsics;
		hr = coordinate_mapper->GetDepthCameraIntrinsics(&depth_camera_intrinsics);
		while (depth_camera_intrinsics.FocalLengthX == 0)
		{
			Sleep(1000);
			hr = coordinate_mapper->GetDepthCameraIntrinsics(&depth_camera_intrinsics);
		}
		
		std::cout << "fx: " << depth_camera_intrinsics.FocalLengthX << " fy: " << depth_camera_intrinsics.FocalLengthY << std::endl;
		std::cout << "px: " << depth_camera_intrinsics.PrincipalPointX << " py: " << depth_camera_intrinsics.PrincipalPointY << std::endl;
		std::cout << "radial dist 2: " << depth_camera_intrinsics.RadialDistortionSecondOrder << "  4: " << depth_camera_intrinsics.RadialDistortionFourthOrder << "  6: " << depth_camera_intrinsics.RadialDistortionSixthOrder << std::endl;

		//SafeRelease(coordinate_mapper);
		std::cout << "Success" << std::endl;
		
	}

	return hr;
}

HRESULT Kinect2_impl::ShutdownKinect()
{
	HRESULT hr = E_FAIL;

	this->DisableSensors();

	t1.interrupt();
	t1.join();

	if (kinect)
		hr = kinect->Close();
	SafeRelease(kinect);
	return hr;
}

uint8_t Kinect2_impl::EnableSensors(sensors::kinect2::KinectMultiSourcePtr s)
{
	if (this->enabledSources > 0)
	{
		std::cout << "Already Streaming.  Disable and then Re-Enable" << std::endl;
		return 0;	
	}
	this->enabledSources |= (s->Body > 0) ? FrameSourceTypes_Body : FrameSourceTypes_None;
	this->enabledSources |= (s->BodyIndex > 0) ? FrameSourceTypes_BodyIndex : FrameSourceTypes_None;
	this->enabledSources |= (s->Color > 0) ? FrameSourceTypes_Color : FrameSourceTypes_None;
	this->enabledSources |= (s->Depth > 0) ? FrameSourceTypes_Depth : FrameSourceTypes_None;
	this->enabledSources |= (s->Infrared > 0) ? FrameSourceTypes_Infrared : FrameSourceTypes_None;
	this->enabledSources |= (s->LongExposureInfrared > 0) ? FrameSourceTypes_LongExposureInfrared : FrameSourceTypes_None;
	
	// Open MultiSource Reader with user-specified sources open
	if SUCCEEDED(this->kinect->OpenMultiSourceFrameReader(this->enabledSources, &this->multi_reader))
	{
		multi_reader->SubscribeMultiSourceFrameArrived(&this->h_event);
		return 1;
	}
	else
		return 0;
}

uint8_t Kinect2_impl::DisableSensors()
{
	HRESULT hr;
	// Close MultiSource Reader
	this->enabledSources = FrameSourceTypes_None;
	if (multi_reader != NULL)
	{
		hr = this->multi_reader->put_IsPaused(true);
		if SUCCEEDED(hr)
			std::cout << "Successfully Paused Multi-Source Reader" << std::endl;

		SafeRelease(this->multi_reader);
	}

	return 1;
}

RR_INTRUSIVE_PTR<sensors::kinect2::KinectMultiSource > Kinect2_impl::SensorsEnabled()
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	RR_INTRUSIVE_PTR<sensors::kinect2::KinectMultiSource> S(new sensors::kinect2::KinectMultiSource());
	
	S->Body = (this->enabledSources & FrameSourceTypes_Body) ? 1 : 0;
	S->BodyIndex = (this->enabledSources & FrameSourceTypes_BodyIndex) ? 1 : 0;
	S->Color = (this->enabledSources & FrameSourceTypes_Color) ? 1 : 0;
	S->Depth = (this->enabledSources & FrameSourceTypes_Depth) ? 1 : 0;
	S->Infrared = (this->enabledSources & FrameSourceTypes_Infrared) ? 1 : 0;
	S->LongExposureInfrared = (this->enabledSources & FrameSourceTypes_LongExposureInfrared) ? 1 : 0;

	return S;
}

RR_INTRUSIVE_PTR<sensors::kinect2::ImageHeader > Kinect2_impl::getColorImageHeader()
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	RR_INTRUSIVE_PTR<sensors::kinect2::ImageHeader> I(new sensors::kinect2::ImageHeader());

	I->width = this->color_image_width;
	I->height = this->color_image_height;
	I->channels = 4;
	I->step = 1;

	return I;

}
RR_INTRUSIVE_PTR<sensors::kinect2::ImageHeader > Kinect2_impl::getDepthImageHeader()
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	RR_INTRUSIVE_PTR<sensors::kinect2::ImageHeader> I(new sensors::kinect2::ImageHeader());

	I->width = this->depth_image_width;
	I->height = this->depth_image_height;
	I->channels = 1;
	I->step = 2;

	return I;

}

RR_INTRUSIVE_PTR<sensors::kinect2::Image > Kinect2_impl::getCurrentColorImage()
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	RR_INTRUSIVE_PTR<sensors::kinect2::Image> I(new sensors::kinect2::Image());

	I->width = this->color_image_width;
	I->height = this->color_image_height;
	I->channels = 4;
	I->data = RobotRaconteur::AttachRRArrayCopy<uint8_t>(reinterpret_cast<uint8_t *>(this->color_image_data), I->width * I->height * I->channels);

	return I;

}
RR_INTRUSIVE_PTR<sensors::kinect2::DepthImage > Kinect2_impl::getCurrentDepthImage()
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	RR_INTRUSIVE_PTR<sensors::kinect2::DepthImage> I(new sensors::kinect2::DepthImage());

	I->width = this->depth_image_width;
	I->height = this->depth_image_height;
	I->channels = 1;
	I->data = RobotRaconteur::AttachRRArrayCopy<uint16_t>(reinterpret_cast<uint16_t *>(this->depth_image_data), I->width * I->height * I->channels);

	return I;

}
RR_INTRUSIVE_PTR<sensors::kinect2::DepthImage > Kinect2_impl::getCurrentInfraredImage()
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	RR_INTRUSIVE_PTR<sensors::kinect2::DepthImage> I(new sensors::kinect2::DepthImage());

	I->width = this->depth_image_width;
	I->height = this->depth_image_height;
	I->channels = 1;
	I->data = RobotRaconteur::AttachRRArrayCopy<uint16_t>(reinterpret_cast<uint16_t *>(this->infrared_image_data), I->width * I->height * I->channels);

	return I;

}


RR_INTRUSIVE_PTR<sensors::kinect2::Image > Kinect2_impl::getCurrentBodyIndexImage()
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	RR_INTRUSIVE_PTR<sensors::kinect2::Image> I(new sensors::kinect2::Image());

	I->width = this->depth_image_width;
	I->height = this->depth_image_height;
	I->channels = 1;
	I->data = RobotRaconteur::AttachRRArrayCopy<uint8_t>(reinterpret_cast<uint8_t *>(this->bodyindex_image_data), I->width * I->height * I->channels);

	return I;
}

RR_INTRUSIVE_PTR<sensors::kinect2::DepthImage > Kinect2_impl::getCurrentLongExposureInfraredImage()
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	RR_INTRUSIVE_PTR<sensors::kinect2::DepthImage> I(new sensors::kinect2::DepthImage());

	I->width = this->depth_image_width;
	I->height = this->depth_image_height;
	I->channels = 1;
	I->data = RobotRaconteur::AttachRRArrayCopy<uint16_t>(reinterpret_cast<uint16_t *>(this->longexposure_infrared_image_data), I->width * I->height * I->channels);

	return I;
}


RR_INTRUSIVE_PTR<RobotRaconteur::RRArray<uint64_t > > Kinect2_impl::getTrackedBodyIDs()
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	RR_INTRUSIVE_PTR<RobotRaconteur::RRArray<uint64_t > > ids = RobotRaconteur::AttachRRArrayCopy<uint64_t >(reinterpret_cast<uint64_t *>(&this->tracked_body_ids), 6);
	
	return ids;

}

RR_INTRUSIVE_PTR<sensors::kinect2::KinectBody > Kinect2_impl::getDetectedBody(int32_t index)
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	HRESULT hr;

	RR_INTRUSIVE_PTR<sensors::kinect2::KinectBody > body(new sensors::kinect2::KinectBody());
	
	if (index >= BODY_COUNT)
		return body;
	
	double body_joint_positions[JointType::JointType_Count * 3];
	double body_joint_orientations[JointType::JointType_Count * 4];
	uint8_t body_joint_states[JointType::JointType_Count];
	uint8_t body_hand_states[2];
	uint8_t body_hand_confidence[2];
	double body_lean[2];
	uint64_t tracking_id;
	BOOLEAN is_tracked = false;

	hr = kinect_bodies[index]->get_TrackingId(&tracking_id);
	hr = kinect_bodies[index]->get_IsTracked(&is_tracked);

	if (is_tracked)
	{
		Joint my_joint_positions[JointType::JointType_Count];
		JointOrientation my_joint_orientations[JointType::JointType_Count];
		HandState my_hand_state = HandState::HandState_Unknown;
		TrackingConfidence my_hand_confidence = TrackingConfidence::TrackingConfidence_Low;
		PointF my_lean;

		hr = kinect_bodies[index]->GetJoints(JointType::JointType_Count, my_joint_positions);
		hr = kinect_bodies[index]->GetJointOrientations(JointType::JointType_Count, my_joint_orientations);
		for (int i = 0; i < JointType::JointType_Count; i++)
		{
			body_joint_positions[i * 3] = my_joint_positions[i].Position.X;
			body_joint_positions[i * 3 + 1] = my_joint_positions[i].Position.Y;
			body_joint_positions[i * 3 + 2] = my_joint_positions[i].Position.Z;

			body_joint_orientations[i * 4] = my_joint_orientations[i].Orientation.w;
			body_joint_orientations[i * 4 + 1] = my_joint_orientations[i].Orientation.x;
			body_joint_orientations[i * 4 + 2] = my_joint_orientations[i].Orientation.y;
			body_joint_orientations[i * 4 + 3] = my_joint_orientations[i].Orientation.z;

			body_joint_states[i] = my_joint_positions[i].TrackingState;
		}

		hr = kinect_bodies[index]->get_HandLeftState(&my_hand_state);
		hr = kinect_bodies[index]->get_HandLeftConfidence(&my_hand_confidence);
		body_hand_states[0] = my_hand_state;
		body_hand_confidence[0] = my_hand_confidence;
		hr = kinect_bodies[index]->get_HandRightState(&my_hand_state);
		hr = kinect_bodies[index]->get_HandRightConfidence(&my_hand_confidence);
		body_hand_states[1] = my_hand_state;
		body_hand_confidence[1] = my_hand_confidence;

		hr = kinect_bodies[index]->get_Lean(&my_lean);
		body_lean[0] = my_lean.X;
		body_lean[1] = my_lean.Y;
	}

	body->tracked = (is_tracked) ? 1 : 0;
	body->tracking_id = tracking_id;
	body->joint_positions = RobotRaconteur::AttachRRArrayCopy<double >(reinterpret_cast<double *>(&body_joint_positions), JointType::JointType_Count * 3);
	body->joint_orientations = RobotRaconteur::AttachRRArrayCopy<double >(reinterpret_cast<double *>(&body_joint_orientations), JointType::JointType_Count * 4);
	body->joint_tracking_state = RobotRaconteur::AttachRRArrayCopy<uint8_t >(reinterpret_cast<uint8_t *>(&body_joint_states), JointType::JointType_Count);
	body->hand_state = RobotRaconteur::AttachRRArrayCopy<uint8_t >(reinterpret_cast<uint8_t *>(&body_hand_states), 2);
	body->hand_confidence = RobotRaconteur::AttachRRArrayCopy<uint8_t >(reinterpret_cast<uint8_t *>(&body_hand_confidence), 2);
	body->lean = RobotRaconteur::AttachRRArrayCopy<double >(reinterpret_cast<double *>(&body_lean), 2);

	return body;
}

RR_INTRUSIVE_PTR<sensors::kinect2::PointCloud > Kinect2_impl::getPointCloud()
{
	

	HRESULT hr = NULL;
	ICoordinateMapper *coordinate_mapper;
	hr = this->kinect->get_CoordinateMapper(&coordinate_mapper);
	
	boost::lock_guard<boost::mutex> guard(mtx_);
	
	RR_INTRUSIVE_PTR<sensors::kinect2::PointCloud > pointcloud(new sensors::kinect2::PointCloud());
	RR::RRNamedArrayPtr<sensors::kinect2::point3f> cloud = RR::AllocateEmptyRRNamedArray<sensors::kinect2::point3f>(depth_image_width*depth_image_height);
	sensors::kinect2::point3f* cloud_ptr = &cloud->at(0);

	
	for (int y = 0; y < depth_image_height; y++) {
		for (int x = 0; x < depth_image_width ; x++) {
			

			DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
			UINT16 depth = depth_image_data[y * depth_image_width + x];
			
			// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
			CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
			
			
			
			coordinate_mapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
			
			cloud_ptr->s.x = cameraSpacePoint.X;
			cloud_ptr->s.y = cameraSpacePoint.Y;			
			cloud_ptr->s.y = cameraSpacePoint.Z;			
			cloud_ptr++;			
		}
		
	}
	pointcloud->points = cloud;
	
	return pointcloud;
}
void Kinect2_impl::MultiSourceFrameArrived(IMultiSourceFrameArrivedEventArgs* pArgs)
{
	HRESULT hr;
	IMultiSourceFrameReference *multi_frame_reference = NULL;
	IMultiSourceFrame *multi_frame = NULL;


	//std::cout << "Event Triggered" << std::endl;

	hr = pArgs->get_FrameReference(&multi_frame_reference);
	if FAILED(hr)
	{
		//std::cout << "Could not access multi-source reference.  Failed" << std::endl;
		SafeRelease(multi_frame_reference);
		return;
	}

	multi_frame_reference->AcquireFrame(&multi_frame);

	if (this->enabledSources & FrameSourceTypes_Body)
	{
		//std::cout << "Looking at Body Frame Data" << std::endl;
		IBodyFrameReference *body_frame_reference = NULL;
		IBodyFrame *body_frame = NULL;
		
		hr = multi_frame->get_BodyFrameReference(&body_frame_reference);
		if SUCCEEDED(hr)
			hr = body_frame_reference->AcquireFrame(&body_frame);

		if ((body_frame) && SUCCEEDED(hr))
		{
			//std::cout << "Copying to buffer...";
			hr = body_frame->GetAndRefreshBodyData(BODY_COUNT, this->kinect_bodies);
			//std::cout << ((SUCCEEDED(hr)) ? "Success" : "Failed") << std::endl;
			for (int i = 0; i < BODY_COUNT; i++)
				hr = this->kinect_bodies[i]->get_TrackingId(&this->tracked_body_ids[i]);
			
		}
		
		SafeRelease(body_frame);
		SafeRelease(body_frame_reference);
	}
	if (this->enabledSources & FrameSourceTypes_BodyIndex)
	{
		//std::cout << "Looking at BodyIndex Frame Data" << std::endl;
		IBodyIndexFrameReference *bodyindex_frame_reference = NULL;
		IBodyIndexFrame *bodyindex_frame = NULL;

		hr = multi_frame->get_BodyIndexFrameReference(&bodyindex_frame_reference);
		if SUCCEEDED(hr)
			hr = bodyindex_frame_reference->AcquireFrame(&bodyindex_frame);
		
		if SUCCEEDED(hr)
		{
			//std::cout << "Copying to buffer...";
			UINT buffer_size = this->depth_image_width * this->depth_image_height;
			hr = bodyindex_frame->CopyFrameDataToArray(buffer_size, reinterpret_cast<BYTE *>(this->bodyindex_image_data));
			//std::cout << ((SUCCEEDED(hr)) ? "Success" : "Failed") << std::endl;
		}
		SafeRelease(bodyindex_frame);
		SafeRelease(bodyindex_frame_reference);

	}
	if (this->enabledSources & FrameSourceTypes_Color)
	{
		//std::cout << "Looking at Color Frame Data" << std::endl;
		IColorFrameReference *color_frame_reference = NULL;
		IColorFrame *color_frame = NULL;
		hr = multi_frame->get_ColorFrameReference(&color_frame_reference);
		// Acquire Frame
		if SUCCEEDED(hr)
			hr = color_frame_reference->AcquireFrame(&color_frame);

		if SUCCEEDED(hr)
		{
			//std::cout << "Copying to buffer...";
			UINT buffer_size = this->color_image_width * this->color_image_height * sizeof(RGBQUAD);
			hr = color_frame->CopyConvertedFrameDataToArray(buffer_size, reinterpret_cast<BYTE *>(this->color_image_data), ColorImageFormat_Bgra);
			//std::cout << ((SUCCEEDED(hr)) ? "Success" : "Failed") << std::endl;
		}

		SafeRelease(color_frame);
		SafeRelease(color_frame_reference);

	}
	if (this->enabledSources & FrameSourceTypes_Depth)
	{
		//std::cout << "Looking at Depth Frame Data" << std::endl;
		IDepthFrameReference *depth_frame_reference = NULL;
		IDepthFrame *depth_frame = NULL;
		hr = multi_frame->get_DepthFrameReference(&depth_frame_reference);
		if (SUCCEEDED(hr))
			hr = depth_frame_reference->AcquireFrame(&depth_frame);

		if (SUCCEEDED(hr))
		{
			//std::cout << "Copying to buffer...";
			UINT buffer_size = this->depth_image_width * this->depth_image_height;
			hr = depth_frame->CopyFrameDataToArray(buffer_size, reinterpret_cast<UINT16 *>(this->depth_image_data));
			//std::cout << ((SUCCEEDED(hr)) ? "Success" : "Failed") << std::endl;
		}
		SafeRelease(depth_frame);
		SafeRelease(depth_frame_reference);

	}
	if (this->enabledSources & FrameSourceTypes_Infrared)
	{
		//std::cout << "Looking at Infrared Frame Data" << std::endl;
		IInfraredFrameReference *infrared_frame_reference = NULL;
		IInfraredFrame *infrared_frame = NULL;
		UINT buffer_size = this->depth_image_width * this->depth_image_height;
		hr = multi_frame->get_InfraredFrameReference(&infrared_frame_reference);
		if SUCCEEDED(hr)
			hr = infrared_frame_reference->AcquireFrame(&infrared_frame);

		if (SUCCEEDED(hr))
		{
			//std::cout << "Copying to buffer...";
			UINT buffer_size = this->depth_image_width * this->depth_image_height;
			hr = infrared_frame->CopyFrameDataToArray(buffer_size, reinterpret_cast<UINT16 *>(this->infrared_image_data));
			//std::cout << ((SUCCEEDED(hr)) ? "Success" : "Failed") << std::endl;
		}

		SafeRelease(infrared_frame);
		SafeRelease(infrared_frame_reference);

	}
	if (this->enabledSources & FrameSourceTypes_LongExposureInfrared)
	{
		//std::cout << "Looking at Long-Exposure Infrared Frame Data" << std::endl;
		ILongExposureInfraredFrameReference *longexposure_frame_reference = NULL;
		ILongExposureInfraredFrame *longexposure_frame = NULL;
		hr = multi_frame->get_LongExposureInfraredFrameReference(&longexposure_frame_reference);
		if SUCCEEDED(hr)
			hr = longexposure_frame_reference->AcquireFrame(&longexposure_frame);

		if (SUCCEEDED(hr))
		{
			//std::cout << "Copying to buffer...";
			UINT buffer_size = this->depth_image_width * this->depth_image_height;
			hr = longexposure_frame->CopyFrameDataToArray(buffer_size, reinterpret_cast<UINT16 *>(this->longexposure_infrared_image_data));
			//std::cout << ((SUCCEEDED(hr)) ? "Success" : "Failed" ) << std::endl;
		}

		SafeRelease(longexposure_frame);
		SafeRelease(longexposure_frame_reference);

	}

	SafeRelease(multi_frame);
	SafeRelease(multi_frame_reference);
}

void Kinect2_impl::backgroundPollingThread()
{
	HRESULT hr;
	DWORD res;
	std::cout << "Starting up background thread" << std::endl;
	while (true)
	{
		try
		{
			res = WaitForSingleObjectEx(reinterpret_cast<HANDLE>(h_event), 1000, false);
			if (multi_reader != NULL)
			{
				IMultiSourceFrameArrivedEventArgs *pArgs = nullptr;
				hr = this->multi_reader->GetMultiSourceFrameArrivedEventData(this->h_event, &pArgs);
				if (SUCCEEDED(hr))
				{
					MultiSourceFrameArrived(pArgs);
					pArgs->Release();
				}
				else
					std::cout << "Dropped Frame" << std::endl;

			}
			boost::this_thread::interruption_point();
		}
		catch (...) {
			break;
		}
	}

	std::cout << "Exiting Background Thread" << std::endl;
}