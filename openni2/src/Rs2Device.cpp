#include "Rs2Driver.h"
#include "Rs2Commands.h"
//#include <librealsense2/rsutil.h>

#define WORKER_THREAD_IDLE_MS 500
#define WORKER_THREAD_STOP_TIMEOUT_MS 5000
#define WAIT_FRAMESET_TIMEOUT_MS 2000
#define WAIT_ALIGNED_DEPTH_TIMEOUT_MS 100

void* handle = nullptr;
namespace oni { namespace driver {

Rs2Device::Rs2Device(Rs2Driver* driver, MV3D_RGBD_DEVICE_INFO* device)
: 
	m_driver(driver),
	m_device(device),
	m_registrationMode(ONI_IMAGE_REGISTRATION_OFF),
	m_config(0),
	//m_pipeline(nullptr),
	//m_pipelineProfile(nullptr),
	//m_alignQueue(nullptr),
	//m_alignProcessor(nullptr),
	m_runFlag(false),
	m_configId(0),
	m_framesetId(0)
{
	rsLogDebug("+Rs2Device");
}

Rs2Device::~Rs2Device()
{
	rsLogDebug("~Rs2Device");

	shutdown();
}

OniStatus Rs2Device::initialize()
{
	rsTraceFunc("");
	/*
	printf("initial sizeof(devs) %d nDevNum=%d\n", devs1.size(), nDevNum1);
	MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB, &nDevNum1);
	devs1.resize(nDevNum1);
	printf("initial sizeof(devs) %d nDevNum=%d\n", devs1.size(), nDevNum1);
	//容量必须给至少是nDevNum值
	MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB, &devs1[0], nDevNum1, &nDevNum1);
	*/

	for (;;)
	{
		Rs2ScopedMutex lock(m_stateMx);

		if (m_thread.get())
		{
			rsTraceError("Already initialized");
			break;
		}

		if (queryDeviceInfo(m_device, &m_info) != ONI_STATUS_OK)
		{
			rsTraceError("queryDeviceInfo failed");
			break;
		}
		//printf("queryDeviceInfo success\n");
		{
			Rs2ScopedMutex streamLock(m_streamsMx);

			if (initializeStreams() != ONI_STATUS_OK)
			{
				rsTraceError("initializeStreams failed");
				break;
			}
		}
		//printf("initializeStreams success\n");
		m_configId = 0;
		m_runFlag = true;

		try {
			m_thread.reset(new std::thread(&Rs2Device::mainLoop, this));
		}
		catch (std::exception& ex) {
			rsTraceError("std::thread failed: %s", ex.what());
			break;
		}

		return ONI_STATUS_OK;
	}

	shutdown();
	return ONI_STATUS_ERROR;
}

void Rs2Device::shutdown()
{
	if (m_device) { rsTraceFunc(""); }

	Rs2ScopedMutex lock(m_stateMx);

	m_runFlag = false;
	if (m_thread)
	{
		m_thread->join();
		m_thread = nullptr;
	}

	{
		Rs2ScopedMutex streamLock(m_streamsMx);

		for (auto iter = m_createdStreams.begin(); iter != m_createdStreams.end(); ++iter) { delete(*iter); }
		m_createdStreams.clear();

		for (auto iter = m_streams.begin(); iter != m_streams.end(); ++iter) { delete(*iter); }
		m_streams.clear();
	}

	for (auto iter = m_sensorInfo.begin(); iter != m_sensorInfo.end(); ++iter) { delete[](iter->pSupportedVideoModes); }
	m_sensorInfo.clear();
	m_profiles.clear();
	/*
	if (m_device)
	{
		rs2_delete_device(m_device);
		m_device = nullptr;
	}*/
}

OniStatus Rs2Device::queryDeviceInfo(MV3D_RGBD_DEVICE_INFO* device, OniDeviceInfo* deviceInfo)
{
	//rsTraceFunc("");
	static int i = 0;
	//Rs2Error e;
	const char* serial = device->chSerialNumber;
	/*
	const char* serial = rs2_get_device_info(device, RS2_CAMERA_INFO_SERIAL_NUMBER, &e);
	if (!e.success()) return ONI_STATUS_ERROR;
	
	const char* name = rs2_get_device_info(device, RS2_CAMERA_INFO_NAME, &e);
	if (!e.success()) return ONI_STATUS_ERROR;
	*/
	const char* name = device->chModelName;

	//strncpy(deviceInfo->uri, serial, sizeof(deviceInfo->uri) - 1);
	strncpy(deviceInfo->uri, serial, sizeof(deviceInfo->uri) - 1);

	#ifdef RS2_EMULATE_PRIMESENSE_HARDWARE
		strncpy(deviceInfo->name, "PS1080", sizeof(deviceInfo->name) - 1);
		strncpy(deviceInfo->vendor, "PrimeSense", sizeof(deviceInfo->vendor) - 1);
		deviceInfo->usbVendorId = 7463;
		deviceInfo->usbProductId = 1537;
	#else
		strncpy(deviceInfo->name, name, sizeof(deviceInfo->name) - 1);
		strncpy(deviceInfo->vendor, "", sizeof(deviceInfo->vendor) - 1);
		deviceInfo->usbVendorId = 0;
		deviceInfo->usbProductId = 0;
	#endif

	//rsLogDebug("i=%d DEVICE serial=%s name=%s",i+=1, serial, name);
		//rsLogDebug(" DEVICE serial=%s name=%s", serial, name);
	return ONI_STATUS_OK;
}

//=============================================================================
// DeviceBase overrides
//=============================================================================

OniStatus Rs2Device::getSensorInfoList(OniSensorInfo** sensors, int* numSensors)
{
	rsTraceFunc("");

	Rs2ScopedMutex lock(m_stateMx);

	*numSensors = (int)m_sensorInfo.size();
	*sensors = ((*numSensors > 0) ? &m_sensorInfo[0] : nullptr);

	return ONI_STATUS_OK;
}

StreamBase* Rs2Device::createStream(OniSensorType sensorType)
{
	printf("Rs2Device.cpp\n");
	rsTraceFunc("sensorType=%d", (int)sensorType);
	printf("Rs2Device.cpp\n");
	Rs2ScopedMutex lock(m_streamsMx);

	for (auto iter = m_streams.begin(); iter != m_streams.end(); ++iter)
	{
		Rs2Stream* streamObj = *iter;
		if (streamObj->getOniType() == sensorType)
		{
			m_createdStreams.push_back(streamObj);
			m_streams.remove(streamObj);
			printf("m_createdStreams.push_back(sensorType)%d\n", sensorType);
			return streamObj;
		}
	}

	return nullptr;
}

void Rs2Device::destroyStream(StreamBase* streamBase)
{
	rsTraceFunc("ptr=%p", streamBase);

	if (streamBase)
	{
		Rs2ScopedMutex lock(m_streamsMx);

		Rs2Stream* streamObj = (Rs2Stream*)streamBase;
		streamObj->stop();

		m_streams.push_back(streamObj);
		m_createdStreams.remove(streamObj);
	}
}
/*
OniStatus Rs2Device::invoke(int commandId, void* data, int dataSize)
{
	if (commandId == RS2_PROJECT_POINT_TO_PIXEL && data && dataSize == sizeof(Rs2PointPixel))
	{
		for (auto iter = m_createdStreams.begin(); iter != m_createdStreams.end(); ++iter)
		{
			Rs2Stream* stream = *iter;
			if (stream->getOniType() == ONI_SENSOR_DEPTH)
			{
				auto pp = (Rs2PointPixel*)data;
				rs2_project_point_to_pixel(pp->pixel, &stream->m_intrinsics, pp->point);
				return ONI_STATUS_OK;
			}
		}
		return ONI_STATUS_NO_DEVICE;
	}

	#if defined(RS2_TRACE_NOT_SUPPORTED_CMDS)
		rsTraceError("Not supported: commandId=%d", commandId);
	#endif
	return ONI_STATUS_NOT_SUPPORTED;
}
*/
OniBool Rs2Device::isCommandSupported(int commandId)
{
	switch (commandId)
	{
		case RS2_PROJECT_POINT_TO_PIXEL: return true;
	}

	return false;
}

OniStatus Rs2Device::tryManualTrigger()
{
	return ONI_STATUS_OK;
}

OniBool Rs2Device::isImageRegistrationModeSupported(OniImageRegistrationMode mode)
{
	return true;
}

//=============================================================================
// Start/Stop
//=============================================================================

OniStatus Rs2Device::startPipeline()
{
	rsTraceFunc("");

	for (;;)
	{
		if (m_config)
		{
			rsTraceError("Already started");
			break;
		}

		//Rs2Error e;

		rsLogDebug("rs2_create_config");
		m_config = 1;
		/*
		if (!e.success())
		{
			rsTraceError("rs2_create_config failed: %s", e.get_message());
			break;
		}
		*/
		rsLogDebug("rs2_config_enable_device %s", m_info.uri);
		//通过序列号选择设备给pipeline
		//rs2_config_enable_device(m_config, m_info.uri, &e);
		//通过序列号打开设备
		//MV3D_RGBD_OpenDeviceBySerialNumber((HANDLE*)handle, m_info.uri);
		/*
		if (!e.success())
		{
			rsTraceError("rs2_config_enable_device failed: %s", e.get_message());
			break;
		}

		rs2_config_disable_all_streams(m_config, &e);
		*/
		bool enableStreamError = false;

		{
			Rs2ScopedMutex lock(m_streamsMx);

			for (auto iter = m_createdStreams.begin(); iter != m_createdStreams.end(); ++iter)
			{
				Rs2Stream* stream = *iter;
				if (stream->isEnabled())
				{
					printf("stream->isEnabled()\n");
					const OniVideoMode& mode = stream->getVideoMode();

					rsLogDebug("ENABLE STREAM type=%d sensorId=%d streamId=%d %dx%d @%d", 
						(int)stream->getRsType(), stream->getSensorId(), stream->getStreamId(), mode.resolutionX, mode.resolutionY, mode.fps);
					
					
					/*
					//使用选定的流参数显示启用设备
					rs2_config_enable_stream(
						m_config, stream->getRsType(), stream->getStreamId(), 
						mode.resolutionX, mode.resolutionY, convertPixelFormat(mode.pixelFormat), mode.fps, &e
					);
					if (!e.success())
					{
						rsTraceError("rs2_config_enable_stream failed: %s", e.get_message());
						enableStreamError = true;
						break;
					}
					*/
					printf("stream->onStreamStarted\n");
					stream->onStreamStarted();
				}
			}
		}

		if (enableStreamError)
		{
			rsTraceError("enable_stream error");
			break;
		}
		/*
		// pipeline

		rs2_context* context = getDriver()->getRsContext();
		rsLogDebug("rs2_create_pipeline");
		m_pipeline = rs2_create_pipeline(context, &e);
		if (!e.success())
		{
			rsTraceError("rs2_create_pipeline failed: %s", e.get_message());
			break;
		}

		rsLogDebug("rs2_pipeline_start_with_config");
		m_pipelineProfile = rs2_pipeline_start_with_config(m_pipeline, m_config, &e);
		if (!e.success())
		{
			rsTraceError("rs2_pipeline_start_with_config failed: %s", e.get_message());
			break;
		}
		*/
		{
			Rs2ScopedMutex lock(m_streamsMx);

			for (auto iter = m_createdStreams.begin(); iter != m_createdStreams.end(); ++iter)
			{
				Rs2Stream* stream = *iter;
				if (stream->isEnabled())
				{
					stream->onPipelineStarted();
				}
			}
		}

		// depth to color aligner
		//创建一个最大容量为一的帧的队列
		/*
		m_alignQueue = rs2_create_frame_queue(1, &e);
		if (!e.success())
		{
			rsTraceError("rs2_create_frame_queue failed: %s", e.get_message());
			break;
		}
		//创建一个流中帧对齐进程块（深度对齐到color？）
		m_alignProcessor = rs2_create_align(RS2_STREAM_COLOR, &e);
		if (!e.success())
		{
			rsTraceError("rs2_create_align failed: %s", e.get_message());
			break;
		}

		rs2_start_processing_queue(m_alignProcessor, m_alignQueue, &e);
		if (!e.success())
		{
			rsTraceError("rs2_start_processing_queue failed: %s", e.get_message());
			break;
		}
		*/
		rsLogDebug("STARTED");
		return ONI_STATUS_OK;
	}

	stopPipeline();
	return ONI_STATUS_ERROR;
}

void Rs2Device::stopPipeline()
{
	/*
	if (m_pipeline) { rsTraceFunc(""); }

	if (m_pipeline)
	{
		Rs2Error e;
		rsLogDebug("rs2_pipeline_stop");
		rs2_pipeline_stop(m_pipeline, &e);
	}
	if (m_alignProcessor)
	{
		rs2_delete_processing_block(m_alignProcessor);
		m_alignProcessor = nullptr;
	}
	if (m_alignQueue)
	{
		rs2_delete_frame_queue(m_alignQueue);
		m_alignQueue = nullptr;
	}
	
	if (m_pipelineProfile)
	{
		rsLogDebug("rs2_delete_pipeline_profile");
		rs2_delete_pipeline_profile(m_pipelineProfile);
		m_pipelineProfile = nullptr;
	}
	*/
	if (m_config)
	{
		rsLogDebug("rs2_delete_config");
		//rs2_delete_config(m_config);
		m_config = 0;
	}
	/*
	if (m_pipeline)
	{
		rsLogDebug("rs2_delete_pipeline");
		rs2_delete_pipeline(m_pipeline);
		m_pipeline = nullptr;
		rsLogDebug("STOPPED");
	}
	*/
}

void Rs2Device::restartPipeline()
{
	rsTraceFunc("");
	printf("restartPipeline\n");
	stopPipeline();

	bool hasStreams;
	{
		printf("restartPipeline-hasStreams\n");
		Rs2ScopedMutex lock(m_streamsMx);
		hasStreams = hasEnabledStreams();
	}

	if (hasStreams && m_runFlag)
	{
		printf("startPipeline\n");
		startPipeline();
	}
}

//=============================================================================
// MainLoop
//=============================================================================

void Rs2Device::mainLoop()
{
	
	rsTraceFunc("");
	
	try
	{
		//MV3D_RGBD_Start(handle);

		int configId = 0;
		while (m_runFlag)
		{
			printf("111111111111111111 m_configId=%d \n", m_configId);
			waitForFrames();
			/*
			const int curConfigId = m_configId;
			if (configId != curConfigId) // configuration changed since last tick
			{
				printf("222222222222222222222\n");
				configId = curConfigId;
				//restartPipeline();
			}*/
			//printf("33333333333333333333\n");
			/*
			if (m_runFlag)
			{
				printf("44444444444444444444\n");
				waitForFrames();
				
			}
			else
			{
				
				printf("555555555555555555555\n");
				std::this_thread::sleep_for(std::chrono::milliseconds(WORKER_THREAD_IDLE_MS));
			}*/
		}
	}
	catch (...)
	{
		rsTraceError("Unhandled exception");
	}

	stopPipeline();
}

void Rs2Device::updateConfiguration()
{
	rsTraceFunc("");

	m_configId++;
}

void Rs2Device::waitForFrames()
{
	//printf("waitForFrames SCOPED_PROFILER\n");
	SCOPED_PROFILER;

	//Rs2Error e;
	//rs2_frame* frameset;
	MV3D_RGBD_FRAME_DATA frameset;
	{
		
		//printf("waitForFrames NAMED_PROFILER\n");
		NAMED_PROFILER("rs2_pipeline_wait_for_frames");
		//frameset = rs2_pipeline_wait_for_frames(m_pipeline, WAIT_FRAMESET_TIMEOUT_MS, &e);
		MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
		
		memcpy(&frameset, &stFrameData, sizeof(frameset));
		//printf("图像个数= %d, 图像类型 =%d\n",frameset.nImageCount, frameset.stImageData[0].enImageType);
		
	}


	/*
	if (!e.success())
	{
		rsTraceError("rs2_pipeline_wait_for_frames failed: %s", e.get_message());
		return;
	}
	*/
	//const int nframes = rs2_embedded_frames_count(frameset, &e);
	//rsLogDebug("frameset %llu (%d)", m_framesetId, nframes);
	//复合帧中帧数
	const int nframes = 1;

	if (m_registrationMode == ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR)
	{
		//创建帧的引用，不复制数据
		//MV3D_RGBD_FRAME_DATA* frameset1 = frameset;
		/*
		rs2_frame_add_ref(frameset, &e);
		{
			NAMED_PROFILER("rs2_process_frame");
			rs2_process_frame(m_alignProcessor, frameset, &e);
			if (!e.success())
			{
				rsTraceError("rs2_process_frame failed: %s", e.get_message());
			}
		}*/
	}

	for (int i = 0; i < nframes; ++i)
    {
		MV3D_RGBD_FRAME_DATA* frame = &frameset;// rs2_extract_frame(frameset, i, &e);
		if (frame)
		{
			//printf("图像个数= %d, 图像类型 =%d\n", frame->nImageCount, frame->stImageData[0].enImageType);
			//printf("processFrame success\n");
		
			
			for (int framecount = 0; framecount < frame->nImageCount; ++framecount)
			{
				//printf("frame->nImageCount=%d ????????\n", frame->nImageCount);
				processFrame(frame, framecount);
			}
			
			//printf("图像个数= %d, 图像类型 =%d\n", frame->nImageCount, frame->stImageData[0].enImageType);

			//printf("releaseFrame success\n");
			//releaseFrame(frame);
		}
	}

	//releaseFrame(&frameset);

	++m_framesetId;
	printf("++m_framesetId=%d \n", m_framesetId);
	//深度图对齐到color？
	
	if (m_registrationMode == ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR)
	{
		waitAlignedDepth();
	}
}

void Rs2Device::processFrame(MV3D_RGBD_FRAME_DATA* frame,int framecount)
{
	SCOPED_PROFILER;

	RIFrameInfo spi;
	Rs2Stream* stream = getFrameStream(frame, &spi, framecount);
	printf("stream \n");

	if (stream)
	{
		printf("stream1 \n");

		if (m_registrationMode == ONI_IMAGE_REGISTRATION_OFF || spi.streamType != ImageType_Depth)
		{
			printf("流id= %d, 流(图像)类型 =%d\n", spi.streamId, spi.streamType);
			OniFrame* oniFrame = createOniFrame(frame, stream, &spi);
			
			if (oniFrame)
			{
				printf("数据大小= %d, frameIndex =%d 图像类型=%d\n", oniFrame->dataSize, oniFrame->frameIndex,oniFrame->sensorType);

				submitOniFrame(oniFrame, stream);
			}
		}
	}
	

}

void Rs2Device::waitAlignedDepth()
{
	printf("waitAlignedDepth NAMED_PROFILER\n");
	SCOPED_PROFILER;

	//Rs2Error e;
	MV3D_RGBD_FRAME_DATA frameset;
	{
		printf("waitAlignedDepth NAMED_PROFILER\n");
		NAMED_PROFILER("rs2_wait_for_frame");
		//frameset = rs2_wait_for_frame(m_alignQueue, WAIT_ALIGNED_DEPTH_TIMEOUT_MS, &e);
		printf("waitAlignedDepth NAMED_PROFILER\n");
		NAMED_PROFILER("rs2_pipeline_wait_for_frames");
		//frameset = rs2_pipeline_wait_for_frames(m_pipeline, WAIT_FRAMESET_TIMEOUT_MS, &e);
		MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
		memcpy(&frameset, &stFrameData, sizeof(frameset));
	}
	/*
	if (!e.success())
	{
		rsTraceError("rs2_wait_for_frame failed: %s", e.get_message());
		return;
	}
	*/
	//const int nframes = rs2_embedded_frames_count(frameset, &e);
	//设置每次的帧数为1
	const int nframes = 1;
	for (int i = 0; i < nframes; ++i)
	{
		MV3D_RGBD_FRAME_DATA* frame = &frameset;
		if (frame)
		{
			//for (int framecount = 0; framecount < frame->nImageCount; ++framecount)
			{
				RIFrameInfo spi;
				Rs2Stream* stream = getFrameStream(frame, &spi,0);
				if (stream && spi.streamType == ImageType_Depth)
				{
					OniFrame* oniFrame = createOniFrame(frame, stream, &spi);
					if (oniFrame)
					{
						printf("OniFrame create success\n");
						submitOniFrame(oniFrame, stream);
					}
				}
				
			}
			
			//releaseFrame(frame);
		}
	}

	//releaseFrame(&frameset);
}

OniFrame* Rs2Device::createOniFrame(MV3D_RGBD_FRAME_DATA* frame, Rs2Stream* stream, RIFrameInfo* spi)
{
	SCOPED_PROFILER;

	if (!frame || !stream || !stream->isEnabled() || !spi)
	{
		printf("createOniFrame fail\n");
		return nullptr;
	}

	//Rs2Error e;
	OniFrame* oniFrame;
	//OniFrame oniFrame1;
	//printf("oniFrame->dataSize=%d\n", oniFrame->dataSize);
	{
		
		NAMED_PROFILER("StreamServices::acquireFrame");
		oniFrame = stream->getServices().acquireFrame();
		//printf("oniFrame->dataSize=%d\n",oniFrame->dataSize);
		
		if (!oniFrame)
		{
			rsTraceError("acquireFrame failed");
			printf("申请内存失败\n");
			return nullptr;
		}
		printf("申请内存成功,内存地址=%p\n", oniFrame);
		
	}
	
	oniFrame->sensorType = stream->getOniType();
	oniFrame->timestamp = (uint64_t)(frame->stImageData[spi->streamId].nTimeStamp * 1000.0); // millisecond to microsecond
	oniFrame->frameIndex = (int)frame->stImageData[spi->streamId].nFrameNum;

	oniFrame->width = frame->stImageData[spi->streamId].nWidth;
	oniFrame->height = frame->stImageData[spi->streamId].nHeight;
	//oniFrame->stride = rs2_get_frame_stride_in_bytes(frame, &e);
	//const size_t frameSize = oniFrame->stride * oniFrame->height;
	const size_t frameSize = frame->stImageData[spi->streamId].nDataLen;
	oniFrame->data = frame->stImageData[spi->streamId].pData;

	OniVideoMode mode;
	mode.pixelFormat = convertPixelFormat(spi->streamType);
	mode.resolutionX = oniFrame->width;
	mode.resolutionY = oniFrame->height;

	MV3D_RGBD_PARAM pstValue;
	


	int nRet = MV3D_RGBD_OK;
	//获得宽
	memset(&pstValue, 0, sizeof(MV3D_RGBD_PARAM));
	//获得帧率
	nRet = MV3D_RGBD_GetParam(handle, MV3D_RGBD_FLOAT_FRAMERATE, &pstValue);
	if (MV3D_RGBD_OK != nRet)
	{
		return nullptr;
	}
	mode.fps = pstValue.ParamInfo.stFloatParam.fCurValue;
	//mode.fps = spi->framerate;

	oniFrame->videoMode = mode;
	oniFrame->croppingEnabled = false;
	oniFrame->cropOriginX = 0;
	oniFrame->cropOriginY = 0;

	printf("invalid frame: rsSize=%u oniSize=%u oniFrame->data=%p\n", (unsigned int)frameSize, (unsigned int)oniFrame->dataSize, oniFrame->data);
	printf("stride=%d frameheight=%d framewideth=%d frameIndex=%d \n", oniFrame->stride, oniFrame->height,oniFrame->width, oniFrame->frameIndex);
	oniFrame->dataSize = frameSize;
	if (frameSize != oniFrame->dataSize)
	{
		rsTraceError("invalid frame: rsSize=%u oniSize=%u", (unsigned int)frameSize, (unsigned int)oniFrame->dataSize);
		stream->getServices().releaseFrame(oniFrame);
		return nullptr;
	}
	
	const void* frameData = frame->stImageData[spi->streamId].pData;
	if (!frameData)
	{
		//rsTraceError("rs2_get_frame_data failed: %s", e.get_message());
		stream->getServices().releaseFrame(oniFrame);
		printf("stream->getServices().releaseFrame(oniFrame);\n");
		return nullptr;
	}
	
	{
		NAMED_PROFILER("_copyFrameData");
		printf("memcpy(oniFrame->data, frameData, frameSize)\n");
		memcpy(oniFrame->data, frameData, frameSize);
	}

	return oniFrame;
}

void Rs2Device::submitOniFrame(OniFrame* oniFrame, Rs2Stream* stream)
{
	SCOPED_PROFILER;

	if (stream->getOniType() == ONI_SENSOR_DEPTH) // HACK: clamp depth to OpenNI hardcoded max value
	{
		NAMED_PROFILER("_clampDepth");
		uint16_t* depth = (uint16_t*)oniFrame->data;
		for (int i = 0; i < oniFrame->width * oniFrame->height; ++i)
		{
			if (*depth >= ONI_MAX_DEPTH) { *depth = ONI_MAX_DEPTH - 1; }
			++depth;
		}
	}
	{
		NAMED_PROFILER("StreamServices::raiseNewFrame");
		stream->raiseNewFrame(oniFrame);
	}
	{
		NAMED_PROFILER("StreamServices::releaseFrame");
		stream->getServices().releaseFrame(oniFrame);
	}
}

Rs2Stream* Rs2Device::getFrameStream(MV3D_RGBD_FRAME_DATA* frame, RIFrameInfo* spi,int framecount)
{
	SCOPED_PROFILER;
	/*
	Rs2Error e;
	//const rs2_stream_profile* profile = rs2_get_frame_stream_profile(frame, &e);
	if (e.success())
	{
		memset(spi, 0, sizeof(RIFrameInfo));
		
		rs2_get_stream_profile_data(profile, &spi->streamType, &spi->format, &spi->streamId, &spi->profileId, &spi->framerate, &e);
		if (e.success())
		{*/
	//深度图像

		spi->streamType = frame->stImageData[framecount].enImageType;
		spi->nHeight = frame->stImageData[framecount].nHeight;
		spi->nWidth = frame->stImageData[framecount].nWidth;
		spi->pData = frame->stImageData[framecount].pData;
		spi->nFrameNum = frame->stImageData[framecount].nFrameNum;
		spi->nFrameLength = frame->stImageData[framecount].nDataLen;
		spi->streamId = framecount;
		//printf("spi->streamType=%d----------------------\n", frame->stImageData[framecount].enImageType);

		MV3D_RGBD_PARAM pstValue;
		int nRet = MV3D_RGBD_OK;
		memset(&pstValue, 0, sizeof(MV3D_RGBD_PARAM));
		//获得帧率
		nRet = MV3D_RGBD_GetParam(handle, MV3D_RGBD_FLOAT_FRAMERATE, &pstValue);

		
		spi->framerate = pstValue.ParamInfo.stFloatParam.fCurValue;

		printf("stream spi->streamType=%d-------------------\n", spi->streamType);

			OniSensorType sensorType = convertStreamType(spi->streamType);
			{
				//printf("stream spi->streamType=%d-------------------\n", spi->streamType);
				Rs2ScopedMutex lock(m_streamsMx);
				printf("stream spi->streamId=%d sensorType=%d-------------------\n", spi->streamId, sensorType);

				return findStream(sensorType, spi->streamId);
			}
			/*
		}
	}*/
			
	return nullptr;
}

void Rs2Device::releaseFrame(MV3D_RGBD_FRAME_DATA* frame)
{
	NAMED_PROFILER("rs2_release_frame");
	delete frame;
	//rs2_release_frame(frame);
}

//=============================================================================
// Stream initialization //初始化流时，需要存储不同的流类型
//=============================================================================

OniStatus Rs2Device::initializeStreams()
{
	rsTraceFunc("");
	std::map<int, Mv3dRgbdImageType> Mv3d_sensorStreams;
	//std::map<int, rs2_stream> sensorStreams;

	//Rs2Error e;
	/*
	rs2_sensor_list* sensorList = rs2_query_sensors(m_device, &e);
	if (sensorList)
	{
		const int nsensors = rs2_get_sensors_count(sensorList, &e);
		for (int sensorId = 0; sensorId < nsensors; sensorId++)
		{
			rsLogDebug("SENSOR %d", sensorId);
			
			rs2_sensor* sensor = rs2_create_sensor(sensorList, sensorId, &e);
			if (sensor)
			{
				sensorStreams.clear();

				rs2_stream_profile_list* profileList = rs2_get_stream_profiles(sensor, &e);
				if (profileList)
				{
					const int nprofiles = rs2_get_stream_profiles_count(profileList, &e);
					for (int profileId = 0; profileId < nprofiles; profileId++)
					{
						const rs2_stream_profile* profile = rs2_get_stream_profile(profileList, profileId, &e);
						if (profile)
						{*/

			RIFrameInfo MV3D_spi;
			
			MV3D_RGBD_PARAM pstValue;
			int nRet = MV3D_RGBD_OK;
			char * pParamName = MV3D_RGBD_INT_WIDTH;
			//获得宽
			memset(&pstValue, 0, sizeof(MV3D_RGBD_PARAM));
			nRet = MV3D_RGBD_GetParam(handle, MV3D_RGBD_INT_WIDTH, &pstValue);
			printf("宽= %d\n", pstValue.ParamInfo.stIntParam.nCurValue);
			if (MV3D_RGBD_OK != nRet)
			{
				return ONI_STATUS_ERROR;
			}
			MV3D_spi.nWidth = pstValue.ParamInfo.stIntParam.nCurValue;
			
			//获得高
			nRet = MV3D_RGBD_GetParam(handle, MV3D_RGBD_INT_HEIGHT, &pstValue);
			printf("高= %d\n", pstValue.ParamInfo.stIntParam.nCurValue);
			if (MV3D_RGBD_OK != nRet)
			{
				return ONI_STATUS_ERROR;
			}
			MV3D_spi.nHeight = pstValue.ParamInfo.stIntParam.nCurValue;
			
			//获得像素格式
			nRet = MV3D_RGBD_GetParam(handle, MV3D_RGBD_ENUM_PIXELFORMAT, &pstValue);
			printf("像素格式= %d\n", (Mv3dRgbdImageType)pstValue.ParamInfo.stIntParam.nCurValue);
			if (MV3D_RGBD_OK != nRet)
			{
				return ONI_STATUS_ERROR;
			}
			//流类型
			//MV3D_spi.streamType = (Mv3dRgbdImageType)pstValue.ParamInfo.stIntParam.nCurValue;
			//流id
			//MV3D_spi.streamId = (Mv3dRgbdImageType)pstValue.ParamInfo.stIntParam.nCurValue;
			
			//获得帧率
			nRet = MV3D_RGBD_GetParam(handle, MV3D_RGBD_FLOAT_FRAMERATE, &pstValue);
			printf("帧率= %f\n", pstValue.ParamInfo.stFloatParam.fCurValue);
			if (MV3D_RGBD_OK != nRet)
			{
				return ONI_STATUS_ERROR;
			}
			MV3D_spi.framerate = pstValue.ParamInfo.stFloatParam.fCurValue;
			printf("打印帧的各种数值\n");

			//MV3D_spi.streamId = 0;
			//MV3D_spi.sensorId = 0;


			//流类型
			//MV3D_spi.streamType = ImageType_Depth;
			//流id
			//MV3D_spi.streamId = 0;
			//传感器id
			//MV3D_spi.sensorId = ImageType_Depth;
			
			//循环初始化不同的流
			for (int streamNumber = 0; streamNumber < 3; streamNumber++)
			{
				
				switch (streamNumber)
				{
					case 0: {
						//流类型
						MV3D_spi.streamType = ImageType_Depth;
						//流id
						MV3D_spi.streamId = 0;
						//传感器id
						MV3D_spi.sensorId = ImageType_Depth;
						break;
					}
					case 1: {
						MV3D_spi.streamType = ImageType_YUV422;
						MV3D_spi.streamId = 1;
						//传感器id
						MV3D_spi.sensorId = ImageType_YUV422;
						break;
					}
					case 2: {
						MV3D_spi.streamType = ImageType_Mono8;
						MV3D_spi.streamId = 2;
						//传感器id
						MV3D_spi.sensorId = ImageType_Mono8;
						break;
					}
				}
				
			//Rs2StreamProfileInfo spi;
			//spi.profile = profile;
			//rs2_get_stream_profile_data(profile, &spi.streamType, &spi.format, &spi.streamId, &spi.profileId, &spi.framerate, &e);

			//if (isSupportedStreamType(spi.streamType) && isSupportedPixelFormat(spi.format))
				if (isSupportedStreamType(MV3D_spi.streamType))
				{
					/*
					//rs2_get_video_stream_resolution(profile, &spi.width, &spi.height, &e);
				
					if (e.success())
					{*/
						#if 1
					rsLogDebug("\ttype=%d sensorId=%d streamId=%d width=%d height=%d framerate=%d 2\n", 
						(int)MV3D_spi.streamType, (int)MV3D_spi.sensorId, (int)MV3D_spi.streamId, (int)MV3D_spi.nWidth, (int)MV3D_spi.nHeight,(int)MV3D_spi.framerate);
					#endif

					m_profiles.push_back(MV3D_spi);
					Mv3d_sensorStreams[MV3D_spi.streamId] = MV3D_spi.streamType;
					//}
				}
							/*
						}
					}
					rs2_delete_stream_profiles_list(profileList);
				}
				
				for (auto iter = sensorStreams.begin(); iter != sensorStreams.end(); ++iter)
				{
					rsLogDebug("UNIQ streamId (%d) -> type (%d)", iter->first, (int)iter->second);
				}

				for (auto iter = sensorStreams.begin(); iter != sensorStreams.end(); ++iter)
				{
					const OniSensorType oniType = convertStreamType(iter->second);

					std::vector<Rs2StreamProfileInfo> profiles;
					findStreamProfiles(&profiles, oniType, iter->first);

					if (addStream(sensor, oniType, sensorId, iter->first, &profiles) == ONI_STATUS_OK)
					{
						sensor = nullptr;
					}
				}
				*/
				
			}
			
			for (auto iter = Mv3d_sensorStreams.begin(); iter != Mv3d_sensorStreams.end(); ++iter)
			{
				const OniSensorType oniType = convertStreamType(iter->second);
				printf("\ttype=%d Mv3dRgbdImageType=%d 2\n", (int)oniType, iter->second);

				std::vector<RIFrameInfo> profiles;
				findStreamProfiles(&profiles, oniType, iter->first);

				if (addStream(oniType, iter->second, iter->first, &profiles) == ONI_STATUS_OK)
				{
					//printf("addStream success\n");
					//sensor = nullptr;
				}
			}

				/*
				if (sensor) { rs2_delete_sensor(sensor); }
			}
		}
		
		rs2_delete_sensor_list(sensorList);
	}*/

	rsLogDebug("FILL OniSensorInfo");
	for (auto iter = m_streams.begin(); iter != m_streams.end(); ++iter)
	{
		Rs2Stream* stream = *iter;
		#if 1
		rsLogDebug("STREAM type=%d sensorId=%d streamId=%d", (int)stream->getRsType(), stream->getSensorId(), stream->getStreamId());
		#endif

		std::vector<RIFrameInfo> profiles;
		findStreamProfiles(&profiles, stream->getOniType(), stream->getStreamId());

		//printf(" 1 1 1 1 1 1 1 1 1 \n");
		OniSensorInfo info;
		info.sensorType = stream->getOniType();
		info.numSupportedVideoModes = (int)profiles.size();
		info.pSupportedVideoModes = nullptr;
		//printf(" 2 2 2 2 2  2 2 2 2 2  2\n");
		if (info.numSupportedVideoModes > 0)
		{
			info.pSupportedVideoModes = new OniVideoMode[info.numSupportedVideoModes];
			int modeId = 0;

			for (auto p = profiles.begin(); p != profiles.end(); ++p)
			{
				//printf(" 3 3 3 3  3 3 3 3 3 3  3 3 3 \n");
				OniVideoMode& mode = info.pSupportedVideoModes[modeId];
				mode.pixelFormat = convertPixelFormat(p->streamType);
				//printf(" 4 4 4 4  4 4 4 4 4 4 4  \n");
				mode.resolutionX = p->nWidth;
				mode.resolutionY = p->nHeight;
				mode.fps = p->framerate;
				modeId++;

				#if 1
				rsLogDebug("\ttype=%d sensorId=%d streamId=%d width=%d height=%d framerate=%d",
					(int)p->streamType, (int)p->sensorId, (int)p->streamId, (int)p->nWidth, (int)p->nHeight, (int)p->framerate);
				#endif
			}

			m_sensorInfo.push_back(info);
		}
	}

	return ONI_STATUS_OK;
}

OniStatus Rs2Device::addStream(OniSensorType sensorType, int sensorId, int streamId, std::vector<RIFrameInfo>* profiles)
{
	rsTraceFunc("type=%d sensorId=%d streamId=%d", (int)convertStreamType(sensorType), sensorId, streamId);

	Rs2Stream* streamObj = nullptr;
	switch (sensorType)
	{
		case ONI_SENSOR_IR: streamObj = new Rs2InfraredStream(); break;
		case ONI_SENSOR_COLOR: streamObj = new Rs2ColorStream(); break;
		case ONI_SENSOR_DEPTH: streamObj = new Rs2DepthStream(); break;

		default:
		{
			rsTraceError("Invalid type=%d", (int)sensorType);
			return ONI_STATUS_ERROR;
		}
	}

	if (streamObj->initialize(this, sensorId, streamId, profiles) != ONI_STATUS_OK)
	{
		rsTraceError("Rs2Stream::initialize failed");
		delete(streamObj);
		return ONI_STATUS_ERROR;
	}

	m_streams.push_back(streamObj);
	return ONI_STATUS_OK;
}

void Rs2Device::findStreamProfiles(std::vector<RIFrameInfo>* dst, OniSensorType sensorType, int streamId)
{
	const Mv3dRgbdImageType rsType = convertStreamType(sensorType);
	
	for (auto iter = m_profiles.begin(); iter != m_profiles.end(); ++iter)
	{
		RIFrameInfo& p = *iter;
		if (p.streamType == rsType && p.streamId == streamId)
		{
			dst->push_back(p);
		}
	}
	printf("findStreamProfiles\n");
}

Rs2Stream* Rs2Device::findStream(OniSensorType sensorType, int streamId)
{
	printf("findStream\n");
	for (auto iter = m_createdStreams.begin(); iter != m_createdStreams.end(); ++iter)
	{
		Rs2Stream* stream = *iter;
		
		//printf("stream->getOniType()= %d sensorType=%d stream->getStreamId()= %d streamId=%d\n", stream->getOniType(),sensorType, stream->getStreamId(), streamId);

		if (stream->getOniType() == sensorType && stream->getStreamId() == streamId)
		{
			printf("return stream\n");

			return stream;
		}
	}
	printf("return nullptr\n");
	return nullptr;
}

bool Rs2Device::hasEnabledStreams()
{
	for (auto iter = m_createdStreams.begin(); iter != m_createdStreams.end(); ++iter)
	{
		Rs2Stream* stream = *iter;
		if (stream->isEnabled())
		{
			return true;
		}
	}

	return false;
}

}} // namespace
