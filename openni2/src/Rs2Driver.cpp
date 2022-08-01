#include "Rs2Driver.h"

#if defined(PROF_ENABLED)
#include "Profiler.inl"
#endif

namespace oni { namespace driver {

Rs2Driver::Rs2Driver(OniDriverServices* driverServices)
:
	DriverBase(driverServices)
	//m_context(nullptr)
{
	rsLogDebug("+Rs2Driver");
	INIT_PROFILER;
}

Rs2Driver::~Rs2Driver()
{
	rsLogDebug("~Rs2Driver");
	shutdown();
	SHUT_PROFILER;
}

OniStatus Rs2Driver::initialize(
	DeviceConnectedCallback connectedCallback, 
	DeviceDisconnectedCallback disconnectedCallback, 
	DeviceStateChangedCallback deviceStateChangedCallback, 
	void* cookie)
{
	rsTraceFunc("");
	
	for (;;)
	{
		
		Rs2ScopedMutex lock(m_stateMx);
		/*
		if (m_context)
		{
			rsTraceError("Already initialized");
			break;
		}*/

		if (DriverBase::initialize(connectedCallback, disconnectedCallback, deviceStateChangedCallback, cookie) != ONI_STATUS_OK)
		{
			rsTraceError("DriverBase::initialize failed");
			break;
		}
		
		MV3D_RGBD_VERSION_INFO stVersion;
		MV3D_RGBD_GetSDKVersion(&stVersion);
		
		
		//MV3D_RGBD_OK
		/*
		Rs2Error e;
		struct rs2_context* m_context = rs2_create_context(RS2_API_VERSION, &e);
		
		if (!e.success())
		{
			rsTraceError("rs2_create_context failed: %s", e.get_message());
			break;
		}
		*/
		MV3D_RGBD_Initialize();
		enumerateDevices();
		
		//std::vector<MV3D_RGBD_DEVICE_INFO> devs(nDevNum);

		//容量必须给至少是nDevNum值
		//MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB, &devs[0], nDevNum, &nDevNum);
		//printf("2222222222222222222222");
		devicesChangedCallback(nullptr, &devs,this);
		//printf("1111111111111111111111");
			/*
		rs2_set_devices_changed_callback(m_context, devicesChangedCallback, this, &e);
		if (!e.success())
		{
			rsTraceError("rs2_set_devices_changed_callback failed: %s", e.get_message());
			break;
		}
		*/
		
		

		rsLogDebug("Rs2Driver INITIALIZED");
		return ONI_STATUS_OK;
	}

	shutdown();
	return ONI_STATUS_ERROR;
}

void Rs2Driver::shutdown()
{
	//if (m_context) { rsTraceFunc(""); }

	Rs2ScopedMutex lock(m_stateMx);

	{
		Rs2ScopedMutex devLock(m_devicesMx);
		for (auto iter = m_devices.begin(); iter != m_devices.end(); ++iter) { delete(iter->second); }
		m_devices.clear();
	}
	/*
	if (m_context)
	{
		rs2_delete_context(m_context);
		m_context = nullptr;
	}
	*/
}


void Rs2Driver::enumerateDevices()
{
	rsTraceFunc("");
	nDevNum = 0;
	//Rs2Error e;
	//printf("nDevNum %d---------------\n", nDevNum);
	MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB, &nDevNum);
	//printf("initial sizeof(devs) %d nDevNum=%d\n", devs.size(), nDevNum);
	devs.resize(nDevNum);
	//printf("initial sizeof(devs) %d nDevNum=%d\n", devs.size(), nDevNum);
	//容量必须给至少是nDevNum值
	MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB, &devs[0], nDevNum, &nDevNum);
	//printf("nDevNum %d+++++++++++++++\n", nDevNum);
	/*
	for (unsigned int i = 0; i < nDevNum; i++)
	{
		//printf("Index[%d]. SerialNum[%s] IP[%s] userdefinename[%s] chmodename[%s].\r\n", i, devs[i].chSerialNumber, devs[i].SpecialInfo.stNetInfo.chCurrentIp, devs[i].chUserDefinedName, devs[i].chModelName);
	}*/
	//rs2_device_list* deviceList = rs2_query_devices(m_context, &e);
	/*
	if (!e.success())
	{
		rsTraceError("rs2_query_devices failed: %s", e.get_message());
	}
	else
	{*/
		devicesChanged(nullptr, &devs);
		//printf("devicesChanged----------------\n");
		//rs2_delete_device_list(devs);
		
		//delete(&devs);
	//}
		//system("pause");
		
}

void Rs2Driver::devicesChangedCallback(std::vector<MV3D_RGBD_DEVICE_INFO>* removed, std::vector<MV3D_RGBD_DEVICE_INFO>* added, void* param)
{
	((Rs2Driver*)param)->devicesChanged(removed, added);
}

void Rs2Driver::devicesChanged(std::vector<MV3D_RGBD_DEVICE_INFO>* removed, std::vector<MV3D_RGBD_DEVICE_INFO>* added)
{
	rsTraceFunc("-----removed=%p ****added=%p", removed, added);
	//std::vector<MV3D_RGBD_DEVICE_INFO> deviceList(nDevNum);
	

	memcpy(&devs, added, devs.size());
	//printf("sizeof(devs) %d \n", devs.size());
	//std::vector<MV3D_RGBD_DEVICE_INFO> deviceListed(nDevNum);
	//MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB, &deviceListed[0], 0, &nDevNum);

	Rs2ScopedMutex lock(m_devicesMx);
	//Rs2Error e;
	
	if (removed)
	{
		//rsLogDebug("removed****************");
		std::list<Rs2Device*> removedList;

		for (auto iter = m_devices.begin(); iter != m_devices.end(); ++iter)
		{
			//查找是否在removed中已包含一个device
			/*
			if (rs2_device_list_contains(removed, iter->second->getRsDevice(), &e))
			{
				removedList.push_back(iter->second);
			}
			*/
			MV3D_RGBD_DEVICE_INFO* device1 = iter->second->getRsDevice();
			for (int i = 0; i < removed->size(); ++i)
			{
				MV3D_RGBD_DEVICE_INFO* device2;
				memcpy(device2,&(removed[i]),sizeof(*device2));
				if (device1 == device2) {
					removedList.push_back(iter->second);
				}
				delete device2;

			}
			delete device1;
			
		}

		for (auto iter = removedList.begin(); iter != removedList.end(); ++iter)
		{
			rsLogDebug("deviceDisconnected1");
			deviceDisconnected((*iter)->getInfo());
		}
	}
	
	
	if (added)
	{
		const int count = added->size();
		
		for (int i = 0; i < count; i++)
		{
			MV3D_RGBD_DEVICE_INFO device;
			memcpy(&device, &devs[i], sizeof(device));
			//rs2_device* device = rs2_create_device(added, i, &e);
			/*
			if (!e.success())
			{
				rsTraceError("rs2_create_device failed: %s", e.get_message());
			}
			else
			{
				*/
				//rsLogDebug("deviceConnected2");
				
				OniDeviceInfo info;
				Rs2Device::queryDeviceInfo(&device, &info);
				//printf("查询结束\n");
				deviceConnected(&info);
				//printf("deviceconnected\n");
				/*
				rs2_delete_device(device);
			}*/
		}
	}
	
	//printf("函数运行结束\n");
}


DeviceBase* Rs2Driver::deviceOpen(const char* uri, const char* mode)
{
	rsTraceFunc("uri=%s", uri);

	//std::vector<MV3D_RGBD_DEVICE_INFO> deviceList(nDevNum);
	MV3D_RGBD_DEVICE_INFO device;
	Rs2Device* deviceObj = nullptr;

	MV3D_RGBD_OpenDevice(&handle, &devs[nDevNum - 1]);
	
	MV3D_RGBD_Start(handle);

	
	for (;;)
	{
		Rs2ScopedMutex lock(m_devicesMx);

		if (m_devices.find(uri) != m_devices.end())
		{
			rsTraceError("Already opened");
			break;
		}
		
		//Rs2Error e;
		//MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB, &devs[0], 0, &nDevNum);
		/*
		//deviceList = rs2_query_devices(m_context, &e);
		if (!e.success())
		{
			rsTraceError("rs2_query_devices failed: %s", e.get_message());
			break;
		}
		*/

		//const int count = rs2_get_device_count(deviceList, &e);
		const int count = nDevNum;
		for (int i = 0; i < count; i++)
		{
			memcpy(&device, &devs[i],sizeof(device));
			printf("devs[%d]serial= %s\n",i+1, devs[i].chSerialNumber);
			
			//****
			//MV3D_RGBD_OpenDevice(&handle, &deviceList[i]);

			//if (device) { rs2_delete_device(device); }
			
			/*
			device = rs2_create_device(deviceList, i, &e);
			if (!e.success())
			{
				rsTraceError("rs2_create_device failed: %s", e.get_message());
				break;
			}*/
			const char* serial = device.chSerialNumber;
			printf("serial= %s\n", serial);
			/*
			const char* serial = rs2_get_device_info(device, RS2_CAMERA_INFO_SERIAL_NUMBER, &e);
			if (!e.success())
			{
				rsTraceError("rs2_get_device_info failed: %s", e.get_message());
				break;
			}*/
			
			if (strcmp(uri, serial) == 0)
			{
				deviceObj = new Rs2Device(this, &device);
				if (deviceObj->initialize() != ONI_STATUS_OK)
				{
					rsTraceError("Rs2Device::initialize failed");
					delete(deviceObj);
					deviceObj = nullptr;
				}
				else
				{
					m_devices[serial] = deviceObj;
					//device = NULL; // don't release device handle
				}
				break;
			}
		}

		break;
	}

	//if (device) { rs2_delete_device(device); }
	//if (deviceList) { rs2_delete_device_list(deviceList); }

	return deviceObj;
}

void Rs2Driver::deviceClose(DeviceBase* deviceBase)
{
	rsTraceFunc("ptr=%p", deviceBase);

	if (deviceBase)
	{
		Rs2ScopedMutex lock(m_devicesMx);

		Rs2Device* deviceObj = (Rs2Device*)deviceBase;
		m_devices.erase(deviceObj->getInfo()->uri);
		delete(deviceObj);
	}
}

OniStatus Rs2Driver::tryDevice(const char* uri)
{
	rsTraceFunc("uri=%s", uri);

	return ONI_STATUS_ERROR;
}

void* Rs2Driver::enableFrameSync(StreamBase** streams, int streamCount)
{
	rsTraceError("FrameSync not supported");

	return nullptr;
}

void Rs2Driver::disableFrameSync(void* frameSyncGroup)
{
	rsTraceError("FrameSync not supported");
}

#if !defined(XN_NEW)
#define XN_NEW(type, arg) new type(arg)
#endif

#if !defined(XN_DELETE)
#define XN_DELETE(arg) delete arg
#endif

ONI_EXPORT_DRIVER(Rs2Driver);

}} // namespace
