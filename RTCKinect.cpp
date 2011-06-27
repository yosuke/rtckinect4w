// -*- C++ -*-
/*!
 * @file  RTCKinect.cpp
 * @brief RTC Kinect 4 Windows
 * @date $Date$
 *
 * $Id$
 */


#include <windows.h>
#include <assert.h>
#include <avrt.h>
#include <MSR_NuiApi.h>

#include "RTCKinect.h"

// Module specification
// <rtc-template block="module_spec">
static const char* rtckinect_spec[] =
  {
    "implementation_id", "RTCKinect",
    "type_name",         "RTCKinect",
    "description",       "RTC Kinect 4 Windows",
    "version",           "1.0.0",
    "vendor",            "ysuga.net and Yosuke Matsusaka",
    "category",          "Experimental",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "0",
    "conf.default.enable_camera", "1",
    "conf.default.enable_depth", "1",
    "conf.default.enable_microphone", "1",
    "conf.default.camera_width", "640",
    "conf.default.camera_height", "480",
    "conf.default.depth_width", "320",
    "conf.default.depth_height", "240",
    "conf.default.player_index", "0",
    // Widget
    "conf.__widget__.debug", "text",
    "conf.__widget__.enable_camera", "text",
    "conf.__widget__.enable_depth", "text",
    "conf.__widget__.enable_microphone", "text",
    "conf.__widget__.camera_width", "text",
    "conf.__widget__.camera_height", "text",
    "conf.__widget__.depth_width", "text",
    "conf.__widget__.depth_height", "text",
    "conf.__widget__.player_index", "text",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
RTCKinect::RTCKinect(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_targetElevationIn("targetElevation", m_targetElevation),
    m_imageOut("image", m_image),
    m_depthOut("depth", m_depth),
    m_currentElevationOut("currentElevation", m_currentElevation),
    m_skeletonOut("skeleton", m_skeleton),
    m_soundOut("sound", m_sound),
    m_soundMonitorOut("soundMonitor", m_soundMonitor),
    m_soundSourceOut("soundSource", m_soundSource)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
RTCKinect::~RTCKinect()
{
}



RTC::ReturnCode_t RTCKinect::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("targetElevation", m_targetElevationIn);
  
  // Set OutPort buffer
  addOutPort("image", m_imageOut);
  addOutPort("depth", m_depthOut);
  addOutPort("currentElevation", m_currentElevationOut);
  addOutPort("skeleton", m_skeletonOut);
  addOutPort("sound", m_soundOut);
  addOutPort("soundMonitor", m_soundMonitorOut);
  addOutPort("soundSource", m_soundSourceOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "0");
  bindParameter("enable_camera", m_enable_camera, "1");
  bindParameter("enable_microphone", m_enable_microphone, "1");
  bindParameter("enable_depth", m_enable_depth, "1");
  bindParameter("camera_width", m_camera_width, "640");
  bindParameter("camera_height", m_camera_height, "480");
  bindParameter("depth_width", m_depth_width, "320");
  bindParameter("depth_height", m_depth_height, "240");
  bindParameter("player_index", m_player_index, "0");
  
  // </rtc-template>
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RTCKinect::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RTCKinect::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RTCKinect::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t RTCKinect::onActivated(RTC::UniqueId ec_id)
{
    /**
	 * The configured values should be reflected to the initialization process.
	 *
	 * m_enable_camera -> camera image
	 * m_enable_depth  -> depth image
	 * m_player_index  -> player index detection
	 *
	 * important: if player indexing is enabled, depth map image is limited up to 320x240
	 */

	DWORD dwFlag = NUI_INITIALIZE_FLAG_USES_SKELETON;
	if(m_enable_camera) {
		dwFlag |= NUI_INITIALIZE_FLAG_USES_COLOR;
	}
	if(m_enable_depth) {
		if(m_player_index) {
			dwFlag |= NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX;
		} else {
			dwFlag |= NUI_INITIALIZE_FLAG_USES_DEPTH;
		}
	}


	HRESULT hr = NuiInitialize(dwFlag); 
    if( FAILED( hr ) ) {
		std::cout << "NUI Initialize Failed." << std::endl;
		return RTC::RTC_ERROR;
    }

	if(m_depth_width == 640 && m_depth_height == 480 && m_enable_depth && m_player_index) {
		std::cout << "If PlayerIndex and Depth Map is ON, resolution should be 320X240" << std::endl;
		return RTC::RTC_ERROR;
	}
	NUI_IMAGE_RESOLUTION eResolution;
	if(m_camera_width == 640 && m_camera_height == 480) {
		eResolution = NUI_IMAGE_RESOLUTION_640x480;
	} else {
		std::cout << "Invalid Image Resolution" << std::endl;
		return RTC::RTC_ERROR;
	}
	if(m_enable_camera) {
		hr = NuiImageStreamOpen(::NUI_IMAGE_TYPE_COLOR, eResolution, 0, 2, NULL, &m_pVideoStreamHandle );
		if( FAILED( hr ) )
		{
			std::cout << "NUI Image Stream Open Failed." << std::endl;
			return RTC::RTC_ERROR;
		}
	}

	if(m_depth_width == 640 && m_depth_height == 480) {
		eResolution = NUI_IMAGE_RESOLUTION_640x480;
	} else if(m_depth_width == 320 && m_depth_height == 240) {
		eResolution = NUI_IMAGE_RESOLUTION_320x240;
	} else {
		std::cout << "Invalid Image Resolution" << std::endl;
		return RTC::RTC_ERROR;
	}
	if(m_enable_depth) {
		if(m_player_index) {
			hr = NuiImageStreamOpen(::NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, eResolution, 0, 2, NULL, &m_pDepthStreamHandle );
		} else {
			hr = NuiImageStreamOpen(::NUI_IMAGE_TYPE_DEPTH, eResolution, 0, 2, NULL, &m_pDepthStreamHandle );
		}
	}
    if( FAILED( hr ) ) {
		std::cout << "NUI Image Stream Open Failed." << std::endl;
		return RTC::RTC_ERROR;
    }

	this->m_image.width = m_camera_width;
	this->m_image.height = m_camera_height;
	this->m_image.pixels.length(m_camera_width*m_camera_height*3);

	this->m_depth.width = m_depth_width;
	this->m_depth.height = m_depth_height;
	this->m_depth.pixels.length(m_depth_width*m_depth_height*3);

    /**
	 * Initialization for raw sound input.
	 */
	if (m_enable_microphone) {
		UINT deviceCount;
		IMMDeviceEnumerator *deviceEnumerator = NULL;
		IMMDeviceCollection *deviceCollection = NULL;

		hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), NULL, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&deviceEnumerator));
		if (FAILED(hr)) {
			std::cout << "Unable to instantiate device enumerator." << std::endl;
			return RTC::RTC_ERROR;
		}
		hr = deviceEnumerator->EnumAudioEndpoints(eCapture, DEVICE_STATE_ACTIVE, &deviceCollection);
		if (FAILED(hr)) {
			std::cout << "Unable to retrieve device collection." << std::endl;
			return RTC::RTC_ERROR;
		}
		hr = deviceCollection->GetCount(&deviceCount);
		if (FAILED(hr)) {
			std::cout << "Unable to get device collection length." << std::endl;
			return RTC::RTC_ERROR;
		}
		for (UINT i = 0; i < deviceCount; i++) {
			IPropertyStore *propertyStore;
			PROPVARIANT friendlyName;
			IMMDevice *endpoint;
			PropVariantInit(&friendlyName);

			hr = deviceCollection->Item(i, &endpoint);
			if (FAILED(hr)) {
				std::cout << "Unable to get device collection item." << std::endl;
				return RTC::RTC_ERROR;
			}

			hr = endpoint->OpenPropertyStore(STGM_READ, &propertyStore);
			if (FAILED(hr)) {
				std::cout << "Unable to open device property store." << std::endl;
				return RTC::RTC_ERROR;
			}

			hr = propertyStore->GetValue(PKEY_Device_FriendlyName, &friendlyName);
			SafeRelease(&propertyStore);
			if (FAILED(hr)) {
				std::cout << "Unable to retrieve friendly name for device." << std::endl;
				return RTC::RTC_ERROR;
			}

			std::cout << "Scanning for Kinect Audio device..." << std::endl;
			if (friendlyName.vt == VT_LPWSTR) {
				wprintf(L"  %s\n", friendlyName.pwszVal);
				if (wcscmp(friendlyName.pwszVal, L"Kinect USB Audio") != 0) {
					std::cout << "  Found Kinect Audio device" << std::endl;
					m_pAudioEndpoint = endpoint;
					m_pAudioEndpoint->AddRef();
					PropVariantClear(&friendlyName);
					SafeRelease(&endpoint);
					break;
				}
			}
			PropVariantClear(&friendlyName);
			SafeRelease(&endpoint);
		}
		SafeRelease(&deviceCollection);
		SafeRelease(&deviceEnumerator);

		m_AudioShutdownEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
		if (m_AudioShutdownEvent == NULL) {
			std::cout << "Unable to create shutdown event." << std::endl;
			return RTC::RTC_ERROR;
		}    
		hr = m_pAudioEndpoint->Activate(__uuidof(IAudioClient), CLSCTX_INPROC_SERVER, NULL, reinterpret_cast<void **>(&m_pAudioClient));
		if (FAILED(hr)) {
			std::cout << "Unable to activate audio client." << std::endl;
			return RTC::RTC_ERROR;
		}
		hr = m_pAudioClient->GetMixFormat(&m_pAudioMixFormat);
		if (FAILED(hr)) {
			std::cout << "Unable to get mix format on audio client." << std::endl;
			return RTC::RTC_ERROR;
		}
	    m_AudioFrameSize = (m_pAudioMixFormat->wBitsPerSample / 8) * m_pAudioMixFormat->nChannels;
		m_AudioCaptureBufferSize = m_pAudioMixFormat->nSamplesPerSec * 5 * m_AudioFrameSize;
		m_pAudioCaptureBuffer = new (std::nothrow) BYTE[m_AudioCaptureBufferSize];
		if (m_pAudioCaptureBuffer == NULL) {
			std::cout << "Unable to allocate capture buffer." << std::endl;
			return RTC::RTC_ERROR;
		}
		m_AudioCurrentCaptureIndex = 0;
		std::cout << "Audio capture format (" << m_pAudioMixFormat->nChannels << " channels, " << m_pAudioMixFormat->wBitsPerSample << " bits)"<< std::endl;
		m_AudioLatency = 10;
		hr = m_pAudioClient->Initialize(AUDCLNT_SHAREMODE_SHARED, AUDCLNT_STREAMFLAGS_NOPERSIST, m_AudioLatency*10000, 0, m_pAudioMixFormat, NULL);
		if (FAILED(hr)) {
			std::cout << "Unable to initialize audio client." << std::endl;
			return RTC::RTC_ERROR;
		}
		hr = m_pAudioClient->GetService(IID_PPV_ARGS(&m_pAudioCaptureClient));
		if (FAILED(hr)) {
			std::cout << "Unable to get audio capture client." << std::endl;
			return RTC::RTC_ERROR;
		}
		m_AudioCaptureThread = CreateThread(NULL, 0, AudioCaptureThread, this, 0, NULL);
		if (m_AudioCaptureThread == NULL) {
			std::cout << "Unable to create transport thread: " << GetLastError() << std::endl;
			return RTC::RTC_ERROR;
		}
		hr = m_pAudioClient->Start();
		if (FAILED(hr)) {
			std::cout << "Unable to start audio capture client." << std::endl;
			return RTC::RTC_ERROR;
		}
	}

	return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCKinect::onDeactivated(RTC::UniqueId ec_id)
{
	NuiShutdown( );
	if (m_enable_microphone) {
		HRESULT hr;
		if (m_AudioShutdownEvent) {
			SetEvent(m_AudioShutdownEvent);
		}
		hr = m_pAudioClient->Stop();
		if (FAILED(hr)) {
			std::cout << "Unable to stop audio client." << std::endl;
		}
		if (m_AudioCaptureThread) {
			WaitForSingleObject(m_AudioCaptureThread, INFINITE);
			CloseHandle(m_AudioCaptureThread);
			m_AudioCaptureThread = NULL;
		}
		SafeRelease(&m_pAudioEndpoint);
		SafeRelease(&m_pAudioClient);
		SafeRelease(&m_pAudioCaptureClient);
		CoTaskMemFree(m_pAudioMixFormat);
		delete [] m_pAudioCaptureBuffer;
	}
	return RTC::RTC_OK;
}


/**
 * Writing camera image to image port
 *
 * Color space conversion should be accelerated
 * (OpenCV or Intel Performance Primitive Library 
 *  would be useful for this case. OpenMP will 
 * also useful for this kind of processing)
 *
 *
 * Aquired Image by NUI     -> BGRW (4 bytes)
 * RTC::CameraImage struct  -> BGR (3 bytes)
 *
 */

HRESULT RTCKinect::WriteColorImage(void)
{
	static const long TIMEOUT_IN_MILLI = 100;
	const NUI_IMAGE_FRAME * pImageFrame = NULL;
    HRESULT hr = NuiImageStreamGetNextFrame(m_pVideoStreamHandle, TIMEOUT_IN_MILLI, &pImageFrame );
    if( FAILED( hr ) ) {
		std::cout << "NuiImageStreamGetNextFrame failed." << std::endl;
		return hr;
    }

    NuiImageBuffer * pTexture = pImageFrame->pFrameTexture;
    KINECT_LOCKED_RECT LockedRect;
    pTexture->LockRect( 0, &LockedRect, NULL, 0 );
    if( LockedRect.Pitch != 0 )
    {
        BYTE * pBuffer = (BYTE*) LockedRect.pBits;
		for(int h = 0;h < m_camera_height;h++) {
			for(int w = 0;w < m_camera_width;w++) {
				BYTE* pixel = pBuffer + (h * m_camera_width * 4) + w * 4;
				BYTE b = pixel[0];
				BYTE g = pixel[1];
				BYTE r = pixel[2];
				int offset = h*m_camera_width*3+w*3;
				m_image.pixels[offset + 0] = b;
				m_image.pixels[offset + 1] = g;
				m_image.pixels[offset + 2] = r;
			}
			m_imageOut.write();
		}

    }
    else {
		std::cout << "Buffer length of received texture is bogus\r\n" << std::endl;
    }

    NuiImageStreamReleaseFrame( m_pVideoStreamHandle, pImageFrame );

	return S_OK;
}

/**
 * Writing Depth Image to the Outport
 *
 * In this release the depth map is converted to the gray scale image.
 * But in the future, the depth map should have its own data type, 
 * because the aquired data includes not only depth data [mm]
 * but also the tracking user id!
 *
 */
HRESULT RTCKinect::WriteDepthImage(void)
{
	static const long TIMEOUT_IN_MILLI = 100;
	const NUI_IMAGE_FRAME * pImageFrame = NULL;
    HRESULT hr = NuiImageStreamGetNextFrame(m_pDepthStreamHandle, TIMEOUT_IN_MILLI, &pImageFrame );
    if( FAILED( hr ) ) {
		std::cout << "NuiImageStreamGetNextFrame failed." << std::endl;
		return hr;
    }

    NuiImageBuffer * pTexture = pImageFrame->pFrameTexture;
    KINECT_LOCKED_RECT LockedRect;
    pTexture->LockRect( 0, &LockedRect, NULL, 0 );
    if( LockedRect.Pitch != 0 )
    {
        BYTE * pBuffer = (BYTE*) LockedRect.pBits;
		for(int h = 0;h < m_depth_height;h++) {
			for(int w = 0;w < m_depth_width;w++) {
				USHORT* pixel = (USHORT*)(pBuffer + (h * m_depth_width * sizeof(USHORT)) + w * sizeof(USHORT));

				USHORT RealDepth = (*pixel & 0xfff8) >> 3;
				USHORT Player = *pixel & 7;

				// transform 13-bit depth information into an 8-bit intensity appropriate
				// for display (we disregard information in most significant bit)
				BYTE depth = 255 - (BYTE)(256*RealDepth/0x0fff);

				unsigned char r, g, b;
				r=g=b = depth/2;
				int offset = h*m_depth_width*3+w*3;
				m_depth.pixels[offset + 0] = b;
				m_depth.pixels[offset + 1] = g;
				m_depth.pixels[offset + 2] = r;
			}
			m_depthOut.write();
		}

    }
    else {
		std::cout << "Buffer length of received texture is bogus\r\n" << std::endl;
    }

    NuiImageStreamReleaseFrame( m_pDepthStreamHandle, pImageFrame );

	return S_OK;
}


/**
 * Controlling Elevation Motor
 */
HRESULT RTCKinect::WriteElevation()
{
	HRESULT hr;
	LONG angle;
	if(m_targetElevationIn.isNew()) {
		hr = NuiCameraElevationSetAngle(m_targetElevation.data);
		if( FAILED(hr) ) {
			return hr;
		}
	}
	hr = NuiCameraElevationGetAngle(&angle);
	if( FAILED(hr) ) {
		return hr;
	}
	m_currentElevation.data = angle;
	m_currentElevationOut.write();
	return S_OK;
}

void operator<<=(Kinect::Vector4& dst, ::Vector4& src) {
	dst.v[0] = src.v[0];
	dst.v[1] = src.v[1];
	dst.v[2] = src.v[2];
	dst.v[3] = src.v[3];
}
/*
struct _NUI_SKELETON_DATA {
    NUI_SKELETON_TRACKING_STATE eTrackingState;
    DWORD dwTrackingID;
    DWORD dwEnrollmentIndex;
    DWORD dwUserIndex;
    Vector4 Position;
    Vector4 SkeletonPositions[NUI_SKELETON_POSITION_COUNT];
    NUI_SKELETON_POSITION_TRACKING_STATE 
        eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_COUNT];
    DWORD dwQualityFlags;
*/
void operator<<=(Kinect::NuiSkeletonData& dst, ::NUI_SKELETON_DATA& src) {
	dst.trackingID = src.dwTrackingID;
	dst.trackingState = (Kinect::NUI_SKELETON_TRACKING_STATE)src.eTrackingState;
	dst.enrollmentIndex = src.dwEnrollmentIndex;
	dst.userIndex = src.dwUserIndex;
	dst.position <<= src.Position;
	for(int i = 0;i < 20;i++) {
		dst.skeletonPositions[i] <<= src.SkeletonPositions[i];
		dst.eSkeletonPositionTrackingState[i]  = (Kinect::NUI_SKELETON_POSITION_TRACKING_STATE)src.eSkeletonPositionTrackingState[i];
	}
	dst.qualityFlags = src.dwQualityFlags;
}


/**
 *
 */
HRESULT RTCKinect::WriteSkeleton()
{
	static const long TIMEOUT_IN_MILI = 100;

	NUI_SKELETON_FRAME skeletonFrame;
	HRESULT ret = NuiSkeletonGetNextFrame(TIMEOUT_IN_MILI, & skeletonFrame);
	if(FAILED(ret)) {
		return ret;
	}

	/*
	    LARGE_INTEGER liTimeStamp;
    DWORD dwFrameNumber;
    DWORD dwFlags;
    Vector4 vFloorClipPlane;
    Vector4 vNormalToGravity;
    NUI_SKELETON_DATA SkeletonData[NUI_SKELETON_COUNT];
} NUI_SKELETON_FRAME;
   */
	m_skeleton.frameNumber = (long)skeletonFrame.dwFrameNumber;
	//m_skeleton.timeStamp   = (long)skeletonFrame.liTimeStamp;
	m_skeleton.flags       = skeletonFrame.dwFlags;
	m_skeleton.floorClipPlane <<= skeletonFrame.vFloorClipPlane;
	m_skeleton.normalToGravity <<= skeletonFrame.vNormalToGravity;

	// int NUI_SKELETON_COUNT=6; Undefined?
	m_skeleton.skeletonData.length(6);
	for(int i = 0;i < 6;i++) {
		m_skeleton.skeletonData[i] <<= skeletonFrame.SkeletonData[i];
	}
	m_skeletonOut.write();
	
	return S_OK;
}

/**
 *
 */
HRESULT RTCKinect::WriteRawSound()
{
	m_AudioBufferMutex.lock();
	size_t bsize = m_AudioCurrentCaptureIndex;
	size_t nsamples = bsize / m_AudioFrameSize;
#if 0
	std::cout << nsamples << std::endl;
#endif
	if (bsize > 0) {
		m_sound.data.length(bsize);
        CopyMemory(&m_sound.data[0], m_pAudioCaptureBuffer, bsize);
		m_soundMonitor.data.length(nsamples * 2);
		for (size_t i = 0; i < nsamples; i++) {
#if 0
            INT32 sample = (INT32)(m_pAudioCaptureBuffer[i * m_AudioFrameSize + 0]
							    + (m_pAudioCaptureBuffer[i * m_AudioFrameSize + 1] << 8)
						     	+ (m_pAudioCaptureBuffer[i * m_AudioFrameSize + 2] << 16)
								+ (m_pAudioCaptureBuffer[i * m_AudioFrameSize + 3] << 24));
			INT16 sample16 = (INT16)(sample / 65536);
			m_soundMonitor.data[i*2+0] = (BYTE)(0xff & sample16);
			m_soundMonitor.data[i*2+1] = (BYTE)(0xff & (sample16>>8));
#endif
			m_soundMonitor.data[i*2+0] = m_pAudioCaptureBuffer[i * m_AudioFrameSize + 2];
			m_soundMonitor.data[i*2+1] = m_pAudioCaptureBuffer[i * m_AudioFrameSize + 3];
		}
    }
	m_AudioCurrentCaptureIndex = 0;
	m_AudioBufferMutex.unlock();
	if (bsize > 0) {
		m_soundOut.write();
		m_soundMonitorOut.write();
    }
	return S_OK;
}

RTC::ReturnCode_t RTCKinect::onExecute(RTC::UniqueId ec_id)
{
	if(m_enable_camera) {
		if( FAILED(WriteColorImage())) {
			return RTC::RTC_ERROR;
		}
	}

	if(m_enable_depth) {
		if( FAILED(WriteDepthImage())) {
			return RTC::RTC_ERROR;
		}
	}

	if( FAILED(WriteElevation()) ) {
		return RTC::RTC_ERROR;
	}

	if( FAILED(WriteSkeleton()) ) {
		return RTC::RTC_ERROR;
	}

	if(m_enable_microphone) {
		if( FAILED(WriteRawSound())) {
			return RTC::RTC_ERROR;
		}
	}

	return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RTCKinect::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RTCKinect::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t RTCKinect::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RTCKinect::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RTCKinect::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

//
//  Capture thread - processes samples from the audio engine
//
DWORD RTCKinect::AudioCaptureThread(LPVOID Context)
{
    RTCKinect *capturer = static_cast<RTCKinect *>(Context);
    return capturer->DoAudioCaptureThread();
}

DWORD RTCKinect::DoAudioCaptureThread()
{
    bool loop = true;
    HANDLE mmcssHandle = NULL;
    DWORD mmcssTaskIndex = 0;

	// create COM client
	HRESULT hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);
    if (FAILED(hr)) {
		std::cout << "Unable to initialize COM in render thread." << std::endl;
        return hr;
    }

	// give multimedia priority to this thread
	mmcssHandle = AvSetMmThreadCharacteristics("Audio", &mmcssTaskIndex);
    if (mmcssHandle == NULL) {
		std::cout << "Unable to enable MMCSS on capture thread: " << GetLastError() << std::endl;
    }
    
	std::cout << "Starting audio capture thread." << std::endl;

    while (loop) {
        HRESULT hr;
		DWORD waitResult = WaitForSingleObject(m_AudioShutdownEvent, m_AudioLatency / 2);
        switch (waitResult) {
        case WAIT_OBJECT_0:
            loop = false;
            break;
        case WAIT_TIMEOUT:
            BYTE *pData;
            UINT32 framesAvailable;
            DWORD  flags;

			hr = m_pAudioCaptureClient->GetBuffer(&pData, &framesAvailable, &flags, NULL, NULL);
			if (SUCCEEDED(hr)) {
				m_AudioBufferMutex.lock();
				UINT32 framesToCopy = min(framesAvailable, static_cast<UINT32>((m_AudioCaptureBufferSize - m_AudioCurrentCaptureIndex) / m_AudioFrameSize));
				if (framesToCopy != 0) {
					if (flags & AUDCLNT_BUFFERFLAGS_SILENT) {
	                    ZeroMemory(&(m_pAudioCaptureBuffer[m_AudioCurrentCaptureIndex]), framesToCopy * m_AudioFrameSize);
					} else {
	                    CopyMemory(&(m_pAudioCaptureBuffer[m_AudioCurrentCaptureIndex]), pData, framesToCopy * m_AudioFrameSize);
					}
					m_AudioCurrentCaptureIndex += framesToCopy * m_AudioFrameSize;
				}
                hr = m_pAudioCaptureClient->ReleaseBuffer(framesAvailable);
                if (FAILED(hr)) {
					std::cout << "Unable to release audio capture buffer." << std::endl;
                }
				m_AudioBufferMutex.unlock();
			}
            break;
        }
    }
    
	std::cout << "Exiting audio capture thread." << std::endl;

	AvRevertMmThreadCharacteristics(mmcssHandle);
    CoUninitialize();
    return 0;
}



extern "C"
{
 
  void RTCKinectInit(RTC::Manager* manager)
  {
    coil::Properties profile(rtckinect_spec);
    manager->registerFactory(profile,
                             RTC::Create<RTCKinect>,
                             RTC::Delete<RTCKinect>);
  }
  
};


