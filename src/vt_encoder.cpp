//
// Created by sunchongyang on 2021/12/29.
//

#include <libyuv/convert.h>
#include "vt_encoder.h"
#include "glog/logging.h"

#define START_CODE_SIZE 4
static uint8_t start_code[] = {0x00, 0x00, 0x00, 0x01};

static uint32_t ReverseBytes(uint32_t value) {
  return (value & 0x000000FFU) << 24 | (value & 0x0000FF00U) << 8 |
      (value & 0x00FF0000U) >> 8 | (value & 0xFF000000U) >> 24;
}

static void PrintData (uint8_t* data, int data_len) {
  for (int i = 0; i < data_len; i++) {
    printf("0x%02x ", data[i]);
    if ((i+1) % 16 == 0) {
      printf("\n");
    }
  }
  printf("\n");
}

static void videotoolbox_encoder_callback(
  void *outputCallbackRefCon,
  void *sourceFrameRefCon,
  OSStatus status,
  VTEncodeInfoFlags infoFlags,
  CMSampleBufferRef sampleBuffer
  ) {
//  LOG(INFO) << "didCompressH264 called with status " << status << " infoFlags " << infoFlags;
  if (status != 0) {
    return;
  }

  if (!CMSampleBufferDataIsReady(sampleBuffer)) {
    LOG(INFO) << "didCompressH264 data is not ready ";
    return;
  }

  auto encoder = (VTEncoder*)outputCallbackRefCon;
  CFDictionaryRef dict = static_cast<CFDictionaryRef>(CFArrayGetValueAtIndex(CMSampleBufferGetSampleAttachmentsArray(sampleBuffer, true),0));
  bool keyframe = !CFDictionaryContainsKey(dict, kCMSampleAttachmentKey_NotSync);
  // 判断当前帧是否为关键帧
  // 获取sps & pps数据
  if (keyframe) {
    CMFormatDescriptionRef format = CMSampleBufferGetFormatDescription(sampleBuffer);
    size_t sparameterSetSize, sparameterSetCount;
    const uint8_t* sparameterSet;
    size_t pparameterSetSize, pparameterSetCount;
    const uint8_t* pparameterSet;
    // Found sps pps and check for sps pps
    OSStatus statusCode1 = CMVideoFormatDescriptionGetH264ParameterSetAtIndex(format, 0, &sparameterSet, &sparameterSetSize, &sparameterSetCount, 0 );
    OSStatus statusCode2 = CMVideoFormatDescriptionGetH264ParameterSetAtIndex(format, 1, &pparameterSet, &pparameterSetSize, &pparameterSetCount, 0 );
    if (statusCode1 == noErr && statusCode2 == noErr && encoder) {
      uint8_t* sps = const_cast<uint8_t *>(sparameterSet);
      uint8_t* pps = const_cast<uint8_t *>(pparameterSet);
      encoder->GetSpsPps(sps, sparameterSetSize, pps, pparameterSetSize);
    }
  }

  CMBlockBufferRef dataBuffer = CMSampleBufferGetDataBuffer(sampleBuffer);
  size_t length, totalLength;
  char* dataPointer;
  static int frame_count = 0;
  OSStatus statusCodeRet = CMBlockBufferGetDataPointer(dataBuffer, 0, &length, &totalLength, &dataPointer);
  if (statusCodeRet == noErr) {
    size_t bufferOffset = 0;
    bool first_data = true;
    // 循环获取nalu数据
    while (bufferOffset < totalLength - START_CODE_SIZE) {
      uint32_t NALUnitLength = 0;
      // Read the NAL unit length
      memcpy(&NALUnitLength, dataPointer + bufferOffset, START_CODE_SIZE);

      // 从大端转系统端
      NALUnitLength = ReverseBytes(NALUnitLength);
//      LOG(INFO) << "NALUnitLength: " <<NALUnitLength;
      if (keyframe && first_data) {
        first_data = false;
      } else {
        encoder->CreatePacket(reinterpret_cast<uint8_t *>(dataPointer + bufferOffset + START_CODE_SIZE), NALUnitLength, frame_count * 40, keyframe);
        frame_count++;
      }
      // Move to the next NAL unit in the block buffer
      bufferOffset += START_CODE_SIZE + NALUnitLength;

    }
  }
//  CFRelease(dataBuffer);
}

static CFDictionaryRef videotoolbox_buffer_attributes_create(
    int width,
    int height,
    OSType pix_fmt) {
  CFMutableDictionaryRef buffer_attributes;
  CFMutableDictionaryRef io_surface_properties;
  CFNumberRef cv_pix_fmt;
  CFNumberRef w;
  CFNumberRef h;

  w = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &width);
  h = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &height);
  cv_pix_fmt = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &pix_fmt);

  buffer_attributes = CFDictionaryCreateMutable(kCFAllocatorDefault,
                                                4,
                                                &kCFTypeDictionaryKeyCallBacks,
                                                &kCFTypeDictionaryValueCallBacks);
  io_surface_properties = CFDictionaryCreateMutable(kCFAllocatorDefault,
                                                    0,
                                                    &kCFTypeDictionaryKeyCallBacks,
                                                    &kCFTypeDictionaryValueCallBacks);

  if (pix_fmt) {
    CFDictionarySetValue(buffer_attributes, kCVPixelBufferPixelFormatTypeKey, cv_pix_fmt);
  }
  CFDictionarySetValue(buffer_attributes, kCVPixelBufferIOSurfacePropertiesKey, io_surface_properties);//linesize?
  CFDictionarySetValue(buffer_attributes, kCVPixelBufferWidthKey, w);
  CFDictionarySetValue(buffer_attributes, kCVPixelBufferHeightKey, h);
#if TARGET_OS_IPHONE
  CFDictionarySetValue(buffer_attributes, kCVPixelBufferOpenGLESCompatibilityKey, kCFBooleanTrue);
#else
  CFDictionarySetValue(buffer_attributes, kCVPixelBufferIOSurfaceOpenGLTextureCompatibilityKey, kCFBooleanTrue);
#endif

  CFRelease(io_surface_properties);
  CFRelease(cv_pix_fmt);
  CFRelease(w);
  CFRelease(h);

  return buffer_attributes;
}

static CMSampleBufferRef Packet2SampleBuffer(
    CMFormatDescriptionRef fmt_desc,
    AVPacket *packet) {
  OSStatus status;
  CMBlockBufferRef  block_buf;
  CMSampleBufferRef sample_buf;

  block_buf  = nullptr;
  sample_buf = nullptr;

  status = CMBlockBufferCreateWithMemoryBlock(
      kCFAllocatorDefault,// structureAllocator
      packet->data,       // memoryBlock
      packet->size,       // blockLength
      kCFAllocatorNull,   // blockAllocator
      nullptr,            // customBlockSource
      0,                  // offsetToData
      packet->size,       // dataLength
      0,                  // flags
      &block_buf
  );

  if (status == kCMBlockBufferNoErr) {

    // 需要设置时间戳
    CMSampleTimingInfo timing_info;
    timing_info.presentationTimeStamp = CMTimeMake(packet->pts/1000, 1000000);
    timing_info.duration = CMTimeMake(packet->duration/1000, 1000000);
    timing_info.decodeTimeStamp = CMTimeMake(packet->dts/1000, 1000000);
//    LOG(INFO) << "SendDecoder packet->dts=" << packet->dts/1000 << " pts=" << packet->pts/1000;

    const size_t sample_size_array[] = { (size_t)packet->size };
    status = CMSampleBufferCreate(
        kCFAllocatorDefault,  // allocator
        block_buf,            // dataBuffer
        TRUE,                 // dataReady
        0,                    // makeDataReadyCallback
        0,                    // makeDataReadyRefcon
        fmt_desc,             // formatDescription
        1,                    // numSamples
        1,                    // numSampleTimingEntries
        &timing_info,         // sampleTimingArray
        1,                    // numSampleSizeEntries
        sample_size_array,    // sampleSizeArray
        &sample_buf
    );
  }

  if (block_buf) {
    CFRelease(block_buf);
  }

  return sample_buf;
}


VTEncoder::VTEncoder() :
  session_(nullptr),
  extradata_(nullptr),
  extradata_size_(0),
  attributes(nullptr),
  poolRef(nullptr){}

VTEncoder::~VTEncoder() {
  Finish();
}


int VTEncoder::InitEncoder(int width, int height, int64_t bitrate) {
  OSStatus status = VTCompressionSessionCreate(
      nullptr,                     // allocator 传nullptr或KCFAllocatorDefault
      width, height,                        // width height
      kCMVideoCodecType_H264,    // 编码类型
      nullptr,            // 编码规范 传NULL，videotoolbox自行选择
      nullptr,       // 源像素缓冲区
      nullptr,         // 压缩数据分配器
      videotoolbox_encoder_callback,        // 回调函数
      this,               // 回调函数的引用
      &session_);                           // 编码会话对象指针
  if (status != 0) {
    switch (status) {
      case kVTVideoDecoderNotAvailableNowErr:
        LOG(INFO) << "VideoToolbox session not available.";
        return AVERROR(ENOSYS);
      case kVTVideoDecoderUnsupportedDataFormatErr:
        LOG(INFO) << "VideoToolbox does not support this format.";
        return AVERROR(ENOSYS);
      case kVTCouldNotFindVideoEncoderErr:
        LOG(INFO) << "VideoToolbox encoder for this format not found.";
        return AVERROR(ENOSYS);
      case kVTVideoEncoderMalfunctionErr:
        LOG(INFO) << "VideoToolbox malfunction.";
        return AVERROR(EINVAL);
      default:
        LOG(INFO) << "Unknown VideoToolbox session creation error " << (int)status;
        return AVERROR_UNKNOWN;
    }
  }
  // 设置实时编码输出（避免延迟）
  VTSessionSetProperty(session_, kVTCompressionPropertyKey_RealTime, kCFBooleanTrue);
  VTSessionSetProperty(session_, kVTCompressionPropertyKey_ProfileLevel, kVTProfileLevel_H264_Baseline_AutoLevel);

  // 设置关键帧（GOPsize)间隔
  int frameInterval = 25;
  CFNumberRef  frameIntervalRef = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &frameInterval);
  VTSessionSetProperty(session_, kVTCompressionPropertyKey_MaxKeyFrameInterval, frameIntervalRef);

  // 设置期望帧率
  int fps = 25;
  CFNumberRef  fpsRef = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &fps);
  VTSessionSetProperty(session_, kVTCompressionPropertyKey_ExpectedFrameRate, fpsRef);

  //使用默认码率和码率上限
  //设置码率，均值，单位是bit
  int bitRate = width * height * 12 * 8;
  //设置码率，上限，单位是byte
  int bitRateLimit = width * height * 12;
  if (bitrate != 0) {
    bitRate = bitrate;
    bitRateLimit = bitrate / 8;
  }
  CFNumberRef bitRateRef = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &bitRate);
  VTSessionSetProperty(session_, kVTCompressionPropertyKey_AverageBitRate, bitRateRef);
  CFNumberRef bitRateLimitRef = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &bitRateLimit);
  VTSessionSetProperty(session_, kVTCompressionPropertyKey_DataRateLimits, bitRateLimitRef);

  // Tell the encoder to start encoding
  VTCompressionSessionPrepareToEncodeFrames(session_);

  return 0;
}

int VTEncoder::Encode(AVFrame* frame) {
  CVImageBufferRef imageBuffer = Frame2ImageBuffer2(frame);
  // 帧时间，如果不设置会导致时间轴过长。
  int64_t pts = 0;
  if (frame) {
    pts = av_rescale_q(frame->pts, (AVRational) VIDEO_TIME_BASE, (AVRational) FFMPEG_TIME_BASE);
  } else {
    OSStatus completeCode = VTCompressionSessionCompleteFrames(session_, kCMTimeInvalid);
    if (completeCode != noErr) {
      LOG(INFO) << "flush frame failed";
      return -1;
    } else {
      LOG(INFO) << "read over";
      return 0;
    }
  }
  CMTime presentationTimeStamp = CMTimeMake(pts, 1);
//  LOG(INFO) << "pts: " <<pts;
  VTEncodeInfoFlags flags;
  OSStatus statusCode = VTCompressionSessionEncodeFrame(
      session_,
      imageBuffer,                                      // 要编码的数据
      presentationTimeStamp,                            // 时间戳
      kCMTimeInvalid,                                   // 帧持续时间
      nullptr,                            // 额外数据
      nullptr,                         // 帧数据的引用，将被传递给回调函数
      &flags);                                          // 接收编码信息操作
  if (statusCode != noErr) {
    LOG(INFO) << "H264: VTCompressionSessionEncodeFrame failed with " << statusCode;
    return -1;
  }
  return 0;
}

void VTEncoder::GetSpsPps(uint8_t* sps, size_t sps_size, uint8_t* pps, size_t pps_size) {
  extradata_size_ = 2 * START_CODE_SIZE + sps_size + pps_size;
  extradata_ = new uint8_t [extradata_size_];
  uint8_t *position = extradata_;
  memcpy(position, start_code, START_CODE_SIZE);
  position += START_CODE_SIZE;
  memcpy(position, sps, sps_size);
  position += sps_size;
  memcpy(position, start_code, START_CODE_SIZE);
  position += START_CODE_SIZE;
  memcpy(position, pps, pps_size);
  position += pps_size;
//  PrintData(extradata_, extradata_size_);

}

void VTEncoder::FindExtradata(AVCodecParameters *vpar) {
  vpar->extradata = new uint8_t[extradata_size_];
  memcpy(vpar->extradata, extradata_, extradata_size_);
  vpar->extradata_size = extradata_size_;
//  PrintData(extradata_, extradata_size_);
}

void VTEncoder::CreatePacket(uint8_t *pData, int nBytes, int64_t pts, bool keyframe) {
  AVPacket *pkt = av_packet_alloc();
  uint8_t *data = new uint8_t[nBytes + START_CODE_SIZE];
  uint8_t *position = data;
  memcpy(position, start_code, START_CODE_SIZE);
  position += START_CODE_SIZE;
  memcpy(position, pData, nBytes);
  pkt->pts = pts;
  pkt->dts = pts;
  pkt->data = data;
  pkt->size = nBytes + START_CODE_SIZE;
  if (keyframe) {
    pkt->flags |= AV_PKT_FLAG_KEY;
  }
  pkts_.push_back(pkt);
}

AVPacket* VTEncoder::ReceivePacket() {
  if (!pkts_.empty()) {
    AVPacket *pkt = pkts_.front();
    pkts_.pop_front();
    return pkt;
  } else {
    return nullptr;
  }
}

CVImageBufferRef VTEncoder::Frame2ImageBuffer(AVFrame *frame) {
  if (!frame) {
    return nullptr;
  }
  CVReturn cv_ret;
  CVPixelBufferRef pixel_buf = nullptr;

  OSType pixelFormatType = frame->color_range == AVCOL_RANGE_MPEG ? kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange : kCVPixelFormatType_420YpCbCr8BiPlanarFullRange;
  cv_ret = CVPixelBufferCreateWithBytes(
      kCFAllocatorDefault,              // structureAllocator
      frame->width,                     //width   Width of the PixelBuffer in pixels
      frame->height,                    //height  Height of the PixelBuffer in pixels
      pixelFormatType,                  //pixelFormatType  Pixel format indentified by its respective OSType.
      frame->data,                      //baseAddress  Address of the memory storing the pixels.
      frame->linesize[0],   //bytesPerRow  Row bytes of the pixel storage memory.
      nullptr,            //releaseCallback  CVPixelBufferReleaseBytePointerCallback function that gets called when the PixelBuffer gets destroyed.
      nullptr,             //releaseRefCon  User data identifying the PixelBuffer for the release callback.
      nullptr,         //pixelBufferAttributes A dictionary with additional attributes for a a pixel buffer. This parameter is optional. See PixelBufferAttributes for more details.
      &pixel_buf                        //pixelBufferOut  The new pixel buffer will be returned here
  );

  if (cv_ret != kCVReturnSuccess) {
    LOG(INFO) << "create pixel buffer failed";
    return nullptr;
  }
  return pixel_buf;
}

CVImageBufferRef VTEncoder::Frame2ImageBuffer2(AVFrame *frame) {
  if (!frame) {
    return nullptr;
  }

  CVReturn cv_ret;

  if (!poolRef) {
    OSType pixelFormatType = frame->color_range == AVCOL_RANGE_MPEG ? kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange : kCVPixelFormatType_420YpCbCr8BiPlanarFullRange;
    attributes = videotoolbox_buffer_attributes_create(frame->width, frame->height, pixelFormatType);
    cv_ret = CVPixelBufferPoolCreate(
        kCFAllocatorDefault,
        nullptr,    //allocator The CFAllocatorRef to use for allocating this buffer pool.  May be NULL.
        attributes,             //attributes   A CFDictionaryRef containing the attributes to be used for creating new PixelBuffers within the pool
        &poolRef                //poolOut   The newly created pool will be placed here
    );
    if (cv_ret != kCVReturnSuccess) {
      LOG(ERROR) << "create pixel buffer pool failed";
      return nullptr;
    }
  }

  CVPixelBufferRef pixel_buf = nullptr;
  cv_ret = CVPixelBufferPoolCreatePixelBuffer(
      kCFAllocatorDefault,
      poolRef,              //pool The CVPixelBufferPool that should create the new CVPixelBuffer.
      &pixel_buf            //pixelBufferOut  The newly created pixel buffer will be placed here
  );
  if (cv_ret != kCVReturnSuccess) {
    LOG(INFO) << "create pixel buffer failed";
    return nullptr;
  }
  size_t w  =frame->width;
  size_t h = frame->height;
  CVPixelBufferLockBaseAddress(pixel_buf, kCVPixelBufferLock_ReadOnly);

  uint8_t *y_src = frame->data[0];
  uint8_t *u_src = frame->data[1];
  uint8_t *v_src = frame->data[2];

  uint8_t *y_dest = (uint8_t *)CVPixelBufferGetBaseAddressOfPlane(pixel_buf, 0);
  uint8_t *u_dest = (uint8_t *)CVPixelBufferGetBaseAddressOfPlane(pixel_buf, 1);
  uint8_t *v_dest = (uint8_t *)CVPixelBufferGetBaseAddressOfPlane(pixel_buf, 2);

  size_t y_src_bytesPerRow  = frame->linesize[0];
  size_t u_src_bytesPerRow  = frame->linesize[1];
  size_t v_src_bytesPerRow  = frame->linesize[2];
  size_t y_dest_bytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixel_buf, 0);
  size_t u_dest_bytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixel_buf, 1);
  size_t v_dest_bytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(pixel_buf, 2);

  libyuv::I420Copy(y_src, y_src_bytesPerRow,
                   u_src, u_src_bytesPerRow,
                   v_src, v_src_bytesPerRow,
                   y_dest, y_dest_bytesPerRow,
                   u_dest, u_dest_bytesPerRow,
                   v_dest, v_dest_bytesPerRow,
                   w, h);

  CVPixelBufferUnlockBaseAddress(pixel_buf, kCVPixelBufferLock_ReadOnly);
  return pixel_buf;
}

void VTEncoder::Finish() {
  if (session_) {
    VTCompressionSessionCompleteFrames(session_, kCMTimeInvalid);
    VTCompressionSessionInvalidate(session_);
    CFRelease(session_);
    session_ = nullptr;
  }
}
