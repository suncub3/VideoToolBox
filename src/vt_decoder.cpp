//
// Created by suncube on 2022/8/13.
//

#include <libyuv.h>
#include "vt_decoder.h"
#include "glog/logging.h"

// !!!HEVC codec tag: only hvc1 can decode
//const OSType kDefaultCVPixFmtType = kCVPixelFormatType_420YpCbCr8Planar;
const OSType kDefaultCVPixFmtType = kCVPixelFormatType_32BGRA;

const int kMaxRefFrames = 5;  // IJKVideoToolBoxSync.m 1018行

static void print_data(uint8_t* data, size_t size) {
  for (size_t i = 0; i < size; i++) {
    printf("0x%02x ", data[i]);
    if ((i+1) % 16 == 0) {
      printf("\n");
    }
  }
  printf("\n");
}

static CFDictionaryRef videotoolbox_decoder_config_create(
    CMVideoCodecType codec_type,
    AVCodecParameters *codec_parameters) {
  CFMutableDictionaryRef config_info = CFDictionaryCreateMutable(
      kCFAllocatorDefault,
      0,
      &kCFTypeDictionaryKeyCallBacks,
      &kCFTypeDictionaryValueCallBacks);

  CFDictionarySetValue(config_info,
                       codec_type == kCMVideoCodecType_HEVC ?
                       kVTVideoDecoderSpecification_EnableHardwareAcceleratedVideoDecoder :
                       kVTVideoDecoderSpecification_RequireHardwareAcceleratedVideoDecoder,
                       kCFBooleanTrue);

  CFMutableDictionaryRef avc_info;
  CFDataRef data = nullptr;

  avc_info = CFDictionaryCreateMutable(
      kCFAllocatorDefault,
      1,
      &kCFTypeDictionaryKeyCallBacks,
      &kCFTypeDictionaryValueCallBacks);

  switch (codec_type) {
    case kCMVideoCodecType_MPEG4Video :
//      if (avctx->extradata_size) {
//        data = videotoolbox_esds_extradata_create(avctx);
//      }
//      if (data) {
//        CFDictionarySetValue(avc_info, CFSTR("esds"), data);
//      }
      break;
    case kCMVideoCodecType_H264 :
    {
//        print_data(codec_parameters->extradata, codec_parameters->extradata_size);
//
//        uint8_t* sps = nullptr; size_t sps_size = 0;
//        uint8_t* pps = nullptr; size_t pps_size = 0;
//        parse_extradata(codec_parameters, &sps, &sps_size, &pps, &pps_size);
//        data = ff_videotoolbox_avcc_extradata_create(sps, sps_size, pps, pps_size);
      data = CFDataCreate(kCFAllocatorDefault, codec_parameters->extradata, codec_parameters->extradata_size);
      if (data) {
        CFDictionarySetValue(avc_info, CFSTR("avcC"), data);
      }
    }
      break;
    case kCMVideoCodecType_HEVC :
//      data = ff_videotoolbox_hvcc_extradata_create(avctx);
      data = CFDataCreate(kCFAllocatorDefault, codec_parameters->extradata, codec_parameters->extradata_size);
      if (data) {
        CFDictionarySetValue(avc_info, CFSTR("hvcC"), data);
      }
      break;
    default:
      break;
  }

  CFDictionarySetValue(config_info,
                       kCMFormatDescriptionExtension_SampleDescriptionExtensionAtoms,
                       avc_info);

  if (data) {
    CFRelease(data);
  }
  CFRelease(avc_info);
  return config_info;
}

static CMVideoFormatDescriptionRef videotoolbox_format_desc_create(
    CMVideoCodecType codec_type,
    CFDictionaryRef decoder_spec,
    int width,
    int height) {
  CMFormatDescriptionRef cm_fmt_desc;
  OSStatus status;

  status = CMVideoFormatDescriptionCreate(kCFAllocatorDefault,
                                          codec_type,
                                          width,
                                          height,
                                          decoder_spec, // Dictionary of extension
                                          &cm_fmt_desc);

  if (status) {
    return nullptr;
  }
  return cm_fmt_desc;
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
  CFDictionarySetValue(buffer_attributes, kCVPixelBufferIOSurfacePropertiesKey, io_surface_properties);
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

static void videotoolbox_decoder_callback(
    void *opaque,
    void *sourceFrameRefCon,
    OSStatus status,
    VTDecodeInfoFlags flags,
    CVImageBufferRef image_buffer,
    CMTime pts,
    CMTime duration
) {
  if (status != noErr) {
    LOG(INFO) << "Video hard decode callback error status=" << (int)status;
    return;
  }

  if (!image_buffer) {
    LOG(INFO) << "vt decoder cb: output image buffer is null";
    return;
  }

  auto vt_decoder = (VTDecoder*)opaque;
  if (vt_decoder) {
    vt_decoder->OnDecodeFrame(sourceFrameRefCon, status, flags, image_buffer, pts, duration);
  }
//  CVPixelBufferRelease(image_buffer);
}

static CMSampleBufferRef videotoolbox_sample_buffer_create(
    CMFormatDescriptionRef fmt_desc,
    AVPacket *packet) {
  if (!packet) {
    return nullptr;
  }
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

  if (!status) {

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

VTDecoder::VTDecoder() :
    refresh_session_(false),
    recovery_drop_packet_(false),
    session_(nullptr),
    cm_fmt_desc_(nullptr),
    texture_cache_(nullptr) {}

VTDecoder::~VTDecoder() {
  if (cm_fmt_desc_) {
    CFRelease(cm_fmt_desc_);
    cm_fmt_desc_ = nullptr;
  }

  if (session_) {
    VTDecompressionSessionInvalidate(session_);
    session_ = nullptr;
  }
}

int VTDecoder::InitDecoder(AVCodecParameters *codec_parameters) {
  VTDecompressionOutputCallbackRecord decoder_cb;
  CFDictionaryRef decoder_spec;
  CFDictionaryRef buf_attr;

  int cm_codec_type;
  switch (codec_parameters->codec_id) {
    case AV_CODEC_ID_H263 :
      cm_codec_type = kCMVideoCodecType_H263;
      break;
    case AV_CODEC_ID_H264 :
      cm_codec_type = kCMVideoCodecType_H264;
      break;
    case AV_CODEC_ID_HEVC :
      cm_codec_type = kCMVideoCodecType_HEVC;
      break;
    case AV_CODEC_ID_MPEG1VIDEO :
      cm_codec_type = kCMVideoCodecType_MPEG1Video;
      break;
    case AV_CODEC_ID_MPEG2VIDEO :
      cm_codec_type = kCMVideoCodecType_MPEG2Video;
      break;
    case AV_CODEC_ID_MPEG4 :
      cm_codec_type = kCMVideoCodecType_MPEG4Video;
      break;
    default :
      return -1;
  }

  decoder_spec = videotoolbox_decoder_config_create(cm_codec_type, codec_parameters);

  if (!decoder_spec) {
    LOG(INFO) << "decoder specification creation failed";
    return -1;
  }

  cm_fmt_desc_ = videotoolbox_format_desc_create(
      cm_codec_type,
      decoder_spec,
      codec_parameters->width,
      codec_parameters->height);
  if (!cm_fmt_desc_) {
    if (decoder_spec) {
      CFRelease(decoder_spec);
    }

    LOG(INFO)<< "format description creation failed";
    return -1;
  }

  OSType cv_pix_fmt_type = kDefaultCVPixFmtType;
  buf_attr = videotoolbox_buffer_attributes_create(
      codec_parameters->width,
      codec_parameters->height,
      cv_pix_fmt_type);

  decoder_cb.decompressionOutputCallback = videotoolbox_decoder_callback;
  decoder_cb.decompressionOutputRefCon   = this;
  LOG(INFO) << "decompressionOutputRefCon this:" << this;

  OSStatus status = VTDecompressionSessionCreate(
      nullptr,                      // allocator
      cm_fmt_desc_,              // videoFormatDescription
      decoder_spec,              // videoDecoderSpecification
      buf_attr,                  // destinationImageBufferAttributes
      &decoder_cb,               // outputCallback
      &session_);                // decompressionSessionOut

  if (decoder_spec) {
    CFRelease(decoder_spec);
  }
  if (buf_attr) {
    CFRelease(buf_attr);
  }

  switch (status) {
    case kVTVideoDecoderNotAvailableNowErr:
      LOG(INFO) << "VideoToolbox session not available.";
      return AVERROR(ENOSYS);
    case kVTVideoDecoderUnsupportedDataFormatErr:
      LOG(INFO) << "VideoToolbox does not support this format.";
      return AVERROR(ENOSYS);
    case kVTCouldNotFindVideoDecoderErr:
      LOG(INFO) << "VideoToolbox decoder for this format not found.";
      return AVERROR(ENOSYS);
    case kVTVideoDecoderMalfunctionErr:
      LOG(INFO) << "VideoToolbox malfunction.";
      return AVERROR(EINVAL);
    case kVTVideoDecoderBadDataErr:
      LOG(INFO) << "VideoToolbox reported invalid data.";
      return AVERROR_INVALIDDATA;
    case 0:
      return 0;
    default:
      LOG(INFO) << "Unknown VideoToolbox session creation error %d" << (int)status;
      return AVERROR_UNKNOWN;
  }
}

int VTDecoder::Decode(AVPacket *packet) {
  CMSampleBufferRef sample_buf = videotoolbox_sample_buffer_create(
      cm_fmt_desc_, packet);
  if (!sample_buf) {
    return -1;
  }

// kVTDecodeFrame_EnableAsynchronousDecompression 异步
// kVTDecodeFrame_EnableTemporalProcessing 排序——排序不起作用(手动排序)
// kVTDecodeFrame_DoNotOutputFrame 不输出
  VTDecodeFrameFlags decode_frame_flags = 0;
  OSStatus status = VTDecompressionSessionDecodeFrame(
      session_,
      sample_buf,
      decode_frame_flags,       // decodeFlags
      nullptr,                  // sourceFrameRefCon
      nullptr);                 // infoFlagsOut
//  LOG(INFO) << __FUNCTION__ << " status=" << status;
  if (status == noErr) {
    status = VTDecompressionSessionWaitForAsynchronousFrames(session_);
  }
  
  if (status == kVTInvalidSessionErr) {
    refresh_session_ = true;
  }
  if (status == kVTVideoDecoderMalfunctionErr) {
    recovery_drop_packet_ = true;
    refresh_session_ = true;
  }

  CFRelease(sample_buf);
  return status;
}

void VTDecoder::OnDecodeFrame(
    void *sourceFrameRefCon,
    OSStatus status,
    VTDecodeInfoFlags flags,
    CVImageBufferRef image_buffer,
    CMTime pts,
    CMTime duration
) {
//  LOG(INFO) << "OnDecode thread id:" << std::this_thread::get_id();

  OSType pixel_format = CVPixelBufferGetPixelFormatType(image_buffer);
  if (pixel_format != kDefaultCVPixFmtType) {
    return;
  }

  ToAVFrame(image_buffer, pts);
}

void VTDecoder::ToAVFrame(
    CVImageBufferRef image_buffer,
    CMTime pts
) {
  size_t w = CVPixelBufferGetWidth(image_buffer);
  size_t h = CVPixelBufferGetHeight(image_buffer);

  auto yuv420_frame = get_video_frame(AV_PIX_FMT_YUV420P, w, h);

  uint32_t map_flags = kCVPixelBufferLock_ReadOnly;
  auto err = CVPixelBufferLockBaseAddress(image_buffer, map_flags);
  if (err != kCVReturnSuccess) {
    LOG(INFO) << "Error locking the pixel buffer.";
    return;
  }

  size_t y_src_bytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(image_buffer, 0);
  size_t u_src_bytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(image_buffer, 1);
  size_t v_src_bytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(image_buffer, 2);

  uint8_t *y_src = (uint8_t *) CVPixelBufferGetBaseAddressOfPlane(image_buffer, 0);
  uint8_t *u_src = (uint8_t *) CVPixelBufferGetBaseAddressOfPlane(image_buffer, 1);
  uint8_t *v_src = (uint8_t *) CVPixelBufferGetBaseAddressOfPlane(image_buffer, 2);

  uint8_t *y_dest = yuv420_frame->data[0];
  uint8_t *u_dest = yuv420_frame->data[1];
  uint8_t *v_dest = yuv420_frame->data[2];

  size_t y_dest_bytesPerRow = yuv420_frame->linesize[0];
  size_t u_dest_bytesPerRow = yuv420_frame->linesize[1];
  size_t v_dest_bytesPerRow = yuv420_frame->linesize[2];

  for (int i = 0; i < h; i ++) {
    memcpy(y_dest, y_src, y_src_bytesPerRow);
    y_src  += y_src_bytesPerRow;
    y_dest += y_dest_bytesPerRow;
  }

  for (int i = 0; i < h/2; i ++) {
    memcpy(u_dest, u_src, u_src_bytesPerRow);
    u_src  += u_src_bytesPerRow;
    u_dest += u_dest_bytesPerRow;
  }
  for (int i = 0; i < h/2; i ++) {
    memcpy(v_dest, v_src, v_src_bytesPerRow);
    v_src  += v_src_bytesPerRow;
    v_dest += v_dest_bytesPerRow;
  }

  CVPixelBufferUnlockBaseAddress(image_buffer, kCVPixelBufferLock_ReadOnly);

  yuv420_frame->pts = pts.value * (1000000000 / pts.timescale);  // ns
  SortQueuePush(yuv420_frame);
}

void VTDecoder::SortQueuePush(AVFrame* frame) {
  auto pos = yuv420_frame_list_.begin();
  for (; pos != yuv420_frame_list_.end(); ++pos) {
    auto tmp = *pos;
    if (frame->pts <= tmp->pts) {
      yuv420_frame_list_.insert(pos, frame);
      return;
    }
  }

  yuv420_frame_list_.push_back(frame);
}

AVFrame* VTDecoder::SortQueuePop(bool eof) {
  if (!eof && yuv420_frame_list_.size() < kMaxRefFrames) {
    return nullptr;
  }
  if (!yuv420_frame_list_.empty()) {
    auto frame = yuv420_frame_list_.front();
    yuv420_frame_list_.pop_front();
    return frame;
  }
  return nullptr;
}

AVFrame* VTDecoder::get_video_frame(enum AVPixelFormat pixfmt,int width,int height)
{
  AVFrame *f = av_frame_alloc();
  f->format = pixfmt;
  f->width = width;
  f->height = height;
  int ret = 0;
  if ((ret = av_frame_get_buffer(f, 0)) < 0) {
    LOG(ERROR) << "video get frame buffer fail: " << ret;
    return nullptr;
  }

  if ((ret =  av_frame_make_writable(f)) < 0) {
    LOG(ERROR) << "video av_frame_make_writable fail: " << ret;
    return nullptr;
  }
  return f;
}
