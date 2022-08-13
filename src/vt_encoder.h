//
// Created by sunchongyang on 2021/12/29.
//

#ifndef VIDEOTOOLBOX_SRC_VT_ENCODER_H_
#define VIDEOTOOLBOX_SRC_VT_ENCODER_H_

#include <VideoToolbox/VideoToolbox.h>
#include <TargetConditionals.h>
#include <list>

extern "C" {
#include <libavutil/pixfmt.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
}

#define FFMPEG_TIME_BASE {1, 1000000000}
#define VIDEO_TIME_BASE {1, 12800}


class VTEncoder {
 public:
  VTEncoder();
  ~VTEncoder();

  int InitEncoder(int width, int height, int64_t bitrate = 0);
  int Encode(AVFrame* frame);
  void FindExtradata(AVCodecParameters *vpar);
  void GetSpsPps(uint8_t* sps, size_t sps_size, uint8_t* pps, size_t pps_size);
  void CreatePacket(uint8_t *pData, int nBytes, int64_t pts, bool keyframe);
  AVPacket* ReceivePacket();
  void Finish();

 private:
  CVImageBufferRef Frame2ImageBuffer(AVFrame *frame);
  CVImageBufferRef Frame2ImageBuffer2(AVFrame *frame);

  VTCompressionSessionRef session_;
  std::list<AVPacket* > pkts_;
  uint8_t *extradata_;
  int extradata_size_;

  CFDictionaryRef attributes;
  CVPixelBufferPoolRef poolRef;



};

#endif //VIDEOTOOLBOX_SRC_VT_ENCODER_H_
