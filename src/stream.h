//
// Created by sunchongyang on 2021/12/29.
//

#ifndef VIDEOTOOLBOX_SRC_STREAM_H_
#define VIDEOTOOLBOX_SRC_STREAM_H_

#include "glog/logging.h"
#include "vt_encoder.h"
#include "vt_decoder.h"
extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/opt.h>
}
class Stream {
 public:
  Stream();
  ~Stream();

  //转码
  bool Transcode(const char *input_url, const char *output_url);
 private:
  bool OpenFile(const char *input_url, const char *output_url);
  bool InitVideoDecoder();
  bool InitVideoEncoder();

  bool InitVtStream();
  int ProcessFrame();
  void VTDecode(AVPacket *pkt);
  void VTEncode(AVFrame *frame);
  void Finish();

  AVFormatContext *ictx_;
  AVFormatContext *octx_;
  int video_in_stream_index_;
  int audio_in_stream_index_;
  int video_out_stream_index_;
  int audio_out_stream_index_;

  VTEncoder *vt_encoder_;
  VTDecoder *vt_decoder_;
};

#endif //VIDEOTOOLBOX_SRC_STREAM_H_
