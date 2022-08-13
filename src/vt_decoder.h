//
// Created by suncube on 2022/8/13.
//

#ifndef VIDEOTOOLBOX_SRC_VT_DECODER_H_
#define VIDEOTOOLBOX_SRC_VT_DECODER_H_

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

class VTDecoder {
 public:
  VTDecoder();
  ~VTDecoder();
  int InitDecoder(AVCodecParameters *codec_parameters);
  int Decode(AVPacket* packet);
  void OnDecodeFrame(
      void *sourceFrameRefCon,
      OSStatus status,
      VTDecodeInfoFlags flags,
      CVImageBufferRef image_buffer,
      CMTime pts,
      CMTime duration
  );

  void SortQueuePush(AVFrame* frame);
  AVFrame* SortQueuePop(bool eof);

 private:
  void ToAVFrame(CVImageBufferRef image_buffer,
                 CMTime pts);
  AVFrame *get_video_frame(enum AVPixelFormat pixfmt,int width,int height);

  bool refresh_session_;  // 当ios程序退到后台，session将不可用
  bool recovery_drop_packet_;  // 重置session之后需要从关键帧开始解码
  VTDecompressionSessionRef session_;
  CMFormatDescriptionRef cm_fmt_desc_;

//  std::shared_ptr<base::GLContext> gl_context_;
  CVOpenGLTextureCacheRef texture_cache_;
  struct ImageBuffer {
    CVImageBufferRef image_buffer;
    CMTime pts;
  };
  std::list<ImageBuffer> image_buffer_list_;

  GLuint arb_program_;
//  std::unique_ptr<GLFrameBuffer> out_frame_buffer_;

  std::list<AVFrame*> yuv420_frame_list_;
};

#endif //VIDEOTOOLBOX_SRC_VT_DECODER_H_
