//
// Created by sunchongyang on 2021/12/29.
//

#include "stream.h"


Stream::Stream() :ictx_(nullptr), octx_(nullptr), vt_decoder_(nullptr), vt_encoder_(nullptr),
                  video_in_stream_index_(-1), video_out_stream_index_(-1),
                  audio_in_stream_index_(-1), audio_out_stream_index_(-1){}

Stream::~Stream() {
  Finish();
}

bool Stream::Transcode(const char *input_url, const char *output_url) {
  if (!OpenFile(input_url, output_url)) {
    LOG(ERROR) << "open url failed";
    return false;
  }

  if (video_in_stream_index_ != -1) {
    if (!InitVideoDecoder()) {
      LOG(ERROR) << "init Video Decoder failed";
      return false;
    }
    if (!InitVideoEncoder()) {
      LOG(ERROR) << "init Video Encoder failed";
      return false;
    }
  }

  InitVtStream();

  // Everything is ready. Now open the output stream.
  if (!(octx_->flags & AVFMT_NOFILE)) {
    if (avio_open2(&octx_->pb, output_url, AVIO_FLAG_WRITE, nullptr, nullptr) < 0) {
      DLOG(ERROR) << "FFMPEG: Could not open";
      return false;
    }
  }

  // Write the container header
  if (avformat_write_header(octx_, nullptr) < 0) {
    DLOG(ERROR) << "FFMPEG: avformat_write_header error!";
    return false;
  }

  av_dump_format(octx_, 0, output_url, 1);

  if (ProcessFrame() < 0) {
    DLOG(ERROR) << "write packet failed";
    return false;
  }
  return true;
}

bool Stream::OpenFile(const char *input_url, const char *output_url) {
  if (!input_url || !output_url) {
    LOG(ERROR) << "url is null";
    return false;
  }
  int ret = 0;
  //open input
  if ((avformat_open_input(&ictx_, input_url, 0, 0)) < 0) {
    DLOG(ERROR) <<"avformat_open_input failed:" << input_url;
    return false;
  }
  // 1.2 解码一段数据，获取流相关信息
  if ((avformat_find_stream_info(ictx_, 0)) < 0) {
    DLOG(ERROR) <<"Failed to retrieve input stream information";
    return false;
  }
  //av_dump_format(ictx_, 0, input_url, 0);
  for (int i = 0; i < ictx_->nb_streams; i++) {
    AVCodecParameters *codecpar = ictx_->streams[i]->codecpar;
    if (codecpar->codec_type == AVMEDIA_TYPE_VIDEO && video_in_stream_index_ == -1) {
      video_in_stream_index_ = i;
    }
    if (codecpar->codec_type == AVMEDIA_TYPE_AUDIO && audio_in_stream_index_ == -1) {
      audio_in_stream_index_ = i;
    }
  }

  //open output
  if (avformat_alloc_output_context2(&octx_, nullptr, nullptr, output_url) < 0) {
    DLOG(ERROR) <<"Could not create output context";
    return false;
  }

  return true;
}

bool Stream::InitVideoDecoder() {
  AVCodecParameters *codecpar = ictx_->streams[video_in_stream_index_]->codecpar;
  vt_decoder_ = new VTDecoder();
  if (vt_decoder_->InitDecoder(codecpar) < 0) {
    return false;
  }
  return true;
}

bool Stream::InitVideoEncoder() {
  AVCodecParameters *codecpar = ictx_->streams[video_in_stream_index_]->codecpar;
  vt_encoder_ = new VTEncoder();
  if (vt_encoder_->InitEncoder(codecpar->width, codecpar->height) < 0) {
    return false;
  }
  return true;
}

bool Stream::InitVtStream() {
  for (int i = 0; i < ictx_->nb_streams; i++) {
    AVStream *in_stream = ictx_->streams[i];
    AVCodecParameters *in_codecpar = in_stream->codecpar;
    AVStream *avs = avformat_new_stream(octx_, nullptr);
    if (!avs) {
      DLOG(ERROR) <<"Failed allocating output stream\n";
      return false;
    }
    int ret = 0;
    if (in_codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
      video_out_stream_index_ = avs->index;
      AVCodecParameters *vpar = avs->codecpar;
      vpar->codec_id = AV_CODEC_ID_H264;
      vpar->codec_type = AVMEDIA_TYPE_VIDEO;
      vpar->width = in_codecpar->width;
      vpar->height = in_codecpar->height;
      vpar->bit_rate = 0;
      //vs->time_base = (AVRational) {1, 25};
      vpar->codec_tag = MKTAG('a', 'v', 'c', '1');
      avs->time_base = (AVRational) VIDEO_TIME_BASE;
    }
    if (in_codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
      audio_out_stream_index_ = avs->index;
      ret = avcodec_parameters_copy(avs->codecpar, in_codecpar);
    }

    if (ret < 0) {
      LOG(ERROR) << "avcodec_parameters_from_context failed, ret=" << ret;
      return false;
    }
    uint32_t src_codec_tag = in_codecpar->codec_tag;
    if (av_codec_get_id(octx_->oformat->codec_tag, src_codec_tag) != avs->codecpar->codec_id) {
      avs->codecpar->codec_tag = 0;
    }
  }
  return true;
}

int Stream::ProcessFrame() {
  AVPacket *pkt = av_packet_alloc();
  while (av_read_frame(ictx_, pkt) == 0) {
    if (pkt->stream_index == video_in_stream_index_) {
      VTDecode(pkt);
    }
    else {
      av_packet_rescale_ts(pkt, ictx_->streams[audio_in_stream_index_]->time_base, octx_->streams[audio_out_stream_index_]->time_base);
      pkt->pos = -1;
      if (av_interleaved_write_frame(octx_, pkt) < 0) {
        DLOG(ERROR) <<"Error muxing packet";
        return -1;
      }
    }
    av_packet_unref(pkt);
  }
  av_packet_free(&pkt);
  return 0;
}

void Stream::VTDecode(AVPacket *pkt) {
  AVFrame* frame = av_frame_alloc();
  bool eof = false;
  if (!pkt) {
    eof = true;
  }
  if (vt_decoder_->Decode(pkt) < 0) {
    return;
  }


  while (true) {
    // 从解码缓冲区接收解码后的数据
    frame = vt_decoder_->SortQueuePop(eof);
    if (!frame) {
      break;
    }
    VTEncode(frame);

  }
  av_frame_free(&frame);
}

void Stream::VTEncode(AVFrame *frame) {
  if (vt_encoder_->Encode(frame) < 0) {
    LOG(ERROR) << "VTEncoder fail";
    return;
  }
  while (true) {
    AVPacket *pkt = av_packet_alloc();
    pkt = vt_encoder_->ReceivePacket();
    if(!pkt) {
      break;
    }
    if (pkt->flags == AV_PKT_FLAG_KEY) {
      vt_encoder_->FindExtradata(octx_->streams[video_out_stream_index_]->codecpar);
    }
    pkt->stream_index = video_out_stream_index_;

    // 将编码后的数据写入文件
    LOG(INFO) << "before pts: "<<pkt->pts;
    av_packet_rescale_ts(pkt, (AVRational) {1,1000}, octx_->streams[video_out_stream_index_]->time_base);
    LOG(INFO) << "after pts: "<<pkt->pts;
    pkt->pos = -1;

    // 将packet写入输出
    if (av_interleaved_write_frame(octx_, pkt) < 0) {
      DLOG(ERROR) <<"Error muxing packet";
      break;
    }
    av_packet_free(&pkt);
  }
}

void Stream::Finish() {
  VTDecode(nullptr);

  if (ictx_) {
    avformat_close_input(&ictx_);
  }
  if (octx_) {
    av_write_trailer(octx_);
    avio_close(octx_->pb);
    avformat_free_context(octx_);
  }
}

