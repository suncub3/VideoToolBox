#include "stream.h"


int main() {
  const char *input_url = "../res/fps25.mp4";
  const char *output_url = "../res/res.mp4";
  std::shared_ptr<Stream> stream = std::make_shared<Stream>();
  LOG(INFO) << "start";
  stream->Transcode(input_url, output_url);
  LOG(INFO) << "finish";
  return 0;
}
