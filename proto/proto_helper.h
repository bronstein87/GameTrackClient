#ifndef PROTO_HELPER_H
#define PROTO_HELPER_H
#include <google/protobuf/message.h>
#include <sstream>
#include <opencv2/core.hpp>
#include <proto/msg.internal.pb.h>
namespace ProtoHelper
{
std::vector<std::string> updateMessage(google::protobuf::Message *outMessage, const google::protobuf::Message *inMessage, const std::string &parentName = std::string());
google::protobuf::Message *searchForField(google::protobuf::Message *inMessage, std::string& fullName, int& indexInMessage);
cv::Rect gtRectToCv(gt::internal::msg::Rect r);
gt::internal::msg::Rect *cvRectToGt(cv::Rect r);
}
#endif // PROTO_HELPER_H
