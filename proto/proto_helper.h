#ifndef PROTO_HELPER_H
#define PROTO_HELPER_H
#include <google/protobuf/message.h>
#include <sstream>

std::vector<std::string> updateMessage(google::protobuf::Message *outMessage, const google::protobuf::Message *inMessage, const std::string &parentName = std::string());
google::protobuf::Message *searchForField(google::protobuf::Message *inMessage, std::string& fullName, int& indexInMessage);
#endif // PROTO_HELPER_H
