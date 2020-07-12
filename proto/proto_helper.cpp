#include <proto/proto_helper.h>

std::vector<std::string> updateMessage(google::protobuf::Message *outMessage, const google::protobuf::Message *inMessage, const std::string& parentName)
{
    std::vector <const google::protobuf::FieldDescriptor*> descs;
    inMessage->GetReflection()->ListFields(*inMessage, &descs);
    std::vector <std::string> fieldNames;
    for (auto& desc : descs)
    {
        auto reflection = outMessage->GetReflection();
        bool notMessage = true;

        if (!desc->is_repeated())
        {

            switch (desc->type())
            {
            case google::protobuf::FieldDescriptor::Type::TYPE_BOOL:
                reflection->SetBool(outMessage, desc, reflection->GetBool(*inMessage, desc));
                break;

            case google::protobuf::FieldDescriptor::Type::TYPE_INT32:
                reflection->SetInt32(outMessage, desc, reflection->GetInt32(*inMessage, desc));
                break;
            case google::protobuf::FieldDescriptor::Type::TYPE_INT64:
                reflection->SetInt64(outMessage, desc, reflection->GetInt64(*inMessage, desc));
                break;
            case google::protobuf::FieldDescriptor::Type::TYPE_UINT64:
                reflection->SetUInt64(outMessage, desc, reflection->GetUInt64(*inMessage, desc));
                break;

            case google::protobuf::FieldDescriptor::Type::TYPE_FLOAT:
                reflection->SetFloat(outMessage, desc, reflection->GetFloat(*inMessage, desc));
                break;

            case google::protobuf::FieldDescriptor::Type::TYPE_DOUBLE:
                reflection->SetDouble(outMessage, desc, reflection->GetDouble(*inMessage, desc));
                break;

            case google::protobuf::FieldDescriptor::Type::TYPE_ENUM:
                reflection->SetEnum(outMessage, desc, reflection->GetEnum(*inMessage, desc));
                break;

            case google::protobuf::FieldDescriptor::Type::TYPE_STRING:
                reflection->SetString(outMessage, desc, reflection->GetString(*inMessage, desc));
                break;
            case google::protobuf::FieldDescriptor::Type::TYPE_MESSAGE:
            {
                notMessage = false;
                reflection->SetAllocatedMessage(outMessage, reflection->GetMessage(*inMessage, desc).New(), desc);
                const google::protobuf::Message* subMessage = &reflection->GetMessage(*inMessage, desc);
                std::vector <const google::protobuf::FieldDescriptor*> v;
                subMessage->GetReflection()->ListFields(*subMessage, &v);
                std::string actualParentName = parentName.empty() ? "" : parentName + ".";
                actualParentName += desc->name();
                auto subFieldNames = updateMessage(reflection->MutableMessage(outMessage, desc), &reflection->GetMessage(*inMessage, desc), actualParentName);
                fieldNames.insert(fieldNames.end(), subFieldNames.begin(), subFieldNames.end());
                break;
            }

            default:
                assert(false);
                break;
            }
        }
        else
        {
            // TODO
            //        switch (desc.type())
            //        {
            //        case google::protobuf::FieldDescriptor::Type::TYPE_BOOL:
            //            reflection->AddBool(outMessage, &desc, reflection->GetBool(*inMessage, &desc));
            //            break;

            //        case google::protobuf::FieldDescriptor::Type::TYPE_INT32:
            //            reflection->AddInt32(outMessage, &desc, reflection->GetInt32(*inMessage, &desc));
            //            break;
            //        case google::protobuf::FieldDescriptor::Type::TYPE_INT64:
            //            reflection->AddInt64(outMessage, &desc, reflection->GetInt64(*inMessage, &desc));
            //            break;
            //        case google::protobuf::FieldDescriptor::Type::TYPE_UINT64:
            //            reflection->AddUInt64(outMessage, &desc, reflection->GetUInt64(*inMessage, &desc));
            //            break;

            //        case google::protobuf::FieldDescriptor::Type::TYPE_FLOAT:
            //            reflection->AddFloat(outMessage, &desc, reflection->GetFloat(*inMessage, &desc));
            //            break;

            //        case google::protobuf::FieldDescriptor::Type::TYPE_DOUBLE:
            //            reflection->AddDouble(outMessage, &desc, reflection->GetDouble(*inMessage, &desc));
            //            break;

            //        case google::protobuf::FieldDescriptor::Type::TYPE_ENUM:
            //            reflection->AddEnum(outMessage, &desc, reflection->GetEnum(*inMessage, &desc));
            //            break;

            //        case google::protobuf::FieldDescriptor::Type::TYPE_STRING:
            //            reflection->AddString(outMessage, &desc, reflection->GetString(*inMessage, &desc));
            //            break;
            //        case google::protobuf::FieldDescriptor::Type::TYPE_MESSAGE:
            //        {
            //            notMessage = false;
            //            reflection->SetAllocatedMessage(outMessage, reflection->GetMessage(*inMessage, &desc).New(), &desc);
            //            const google::protobuf::Message* subMessage = &reflection->GetMessage(*inMessage, &desc);
            //            std::vector <const google::protobuf::FieldDescriptor*> v;
            //            subMessage->GetReflection()->ListFields(*subMessage, &v);
            //            for (auto& descriptor : v)
            //            {
            //                auto v = updateField(reflection->MutableMessage(outMessage, &desc), &reflection->GetMessage(*inMessage, &desc), *descriptor);
            //                fieldNames.insert(fieldNames.end(), v.begin(), v.end());
            //            }

            //            break;
            //        }

            //        default:
            //            assert(false);
            //            break;
            //        }
        }
        if (notMessage)
        {
            fieldNames.push_back(parentName + "." + desc->name());
        }
    }


    return fieldNames;
}


google::protobuf::Message *searchForField(google::protobuf::Message *inMessage, std::string &fullName, int &indexInMessage)
{
     std::vector <std::string> strings;
     std::istringstream f(fullName);
     std::string s;
     while (std::getline(f, s, '.'))
     {
         strings.push_back(s);
     }
     google::protobuf::Message *outMessage = inMessage;
     for (int i = 0; i < strings.size() - 1; ++i)
     {
         auto desc = outMessage->GetDescriptor()->FindFieldByName(strings[i]);
         if (!desc)
         {
             continue;
         }
         assert(desc);
         outMessage = outMessage->GetReflection()->MutableMessage(outMessage, desc);
     }
     indexInMessage = outMessage->GetDescriptor()->FindFieldByName(strings.back())->index();
     return outMessage;
}
