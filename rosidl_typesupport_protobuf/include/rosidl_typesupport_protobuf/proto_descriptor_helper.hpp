#pragma once

#include <string>
#include <algorithm>
#include <string>
#include <vector>

// protobuf includes
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4100 4127 4146 4800) // disable proto warnings
#endif
#include <google/protobuf/descriptor.pb.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif


inline bool HasFile(const google::protobuf::FileDescriptorSet& fset_, const std::string& fname_)
{
  for (auto findex = 0; findex < fset_.file_size(); ++findex)
  {
    if (fset_.file(findex).name() == fname_)
    {
      return(true);
    }
  }
  return(false);
}

inline bool GetFileDescriptor(const google::protobuf::Descriptor* desc_, google::protobuf::FileDescriptorSet& fset_)
{
  
  if (desc_ == nullptr) return(false);
  const google::protobuf::FileDescriptor* fdesc = desc_->file();
  
  for (auto dep = 0; dep < fdesc->dependency_count(); ++dep)
  {
    // iterate containing messages
    const google::protobuf::FileDescriptor* sfdesc = fdesc->dependency(dep);
    
    for (auto mtype = 0; mtype < sfdesc->message_type_count(); ++mtype)
    {
      const google::protobuf::Descriptor* desc = sfdesc->message_type(mtype);
      GetFileDescriptor(desc, fset_);
    }

    // containing enums ?
    if (sfdesc->enum_type_count() > 0)
    {
      const google::protobuf::EnumDescriptor* edesc = sfdesc->enum_type(0);
      const google::protobuf::FileDescriptor* efdesc = edesc->file();

      if (!HasFile(fset_, efdesc->name()))
      {
        google::protobuf::FileDescriptorProto* epdesc = fset_.add_file();
        efdesc->CopyTo(epdesc);
      }
    }

  
    // containing services ?
    if (sfdesc->service_count() > 0)
    {
      const google::protobuf::ServiceDescriptor* svdesc = sfdesc->service(0);
      const google::protobuf::FileDescriptor*    svfdesc = svdesc->file();

      if (!HasFile(fset_, svfdesc->name()))
      {
        google::protobuf::FileDescriptorProto* svpdesc = fset_.add_file();
        svfdesc->CopyTo(svpdesc);
      }
    }
  }

  if (HasFile(fset_, fdesc->name())) return(true);

  google::protobuf::FileDescriptorProto* pdesc = fset_.add_file();
  fdesc->CopyTo(pdesc);
  for (auto field = 0; field < desc_->field_count(); ++field)
  {
    const google::protobuf::FieldDescriptor* fddesc = desc_->field(field);
    const google::protobuf::Descriptor*      desc = fddesc->message_type();
    GetFileDescriptor(desc, fset_);
  }

  return(true);
}

template<typename T>
inline std::string GetProtoMessageDescription()
{
  const google::protobuf::Descriptor* desc = T::descriptor();
  google::protobuf::FileDescriptorSet pset;
  if (GetFileDescriptor(desc, pset))
  {
    std::string desc_s = pset.SerializeAsString();
    return(desc_s);
  }
  return("");
}