#include "lio_sam/ground_projection_cache.h"
#include <array>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace lio_sam_hesai {
namespace {
constexpr std::array<char,8> kMagic{{'L','I','O','G','P','R','1','\0'}};
void Validate(std::size_t count,const std::vector<std::uint32_t>& indices,
              int rows,int columns) {
  if(rows<=0||columns<=0 || static_cast<std::size_t>(rows)>
      std::numeric_limits<std::size_t>::max()/static_cast<std::size_t>(columns) ||
      static_cast<std::size_t>(rows)*static_cast<std::size_t>(columns)>
          std::numeric_limits<std::uint32_t>::max())
    throw std::invalid_argument("invalid ground projection dimensions");
  if(count!=indices.size())
    throw std::invalid_argument("ground projection count does not match scan");
  const auto cells=static_cast<std::size_t>(rows)*columns;
  std::unordered_set<std::uint32_t> seen;seen.reserve(indices.size());
  for(const auto index:indices)
    if(index>=cells||!seen.insert(index).second)
      throw std::invalid_argument("duplicate or out-of-range ground projection index");
}
template<class T> void Write(std::ofstream& file,const T& value) {
  file.write(reinterpret_cast<const char*>(&value),sizeof(value));
}
template<class T> void Read(std::ifstream& file,T& value) {
  file.read(reinterpret_cast<char*>(&value),sizeof(value));
}
}
void WriteGroundProjectionFile(const std::filesystem::path& path,std::size_t count,
    const std::vector<std::uint32_t>& indices,int rows,int columns) {
  Validate(count,indices,rows,columns);
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path,std::ios::binary|std::ios::trunc);
  file.write(kMagic.data(),kMagic.size());
  Write(file,static_cast<std::uint32_t>(rows));
  Write(file,static_cast<std::uint32_t>(columns));
  Write(file,static_cast<std::uint64_t>(count));
  if(!indices.empty()) file.write(reinterpret_cast<const char*>(indices.data()),
      indices.size()*sizeof(std::uint32_t));
  file.close();
  if(!file.good()) throw std::runtime_error("cannot write ground projection cache: "+path.string());
}
std::vector<std::uint32_t> ReadGroundProjectionFile(const std::filesystem::path& path,
    std::size_t count,int rows,int columns) {
  std::ifstream file(path,std::ios::binary);
  std::array<char,8> magic{};std::uint32_t stored_rows=0,stored_columns=0;
  std::uint64_t stored_count=0;
  file.read(magic.data(),magic.size());
  Read(file,stored_rows);Read(file,stored_columns);Read(file,stored_count);
  if(!file.good()||magic!=kMagic||stored_count!=count ||
      rows<=0||columns<=0||stored_rows!=static_cast<std::uint32_t>(rows)||
      stored_columns!=static_cast<std::uint32_t>(columns))
    throw std::runtime_error("missing/corrupt ground projection cache: "+path.string());
  if(count>std::numeric_limits<std::size_t>::max()/sizeof(std::uint32_t) ||
      std::filesystem::file_size(path)!=kMagic.size()+2*sizeof(std::uint32_t)+
          sizeof(std::uint64_t)+count*sizeof(std::uint32_t))
    throw std::runtime_error("ground projection cache payload size mismatch: "+path.string());
  std::vector<std::uint32_t> indices(count);
  if(count) file.read(reinterpret_cast<char*>(indices.data()),count*sizeof(std::uint32_t));
  if(!file.good()) throw std::runtime_error("unreadable ground projection cache: "+path.string());
  try { Validate(count,indices,rows,columns); }
  catch(const std::exception& error) {
    throw std::runtime_error("invalid ground projection cache "+path.string()+": "+error.what());
  }
  return indices;
}
}
