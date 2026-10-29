#pragma once
#include <filesystem>
#include <cstddef>
#include <cstdint>
#include <vector>
namespace lio_sam_hesai {
void WriteGroundProjectionFile(const std::filesystem::path& path,
    std::size_t point_count,const std::vector<std::uint32_t>& indices,
    int rows,int columns);
std::vector<std::uint32_t> ReadGroundProjectionFile(
    const std::filesystem::path& path,std::size_t point_count,int rows,int columns);
}
