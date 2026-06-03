#include <ToLoG/vector_concepts.hpp>
#include <ToLoG/utils/indices.hpp>
#include <filesystem>
#include <fstream>
#include <span>

namespace ToLoG::IO
{

template<typename Vertex, typename Line, typename Face>
void write_obj(
    const std::filesystem::path& _path,
    std::span<Vertex> _vertices,
    std::span<Line> _lines,
    std::span<Face> _faces)
{
    std::ofstream file(_path);
    for (const auto& v : _vertices) {
        file << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl;
    }
    for (const auto& l : _lines) {
        file << "l";
        for (const auto& i : l) {
            file << " " << (static_cast<int>(i)+1);
        }
        file << std::endl;
    }
    for (const auto& f : _faces) {
        file << "f";
        for (const auto& i : f) {
            file << " " << (static_cast<int>(i)+1);
        }
        file << std::endl;
    }
    file.close();
}

}
