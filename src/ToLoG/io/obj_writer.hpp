#include <ToLoG/utils/indices.hpp>
#include <ToLoG/AABBTree.hpp>
#include <filesystem>
#include <fstream>

namespace ToLoG
{

template<typename T>
int write_aabb_tree_obj(const std::filesystem::path& _path, const AABBTree<T>& _tree, const bool _include_only_leafs=true)
{
    static_assert(AABBTree<T>::DIM==3);
    std::ofstream file(_path);
    unsigned int idx(1);
    for (uint32_t node_i = 0; node_i < _tree.n_nodes(); ++node_i)
    {
        if (_include_only_leafs && !_tree.is_leaf_node(node_i)) {continue;}
        const auto& aabb = _tree.node_aabb(node_i);
        for (const auto& v : aabb.corners()) {
            file << "v "
                 << v.x() << " "
                 << v.y() << " "
                 << v.z() << std::endl;
        }
        for (const auto& f : cube_vertex_indices) {
            file << "f "
                 << (idx+f[0]) << " "
                 << (idx+f[1]) << " "
                 << (idx+f[2]) << " "
                 << (idx+f[3]) << std::endl;
        }
        idx += 8;
    }
    file.close();
    return 0;
}

}
