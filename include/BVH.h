#include <algorithm>
#include <parallel/algorithm>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <limits>

#include "AABB.h"
#include "polygon.h"
#include <x86intrin.h>

struct alignas(32) avx_double {
    __m256d val;
    avx_double() = default;
    avx_double(__m256d v) : val(v) {}
    operator __m256d&()             { return val; }
    operator const __m256d&() const { return val; }
};

using m256d_vec = std::vector<avx_double>;

struct SIMDEngine {
    enum class Level { SSE4, AVX2, AVX512 } level;
    int lane_width;     // floats per register: 4 / 8 / 16
    int leaf_threshold; // SIMD group size == lane_width

    SIMDEngine() {
        __builtin_cpu_init();
#if defined(__AVX512F__)
        if(__builtin_cpu_supports("avx512f")){
            level = Level::AVX512; lane_width = 16;
        } else
#endif
#if defined(__AVX2__)
        if(__builtin_cpu_supports("avx2")){
            level = Level::AVX2; lane_width = 8;
        } else
#endif
        {
            level = Level::SSE4; lane_width = 4;
        }
        leaf_threshold = lane_width;
    }

    const char* name() const {
        switch(level){
            case Level::AVX512: return "AVX-512 (16 floats/reg)";
            case Level::AVX2:   return "AVX2    ( 8 floats/reg)";
            default:            return "SSE4    ( 4 floats/reg)";
        }
    }
};

struct Environment{
    public:
        std::unordered_map<polygon*, bool> _objects;
        std::pair<float, float> _x_range, _y_range, _z_range;

        Environment(std::pair<float, float> xRange, std::pair<float, float> yRange, std::pair<float, float> zRange) :
        _x_range(xRange), _y_range(yRange), _z_range(zRange){}
};

struct node{
    std::vector<std::pair<polygon*, bool>> _objects;
    std::vector<node*> children;

    enum SplitAxis {X, Y, Z} _split_axis;
    float _x_range, _y_range, _z_range;

    float _volume;
    AABB _region;
    // SoA range: set on nodes that serve as SIMD-leaf groups.
    // _soa_last == _soa_first means "not a SIMD leaf".
    int _soa_first = 0, _soa_last = 0;

    bool isLeaf()    { return children.empty(); }
    bool isSIMDLeaf(){ return children.empty(); }

    node(std::vector<std::pair<polygon*, bool>> objects, AABB bbox):
        _objects(objects), _region(bbox),
        _x_range(bbox._x_max - bbox._x_min),
        _y_range(bbox._y_max - bbox._y_min),
        _z_range(bbox._z_max - bbox._z_min)
    {
        if     (_x_range >= _y_range && _x_range >= _z_range) _split_axis = X;
        else if(_y_range >= _x_range && _y_range >= _z_range) _split_axis = Y;
        else                                                   _split_axis = Z;

        __gnu_parallel::sort(_objects.begin(), _objects.end(),
            [this](const std::pair<polygon*, bool>& p1, const std::pair<polygon*, bool>& p2){
                return _return_value(_split_axis, p1.first) < _return_value(_split_axis, p2.first);
            });
    }

    float _return_value(SplitAxis ax, polygon* p){
        switch(ax){
            case X: return p->_x_centroid;
            case Y: return p->_y_centroid;
            case Z: return p->_z_centroid;
            default: return 0.0f;
        }
    }
};

class BVH{
    private:
        int n;
        Environment env;
        std::vector<node> treeNodes;
        float _threshold;
        SIMDEngine simd_engine;

        // SoA bounds: filled for nodes whose subtree has <= lane_width objects.
        // Padded to a multiple of 16 (widest register) with non-overlapping
        // sentinels so every SIMD variant can over-read without a bounds check.
        std::vector<float> _soa_xmin, _soa_ymin, _soa_zmin;
        std::vector<float> _soa_xmax, _soa_ymax, _soa_zmax;

        void _build_soa(node* n){
            if(n->isLeaf()){
                n->_soa_first = (int)_soa_xmin.size();
                for(auto& p : n->_objects){
                    AABB* a = static_cast<AABB*>(p.first);
                    _soa_xmin.push_back(a->_x_min); _soa_ymin.push_back(a->_y_min); _soa_zmin.push_back(a->_z_min);
                    _soa_xmax.push_back(a->_x_max); _soa_ymax.push_back(a->_y_max); _soa_zmax.push_back(a->_z_max);
                }
                n->_soa_last = (int)_soa_xmin.size();
            } else {
                for(auto c : n->children) _build_soa(c);
            }
        }

        // ── overlap checks ────────────────────────────────────────────────────

        bool checkOverlap(const AABB& a, const AABB& b){
            return (a._x_min <= b._x_max && a._x_max >= b._x_min) &&
                   (a._y_min <= b._y_max && a._y_max >= b._y_min) &&
                   (a._z_min <= b._z_max && a._z_max >= b._z_min);
        }

        bool checkOverlap_simd(const AABB& a, const AABB& b){
            __m128 a_mins = _mm_loadu_ps(&a._x_min);
            __m128 a_maxs = _mm_loadu_ps(&a._x_max);
            __m128 b_mins = _mm_loadu_ps(&b._x_min);
            __m128 b_maxs = _mm_loadu_ps(&b._x_max);
            __m128 cmp = _mm_and_ps(_mm_cmple_ps(a_mins, b_maxs), _mm_cmple_ps(b_mins, a_maxs));
            return (_mm_movemask_ps(cmp) & 0x7) == 0x7;
        }

        int checkOverlap_batch4(const AABB* a0, const AABB* a1,
                                const AABB* b0, const AABB* b1){
            __m128 axmin = _mm_set_ps(a1->_x_min, a1->_x_min, a0->_x_min, a0->_x_min);
            __m128 aymin = _mm_set_ps(a1->_y_min, a1->_y_min, a0->_y_min, a0->_y_min);
            __m128 azmin = _mm_set_ps(a1->_z_min, a1->_z_min, a0->_z_min, a0->_z_min);
            __m128 axmax = _mm_set_ps(a1->_x_max, a1->_x_max, a0->_x_max, a0->_x_max);
            __m128 aymax = _mm_set_ps(a1->_y_max, a1->_y_max, a0->_y_max, a0->_y_max);
            __m128 azmax = _mm_set_ps(a1->_z_max, a1->_z_max, a0->_z_max, a0->_z_max);
            __m128 bxmin = _mm_set_ps(b1->_x_min, b0->_x_min, b1->_x_min, b0->_x_min);
            __m128 bymin = _mm_set_ps(b1->_y_min, b0->_y_min, b1->_y_min, b0->_y_min);
            __m128 bzmin = _mm_set_ps(b1->_z_min, b0->_z_min, b1->_z_min, b0->_z_min);
            __m128 bxmax = _mm_set_ps(b1->_x_max, b0->_x_max, b1->_x_max, b0->_x_max);
            __m128 bymax = _mm_set_ps(b1->_y_max, b0->_y_max, b1->_y_max, b0->_y_max);
            __m128 bzmax = _mm_set_ps(b1->_z_max, b0->_z_max, b1->_z_max, b0->_z_max);
            __m128 ox = _mm_and_ps(_mm_cmple_ps(axmin,bxmax), _mm_cmple_ps(bxmin,axmax));
            __m128 oy = _mm_and_ps(_mm_cmple_ps(aymin,bymax), _mm_cmple_ps(bymin,aymax));
            __m128 oz = _mm_and_ps(_mm_cmple_ps(azmin,bzmax), _mm_cmple_ps(bzmin,azmax));
            return _mm_movemask_ps(_mm_and_ps(_mm_and_ps(ox,oy),oz));
        }

        // ── dispatched leaf SIMD checks ───────────────────────────────────────

        void _leaf_sse4(node* ca, node* cb,
                        std::vector<std::pair<node*,node*>>& result, bool findAll){
            for(int i = ca->_soa_first; i < ca->_soa_last; i++){
                __m128 axmin = _mm_set1_ps(_soa_xmin[i]), aymin = _mm_set1_ps(_soa_ymin[i]),
                       azmin = _mm_set1_ps(_soa_zmin[i]), axmax = _mm_set1_ps(_soa_xmax[i]),
                       aymax = _mm_set1_ps(_soa_ymax[i]), azmax = _mm_set1_ps(_soa_zmax[i]);
                for(int j = cb->_soa_first; j < cb->_soa_last; j += 4){
                    __m128 ox = _mm_and_ps(_mm_cmple_ps(axmin, _mm_loadu_ps(&_soa_xmax[j])),
                                           _mm_cmple_ps(_mm_loadu_ps(&_soa_xmin[j]), axmax));
                    __m128 oy = _mm_and_ps(_mm_cmple_ps(aymin, _mm_loadu_ps(&_soa_ymax[j])),
                                           _mm_cmple_ps(_mm_loadu_ps(&_soa_ymin[j]), aymax));
                    __m128 oz = _mm_and_ps(_mm_cmple_ps(azmin, _mm_loadu_ps(&_soa_zmax[j])),
                                           _mm_cmple_ps(_mm_loadu_ps(&_soa_zmin[j]), azmax));
                    int mask = _mm_movemask_ps(_mm_and_ps(_mm_and_ps(ox,oy),oz));
                    mask &= (1 << std::min(4, cb->_soa_last - j)) - 1;
                    while(mask){ result.push_back({ca,cb}); if(!findAll) return; mask &= mask-1; }
                }
            }
        }

#if defined(__AVX2__)
        void _leaf_avx2(node* ca, node* cb,
                        std::vector<std::pair<node*,node*>>& result, bool findAll){
            for(int i = ca->_soa_first; i < ca->_soa_last; i++){
                __m256 axmin = _mm256_set1_ps(_soa_xmin[i]), aymin = _mm256_set1_ps(_soa_ymin[i]),
                       azmin = _mm256_set1_ps(_soa_zmin[i]), axmax = _mm256_set1_ps(_soa_xmax[i]),
                       aymax = _mm256_set1_ps(_soa_ymax[i]), azmax = _mm256_set1_ps(_soa_zmax[i]);
                for(int j = cb->_soa_first; j < cb->_soa_last; j += 8){
                    __m256 ox = _mm256_and_ps(
                        _mm256_cmp_ps(axmin, _mm256_loadu_ps(&_soa_xmax[j]), _CMP_LE_OS),
                        _mm256_cmp_ps(_mm256_loadu_ps(&_soa_xmin[j]), axmax, _CMP_LE_OS));
                    __m256 oy = _mm256_and_ps(
                        _mm256_cmp_ps(aymin, _mm256_loadu_ps(&_soa_ymax[j]), _CMP_LE_OS),
                        _mm256_cmp_ps(_mm256_loadu_ps(&_soa_ymin[j]), aymax, _CMP_LE_OS));
                    __m256 oz = _mm256_and_ps(
                        _mm256_cmp_ps(azmin, _mm256_loadu_ps(&_soa_zmax[j]), _CMP_LE_OS),
                        _mm256_cmp_ps(_mm256_loadu_ps(&_soa_zmin[j]), azmax, _CMP_LE_OS));
                    int mask = _mm256_movemask_ps(_mm256_and_ps(_mm256_and_ps(ox,oy),oz));
                    mask &= (1 << std::min(8, cb->_soa_last - j)) - 1;
                    while(mask){ result.push_back({ca,cb}); if(!findAll) return; mask &= mask-1; }
                }
            }
        }
#endif

#if defined(__AVX512F__)
        void _leaf_avx512(node* ca, node* cb,
                          std::vector<std::pair<node*,node*>>& result, bool findAll){
            for(int i = ca->_soa_first; i < ca->_soa_last; i++){
                __m512 axmin = _mm512_set1_ps(_soa_xmin[i]), aymin = _mm512_set1_ps(_soa_ymin[i]),
                       azmin = _mm512_set1_ps(_soa_zmin[i]), axmax = _mm512_set1_ps(_soa_xmax[i]),
                       aymax = _mm512_set1_ps(_soa_ymax[i]), azmax = _mm512_set1_ps(_soa_zmax[i]);
                for(int j = cb->_soa_first; j < cb->_soa_last; j += 16){
                    __mmask16 ox = _mm512_cmple_ps_mask(axmin, _mm512_loadu_ps(&_soa_xmax[j])) &
                                   _mm512_cmple_ps_mask(_mm512_loadu_ps(&_soa_xmin[j]), axmax);
                    __mmask16 oy = _mm512_cmple_ps_mask(aymin, _mm512_loadu_ps(&_soa_ymax[j])) &
                                   _mm512_cmple_ps_mask(_mm512_loadu_ps(&_soa_ymin[j]), aymax);
                    __mmask16 oz = _mm512_cmple_ps_mask(azmin, _mm512_loadu_ps(&_soa_zmax[j])) &
                                   _mm512_cmple_ps_mask(_mm512_loadu_ps(&_soa_zmin[j]), azmax);
                    uint16_t mask = ox & oy & oz;
                    mask &= (uint16_t)((1u << std::min(16, cb->_soa_last - j)) - 1);
                    while(mask){ result.push_back({ca,cb}); if(!findAll) return; mask &= mask-1; }
                }
            }
        }
#endif

        void _leaf_simd(node* ca, node* cb,
                        std::vector<std::pair<node*,node*>>& result, bool findAll){
            switch(simd_engine.level){
#if defined(__AVX512F__)
                case SIMDEngine::Level::AVX512: _leaf_avx512(ca, cb, result, findAll); break;
#endif
#if defined(__AVX2__)
                case SIMDEngine::Level::AVX2:   _leaf_avx2  (ca, cb, result, findAll); break;
#endif
                default:                        _leaf_sse4  (ca, cb, result, findAll); break;
            }
        }

    public:
        const SIMDEngine& simd() const { return simd_engine; }

        BVH(Environment _env): env(_env){
            point p1(env._x_range.first,  env._y_range.first,  env._z_range.first);
            point p2(env._x_range.second, env._y_range.second, env._z_range.second);
            std::vector<std::pair<polygon*, bool>> objVec(env._objects.begin(), env._objects.end());
            treeNodes.push_back(node(objVec, AABB(p1, p2)));
        }

        void create_BVH(node* root){
            if(root->_objects.size() <= (size_t)simd_engine.leaf_threshold) return;

            float split_val = root->_return_value(root->_split_axis, &root->_region);

            for(int i = 0; i < 2; i++){
                std::vector<std::pair<polygon*, bool>> objects;
                float rx_min = INFINITY,  ry_min = INFINITY,  rz_min = INFINITY;
                float rx_max = -INFINITY, ry_max = -INFINITY, rz_max = -INFINITY;

                for(auto& k : root->_objects){
                    if(!k.second) continue;
                    float obj_val = root->_return_value(root->_split_axis, k.first);
                    bool inGroup = (i == 0) ? (obj_val <= split_val) : (obj_val > split_val);
                    if(!inGroup) continue;
                    objects.push_back({k.first, true});
                    k.second = false;
                    rx_min = std::min(rx_min, k.first->_x_min); ry_min = std::min(ry_min, k.first->_y_min); rz_min = std::min(rz_min, k.first->_z_min);
                    rx_max = std::max(rx_max, k.first->_x_max); ry_max = std::max(ry_max, k.first->_y_max); rz_max = std::max(rz_max, k.first->_z_max);
                }

                if(objects.empty()) continue;
                if(objects.size() == root->_objects.size()) return;

                node* child = new node(objects, AABB(point(rx_min,ry_min,rz_min), point(rx_max,ry_max,rz_max)));
                create_BVH(child);
                root->children.push_back(child);
            }
        }

        void build(){
            create_BVH(&treeNodes[0]);
            _build_soa(&treeNodes[0]);
            size_t pad = (_soa_xmin.size() + 15) & ~15u;
            _soa_xmin.resize(pad,  std::numeric_limits<float>::infinity());
            _soa_ymin.resize(pad,  std::numeric_limits<float>::infinity());
            _soa_zmin.resize(pad,  std::numeric_limits<float>::infinity());
            _soa_xmax.resize(pad, -std::numeric_limits<float>::infinity());
            _soa_ymax.resize(pad, -std::numeric_limits<float>::infinity());
            _soa_zmax.resize(pad, -std::numeric_limits<float>::infinity());
        }

        bool has_collision(bool findAll = false){
            node* root = &treeNodes[0];
            if(root->isLeaf()){
                auto& objs = root->_objects;
                for(size_t i = 0; i < objs.size(); i++)
                    for(size_t j = i+1; j < objs.size(); j++)
                        if(checkOverlap(*static_cast<AABB*>(objs[i].first),
                                        *static_cast<AABB*>(objs[j].first))) return true;
                return false;
            }
            if(root->children.size() < 2) return false;
            std::vector<std::pair<node*,node*>> result;
            BVHtraversal(root->children[0], root->children[1], result, findAll);
            return !result.empty();
        }

        bool has_collision_simd(bool findAll = false){
            node* root = &treeNodes[0];
            if(root->isSIMDLeaf()){
                std::vector<std::pair<node*,node*>> result;
                _leaf_simd(root, root, result, findAll);
                return !result.empty();
            }
            if(root->children.size() < 2) return false;
            std::vector<std::pair<node*,node*>> result;
            BVHtraversal_simd(root->children[0], root->children[1], result, findAll);
            return !result.empty();
        }

        std::vector<std::pair<node*,node*>> get_collisions(){
            node* root = &treeNodes[0];
            std::vector<std::pair<node*,node*>> result;
            if(root->isLeaf()){
                auto& objs = root->_objects;
                for(size_t i = 0; i < objs.size(); i++)
                    for(size_t j = i+1; j < objs.size(); j++)
                        if(checkOverlap(*static_cast<AABB*>(objs[i].first),
                                        *static_cast<AABB*>(objs[j].first)))
                            result.push_back({root, root});
                return result;
            }
            if(root->children.size() < 2) return result;
            BVHtraversal(root->children[0], root->children[1], result, true);
            return result;
        }

        std::vector<std::pair<node*,node*>> get_collisions_simd(){
            node* root = &treeNodes[0];
            std::vector<std::pair<node*,node*>> result;
            if(root->isSIMDLeaf() || root->children.size() < 2) return result;
            BVHtraversal_simd(root->children[0], root->children[1], result, true);
            return result;
        }

        // ── traversals ────────────────────────────────────────────────────────

        void BVHtraversal(node* a, node* b,
                          std::vector<std::pair<node*,node*>>& result, bool findAll = false){
            std::vector<std::pair<node*,node*>> stack;
            stack.reserve(128);
            stack.push_back({a, b});
            while(!stack.empty()){
                auto [ca, cb] = stack.back(); stack.pop_back();
                if(ca->isLeaf() && cb->isLeaf()){
                    for(auto& pa : ca->_objects)
                        for(auto& pb : cb->_objects)
                            if(checkOverlap(*static_cast<AABB*>(pa.first),
                                            *static_cast<AABB*>(pb.first))){
                                result.push_back({ca, cb});
                                if(!findAll) return;
                            }
                } else if(ca->isLeaf()){
                    for(auto c : cb->children)
                        if(checkOverlap(ca->_region, c->_region)) stack.push_back({ca, c});
                } else if(cb->isLeaf()){
                    for(auto c : ca->children)
                        if(checkOverlap(c->_region, cb->_region)) stack.push_back({c, cb});
                } else {
                    for(auto n1 : ca->children)
                        for(auto n2 : cb->children)
                            if(checkOverlap(n1->_region, n2->_region))
                                stack.push_back({n1, n2});
                }
            }
        }

        // SIMD traversal uses isSIMDLeaf() instead of isLeaf().
        // At a SIMD leaf, all objects in that node's subtree are stored
        // contiguously in SoA — _mm_loadu_ps / _mm256_loadu_ps reads W at once.
        void BVHtraversal_simd(node* a, node* b,
                               std::vector<std::pair<node*,node*>>& result, bool findAll = false){
            std::vector<std::pair<node*,node*>> stack;
            stack.reserve(128);
            stack.push_back({a, b});
            while(!stack.empty()){
                auto [ca, cb] = stack.back(); stack.pop_back();
                if(ca->isSIMDLeaf() && cb->isSIMDLeaf()){
                    _leaf_simd(ca, cb, result, findAll);
                    if(!findAll && !result.empty()) return;
                } else if(ca->isSIMDLeaf()){
                    for(auto c : cb->children)
                        if(checkOverlap_simd(ca->_region, c->_region)) stack.push_back({ca, c});
                } else if(cb->isSIMDLeaf()){
                    for(auto c : ca->children)
                        if(checkOverlap_simd(c->_region, cb->_region)) stack.push_back({c, cb});
                } else {
                    if(ca->children.size() == 2 && cb->children.size() == 2){
                        node* ac0 = ca->children[0]; node* ac1 = ca->children[1];
                        node* bc0 = cb->children[0]; node* bc1 = cb->children[1];
                        int mask = checkOverlap_batch4(&ac0->_region, &ac1->_region,
                                                       &bc0->_region, &bc1->_region);
                        node* pa[4] = {ac0, ac0, ac1, ac1};
                        node* pb[4] = {bc0, bc1, bc0, bc1};
                        for(int i = 0; i < 4; i++)
                            if(mask & (1 << i)) stack.push_back({pa[i], pb[i]});
                    } else {
                        for(auto n1 : ca->children)
                            for(auto n2 : cb->children)
                                if(checkOverlap_simd(n1->_region, n2->_region))
                                    stack.push_back({n1, n2});
                    }
                }
            }
        }

        __m256d BVHintersection_SIMD(m256d_vec a, m256d_vec b){
            __m256d endResult    = _mm256_setzero_pd();
            __m256d all_bits_set = _mm256_set1_pd(-nan(""));
            for(size_t i = 0; i != a.size() / 2; i++){
                __m256d resL    = _mm256_cmp_pd(a[i].val,             b[i].val,             _CMP_LT_OS);
                __m256d resH    = _mm256_cmp_pd(a[a.size()/2+i].val,  b[a.size()/2+i].val,  _CMP_GT_OS);
                endResult = _mm256_or_pd(endResult, _mm256_or_pd(resL, resH));
                if(_mm256_testc_pd(endResult, all_bits_set) == 1) break;
            }
            return endResult;
        }
};
