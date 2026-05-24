#include "include/BVH.h"
#include <cassert>
#include <cstdio>
#include <chrono>
#include <random>

using hrclock  = std::chrono::high_resolution_clock;
using us_t     = std::chrono::microseconds;
using ns_t     = std::chrono::nanoseconds;

static double elapsed_us(hrclock::time_point t0){
    return std::chrono::duration_cast<us_t>(hrclock::now() - t0).count();
}

// Run f() REPS times across TRIALS trials; return best (min) total µs.
// Min across trials eliminates OS scheduling noise.
template<typename F>
static double bench_min(int reps, int trials, F f){
    double best = std::numeric_limits<double>::max();
    for(int t = 0; t < trials; t++){
        auto t0 = hrclock::now();
        for(int i = 0; i < reps; i++) f();
        double us = std::chrono::duration_cast<ns_t>(hrclock::now() - t0).count() / 1000.0;
        if(us < best) best = us;
    }
    return best;
}

int main(){
    // --- AABB basic properties ---
    auto t0 = hrclock::now();
    AABB box1(point(0,0,0), point(2,2,2));
    assert(box1.volume()       == 8.0f);
    assert(box1.surface_area() == 24.0f);
    assert(box1._x_centroid    == 1.0f);
    assert(box1._x_range       == 2.0f);
    printf("AABB properties:  OK  (%.2f us)\n", elapsed_us(t0));

    // --- contains ---
    t0 = hrclock::now();
    point inside(1,1,1), outside(3,3,3);
    assert( box1.contains(&inside));
    assert(!box1.contains(&outside));
    printf("AABB contains:    OK  (%.2f us)\n", elapsed_us(t0));

    // --- operator+ (union / expand) ---
    t0 = hrclock::now();
    AABB box2(point(1,1,1), point(3,3,3));
    polygon& merged = box1 + static_cast<polygon&>(box2);
    assert(merged._x_min == 0.0f && merged._x_max == 3.0f);
    assert(merged._y_min == 0.0f && merged._y_max == 3.0f);
    printf("AABB union:       OK  (%.2f us)\n", elapsed_us(t0));

    // --- BVH: overlapping objects straddling the spatial midpoint ---
    // Environment midpoint is (5,5,5). Object centroids must be on opposite
    // sides so they land in different subtrees (left: centroid <= 5, right: > 5).
    {
        Environment env({0.0f, 10.0f}, {0.0f, 10.0f}, {0.0f, 10.0f});
        AABB* a = new AABB(point(0,0,0), point(6,6,6)); // centroid 3 → left subtree
        AABB* b = new AABB(point(5,5,5), point(9,9,9)); // centroid 7 → right subtree
        // a and b overlap in [5,6]^3
        env._objects[a] = true;
        env._objects[b] = true;

        BVH bvh(env);
        t0 = hrclock::now();
        bvh.build();
        double build_ms = elapsed_us(t0);

        t0 = hrclock::now();
        assert(bvh.has_collision(false) == true);
        double early_us = elapsed_us(t0);

        t0 = hrclock::now();
        assert(bvh.has_collision(true)  == true);
        double full_us = elapsed_us(t0);

        auto pairs = bvh.get_collisions();
        assert(!pairs.empty());
        printf("BVH collision (overlapping):  OK  (%zu pair(s))  build=%.2f us  early-exit=%.2f us  full-scan=%.2f us\n",
               pairs.size(), build_ms, early_us, full_us);

        delete a; delete b;
    }

    // --- BVH: non-overlapping objects on opposite sides ---
    {
        Environment env({0.0f, 10.0f}, {0.0f, 10.0f}, {0.0f, 10.0f});
        AABB* a = new AABB(point(0,0,0), point(3,3,3)); // centroid 1.5 → left
        AABB* b = new AABB(point(7,7,7), point(9,9,9)); // centroid 8   → right
        env._objects[a] = true;
        env._objects[b] = true;

        BVH bvh(env);
        t0 = hrclock::now();
        bvh.build();
        double build_ms = elapsed_us(t0);

        t0 = hrclock::now();
        assert(bvh.has_collision(false) == false);
        assert(bvh.has_collision(true)  == false);
        assert(bvh.get_collisions().empty());
        printf("BVH collision (separate):     OK  build=%.2f us  query=%.2f us\n",
               build_ms, elapsed_us(t0));

        delete a; delete b;
    }

    // --- BVHintersection_SIMD ---
    // Per axis per lane the function sets a bit when: a_min < b_min OR a_max > b_max.
    // Pack 4 scenarios into the 4 lanes of a single __m256d pair (1 axis group).
    //
    //  Lane 0: A=[0,5],  B=[1,4]  → A contains B      → resL(0<1)=T             → bit SET
    //  Lane 1: A=[1,4],  B=[0,5]  → B contains A      → resL(1<0)=F, resH(4>5)=F → bit CLEAR
    //  Lane 2: A=[0,3],  B=[2,4]  → partial overlap   → resL(0<2)=T             → bit SET
    //  Lane 3: A=[6,8],  B=[0,1]  → no overlap (A right of B) → resH(8>1)=T      → bit SET
    //
    // _mm256_set_pd(e3,e2,e1,e0): last arg = lane 0.
    {
        __m256d a_lower = _mm256_set_pd(6.0, 0.0, 1.0, 0.0); // lanes 3,2,1,0
        __m256d a_upper = _mm256_set_pd(8.0, 3.0, 4.0, 5.0);
        __m256d b_lower = _mm256_set_pd(0.0, 2.0, 0.0, 1.0);
        __m256d b_upper = _mm256_set_pd(1.0, 4.0, 5.0, 4.0);

        Environment env({0.0f,10.0f},{0.0f,10.0f},{0.0f,10.0f});
        BVH bvh(env);

        t0 = hrclock::now();
        __m256d result = bvh.BVHintersection_SIMD(m256d_vec{a_lower, a_upper},
                                                   m256d_vec{b_lower, b_upper});
        double simd_us = elapsed_us(t0);
        int mask = _mm256_movemask_pd(result);
        // Expected: lanes 0,2,3 set → 0b1101 = 0xD
        // Lane 1 (B contains A) produces no bits — known limitation of this check.
        assert(mask == 0xD);
        printf("SIMD intersection: OK (mask=0x%X)  %.2f us\n", mask, simd_us);
    }

    // --- Benchmark: 100 random objects in [0,100]^3 ---
    printf("\n--- Benchmark (10000 random objects vol>=1000 in [0,100]^3, 10000 iterations) ---\n");
    {
        Environment _tmp({0,1},{0,1},{0,1});
        BVH _tmp_bvh(_tmp);
        printf("  SIMD engine: %s  (leaf threshold=%d)\n",
               _tmp_bvh.simd().name(), _tmp_bvh.simd().leaf_threshold);
    }
    {
        const int   N    = 10000;
        const int   REPS = 10000;
        // Volume >= 1000 → side length >= cbrt(1000) = 10.
        // Use uniform random side in [10, 20] for variety.
        const float W_MIN = 10.0f;
        const float W_MAX = 20.0f;

        std::mt19937 rng(42);
        std::uniform_real_distribution<float> pos_dist(0.0f, 100.0f - W_MAX);
        std::uniform_real_distribution<float> w_dist(W_MIN, W_MAX);

        Environment env({0.0f, 100.0f}, {0.0f, 100.0f}, {0.0f, 100.0f});
        std::vector<AABB*> boxes;
        boxes.reserve(N);

        for(int i = 0; i < N; i++){
            float x = pos_dist(rng), y = pos_dist(rng), z = pos_dist(rng);
            float w = w_dist(rng);
            AABB* b = new AABB(point(x, y, z), point(x+w, y+w, z+w));
            env._objects[b] = true;
            boxes.push_back(b);
        }

        // Build
        BVH bvh(env);
        t0 = hrclock::now();
        bvh.build();
        printf("  build:            %.2f us\n", elapsed_us(t0));

        // Count pairs once (full scan) to show scene density
        size_t total_pairs = bvh.get_collisions().size();
        printf("  colliding pairs:  %zu\n", total_pairs);

        const int TRIALS    = 7;
        const int REPS_FULL = 100;  // full scan allocates ~227k pairs per call

        // --- Early-exit (first collision only) ---
        volatile bool scalar_hit = false, simd_hit = false;
        double scalar_early = bench_min(REPS,      TRIALS, [&]{ scalar_hit = bvh.has_collision(false); });
        double simd_early   = bench_min(REPS,      TRIALS, [&]{ simd_hit   = bvh.has_collision_simd(false); });

        printf("\n  [early-exit]\n");
        printf("  BVH scalar (best of %d): %.4f us/query  (result=%d)\n",
               TRIALS, scalar_early / REPS, (int)scalar_hit);
        printf("  BVH SIMD   (best of %d): %.4f us/query  (result=%d)\n",
               TRIALS, simd_early   / REPS, (int)simd_hit);
        printf("  SIMD speedup: %.2fx\n", scalar_early / simd_early);

        // --- Full scan (all colliding pairs) ---
        volatile size_t scalar_np = 0, simd_np = 0;
        double scalar_full = bench_min(REPS_FULL, TRIALS, [&]{ scalar_np = bvh.get_collisions().size(); });
        double simd_full   = bench_min(REPS_FULL, TRIALS, [&]{ simd_np   = bvh.get_collisions_simd().size(); });

        printf("\n  [full scan — all pairs]\n");
        printf("  BVH scalar (best of %d): %.2f us / %d reps  →  %.2f us/query  (pairs=%zu)\n",
               TRIALS, scalar_full, REPS_FULL, scalar_full / REPS_FULL, (size_t)scalar_np);
        printf("  BVH SIMD   (best of %d): %.2f us / %d reps  →  %.2f us/query  (pairs=%zu)\n",
               TRIALS, simd_full,   REPS_FULL, simd_full   / REPS_FULL, (size_t)simd_np);
        printf("  SIMD speedup: %.2fx\n", scalar_full / simd_full);

        for(auto b : boxes) delete b;
    }

    printf("\nAll tests passed.\n");
    return 0;
}
