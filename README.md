### SIMD accelerated BVH

[Basic Bounding Value Heirarchy read](https://pbr-book.org/3ed-2018/Primitives_and_Intersection_Acceleration/Bounding_Volume_Hierarchies)

SIMDop algorithm:

- Create a BVH from set of polygonal shapes
  - Top down breaking of the C-space
  - Wrapped hierarchy for BV struct.
  - Splitting criterion: Batch Neural Clustering
    - Use center of polygons to cluster the C-space polygons.
    - Read more into how to do BNC.
    - Ask authors what's up with their polygon implementation that they did not mention in paper.
  - 