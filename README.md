# raylib-obb
![obb-collision](https://github.com/user-attachments/assets/03360a27-6ebe-4e97-9538-3d8c7e30462c)


**Oriented Bounding Box (OBB) Collision System for raylib (C)**

This is a lightweight, raylib-compatible OBB collision module written in C. It provides high-performance functions for 3D collision detection using oriented bounding boxes, including:

- OBB vs OBB (Separating Axis Theorem)
- OBB vs BoundingBox (AABB)
- OBB vs Sphere
- Ray vs OBB
- Point inside OBB check
- OBB corner generation
- Wireframe OBB debug drawing

---

## Features

- Uses `Vector3`, `Quaternion`, `BoundingBox`, `Matrix`, etc.
- No dynamic allocations
- Easy to integrate: just include `obb.h`
