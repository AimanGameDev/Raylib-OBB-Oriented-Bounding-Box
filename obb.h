#ifndef OBB_H
#define OBB_H

#include "raylib.h"
#include "raymath.h"

typedef struct OBB
{
    Quaternion rotation;
    Vector3 center;
    Vector3 halfExtents;
};

inline void OBB_GetAxes(const OBB *obb, Vector3 *right, Vector3 *up, Vector3 *forward)
{
    Matrix rot = QuaternionToMatrix(obb->rotation);

    *right = (Vector3){rot.m0, rot.m1, rot.m2};
    *up = (Vector3){rot.m4, rot.m5, rot.m6};
    *forward = (Vector3){rot.m8, rot.m9, rot.m10};
}

inline void OBB_GetCorners(const OBB *obb, Vector3 corners[8])
{
    Vector3 right, up, forward;
    OBB_GetAxes(obb, &right, &up, &forward);

    right = Vector3Scale(right, obb->halfExtents.x);
    up = Vector3Scale(up, obb->halfExtents.y);
    forward = Vector3Scale(forward, obb->halfExtents.z);

    corners[0] = Vector3Add(Vector3Add(Vector3Add(obb->center, right), up), forward);
    corners[1] = Vector3Add(Vector3Add(Vector3Subtract(obb->center, right), up), forward);
    corners[2] = Vector3Add(Vector3Add(Vector3Subtract(obb->center, right), up), Vector3Negate(forward));
    corners[3] = Vector3Add(Vector3Add(Vector3Add(obb->center, right), up), Vector3Negate(forward));

    corners[4] = Vector3Add(Vector3Add(Vector3Add(obb->center, right), Vector3Negate(up)), forward);
    corners[5] = Vector3Add(Vector3Add(Vector3Subtract(obb->center, right), Vector3Negate(up)), forward);
    corners[6] = Vector3Add(Vector3Add(Vector3Subtract(obb->center, right), Vector3Negate(up)), Vector3Negate(forward));
    corners[7] = Vector3Add(Vector3Add(Vector3Add(obb->center, right), Vector3Negate(up)), Vector3Negate(forward));
}

inline void OBB_DrawWireframe(const OBB *obb, Color color)
{
    Vector3 c[8];
    OBB_GetCorners(obb, c);

    DrawLine3D(c[0], c[1], color);
    DrawLine3D(c[1], c[2], color);
    DrawLine3D(c[2], c[3], color);
    DrawLine3D(c[3], c[0], color);

    DrawLine3D(c[4], c[5], color);
    DrawLine3D(c[5], c[6], color);
    DrawLine3D(c[6], c[7], color);
    DrawLine3D(c[7], c[4], color);

    DrawLine3D(c[0], c[4], color);
    DrawLine3D(c[1], c[5], color);
    DrawLine3D(c[2], c[6], color);
    DrawLine3D(c[3], c[7], color);
}

inline bool OBB_ContainsPoint(const OBB *obb, Vector3 point)
{
    Vector3 local = Vector3Subtract(point, obb->center);

    Quaternion inverseRot = QuaternionInvert(obb->rotation);
    local = Vector3RotateByQuaternion(local, inverseRot);

    return fabsf(local.x) <= obb->halfExtents.x &&
           fabsf(local.y) <= obb->halfExtents.y &&
           fabsf(local.z) <= obb->halfExtents.z;
}

inline void ProjectBoundingBoxOntoAxis(const BoundingBox *box, Vector3 axis, float *outMin, float *outMax)
{
    Vector3 corners[8] = {
        {box->min.x, box->min.y, box->min.z},
        {box->max.x, box->min.y, box->min.z},
        {box->max.x, box->max.y, box->min.z},
        {box->min.x, box->max.y, box->min.z},
        {box->min.x, box->min.y, box->max.z},
        {box->max.x, box->min.y, box->max.z},
        {box->max.x, box->max.y, box->max.z},
        {box->min.x, box->max.y, box->max.z}};

    float min = Vector3DotProduct(corners[0], axis);
    float max = min;

    for (int i = 1; i < 8; ++i)
    {
        float projection = Vector3DotProduct(corners[i], axis);
        if (projection < min)
            min = projection;
        if (projection > max)
            max = projection;
    }

    *outMin = min;
    *outMax = max;
}

inline void ProjectOBBOntoAxis(const OBB *obb, Vector3 axis, float *outMin, float *outMax)
{
    Vector3 right, up, forward;
    OBB_GetAxes(obb, &right, &up, &forward);

    float r =
        fabsf(Vector3DotProduct(right, axis)) * obb->halfExtents.x +
        fabsf(Vector3DotProduct(up, axis)) * obb->halfExtents.y +
        fabsf(Vector3DotProduct(forward, axis)) * obb->halfExtents.z;

    float centerProj = Vector3DotProduct(obb->center, axis);
    *outMin = centerProj - r;
    *outMax = centerProj + r;
}

inline bool CheckCollisionBoundingBoxVsOBB(const BoundingBox *box, const OBB *obb)
{
    Vector3 aabbAxes[3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}};

    Vector3 obbAxes[3];
    OBB_GetAxes(obb, &obbAxes[0], &obbAxes[1], &obbAxes[2]);

    Vector3 testAxes[15];
    int axisCount = 0;

    for (int i = 0; i < 3; i++)
        testAxes[axisCount++] = aabbAxes[i];

    for (int i = 0; i < 3; i++)
        testAxes[axisCount++] = obbAxes[i];

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Vector3 cross = Vector3CrossProduct(aabbAxes[i], obbAxes[j]);
            if (Vector3LengthSqr(cross) > 0.000001f)
            {
                testAxes[axisCount++] = Vector3Normalize(cross);
            }
        }
    }

    for (int i = 0; i < axisCount; ++i)
    {
        Vector3 axis = testAxes[i];
        float minA, maxA, minB, maxB;

        ProjectBoundingBoxOntoAxis(box, axis, &minA, &maxA);
        ProjectOBBOntoAxis(obb, axis, &minB, &maxB);

        if (maxA < minB || maxB < minA)
        {
            return false;
        }
    }

    return true;
}

inline bool CheckCollisionOBBvsOBB(const OBB *a, const OBB *b)
{
    Vector3 axesA[3], axesB[3];
    OBB_GetAxes(a, &axesA[0], &axesA[1], &axesA[2]);
    OBB_GetAxes(b, &axesB[0], &axesB[1], &axesB[2]);

    Vector3 testAxes[15];
    int axisCount = 0;

    for (int i = 0; i < 3; ++i)
        testAxes[axisCount++] = axesA[i];

    for (int i = 0; i < 3; ++i)
        testAxes[axisCount++] = axesB[i];

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            Vector3 cross = Vector3CrossProduct(axesA[i], axesB[j]);
            float len = Vector3Length(cross);
            if (len > 0.0001f)
            {
                testAxes[axisCount++] = Vector3Scale(cross, 1.0f / len);
            }
        }
    }

    for (int i = 0; i < axisCount; ++i)
    {
        Vector3 axis = testAxes[i];

        float minA, maxA, minB, maxB;
        ProjectOBBOntoAxis(a, axis, &minA, &maxA);
        ProjectOBBOntoAxis(b, axis, &minB, &maxB);

        if (maxA < minB || maxB < minA)
        {
            return false;
        }
    }

    return true;
}

inline RayCollision GetRayCollisionOBB(Ray ray, const OBB *obb)
{
    RayCollision result = {0};
    result.hit = false;
    result.distance = 0;
    result.normal = (Vector3){0.0f, 0.0f, 0.0f};
    result.point = (Vector3){0.0f, 0.0f, 0.0f};

    // Move ray into OBB's local space
    Vector3 localOrigin = Vector3Subtract(ray.position, obb->center);
    Quaternion inverseRot = QuaternionInvert(obb->rotation);
    Vector3 localRayOrigin = Vector3RotateByQuaternion(localOrigin, inverseRot);
    Vector3 localRayDir = Vector3RotateByQuaternion(ray.direction, inverseRot);

    Vector3 boxMin = Vector3Negate(obb->halfExtents);
    Vector3 boxMax = obb->halfExtents;

    // Ray vs AABB in OBB-local space
    float tmin = -INFINITY;
    float tmax = INFINITY;
    Vector3 normal = {0};

    for (int i = 0; i < 3; ++i)
    {
        float origin = (&localRayOrigin.x)[i];
        float dir = (&localRayDir.x)[i];
        float min = (&boxMin.x)[i];
        float max = (&boxMax.x)[i];

        if (fabsf(dir) < 0.0001f)
        {
            if (origin < min || origin > max)
                return result;
        }
        else
        {
            float ood = 1.0f / dir;
            float t1 = (min - origin) * ood;
            float t2 = (max - origin) * ood;
            int axis = i;

            if (t1 > t2)
            {
                float temp = t1;
                t1 = t2;
                t2 = temp;
                axis = -axis;
            }

            if (t1 > tmin)
            {
                tmin = t1;
                normal = (Vector3){0};
                (&normal.x)[abs(axis)] = axis >= 0 ? -1.0f : 1.0f;
            }

            if (t2 < tmax)
            {
                tmax = t2;
            }

            if (tmin > tmax)
                return result;
        }
    }

    // Convert result to world space
    result.hit = true;
    result.distance = tmin;
    result.point = Vector3Add(ray.position, Vector3Scale(ray.direction, tmin));
    result.normal = Vector3RotateByQuaternion(normal, obb->rotation);

    return result;
}

inline bool CheckCollisionSphereVsOBB(Vector3 sphereCenter, float radius, const OBB *obb)
{
    Vector3 localCenter = Vector3Subtract(sphereCenter, obb->center);
    Quaternion invRot = QuaternionInvert(obb->rotation);
    localCenter = Vector3RotateByQuaternion(localCenter, invRot);

    Vector3 clamped = {
        Clamp(localCenter.x, -obb->halfExtents.x, obb->halfExtents.x),
        Clamp(localCenter.y, -obb->halfExtents.y, obb->halfExtents.y),
        Clamp(localCenter.z, -obb->halfExtents.z, obb->halfExtents.z)};

    Vector3 worldClamped = Vector3RotateByQuaternion(clamped, obb->rotation);
    worldClamped = Vector3Add(worldClamped, obb->center);

    float distSq = Vector3DistanceSqr(sphereCenter, worldClamped);
    return distSq <= radius * radius;
}

#endif
