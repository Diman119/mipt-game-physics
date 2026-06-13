using System.Collections.Generic;
using UnityEngine;

public static class Narrowphase {
    // Static list to store contacts
    static readonly List<Contact> _contacts = new List<Contact>();

    public struct Contact {
        public Vector3 normal;
        public Vector3 anchorA; // contact point in body A's local space, moved along normal
        public Vector3 anchorB; // contact point in body B's local space, moved along normal
        public int bodyIndexA;
        public int bodyIndexB;
        public float lambda;
    }

    // Static arrays to avoid allocations
    static readonly Vector3[] _cornersA = new Vector3[8];
    static readonly Vector3[] _cornersB = new Vector3[8];
    static readonly Vector4[] _planesA = new Vector4[6];
    static readonly Vector4[] _planesB = new Vector4[6];

    // Project box corners onto an axis (for SAT)
    static void ProjectShape(Vector3[] corners, Vector3 axis, out float min, out float max) {
        min = float.MaxValue;
        max = float.MinValue;
        float dot;
        for (int i = 0; i < corners.Length; i++) {
            dot = Vector3.Dot(corners[i], axis);
            if (dot < min) min = dot;
            if (dot > max) max = dot;
        }
    }

    // Test a single SAT axis for overlap, returns true if overlapping
    // Penetration depth is the depth of overlap
    static bool BoxSatSubtest(Vector3 axis, ref Vector3 bestAxis, ref float bestDepth) {
        ProjectShape(_cornersA, axis, out float minA, out float maxA);
        ProjectShape(_cornersB, axis, out float minB, out float maxB);

        // Check for separation
        if (maxA < minB || maxB < minA) {
            return false;
        }

        // Calculate penetration depth (overlap amount)
        float overlapA = maxA - minB;
        float overlapB = maxB - minA;
        float depth = Mathf.Min(overlapA, overlapB);
        if (depth < bestDepth) {
            bestDepth = depth;
            bestAxis = axis;
        }
        return true;
    }

    static bool BoxSatTest(out Vector3 bestAxis, out float penetrationDepth) {
        bestAxis = Vector3.zero;
        penetrationDepth = float.PositiveInfinity;
        
        for (int i = 0; i < 6; i += 2) {
            if (!BoxSatSubtest(_planesA[i], ref bestAxis, ref penetrationDepth)) return false;
            if (!BoxSatSubtest(_planesB[i], ref bestAxis, ref penetrationDepth)) return false;
            
            for (int j = 0; j < 6; j += 2) {
                var cross = Vector3.Cross(_planesA[i], _planesB[j]);
                var sqm = cross.sqrMagnitude;
                if (sqm < 1e-6) continue;
                cross /= Mathf.Sqrt(sqm);
                if (!BoxSatSubtest(cross, ref bestAxis, ref penetrationDepth)) return false;
            }
        }

        return true;
    }

    // Get the 8 corners of a box (OBB)
    static void GetBoxCorners(Vector3 center, Vector3 size, Quaternion rotation, Vector3[] corners) {
        var ext = size / 2f;
        var corner = rotation * ext;
        corners[0] = center + corner;
        corners[1] = center - corner;
        corner = rotation * new Vector3(-ext.x, ext.y, ext.z);
        corners[2] = center + corner;
        corners[3] = center - corner;
        corner = rotation * new Vector3(ext.x, -ext.y, ext.z);
        corners[4] = center + corner;
        corners[5] = center - corner;
        corner = rotation * new Vector3(ext.x, ext.y, -ext.z);
        corners[6] = center + corner;
        corners[7] = center - corner;
    }

    // Get face planes for a box (plane normal.xyz and distance.w for each of 6 faces)
    static void GetBoxPlanes(Vector3 center, Vector3 size, Quaternion rotation, Vector4[] planes) {
        var ext = size / 2f;

        // Right (x+)
        Vector3 right = rotation * Vector3.right;
        var dot = Vector3.Dot(center, right);
        planes[0] = new(right.x, right.y, right.z, dot + ext.x);
        planes[1] = new(-right.x, -right.y, -right.z, -dot + ext.x);

        // Top (y+)
        Vector3 up = rotation * Vector3.up;
        dot = Vector3.Dot(center, up);
        planes[2] = new(up.x, up.y, up.z, dot + ext.y);
        planes[3] = new(-up.x, -up.y, -up.z, -dot + ext.y);

        // Front (z+)
        Vector3 forward = rotation * Vector3.forward;
        dot = Vector3.Dot(center, forward);
        planes[4] = new(forward.x, forward.y, forward.z, dot + ext.z);
        planes[5] = new(-forward.x, -forward.y, -forward.z, -dot + ext.z);
    }

    // Check if point is inside all planes of a box, returns true if inside
    static bool PointInsidePlanes(Vector3 point, Vector4[] planes, out float minPenetration,
        out Vector3 closestNormal) {
        minPenetration = float.MaxValue;
        closestNormal = Vector3.zero;
        for (int i = 0; i < 6; i++) {
            float dist = Vector3.Dot(point, planes[i]) - planes[i].w;
            if (dist > 0) {
                return false; // Point is outside
            }

            if (-dist < minPenetration) {
                minPenetration = -dist;
                closestNormal = planes[i];
            }
        }

        return true; // Point is inside (all distances <= 0)
    }

    // Generate contact points between two boxes
    public static void GenerateContacts(MyRigidbody bodyA, MyRigidbody bodyB, List<Contact> contacts,
        int bodyIndexA = -1, int bodyIndexB = -1) {
        Vector3 centerA = bodyA.transform.position;
        Vector3 sizeA = bodyA.Size;
        Quaternion rotA = bodyA.transform.rotation;

        Vector3 centerB = bodyB.transform.position;
        Vector3 sizeB = bodyB.Size;
        Quaternion rotB = bodyB.transform.rotation;

        // Get corners and planes (reusing static arrays)
        GetBoxCorners(centerA, sizeA, rotA, _cornersA);
        GetBoxCorners(centerB, sizeB, rotB, _cornersB);
        GetBoxPlanes(centerA, sizeA, rotA, _planesA);
        GetBoxPlanes(centerB, sizeB, rotB, _planesB);

        if (!BoxSatTest(out Vector3 bestAxis, out float satDepth)) {
            return;
        }

        var contactCountPrev = contacts.Count;

        // Try generating clean vertex contacts
        // Check vertices of A against box B
        for (int i = 0; i < 8; i++) {
            if (PointInsidePlanes(_cornersA[i], _planesB, out float penetration, out Vector3 closestNormal)) {
                contacts.Add(new() {
                    normal = -closestNormal,
                    anchorA = Quaternion.Inverse(rotA) * (_cornersA[i] - centerA),
                    anchorB = Quaternion.Inverse(rotB) * (_cornersA[i] - centerB + closestNormal * penetration),
                    bodyIndexA = bodyIndexA,
                    bodyIndexB = bodyIndexB
                });
            }
        }
        
        // Check vertices of B against box A
        for (int i = 0; i < 8; i++) {
            if (PointInsidePlanes(_cornersB[i], _planesA, out float penetration, out Vector3 closestNormal)) {
                contacts.Add(new() {
                    normal = closestNormal,
                    anchorA = Quaternion.Inverse(rotA) * (_cornersB[i] - centerA + closestNormal * penetration),
                    anchorB = Quaternion.Inverse(rotB) * (_cornersB[i] - centerB),
                    bodyIndexA = bodyIndexA,
                    bodyIndexB = bodyIndexB
                });
            }
        }

        // Fallback to single basic contact
        if (contacts.Count == contactCountPrev) {
            if (Vector3.Dot(centerB - centerA, bestAxis) < 0f) {
                bestAxis *= -1f;
            }
            var point = Vector3.LerpUnclamped(centerA, centerB,
                (sizeA.x + sizeA.y + sizeA.z) / (sizeA.x + sizeA.y + sizeA.z + sizeB.x + sizeB.y + sizeB.z));
            satDepth /= 2f;
            contacts.Add(new() {
                normal = bestAxis,
                anchorA = Quaternion.Inverse(rotA) * (Vector3.Project(point - centerA, bestAxis) + bestAxis * satDepth),
                anchorB = Quaternion.Inverse(rotB) * (Vector3.Project(point - centerB, bestAxis) - bestAxis * satDepth),
                bodyIndexA = bodyIndexA,
                bodyIndexB = bodyIndexB
            });
        }
    }

    // Combined broadphase + narrowphase - uses static _contacts list
    public static void GenerateContacts(MyRigidbody[] bodies, IEnumerator<Broadphase.IntPair> broadphase) {
        _contacts.Clear();
        while (broadphase.MoveNext()) {
            var pair = broadphase.Current;
            GenerateContacts(bodies[pair.i1], bodies[pair.i2], _contacts, pair.i1, pair.i2);
        }
    }

    // Get the static contacts list
    public static List<Contact> GetContacts() => _contacts;
}