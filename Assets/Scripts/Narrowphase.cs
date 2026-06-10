using System.Collections.Generic;
using UnityEngine;

public static class Narrowphase {
    // Static list to store contacts
    private static readonly List<Contact> _contacts = new List<Contact>();
    public struct Contact {
        public Vector3 normal;
        public Vector3 anchorA;  // contact point in body A's local space, moved along normal
        public Vector3 anchorB;  // contact point in body B's local space, moved along normal
        public int bodyIndexA;
        public int bodyIndexB;
    }

    // Static arrays to avoid allocations
    private static readonly Vector3[] _cornersA = new Vector3[8];
    private static readonly Vector3[] _cornersB = new Vector3[8];
    private static readonly Vector4[] _planesA = new Vector4[6];
    private static readonly Vector4[] _planesB = new Vector4[6];

    // Get the 8 corners of a box (OBB)
    private static void GetBoxCorners(Vector3 center, Vector3 size, Quaternion rotation, Vector3[] corners) {
        Vector3 half = size * 0.5f;

        corners[0] = center + rotation * new Vector3(-half.x, -half.y, -half.z);
        corners[1] = center + rotation * new Vector3(-half.x, -half.y, half.z);
        corners[2] = center + rotation * new Vector3(-half.x, half.y, -half.z);
        corners[3] = center + rotation * new Vector3(-half.x, half.y, half.z);
        corners[4] = center + rotation * new Vector3(half.x, -half.y, -half.z);
        corners[5] = center + rotation * new Vector3(half.x, -half.y, half.z);
        corners[6] = center + rotation * new Vector3(half.x, half.y, -half.z);
        corners[7] = center + rotation * new Vector3(half.x, half.y, half.z);
    }

    // Get face planes for a box (plane normal.xyz() and distance.w for each of 6 faces)
    private static void GetBoxPlanes(Vector3 center, Vector3 size, Quaternion rotation, Vector4[] planes) {
        Vector3 half = size * 0.5f;

        // Right (x+)
        Vector3 right = rotation * Vector3.right;
        Vector3 p0 = center + rotation * new Vector3(half.x, 0, 0);
        planes[0] = new Vector4(right.x, right.y, right.z, Vector3.Dot(p0, right));

        // Left (x-)
        Vector3 left = rotation * -Vector3.right;
        Vector3 p1 = center + rotation * new Vector3(-half.x, 0, 0);
        planes[1] = new Vector4(left.x, left.y, left.z, Vector3.Dot(p1, left));

        // Top (y+)
        Vector3 up = rotation * Vector3.up;
        Vector3 p2 = center + rotation * new Vector3(0, half.y, 0);
        planes[2] = new Vector4(up.x, up.y, up.z, Vector3.Dot(p2, up));

        // Bottom (y-)
        Vector3 down = rotation * -Vector3.up;
        Vector3 p3 = center + rotation * new Vector3(0, -half.y, 0);
        planes[3] = new Vector4(down.x, down.y, down.z, Vector3.Dot(p3, down));

        // Front (z+)
        Vector3 forward = rotation * Vector3.forward;
        Vector3 p4 = center + rotation * new Vector3(0, 0, half.z);
        planes[4] = new Vector4(forward.x, forward.y, forward.z, Vector3.Dot(p4, forward));

        // Back (z-)
        Vector3 backward = rotation * -Vector3.forward;
        Vector3 p5 = center + rotation * new Vector3(0, 0, -half.z);
        planes[5] = new Vector4(backward.x, backward.y, backward.z, Vector3.Dot(p5, backward));
    }

    // Check if point is inside all planes of a box, returns true if inside
    private static bool PointInsidePlanes(Vector3 point, Vector4[] planes, out float minPenetration, out Vector3 closestNormal) {
        minPenetration = float.MaxValue;
        closestNormal = Vector3.zero;
        for (int i = 0; i < 6; i++) {
            float dist = Vector3.Dot(point, planes[i].xyz()) - planes[i].w;
            if (dist > 0) {
                closestNormal = Vector3.zero;
                return false; // Point is outside
            }
            if (dist < minPenetration) {
                minPenetration = dist;
                closestNormal = planes[i].xyz();
            }
        }
        return true; // Point is inside (all distances <= 0)
    }

    // Generate contact points between two boxes
    public static void GenerateContacts(MyRigidbody bodyA, MyRigidbody bodyB, List<Contact> contacts, int bodyIndexA = -1, int bodyIndexB = -1) {
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

        // Check vertices of A against box B
        for (int i = 0; i < 8; i++) {
            if (PointInsidePlanes(_cornersA[i], _planesB, out float penetration, out Vector3 closestNormal)) {
                penetration /= 2f;
                contacts.Add(new Contact {
                    normal = -closestNormal,
                    anchorA = Quaternion.Inverse(rotA) * (_cornersA[i] - centerA + closestNormal * penetration),
                    anchorB = Quaternion.Inverse(rotB) * (_cornersA[i] - centerB - closestNormal * penetration),
                    bodyIndexA = bodyIndexA,
                    bodyIndexB = bodyIndexB
                });
            }
        }

        // Check vertices of B against box A
        for (int i = 0; i < 8; i++) {
            if (PointInsidePlanes(_cornersB[i], _planesA, out float penetration, out Vector3 closestNormal)) {
                penetration /= 2f;
                contacts.Add(new Contact {
                    normal = -closestNormal,
                    anchorA = Quaternion.Inverse(rotA) * (_cornersB[i] - centerA + closestNormal * penetration),
                    anchorB = Quaternion.Inverse(rotB) * (_cornersB[i] - centerB - closestNormal * penetration),
                    bodyIndexA = bodyIndexA,
                    bodyIndexB = bodyIndexB
                });
            }
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
