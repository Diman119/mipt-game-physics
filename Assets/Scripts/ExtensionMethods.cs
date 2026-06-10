using UnityEngine;

public static class ExtensionMethods {
    public static Matrix3x3 Skew(this Vector3 v) {
        var result = Matrix3x3.zero;
        result.m01 = -v.z;
        result.m10 = v.z;
        result.m02 = v.y;
        result.m20 = -v.y;
        result.m12 = -v.x;
        result.m21 = v.x;
        return result;
    }

    public static Vector3 RandomVectorInBounds(Bounds bounds) => new Vector3(
        Random.Range(bounds.min.x, bounds.max.x),
        Random.Range(bounds.min.y, bounds.max.y),
        Random.Range(bounds.min.z, bounds.max.z)
    );

    // Vector4 extension to access xyz as Vector3
    public static Vector3 xyz(this Vector4 v) => new Vector3(v.x, v.y, v.z);
}
