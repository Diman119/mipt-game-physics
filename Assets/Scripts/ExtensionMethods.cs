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
}
