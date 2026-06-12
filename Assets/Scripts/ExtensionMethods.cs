using System.Collections.Generic;
using System.Reflection;
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

    public static Vector3 RandomVectorBetween(Vector3 min, Vector3 max) => new Vector3(
        Random.Range(min.x, max.x),
        Random.Range(min.y, max.y),
        Random.Range(min.z, max.z)
    );

    static class ArrayAccessor<T> {
        public static readonly FieldInfo itemsField =
            typeof(List<T>).GetField("_items", BindingFlags.NonPublic | BindingFlags.Instance);
    }

    public static T[] GetInternalArray<T>(this List<T> list) {
        return (T[])ArrayAccessor<T>.itemsField.GetValue(list);
    }
}