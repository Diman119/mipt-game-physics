using System;
using UnityEngine;

/// <summary>
/// A 3x3 Matrix struct optimized for Unity.
/// Supports multiplication, inversion, and quaternion-based rotation.
/// </summary>
[System.Serializable]
public struct Matrix3x3 : IEquatable<Matrix3x3> {
    // Internal storage: row-major order for consistency with Unity's Matrix4x4
    // m[row * 3 + col]
    public float m00, m01, m02;
    public float m10, m11, m12;
    public float m20, m21, m22;

    #region Constructors

    public Matrix3x3(float m00, float m01, float m02,
        float m10, float m11, float m12,
        float m20, float m21, float m22) {
        this.m00 = m00;
        this.m01 = m01;
        this.m02 = m02;
        this.m10 = m10;
        this.m11 = m11;
        this.m12 = m12;
        this.m20 = m20;
        this.m21 = m21;
        this.m22 = m22;
    }

    public static Matrix3x3 identity => new Matrix3x3(
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    );

    public static Matrix3x3 zero => new Matrix3x3(
        0, 0, 0,
        0, 0, 0,
        0, 0, 0
    );

    #endregion

    #region Indexer Access

    /// <summary>
    /// Access element by row and column.
    /// Note: This involves bounds checking. For high-performance inner loops, 
    /// use direct member access (m00, m01, etc.) if possible.
    /// </summary>
    public float this[int row, int col] {
        get {
            switch (row) {
                case 0:
                    switch (col) {
                        case 0: return m00;
                        case 1: return m01;
                        case 2: return m02;
                    }

                    break;
                case 1:
                    switch (col) {
                        case 0: return m10;
                        case 1: return m11;
                        case 2: return m12;
                    }

                    break;
                case 2:
                    switch (col) {
                        case 0: return m20;
                        case 1: return m21;
                        case 2: return m22;
                    }

                    break;
            }

            throw new IndexOutOfRangeException("Invalid Matrix3x3 index.");
        }
        set {
            switch (row) {
                case 0:
                    switch (col) {
                        case 0:
                            m00 = value;
                            return;
                        case 1:
                            m01 = value;
                            return;
                        case 2:
                            m02 = value;
                            return;
                    }

                    break;
                case 1:
                    switch (col) {
                        case 0:
                            m10 = value;
                            return;
                        case 1:
                            m11 = value;
                            return;
                        case 2:
                            m12 = value;
                            return;
                    }

                    break;
                case 2:
                    switch (col) {
                        case 0:
                            m20 = value;
                            return;
                        case 1:
                            m21 = value;
                            return;
                        case 2:
                            m22 = value;
                            return;
                    }

                    break;
            }

            throw new IndexOutOfRangeException("Invalid Matrix3x3 index.");
        }
    }

    #endregion

    #region Operations

    /// <summary>
    /// Multiplies this matrix by a Vector3.
    /// Treats the vector as a column vector.
    /// </summary>
    public Vector3 MultiplyVector(Vector3 v) {
        return new Vector3(
            m00 * v.x + m01 * v.y + m02 * v.z,
            m10 * v.x + m11 * v.y + m12 * v.z,
            m20 * v.x + m21 * v.y + m22 * v.z
        );
    }

    /// <summary>
    /// Multiplies two 3x3 matrices.
    /// </summary>
    public static Matrix3x3 operator *(Matrix3x3 lhs, Matrix3x3 rhs) {
        Matrix3x3 res;

        // Row 0
        res.m00 = lhs.m00 * rhs.m00 + lhs.m01 * rhs.m10 + lhs.m02 * rhs.m20;
        res.m01 = lhs.m00 * rhs.m01 + lhs.m01 * rhs.m11 + lhs.m02 * rhs.m21;
        res.m02 = lhs.m00 * rhs.m02 + lhs.m01 * rhs.m12 + lhs.m02 * rhs.m22;

        // Row 1
        res.m10 = lhs.m10 * rhs.m00 + lhs.m11 * rhs.m10 + lhs.m12 * rhs.m20;
        res.m11 = lhs.m10 * rhs.m01 + lhs.m11 * rhs.m11 + lhs.m12 * rhs.m21;
        res.m12 = lhs.m10 * rhs.m02 + lhs.m11 * rhs.m12 + lhs.m12 * rhs.m22;

        // Row 2
        res.m20 = lhs.m20 * rhs.m00 + lhs.m21 * rhs.m10 + lhs.m22 * rhs.m20;
        res.m21 = lhs.m20 * rhs.m01 + lhs.m21 * rhs.m11 + lhs.m22 * rhs.m21;
        res.m22 = lhs.m20 * rhs.m02 + lhs.m21 * rhs.m12 + lhs.m22 * rhs.m22;

        return res;
    }
    
    /// <summary>
    /// Adds two 3x3 matrices.
    /// </summary>
    public static Matrix3x3 operator +(Matrix3x3 lhs, Matrix3x3 rhs) {
        Matrix3x3 res;

        res.m00 = lhs.m00 + rhs.m00;
        res.m01 = lhs.m01 + rhs.m01;
        res.m02 = lhs.m02 + rhs.m02;
        res.m10 = lhs.m10 + rhs.m10;
        res.m11 = lhs.m11 + rhs.m11;
        res.m12 = lhs.m12 + rhs.m12;
        res.m20 = lhs.m20 + rhs.m20;
        res.m21 = lhs.m21 + rhs.m21;
        res.m22 = lhs.m22 + rhs.m22;

        return res;
    }
    
    /// <summary>
    /// Subtracts two 3x3 matrices.
    /// </summary>
    public static Matrix3x3 operator -(Matrix3x3 lhs, Matrix3x3 rhs) {
        Matrix3x3 res;

        res.m00 = lhs.m00 - rhs.m00;
        res.m01 = lhs.m01 - rhs.m01;
        res.m02 = lhs.m02 - rhs.m02;
        res.m10 = lhs.m10 - rhs.m10;
        res.m11 = lhs.m11 - rhs.m11;
        res.m12 = lhs.m12 - rhs.m12;
        res.m20 = lhs.m20 - rhs.m20;
        res.m21 = lhs.m21 - rhs.m21;
        res.m22 = lhs.m22 - rhs.m22;

        return res;
    }
    
    /// <summary>
    /// Scales 3x3 matrix by s.
    /// </summary>
    public static Matrix3x3 operator *(Matrix3x3 m, float s) {
        Matrix3x3 res;

        res.m00 = m.m00 * s;
        res.m01 = m.m01 * s;
        res.m02 = m.m02 * s;
        res.m10 = m.m10 * s;
        res.m11 = m.m11 * s;
        res.m12 = m.m12 * s;
        res.m20 = m.m20 * s;
        res.m21 = m.m21 * s;
        res.m22 = m.m22 * s;

        return res;
    }

    /// <summary>
    /// Creates a rotation matrix from a Unity Quaternion.
    /// </summary>
    public static Matrix3x3 FromQuaternion(Quaternion q) {
        // Precompute common terms for efficiency
        float x = q.x;
        float y = q.y;
        float z = q.z;
        float w = q.w;

        float x2 = x + x;
        float y2 = y + y;
        float z2 = z + z;

        float xx = x * x2;
        float xy = x * y2;
        float xz = x * z2;
        float yy = y * y2;
        float yz = y * z2;
        float zz = z * z2;
        float wx = w * x2;
        float wy = w * y2;
        float wz = w * z2;

        Matrix3x3 m;
        m.m00 = 1.0f - (yy + zz);
        m.m01 = xy - wz;
        m.m02 = xz + wy;

        m.m10 = xy + wz;
        m.m11 = 1.0f - (xx + zz);
        m.m12 = yz - wx;

        m.m20 = xz - wy;
        m.m21 = yz + wx;
        m.m22 = 1.0f - (xx + yy);

        return m;
    }

    /// <summary>
    /// Computes the inverse of the matrix.
    /// Returns false if the matrix is singular (determinant is near zero).
    /// </summary>
    public bool TryInverse(out Matrix3x3 inverse) {
        inverse = zero;

        // Calculate cofactors
        float c00 = m11 * m22 - m12 * m21;
        float c01 = m12 * m20 - m10 * m22;
        float c02 = m10 * m21 - m11 * m20;

        float det = m00 * c00 + m01 * c01 + m02 * c02;

        // Check for singularity
        if (Mathf.Abs(det) < 1e-6f) {
            return false;
        }

        float invDet = 1.0f / det;

        // Calculate adjugate matrix (transpose of cofactor matrix)
        // Note: The indices here are transposed compared to cofactor calculation
        inverse.m00 = c00 * invDet;
        inverse.m01 = (m02 * m21 - m01 * m22) * invDet;
        inverse.m02 = (m01 * m12 - m02 * m11) * invDet;

        inverse.m10 = c01 * invDet;
        inverse.m11 = (m00 * m22 - m02 * m20) * invDet;
        inverse.m12 = (m02 * m10 - m00 * m12) * invDet;

        inverse.m20 = c02 * invDet;
        inverse.m21 = (m01 * m20 - m00 * m21) * invDet;
        inverse.m22 = (m00 * m11 - m01 * m10) * invDet;

        return true;
    }

    /// <summary>
    /// Computes the inverse. Throws an exception if the matrix is singular.
    /// </summary>
    public Matrix3x3 Inverse() {
        if (!TryInverse(out Matrix3x3 inv)) {
            throw new InvalidOperationException("Matrix is singular and cannot be inverted.");
        }

        return inv;
    }

    /// <summary>
    /// Calculates the determinant of the matrix.
    /// </summary>
    public float Determinant() {
        return m00 * (m11 * m22 - m12 * m21) -
               m01 * (m10 * m22 - m12 * m20) +
               m02 * (m10 * m21 - m11 * m20);
    }

    /// <summary>
    /// Transposes the matrix.
    /// </summary>
    public Matrix3x3 Transpose() {
        Matrix3x3 m;
        m.m00 = this.m00;
        m.m01 = this.m10;
        m.m02 = this.m20;
        m.m10 = this.m01;
        m.m11 = this.m11;
        m.m12 = this.m21;
        m.m20 = this.m02;
        m.m21 = this.m12;
        m.m22 = this.m22;
        return m;
    }

    #endregion

    #region Equality & Formatting

    public bool Equals(Matrix3x3 other) {
        return Mathf.Approximately(m00, other.m00) &&
               Mathf.Approximately(m01, other.m01) &&
               Mathf.Approximately(m02, other.m02) &&
               Mathf.Approximately(m10, other.m10) &&
               Mathf.Approximately(m11, other.m11) &&
               Mathf.Approximately(m12, other.m12) &&
               Mathf.Approximately(m20, other.m20) &&
               Mathf.Approximately(m21, other.m21) &&
               Mathf.Approximately(m22, other.m22);
    }

    public override bool Equals(object obj) {
        return obj is Matrix3x3 other && Equals(other);
    }

    public override int GetHashCode() {
        // Simple hash combination
        unchecked {
            int hash = 17;
            hash = hash * 23 + m00.GetHashCode();
            hash = hash * 23 + m01.GetHashCode();
            hash = hash * 23 + m02.GetHashCode();
            hash = hash * 23 + m10.GetHashCode();
            hash = hash * 23 + m11.GetHashCode();
            hash = hash * 23 + m12.GetHashCode();
            hash = hash * 23 + m20.GetHashCode();
            hash = hash * 23 + m21.GetHashCode();
            hash = hash * 23 + m22.GetHashCode();
            return hash;
        }
    }

    public override string ToString() {
        return $"[{m00:F4}, {m01:F4}, {m02:F4}]\n" +
               $"[{m10:F4}, {m11:F4}, {m12:F4}]\n" +
               $"[{m20:F4}, {m21:F4}, {m22:F4}]";
    }

    #endregion
}