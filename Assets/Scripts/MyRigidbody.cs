using UnityEngine;

public class MyRigidbody : MonoBehaviour {
    [SerializeField] float _mass = 1f;
    [SerializeField] Vector3 _size = Vector3.one;
    [SerializeField] bool _useGravity = false;
    [SerializeField] float _linearDamp;
    [SerializeField] float _angularDamp;

    [HideInInspector] public Vector3 velocity;
    [HideInInspector] public Vector3 omega;
    Matrix3x3 _localI;
    Matrix3x3 _localI_inv;

    Vector3 _pendingForce;
    Vector3 _pendingTorque;

    Vector3 _prevPosition;
    Quaternion _prevRotation;

    public Matrix3x3 LocalI => _localI;
    public float Mass => _mass;
    public Vector3 Size => _size;

    Bounds _aabb;
    public Bounds AABB => _aabb;

    public void UpdateAABB() {
        var pos = transform.position;
        var rot = transform.rotation;
        var ext = _size / 2f;
        var corner = rot * ext;
        _aabb.SetMinMax(pos - corner, pos + corner);
        corner = rot * new Vector3(-ext.x, ext.y, ext.z);
        _aabb.Encapsulate(pos + corner);
        _aabb.Encapsulate(pos - corner);
        corner = rot * new Vector3(ext.x, -ext.y, ext.z);
        _aabb.Encapsulate(pos + corner);
        _aabb.Encapsulate(pos - corner);
        corner = rot * new Vector3(ext.x, ext.y, -ext.z);
        _aabb.Encapsulate(pos + corner);
        _aabb.Encapsulate(pos - corner);
    }
    
    public Vector3 L {
        get {
            var R = Matrix3x3.FromQuaternion(transform.rotation);
            var Rt = R.Transpose();
            return (R * _localI * Rt).MultiplyVector(omega);
        }
    }

    public Matrix3x3 GlobalI_inv {
        get {
            var R = Matrix3x3.FromQuaternion(transform.rotation);
            var Rt = R.Transpose();
            return R * _localI_inv * Rt;
        }
    }

    float Dt => Time.fixedDeltaTime;

    static Matrix3x3 GetBoxI(float mass, Vector3 size) {
        var result = Matrix3x3.zero;
        result.m00 = (mass / 12) * (size.y * size.y + size.z * size.z);
        result.m11 = (mass / 12) * (size.x * size.x + size.z * size.z);
        result.m22 = (mass / 12) * (size.x * size.x + size.y * size.y);
        return result;
    }

    Vector3 ApplyLinearVelocity(Vector3 v) => transform.position + v * Dt;

    public Quaternion ApplyAngularVelocity(Vector3 w) => ApplyAngularDelta(w * Dt);
    
    public Quaternion ApplyAngularDelta(Vector3 w) {
        var rotation = transform.rotation;
        w /= 2f;
        var delta = new Quaternion(w.x, w.y, w.z, 1f);
        rotation = delta * rotation;
        rotation.Normalize();
        return rotation;
    }

    public void ApplyVelocities() {
        transform.SetLocalPositionAndRotation(ApplyLinearVelocity(velocity), ApplyAngularVelocity(omega));
    }

    public Vector3 IntegrateOmegaLocalExplicitGyro() {
        var omegaLocal = transform.InverseTransformDirection(omega);
        var dOmega = _localI_inv.MultiplyVector(Vector3.Cross(_localI.MultiplyVector(omegaLocal), omegaLocal));
        omegaLocal += dOmega * Dt;
        return transform.TransformDirection(omegaLocal);
    }

    public Vector3 IntegrateOmegaLocalImplicitGyro(Vector3 torque) {
        var omegaLocal = transform.InverseTransformDirection(omega);
        var omegaLocal0 = omegaLocal;
        torque = transform.InverseTransformDirection(torque);

        for (int i = 0; i < 4; ++i) {
            var L = _localI.MultiplyVector(omegaLocal);
            var f = _localI.MultiplyVector(omegaLocal0 - omegaLocal) + (torque - Vector3.Cross(omegaLocal, L)) * Dt;
            var J = _localI + (omegaLocal.Skew() * _localI - L.Skew()) * Dt;
            omegaLocal += J.Inverse().MultiplyVector(f);
        }

        return transform.TransformDirection(omegaLocal);
    }

    public Vector3 IntegrateVelocity(Vector3 force) {
        var v = velocity;
        v += force * (Dt / _mass);
        if (_useGravity) {
            v += Physics.gravity * Dt;
        }

        return v;
    }

    public void IntegrateVelocities() {
        velocity = IntegrateVelocity(_pendingForce);
        omega = IntegrateOmegaLocalImplicitGyro(_pendingTorque);
        DampVelocities();
        _pendingForce = _pendingTorque = Vector3.zero;
    }

    void DampVelocities() {
        if (_linearDamp != 0f) {
            velocity *= Mathf.Exp(-_linearDamp * Dt);
        }
        if (_linearDamp != 0f) {
            omega *= Mathf.Exp(-_angularDamp * Dt);
        }
    }

    public void ApplyForceAtPoint(Vector3 force, Vector3 pointGlobal) {
        var torque = Vector3.Cross(pointGlobal - transform.position, force);
        _pendingForce += force;
        _pendingTorque += torque;
    }

    public void ApplyImpulseToPositions(Vector3 offsetGlobal, Vector3 impulse) {
        transform.SetPositionAndRotation(
            transform.position + impulse / Mass,
            ApplyAngularDelta(GlobalI_inv.MultiplyVector(Vector3.Cross(offsetGlobal, impulse)))
        );
    }

    public void ApplyImpulseToVelocities(Vector3 offsetGlobal, Vector3 impulse) {
        velocity += impulse / Mass;
        omega += GlobalI_inv.MultiplyVector(Vector3.Cross(offsetGlobal, impulse));
    }
    
    public Vector3 GetPointVelocity(Vector3 offsetGlobal) =>
        velocity + Vector3.Cross(omega, offsetGlobal);
    
    public void SavePrevPosition() {
        _prevPosition = transform.position;
        _prevRotation = transform.rotation;
    }

    public void SetVelocitiesFromPrev() {
        velocity = (transform.position - _prevPosition) / Dt;
        var dq = transform.rotation * Quaternion.Inverse(_prevRotation);
        omega = new Vector3(dq.x, dq.y, dq.z) * (Mathf.Sign(dq.w) * 2 / Dt);
    }

    public void SetSizeAndMass(Vector3 newSize, float mass) {
        _size = newSize;
        _mass = mass;
        
        if (transform.childCount > 0) {
            transform.GetChild(0).localScale = newSize;
        }
        
        _localI = GetBoxI(_mass, _size);
        _localI_inv = _localI.Inverse();
    }

    void Awake() {
        SetSizeAndMass(_size, _mass);
    }
}