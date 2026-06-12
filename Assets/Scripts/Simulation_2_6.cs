using UnityEngine;

public class Simulation_2_6 : MonoBehaviour {
    [SerializeField] Transform _anchor1;
    [SerializeField] Transform _anchor2;
    [SerializeField] int _SIIterations = 4;
    [SerializeField] float _softFreq = 0.5f;
    [SerializeField] float _softDamp = 1f;

    float _constraintLength;

    MyRigidbody _rb1;
    MyRigidbody _rb2;
    float _lambda = 0f;

    float Dt => Time.fixedDeltaTime;

    void Awake() {
        _rb1 = _anchor1.GetComponentInParent<MyRigidbody>();
        _rb2 = _anchor2.GetComponentInParent<MyRigidbody>();
        _constraintLength = (_anchor1.position - _anchor2.position).magnitude;
    }

    void Start() {
        _rb1.velocity = new(15f, 0f, 0f);
    }

    void Update() {
        var a1 = _anchor1.position;
        var a2 = _anchor2.position;
        transform.position = a2;
        transform.LookAt(a1, transform.up);
        transform.localScale = new(1.5f, 1.5f, (a2 - a1).magnitude);
    }

    void FixedUpdate() {
        _rb1.IntegrateVelocities();
        _rb2.IntegrateVelocities();
        
        var omega = 2 * Mathf.PI * _softFreq;
        var a1 = 2 * _softDamp + omega * Dt;
        var a2 = Dt * omega * a1;
        var impulseCoeff = 1 / (1 + a2);
        var biasRate = omega / a1;
        var massCoeff = a2 * impulseCoeff;
        
        var r1W = _anchor1.position - _rb1.transform.position;
        var r2W = _anchor2.position - _rb2.transform.position;
        var d = _anchor2.position - _anchor1.position;
        var len = d.magnitude;
        var n = Mathf.Approximately(len, 0f) ? Vector3.up : d / len;
        var c = len - _constraintLength;
        var w = MyRigidbody.GetEffectiveInvMass(_rb1, _rb2, r1W, r2W, n);
        
        _lambda = 0f;
        for (int i = 0; i < _SIIterations; ++i) {
            var relativeVelocity = Vector3.Dot(n, _rb2.GetPointVelocity(r2W) - _rb1.GetPointVelocity(r1W));
            var dLambda = -massCoeff / w * (relativeVelocity + biasRate * c) - impulseCoeff * _lambda;
            _lambda += dLambda;
            _rb1.ApplyImpulseToVelocities(r1W, n * -dLambda);
            _rb2.ApplyImpulseToVelocities(r2W, n * dLambda);
        }

        _rb1.IntegratePositions();
        _rb2.IntegratePositions();
    }
}