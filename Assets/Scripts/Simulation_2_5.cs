using UnityEngine;

public class Simulation_2_5 : MonoBehaviour {
    [SerializeField] Transform _anchor1;
    [SerializeField] Transform _anchor2;
    [SerializeField] int _SIIterations = 4;
    [SerializeField] int _positionIterations = 2;

    float _constraintLength;

    MyRigidbody _rb1;
    MyRigidbody _rb2;

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
        
        var r1W = _anchor1.position - _rb1.transform.position;
        var r2W = _anchor2.position - _rb2.transform.position;
        var d = _anchor2.position - _anchor1.position;
        var len = d.magnitude;
        var n = Mathf.Approximately(len, 0f) ? Vector3.up : d / len;
        var c = len - _constraintLength;
        var w = MyRigidbody.GetEffectiveInvMass(_rb1, _rb2, r1W, r2W, n);
        
        for (int i = 0; i < _SIIterations; ++i) {
            var relativeVelocity = Vector3.Dot(n, _rb2.GetPointVelocity(r2W) - _rb1.GetPointVelocity(r1W));
            var dLambda = -relativeVelocity / w;
            _rb1.ApplyImpulseToVelocities(r1W, n * -dLambda);
            _rb2.ApplyImpulseToVelocities(r2W, n * dLambda);
        }
        
        _rb1.IntegratePositions();
        _rb2.IntegratePositions();
        
        for (int i = 0; i < _positionIterations; ++i) {
            r1W = _anchor1.position - _rb1.transform.position;
            r2W = _anchor2.position - _rb2.transform.position;
            d = _anchor2.position - _anchor1.position;
            len = d.magnitude;
            n = Mathf.Approximately(len, 0f) ? Vector3.up : d / len;
            c = len - _constraintLength;
            if (Mathf.Abs(c) < 1e-6f) {
                break;
            }
            w = MyRigidbody.GetEffectiveInvMass(_rb1, _rb2, r1W, r2W, n);
            
            var dLambda = -c / w;
            _rb1.ApplyImpulseToPositions(r1W, n * -dLambda);
            _rb2.ApplyImpulseToPositions(r2W, n * dLambda);
        }
    }
}