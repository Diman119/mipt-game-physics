using UnityEngine;

public class Simulation_2_3 : MonoBehaviour {
    [SerializeField] Transform _anchor1;
    [SerializeField] Transform _anchor2;
    [SerializeField] float _compliance = 0f;

    float _constraintLength;

    MyRigidbody _rb1;
    MyRigidbody _rb2;
    float _lambda;

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
        _lambda = 0f;
        _rb1.SavePrevPosition();
        _rb2.SavePrevPosition();
        _rb1.IntegrateVelocities();
        _rb2.IntegrateVelocities();
        _rb1.ApplyVelocities();
        _rb2.ApplyVelocities();

        var alpha = _compliance / (Dt * Dt);
        var r1W = _anchor1.position - _rb1.transform.position;
        var r2W = _anchor2.position - _rb2.transform.position;
        var d = _anchor2.position - _anchor1.position;
        var len = d.magnitude;
        var n = Mathf.Approximately(len, 0f) ? Vector3.up : d / len;
        var c = len - _constraintLength;
        var Iinv1 = _rb1.GlobalI_inv;
        var Iinv2 = _rb2.GlobalI_inv;
        var rxn1 = Vector3.Cross(r1W, n);
        var rxn2 = Vector3.Cross(r2W, n);
        var w1 = 1f / _rb1.Mass + Vector3.Dot(rxn1, Iinv1.MultiplyVector(rxn1));
        var w2 = 1f / _rb2.Mass + Vector3.Dot(rxn2, Iinv2.MultiplyVector(rxn2));
        var w = w1 + w2;

        var dLambda = (-c - alpha * _lambda) / (w + alpha);
        _lambda += dLambda;
        _rb1.ApplyImpulseToPositions(r1W, n * -dLambda);
        _rb2.ApplyImpulseToPositions(r2W, n * dLambda);

        _rb1.SetVelocitiesFromPrev();
        _rb2.SetVelocitiesFromPrev();
    }
}