using UnityEngine;

public class Simulation_2_2 : MonoBehaviour {
    [SerializeField] Transform _anchor1;
    [SerializeField] Transform _anchor2;
    [SerializeField] Transform _spring;
    [SerializeField] float _softHz = 0.5f;

    MyRigidbody _rb;
    float _lambda;

    float Dt => Time.fixedDeltaTime;

    void Awake() {
        _rb = GetComponent<MyRigidbody>();
    }
    
    void Update() {
        var a1 = _anchor1.position;
        var a2 = _anchor2.position;
        _spring.position = a2;
        _spring.LookAt(a1, _spring.up);
        _spring.localScale = new(1.5f, 1.5f, (a2 - a1).magnitude);
    }

    void FixedUpdate() {
        _lambda = 0f;
        _rb.SavePrevPosition();
        _rb.IntegrateVelocities();
        _rb.ApplyVelocities();
        
        var omega = 2f * Mathf.PI * _softHz;
        var compliance = 1f / (_rb.Mass * omega * omega);
        var alphaTilde = compliance / (Dt * Dt);

        var Iinv = _rb.GlobalI_inv;
        var attachW = _anchor2.position;
        var rW = attachW - _rb.transform.position;
        var dx = attachW - _anchor1.position;
        var c = dx.magnitude;
        if (c < 1e-10) return;
        var n = dx / c;
        var rxn = Vector3.Cross(rW, n);
        var w = 1f / _rb.Mass + Vector3.Dot(rxn, Iinv.MultiplyVector(rxn));
        var dLambda = (-c - alphaTilde * _lambda) / (w + alphaTilde);
        _lambda += dLambda;
        
        _rb.ApplyImpulseToPositions(rW, n * dLambda);
        _rb.SetVelocitiesFromPrev();
    }
}