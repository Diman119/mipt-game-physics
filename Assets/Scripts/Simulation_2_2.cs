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
        _rb.IntegratePositions();
        
        var omega = 2f * Mathf.PI * _softHz;
        var compliance = 1f / (_rb.Mass * omega * omega);
        var alphaTilde = compliance / (Dt * Dt);

        var attachW = _anchor2.position;
        var rW = attachW - _rb.transform.position;
        var d = attachW - _anchor1.position;
        var len = d.magnitude;
        if (len < 1e-6) {
            return;
        }
        var n = d / len;
        var w = _rb.GetEffectiveInvMass(rW, n);
        var dLambda = (-len - alphaTilde * _lambda) / (w + alphaTilde);
        _lambda += dLambda;
        
        _rb.ApplyImpulseToPositions(rW, n * dLambda);
        _rb.SetVelocitiesFromPrev();
    }
}