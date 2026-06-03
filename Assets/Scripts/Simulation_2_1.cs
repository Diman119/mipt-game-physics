using UnityEngine;

public class SpringForce : MonoBehaviour {
    [SerializeField] Transform _anchor1;
    [SerializeField] Transform _anchor2;
    [SerializeField] Transform _spring;
    [SerializeField] float _k = 45f;
    
    MyRigidbody _rb;

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
        _rb.ApplyForceAtPoint((_anchor1.position - _anchor2.position) * _k, _anchor2.position);
        _rb.IntegrateVelocities();
        _rb.ApplyVelocities();
    }
}
