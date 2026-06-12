using UnityEngine;

public class Simulation_1_2 : MonoBehaviour {
    MyRigidbody _rb;
    Vector3 _L0;
    
    [SerializeField] Transform _LTransform;
    [SerializeField] Transform _L0Transform;
    [SerializeField] float _scale = 0.13f;

    void Visualize(Vector3 v, Transform t) {
        t.LookAt(t.position + v);
        t.localScale = Vector3.one * (v.magnitude * _scale);
    }

    void Awake() {
        _rb = GetComponent<MyRigidbody>();
    }

    void Start() {
        _rb.omega = new(0.5f, 5f, 0f);
        _L0 = _rb.L;
    }

    void FixedUpdate() {
        _rb.transform.rotation = _rb.IntegrateRotation(_rb.omega);
        Visualize(_rb.L, _LTransform);
        Visualize(_L0, _L0Transform);
    }
}
