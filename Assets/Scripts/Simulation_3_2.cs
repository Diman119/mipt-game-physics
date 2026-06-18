using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

public class Simulation_3_2 : MonoBehaviour {
    MyRigidbody[] _bodies;

    [SerializeField] GameObject _bodyPrefab;
    [SerializeField] int _bodyCount = 10;
    [SerializeField] Bounds _spawnBounds;
    [SerializeField] Vector3 _minSize;
    [SerializeField] Vector3 _maxSize;
    [SerializeField] float _bodyDensity = 0.1f;
    [SerializeField] float _baumgarte = 0.2f;
    [SerializeField] int _SIIterations = 4;
    
    float Dt => Time.fixedDeltaTime;
    
    void SpawnBodies() {
        for (int i = 0; i < _bodyCount; ++i) {
            var b = Instantiate(_bodyPrefab, transform).GetComponent<MyRigidbody>();
            var pos = ExtensionMethods.RandomVectorBetween(_spawnBounds.min, _spawnBounds.max);
            var rot = Random.rotation;
            pos.y = i * _minSize.y + _spawnBounds.min.y;
            b.transform.SetPositionAndRotation(pos, rot);
            var size = ExtensionMethods.RandomVectorBetween(_minSize, _maxSize);
            var mass = size.x * size.y * size.z * _bodyDensity;
            b.SetSizeAndMass(size, mass);
        }
    }

    void Awake() {
        SpawnBodies();
    }

    void Start() {
        _bodies = GetComponentsInChildren<MyRigidbody>();
    }

#if UNITY_EDITOR
    void OnDrawGizmos() {
        Handles.color = Color.yellow;
        Handles.DrawWireCube(_spawnBounds.center, _spawnBounds.size);
    }
#endif

    void FixedUpdate() {
        foreach (var b in _bodies) {
            b.UpdateAABB();
            if (b.Mass == 0f) {
                continue;
            }
            b.IntegrateVelocities();
        }
        
        Broadphase.Basic(_bodies);
        Narrowphase.GenerateContacts(_bodies);
        
        var contactList = Narrowphase.GetContacts();
        var contacts = contactList.GetInternalArray();  // for modifying struct fields in place
        for (int i = 0; i < _SIIterations; ++i) {
            for (int j = 0; j < contactList.Count; ++j) {
                var cnt = contacts[j];
                var bodyA = _bodies[cnt.bodyIndexA];
                var bodyB = _bodies[cnt.bodyIndexB];
                var rAW = bodyA.transform.TransformDirection(cnt.anchorA);
                var rBW = bodyB.transform.TransformDirection(cnt.anchorB);
                var penetration = Vector3.Dot(-rAW - bodyA.transform.position + rBW + bodyB.transform.position, cnt.normal);
                var c = penetration + 0.001f;  // to avoid contact jitter
                
                var relativeVelocity =
                    Vector3.Dot(cnt.normal, bodyB.GetPointVelocity(rBW) - bodyA.GetPointVelocity(rAW));
                var w = MyRigidbody.GetEffectiveInvMass(bodyA, bodyB, rAW, rBW, cnt.normal);
                var dLambda = (-relativeVelocity - _baumgarte * c / Dt) / w;
                var newLambda = Mathf.Max(0f, cnt.lambdaN + dLambda);
                dLambda = newLambda - cnt.lambdaN;
                contacts[j].lambdaN = newLambda;
                
                if (dLambda <= 0f) {
                    continue;
                }

                if (bodyA.Mass > 0f) bodyA.ApplyImpulseToVelocities(rAW, cnt.normal * -dLambda);
                if (bodyB.Mass > 0f) bodyB.ApplyImpulseToVelocities(rBW, cnt.normal * dLambda);
            }
        }

        foreach (var b in _bodies) {
            if (b.Mass == 0f) {
                continue;
            }
            b.IntegratePositions();
        }
    }
}