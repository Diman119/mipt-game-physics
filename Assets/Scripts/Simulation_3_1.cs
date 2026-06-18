using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

public enum BroadphaseType {
    Basic,
    SpatialGrid,
    SweepAndPrune,
}

// also used for 3_3 and 4_x
public class Simulation_3_1 : MonoBehaviour {
    MyRigidbody[] _bodies;

    [SerializeField] GameObject _bodyPrefab;
    [SerializeField] int _bodyCount = 10;
    [SerializeField] Bounds _spawnBounds;
    [SerializeField] Vector3 _minSize;
    [SerializeField] Vector3 _maxSize;
    [SerializeField] float _bodyDensity = 0.1f;

    [SerializeField] float _staticFriction = 1f;
    [SerializeField] float _dynamicFriction = 0.4f;

    [SerializeField] int _positionIterations = 3;
    
    [SerializeField] BroadphaseType _broadphaseType = BroadphaseType.Basic;
    [SerializeField] float _gridCellSize = 2f;

    void RunBroadphase() {
        switch (_broadphaseType) {
            case BroadphaseType.Basic: Broadphase.Basic(_bodies); break;
            case BroadphaseType.SpatialGrid: Broadphase.SpatialGrid(_bodies, _gridCellSize); break;
            case BroadphaseType.SweepAndPrune: Broadphase.SweepAndPrune(_bodies); break;
        }
    }

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
        // predict
        foreach (var b in _bodies) {
            b.SavePrevPosition();
            
            if (b.Mass == 0f) {
                b.UpdateAABB();
                continue;
            }
            
            b.IntegrateVelocities();
            b.IntegratePositions();

            const float M = 5e3f;
            var p = b.transform.position;
            b.transform.position = new(Mathf.Clamp(p.x, -M, M), Mathf.Clamp(p.y, -M, M), Mathf.Clamp(p.z, -M, M));
            
            b.UpdateAABB();
        }
        
        // get contacts
        RunBroadphase();
        Narrowphase.GenerateContacts(_bodies);

        var contactList = Narrowphase.GetContacts();
        var contacts = contactList.GetInternalArray();  // for modifying struct fields in place
        for (int i = 0; i < _positionIterations; ++i) {
            for (int j = 0; j < contactList.Count; ++j) {
                var cnt = contacts[j];
                
                // normal solve
                var bodyA = _bodies[cnt.bodyIndexA];
                var bodyB = _bodies[cnt.bodyIndexB];
                var rAW = bodyA.transform.TransformDirection(cnt.anchorA);
                var rBW = bodyB.transform.TransformDirection(cnt.anchorB);
                var pA = rAW + bodyA.transform.position;
                var pB = rBW + bodyB.transform.position;
                var penetration = Vector3.Dot(pB - pA, cnt.normal);
                var c = penetration + 0.001f;  // to avoid contact jitter

                if (c >= 0f) {
                    continue;
                }
                
                var w = MyRigidbody.GetEffectiveInvMass(bodyA, bodyB, rAW, rBW, cnt.normal);
                var dLambdaN = -c / w;
                dLambdaN = Mathf.Min(dLambdaN, 0.01f);
                if (bodyA.Mass > 0f) bodyA.ApplyImpulseToPositions(rAW, cnt.normal * -dLambdaN);
                if (bodyB.Mass > 0f) bodyB.ApplyImpulseToPositions(rBW, cnt.normal * dLambdaN);

                var newLambdaN = cnt.lambdaN + dLambdaN;
                contacts[j].lambdaN = newLambdaN;
                
                // friction solve
                var pAPrev = bodyA.PrevPosition + bodyA.PrevRotation * cnt.anchorA;
                var pBPrev = bodyB.PrevPosition + bodyB.PrevRotation * cnt.anchorB;
                var dA = pA - pAPrev;
                var dB = pB - pBPrev;
                var dP = dB - dA;
                var dPTan = Vector3.ProjectOnPlane(dP, cnt.normal);
                var dPMag = dPTan.magnitude;
                if (dPMag < 1e-6f) continue;
                var frictionDir = dPTan / dPMag;
                var wf = MyRigidbody.GetEffectiveInvMass(bodyA, bodyB, rAW, rBW, frictionDir);
                var totalLambdaT = cnt.lambdaT - dPMag / wf;
                var newLambdaT = totalLambdaT > -_staticFriction * newLambdaN
                    ? totalLambdaT  // static
                    : -_dynamicFriction * newLambdaN;  // dynamic
                var dLambdaT = newLambdaT - cnt.lambdaT;
                if (bodyA.Mass > 0f) bodyA.ApplyImpulseToPositions(rAW, frictionDir * -dLambdaT);
                if (bodyB.Mass > 0f) bodyB.ApplyImpulseToPositions(rBW, frictionDir * dLambdaT);
                contacts[j].lambdaT = newLambdaT;
            }
        }

        foreach (var b in _bodies) {
            if (b.Mass == 0f) {
                continue;
            }
            b.SetVelocitiesFromPrev();
        }
    }
}