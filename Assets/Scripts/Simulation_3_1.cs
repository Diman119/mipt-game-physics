using System.Collections.Generic;
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

    [SerializeField] int _positionIterations = 3;
    
    [SerializeField] BroadphaseType _broadphaseType = BroadphaseType.Basic;
    [SerializeField] float _gridCellSize = 2f;

    IEnumerator<Broadphase.IntPair> SelectedBroadphase {
        get {
            switch (_broadphaseType) {
                case BroadphaseType.Basic: return Broadphase.Basic(_bodies);
                case BroadphaseType.SpatialGrid: return Broadphase.SpatialGrid(_bodies, _gridCellSize);
                case BroadphaseType.SweepAndPrune: return Broadphase.SweepAndPrune(_bodies);
                default: return null;
            }
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
        foreach (var b in _bodies) {
            if (b.Mass == 0f) {
                b.UpdateAABB();
                continue;
            }
            b.SavePrevPosition();
            b.IntegrateVelocities();
            b.IntegratePositions();
            b.UpdateAABB();
        }
        
        Narrowphase.GenerateContacts(_bodies, SelectedBroadphase);

        for (int i = 0; i < _positionIterations; ++i) {
            foreach (var cnt in Narrowphase.GetContacts()) {
                var bodyA = _bodies[cnt.bodyIndexA];
                var bodyB = _bodies[cnt.bodyIndexB];
                var rAW = bodyA.transform.TransformDirection(cnt.anchorA);
                var rBW = bodyB.transform.TransformDirection(cnt.anchorB);
                var penetration = Vector3.Dot(-rAW - bodyA.transform.position + rBW + bodyB.transform.position, cnt.normal);
                var c = penetration + 0.001f;  // to avoid contact jitter

                if (c >= 0f) {
                    continue;
                }
                
                var w = MyRigidbody.GetEffectiveInvMass(bodyA, bodyB, rAW, rBW, cnt.normal);
                var dLambda = -c / w;
                dLambda = Mathf.Min(dLambda, 0.01f);
                if (bodyA.Mass > 0f) bodyA.ApplyImpulseToPositions(rAW, cnt.normal * -dLambda);
                if (bodyB.Mass > 0f) bodyB.ApplyImpulseToPositions(rBW, cnt.normal * dLambda);
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