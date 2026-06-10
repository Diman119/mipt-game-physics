using System.Collections.Generic;
using UnityEngine;

public enum BroadphaseType {
    Basic,
    SpatialGrid
}

public class Simulation_3_1 : MonoBehaviour {
    MyRigidbody[] _bodies;

    [SerializeField] GameObject _bodyPrefab;
    [SerializeField] int _bodyCount = 10;
    [SerializeField] Bounds _spawnBounds;
    [SerializeField] Bounds _sizeBounds;
    [SerializeField] float _bodyDensity = 0.1f;

    [SerializeField] int _positionIterations = 3;
    
    [SerializeField] BroadphaseType _broadphaseType = BroadphaseType.Basic;
    [SerializeField] float _gridCellSize = 2f;

    float Dt => Time.fixedDeltaTime;

    void SpawnBodies() {
        for (int i = 0; i < _bodyCount; ++i) {
            var b = Instantiate(_bodyPrefab, transform).GetComponent<MyRigidbody>();
            b.transform.position = ExtensionMethods.RandomVectorInBounds(_spawnBounds);
            var size = ExtensionMethods.RandomVectorInBounds(_sizeBounds);
            var mass = size.x * size.y * size.z * _bodyDensity;
            b.SetSizeAndMass(size, mass);
        }
    }

    void Awake() {
        Random.InitState(42);
        SpawnBodies();
    }

    void Start() {
        _bodies = GetComponentsInChildren<MyRigidbody>();
    }

    void FixedUpdate() {
        foreach (var b in _bodies) {
          if (b.Mass == 0f) {
            continue;
          }
          b.SavePrevPosition();
          b.IntegrateVelocities();
          b.ApplyVelocities();
        }

        // Run broadphase and narrowphase
        IEnumerator<Broadphase.IntPair> broadphase;
        if (_broadphaseType == BroadphaseType.Basic) {
            broadphase = Broadphase.Basic(_bodies);
        } else {
            broadphase = Broadphase.SpatialGrid(_bodies, _gridCellSize);
        }
        Narrowphase.GenerateContacts(_bodies, broadphase);

        foreach (var b in _bodies) {
            if (b.Mass == 0f) {
                continue;
            }
            b.SetVelocitiesFromPrev();
        }
    }
}