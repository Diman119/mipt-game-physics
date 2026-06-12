using System.Collections.Generic;
using UnityEngine;

public static class Broadphase {
    public struct IntPair {
        public int i1, i2;

        // Encode pair as long for HashSet (smaller index in upper 32 bits)
        public long ToLong() {
            if (i1 < i2) {
                return ((long)i1 << 32) | (uint)i2;
            } else {
                return ((long)i2 << 32) | (uint)i1;
            }
        }

        public static IntPair FromLong(long value) {
            return new IntPair { i1 = (int)(value >> 32), i2 = (int)(value & 0xFFFFFFFF) };
        }
    }

    // Reusable grid for spatial partitioning - persists across frames
    private static readonly Dictionary<long, List<int>> _grid = new Dictionary<long, List<int>>();
    private static readonly Stack<List<int>> _listIntPool = new();
    // Reusable HashSet for deduping pairs
    private static readonly HashSet<long> _pairSet = new HashSet<long>();
    private static readonly List<IntPair> _pairList = new List<IntPair>();

    // Encodes 3D cell coordinates into a single long
    private static long EncodeCell(int ix, int iy, int iz) {
        const long mask = 0x1FFFFF; // 21 bits: 0x1FFFFF = 2097151
        return ((ix & mask) << 42) | ((iy & mask) << 21) | (iz & mask);
    }

    public static IEnumerator<IntPair> Basic(MyRigidbody[] bodies) {
        for (int i = 0; i < bodies.Length; i++) {
            for (int j = i + 1; j < bodies.Length; j++) {
                if (bodies[i].AABB.Intersects(bodies[j].AABB)) {
                    yield return new IntPair { i1 = i, i2 = j };
                }
            }
        }
    }

    public static IEnumerator<IntPair> SpatialGrid(MyRigidbody[] bodies, float cellSize) {
        // Clear all lists in the grid
        foreach (var pair in _grid) {
            pair.Value.Clear();
            _listIntPool.Push(pair.Value);
        }
        _grid.Clear();

        if (bodies.Length == 0) {
            yield break;
        }

        // Calculate global bounds of all objects
        Bounds globalBounds = bodies[0].AABB;
        for (int i = 1; i < bodies.Length; i++) {
            globalBounds.Encapsulate(bodies[i].AABB);
        }

        // Insert each body into overlapping grid cells
        for (int i = 0; i < bodies.Length; i++) {
            var aabb = bodies[i].AABB;
            var minCell = GetCellCoords(aabb.min, cellSize);
            var maxCell = GetCellCoords(aabb.max, cellSize);

            for (int ix = minCell.x; ix <= maxCell.x; ix++) {
                for (int iy = minCell.y; iy <= maxCell.y; iy++) {
                    for (int iz = minCell.z; iz <= maxCell.z; iz++) {
                        long key = EncodeCell(ix, iy, iz);
                        if (!_grid.ContainsKey(key)) {
                            _grid[key] = _listIntPool.Count > 0 ? _listIntPool.Pop() : new();
                        }
                        _grid[key].Add(i);
                    }
                }
            }
        }

        // Check collisions within each cell and dedupe using HashSet
        _pairSet.Clear();
        _pairList.Clear();

        foreach (var pair in _grid) {
            var objects = pair.Value;
            if (objects.Count < 2) continue;

            for (int a = 0; a < objects.Count; a++) {
                for (int b = a + 1; b < objects.Count; b++) {
                    int i1 = objects[a];
                    int i2 = objects[b];
                    var pairKey = ((long)i1 << 32) | (uint)i2;
                    if (_pairSet.Add(pairKey) && bodies[i1].AABB.Intersects(bodies[i2].AABB)) {
                        yield return new IntPair { i1 = i1, i2 = i2 };
                    }
                }
            }
        }
    }

    private static Vector3Int GetCellCoords(Vector3 position, float cellSize) {
        return new Vector3Int(
            (int)Mathf.Floor(position.x / cellSize),
            (int)Mathf.Floor(position.y / cellSize),
            (int)Mathf.Floor(position.z / cellSize)
        );
    }
}
