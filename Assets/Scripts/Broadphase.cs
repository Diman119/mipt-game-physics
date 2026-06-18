using System.Collections.Generic;
using UnityEngine;

public static class Broadphase {
    static readonly List<int> _pairIndices = new();

    public static List<int> GetIndices() => _pairIndices;

    // Basic ================================================================
    public static void Basic(MyRigidbody[] bodies) {
        _pairIndices.Clear();
        for (int i = 0; i < bodies.Length; i++) {
            for (int j = i + 1; j < bodies.Length; j++) {
                if (bodies[i].AABB.Intersects(bodies[j].AABB)) {
                    _pairIndices.Add(i);
                    _pairIndices.Add(j);
                }
            }
        }
    }

    // Spatial grid =========================================================
    // Reusable grid for spatial partitioning - persists across frames
    static readonly Dictionary<long, List<int>> _grid = new();
    static readonly Stack<List<int>> _listIntPool = new();

    // Reusable HashSet for deduping pairs
    static readonly HashSet<long> _pairSet = new();

    // Encodes 3D cell coordinates into a single long
    static long EncodeCell(int ix, int iy, int iz) {
        const long mask = 0x1FFFFF; // 21 bits: 0x1FFFFF = 2097151
        return ((ix & mask) << 42) | ((iy & mask) << 21) | (iz & mask);
    }

    public static void SpatialGrid(MyRigidbody[] bodies, float cellSize) {
        _pairIndices.Clear();
        
        // Clear all lists in the grid
        foreach (var pair in _grid) {
            pair.Value.Clear();
            _listIntPool.Push(pair.Value);
        }
        _grid.Clear();

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

        foreach (var pair in _grid) {
            var objects = pair.Value;
            if (objects.Count < 2) continue;

            for (int a = 0; a < objects.Count; a++) {
                for (int b = a + 1; b < objects.Count; b++) {
                    int i1 = objects[a];
                    int i2 = objects[b];
                    var pairKey = ((long)i1 << 32) | (uint)i2;
                    if (_pairSet.Add(pairKey) && bodies[i1].AABB.Intersects(bodies[i2].AABB)) {
                        _pairIndices.Add(i1);
                        _pairIndices.Add(i2);
                    }
                }
            }
        }
    }

    static Vector3Int GetCellCoords(Vector3 position, float cellSize) {
        return new Vector3Int(
            (int)Mathf.Floor(position.x / cellSize),
            (int)Mathf.Floor(position.y / cellSize),
            (int)Mathf.Floor(position.z / cellSize)
        );
    }

    // Sweep and Prune =========================================================
    static int[] _sortedIndices;

    public static void SweepAndPrune(MyRigidbody[] bodies) {
        _pairIndices.Clear();
        
        if (_sortedIndices is null || bodies.Length != _sortedIndices.Length) {
            _sortedIndices = new int[bodies.Length];
            for (int i = 0; i < _sortedIndices.Length; i++) _sortedIndices[i] = i;
        }
        
        // Sort by Y axis
        for (int i = 1; i < _sortedIndices.Length; i++) {
            int currentIndex = _sortedIndices[i];
            float currentMinY = bodies[currentIndex].AABB.min.y;
            int j = i - 1;

            // Shift elements that are greater than currentMinY to the right
            while (j >= 0 && bodies[_sortedIndices[j]].AABB.min.y > currentMinY) {
                _sortedIndices[j + 1] = _sortedIndices[j];
                j--;
            }

            _sortedIndices[j + 1] = currentIndex;
        }
        
        for (int i = 0; i < _sortedIndices.Length; i++) {
            int idxA = _sortedIndices[i];
            var a = bodies[idxA].AABB;

            float maxX_A = a.max.x;
            float minX_A = a.min.x;
            float maxY_A = a.max.y;
            float minZ_A = a.min.z;
            float maxZ_A = a.max.z;

            for (int j = i + 1; j < _sortedIndices.Length; j++) {
                int idxB = _sortedIndices[j];
                var b = bodies[idxB].AABB;

                // Because the array is sorted by min.y, if the next object's min.y
                // is greater than our max.y, no subsequent objects can overlap on the Y axis.
                if (b.min.y > maxY_A) {
                    break;
                }

                // Y-axis overlap is guaranteed. Check X and Z axes.
                if (maxX_A >= b.min.x && minX_A <= b.max.x &&
                    maxZ_A >= b.min.z && minZ_A <= b.max.z) {
                    _pairIndices.Add(idxA);
                    _pairIndices.Add(idxB);
                }
            }
        }
    }
}