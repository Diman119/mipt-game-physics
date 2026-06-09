using System.Collections.Generic;
using UnityEngine;

public static class Broadphase {
    public struct IntPair {
        public int i1, i2;
    }

    // Reusable grid for spatial partitioning - persists across frames
    private static readonly Dictionary<long, List<int>> _grid = new Dictionary<long, List<int>>();

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
        // Clear all lists in the grid (don't clear dictionary to avoid allocations)
        foreach (var pair in _grid) {
            pair.Value.Clear();
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
                            _grid[key] = new List<int>();
                        }
                        _grid[key].Add(i);
                    }
                }
            }
        }

        // Check collisions within each cell
        foreach (var pair in _grid) {
            var objects = pair.Value;
            if (objects.Count < 2) continue;

            for (int a = 0; a < objects.Count; a++) {
                for (int b = a + 1; b < objects.Count; b++) {
                    int i1 = objects[a];
                    int i2 = objects[b];
                    if (bodies[i1].AABB.Intersects(bodies[i2].AABB)) {
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
