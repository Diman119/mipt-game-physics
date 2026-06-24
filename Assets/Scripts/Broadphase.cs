using System;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

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

    // LBVH ===================================================================================
    static uint[] _mortonCodes;
    static LBVHNode[] _LBVHNodes;
    static int _LBVHinternalNodeCounter;

    struct LBVHNode {
        public Bounds bounds;
        public int left;
        public int right;
        public int bodyIndex;
        public bool IsLeaf => bodyIndex >= 0;
    }

    // Interleave bits: x[9:0], y[9:0], z[9:0] → 30-bit Morton code
    static uint ComputeMortonCode(Vector3 pos, Vector3 minBound, Vector3 maxBound) {
        // Normalize to [0,1]
        float xNorm = (pos.x - minBound.x) / (maxBound.x - minBound.x);
        float yNorm = (pos.y - minBound.y) / (maxBound.y - minBound.y);
        float zNorm = (pos.z - minBound.z) / (maxBound.z - minBound.z);

        // Quantize to 10-bit integers (0–1023)
        int x = Mathf.Clamp((int)(xNorm * 1023.0f), 0, 1023);
        int y = Mathf.Clamp((int)(yNorm * 1023.0f), 0, 1023);
        int z = Mathf.Clamp((int)(zNorm * 1023.0f), 0, 1023);

        // Encode by bit interleaving
        return (SeparateBy2(x) << 2) | (SeparateBy2(y) << 1) | SeparateBy2(z);
    }

    // Insert two zeros after each bit (e.g., abc → a00b00c00)
    static uint SeparateBy2(int a) {
        uint x = (uint)a & 1023;
        x = (x | (x << 16)) & 0x030000FFu;
        x = (x | (x << 8)) & 0x0300F00Fu;
        x = (x | (x << 4)) & 0x030C30C3u;
        x = (x | (x << 2)) & 0x09249249u;
        return x;
    }

    public static void LBVH(MyRigidbody[] bodies) {
        if (bodies.Length < 2) return;

        BuildLBVH(bodies);
        TraverseLBVH();
    }

    static void BuildLBVH(MyRigidbody[] bodies) {
        if (_sortedIndices is null || bodies.Length != _sortedIndices.Length) {
            _sortedIndices = new int[bodies.Length];
            _mortonCodes = new uint[bodies.Length];
            _LBVHNodes = new LBVHNode[bodies.Length * 2 - 1];
            for (int i = 0; i < _sortedIndices.Length; i++) _sortedIndices[i] = i;
        }

        int leafCount = bodies.Length;
        
        Vector3 sceneMin = bodies[0].AABB.min;
        Vector3 sceneMax = bodies[0].AABB.max;
        for (int i = 1; i < leafCount; i++) {
            sceneMin = Vector3.Min(sceneMin, bodies[i].AABB.min);
            sceneMax = Vector3.Max(sceneMax, bodies[i].AABB.max);
        }

        for (int i = 0; i < leafCount; i++) {
            _mortonCodes[i] = ComputeMortonCode(bodies[i].transform.position, sceneMin, sceneMax);
        }

        Array.Sort(_sortedIndices, (a, b) => _mortonCodes[a].CompareTo(_mortonCodes[b]));

        for (int i = 0; i < leafCount; i++) {
            int bodyIdx = _sortedIndices[i];
            _LBVHNodes[i].bounds = bodies[bodyIdx].AABB;
            _LBVHNodes[i].bodyIndex = bodyIdx;
        }

        for (int i = leafCount; i < _LBVHNodes.Length; i++) {
            _LBVHNodes[i].bodyIndex = -1;
        }

        _LBVHinternalNodeCounter = leafCount;

        BuildLBVHRecursive(0, leafCount - 1);
    }

    static int BuildLBVHRecursive(int first, int last) {
        // Базовый случай: если в диапазоне один элемент, это лист.
        // Он уже лежит в списке под индексом `first`.
        if (first == last) {
            return first;
        }

        // Выделяем индекс для текущего внутреннего узла
        int currentNodeIndex = _LBVHinternalNodeCounter++;

        // Находим, где разделить диапазон [first, last] на две части
        int split = FindSplit(first, last);

        // Рекурсивно строим левое и правое поддеревья
        int leftChild = BuildLBVHRecursive(first, split);
        int rightChild = BuildLBVHRecursive(split + 1, last);

        // Записываем связи
        _LBVHNodes[currentNodeIndex].left = leftChild;
        _LBVHNodes[currentNodeIndex].right = rightChild;

        // Рассчитываем Bounds (AABB) снизу вверх
        Bounds leftBounds = _LBVHNodes[leftChild].bounds;
        Bounds rightBounds = _LBVHNodes[rightChild].bounds;

        // Объединяем Bounds левого и правого потомка
        _LBVHNodes[currentNodeIndex].bounds = leftBounds;
        _LBVHNodes[currentNodeIndex].bounds.Encapsulate(rightBounds);

        return currentNodeIndex;
    }

    static int FindSplit(int first, int last) {
        uint firstCode = _mortonCodes[_LBVHNodes[first].bodyIndex];
        uint lastCode = _mortonCodes[_LBVHNodes[last].bodyIndex];

        // Если коды Мортона идентичны (объекты в одной точке), делим диапазон пополам
        if (firstCode == lastCode) {
            return first + (last - first) / 2;
        }

        // Находим количество общих старших битов у первого и последнего кода
        int commonPrefix = CommonPrefixLength(firstCode, lastCode);

        // Используем двоичный поиск, чтобы найти точку, где этот бит меняется с 0 на 1
        int split = first;
        int step = last - first;

        do {
            step = (step + 1) >> 1; // Деление на 2 с округлением вверх
            int newSplit = split + step;

            if (newSplit < last) {
                uint midCode = _mortonCodes[_LBVHNodes[newSplit].bodyIndex];
                if (CommonPrefixLength(firstCode, midCode) > commonPrefix) {
                    split = newSplit; // Префикс совпадает, сдвигаем границу вправо
                }
            }
        } while (step > 1);

        return split;
    }

    // Вычисляет количество общих старших битов у двух чисел (CLZ)
    static int CommonPrefixLength(uint a, uint b) {
        uint xor = a ^ b;
        if (xor == 0) return 32;

        int count = 0;
        if ((xor & 0xFFFF0000) == 0) {
            count += 16;
            xor <<= 16;
        }
        if ((xor & 0xFF000000) == 0) {
            count += 8;
            xor <<= 8;
        }
        if ((xor & 0xF0000000) == 0) {
            count += 4;
            xor <<= 4;
        }
        if ((xor & 0xC0000000) == 0) {
            count += 2;
            xor <<= 2;
        }
        if ((xor & 0x80000000) == 0) {
            count += 1;
        }

        return count;
    }

    static readonly Stack<int> _LBVHTraverseStack = new();

    static void TraverseLBVH() {
        _pairIndices.Clear();
        _LBVHTraverseStack.Clear();
        
        int rootIndex = (_LBVHNodes.Length + 1) / 2; // Корень дерева всегда первая внутренняя нода

        // Помещаем в стек проверку корня самого с собой
        _LBVHTraverseStack.Push(rootIndex);
        _LBVHTraverseStack.Push(rootIndex);

        while (_LBVHTraverseStack.Count > 0) {
            // Достаем пару узлов для проверки
            int idxB = _LBVHTraverseStack.Pop();
            int idxA = _LBVHTraverseStack.Pop();

            LBVHNode nodeA = _LBVHNodes[idxA];
            LBVHNode nodeB = _LBVHNodes[idxB];

            // Если оба узла - листья
            if (nodeA.IsLeaf && nodeB.IsLeaf) {
                var i1 = nodeA.bodyIndex;
                var i2 = nodeB.bodyIndex;

                // Исключаем проверку объекта самого с собой и дубликаты
                if (i1 >= i2) {
                    continue;
                }
                
                // Если AABBs не пересекаются, эта ветка нас не интересует
                if (nodeA.bounds.Intersects(nodeB.bounds)) {
                    _pairIndices.Add(i1);
                    _pairIndices.Add(i2);
                }
                
                continue;
            }
            
            // Если AABBs не пересекаются, эта ветка нас не интересует
            if (!nodeA.bounds.Intersects(nodeB.bounds)) {
                continue;
            }

            // Раскрываем дерево дальше
            // Чтобы избежать избыточных проверок (например, проверять пары (A,B) и (B,A)),
            // мы всегда раскрываем тот узел, который является внутренним, или тот, который "больше/глубже".
            if (nodeB.IsLeaf || (!nodeA.IsLeaf && idxA < idxB)) {
                // Раскрываем узел A, сравниваем его детей с узлом B
                _LBVHTraverseStack.Push(nodeA.left);
                _LBVHTraverseStack.Push(idxB);
                _LBVHTraverseStack.Push(nodeA.right);
                _LBVHTraverseStack.Push(idxB);
            }
            else {
                // Раскрываем узел B, сравниваем узел A с его детьми
                _LBVHTraverseStack.Push(idxA);
                _LBVHTraverseStack.Push(nodeB.left);
                _LBVHTraverseStack.Push(idxA);
                _LBVHTraverseStack.Push(nodeB.right);
            }
        }
    }

#if UNITY_EDITOR
    public static void DrawLBVH() {
        if (_LBVHNodes is null) return;

        // Сохраняем исходный цвет Gizmos, чтобы не сломать отрисовку других компонентов
        Color originalColor = Gizmos.color;

        for (int i = 0; i < _LBVHNodes.Length; i++) {
            LBVHNode node = _LBVHNodes[i];

            if (node.IsLeaf) {
                // Генерируем стабильный псевдослучайный цвет на основе bodyIndex
                Random.InitState(node.bodyIndex + 54321);
                Color leafColor = Random.ColorHSV(0f, 1f, 0.6f, 0.9f, 0.7f, 1f);
                leafColor.a = 0.5f; // Задаем полупрозрачность

                Gizmos.color = leafColor;
                // Рисуем сплошной полупрозрачный куб для листа
                Gizmos.DrawCube(node.bounds.center, node.bounds.size);
            }
            else {
                // Генерируем стабильный цвет для внутренней ноды на основе её индекса в массиве
                Random.InitState(i + 12345);
                Color wireColor = Random.ColorHSV(0f, 1f, 0.8f, 1f, 0.8f, 1f);

                Gizmos.color = wireColor;
                // Рисуем проволочный куб для внутренней ноды
                Gizmos.DrawWireCube(node.bounds.center, node.bounds.size);
            }
        }

        // Возвращаем исходный цвет обратно
        Gizmos.color = originalColor;
    }
#endif
}