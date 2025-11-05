using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Unity.Mathematics;
using PBA.Fluid2D.Main;

using static UnityEngine.Mathf;

namespace PBA.Fluid2D.Helper
{
    public class SpatialHash
    {
        public Vector2[] points;
        public float radius;
        
        // public void USpatialLookup(Vector2[] points, float radius)
        // {
        //     this.points = points;
        //     this.radius = radius;
        //
        //     // create Spatial Lookup
        //     Parallel.For(0, points.Length, i =>
        //     {
        //         (int cellX, int cellY) = position2CellCord(points[i], radius);
        //         uint cellKey = GetKeyFromHash(HashCell(cellX, cellY));
        //         spatialLookup[i] = new Entry(i, cellKey);
        //         startIndices[i] = int.MaxValue; // Reset start index
        //     });
        //
        //     // sort
        //     Array.Sort(spatialLookup);
        //
        //     // calculater start index ...
        //     Parallel.For(0, points.Length, i =>
        //     {
        //         uint key = spatialLookup[i].cellKey;
        //         uint keyPrev = i == 0 ? uint.MaxValue : spatialLookup[i - 1].cellKey;
        //         if (key != keyPrev) startIndices[key] = i;
        //     });
        // }

        public (int x, int y) Pos2CellCord(Vector2 point, float radius)
        {
            int cellX = (int)(point.x / radius);
            int cellY = (int)(point.y / radius);
            return (cellX, cellY);
        }

        public uint HashCell(int cellX, int cellY)
        {
            uint a = (uint)cellX * 15823;
            uint b = (uint)cellY * 9737333;
            return a + b;
        }

        public uint GetKeyFromHash(uint hash)
        {
            return hash /* % (uint)spatialLookup.Length*/;
        }
        
    }
}

