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
    public struct Entry: IComparable<Entry>
    {
        public int index;
        public uint cellKey;
        public Entry(int index, uint cellKey)
        {
            this.index = index;
            this.cellKey = cellKey;
        }
        
        public int CompareTo(Entry other)
        {
            return cellKey.CompareTo(other.cellKey);
        }
    }

    public class SpatialHash
    {
        // public struct Entry
        // {
        //     public int index;
        //     public uint cellKey;
        //     public int PIndex;
        //     public Entry(int index, uint cellKey)
        //     {
        //         this.index = index;
        //         this.cellKey = cellKey;
        //         this.PIndex = -1;
        //     }
        // }

        public Vector2[] points;
        public float radius;

        // public (int, int)[] SHOffsets = new  (int, int)[9]
        // {
        //     (-1, 1),  (0, 1),  (1, 1), 
        //     (-1, 0),  (0, 0),  (1, 0),
        //     (-1, -1), (0, -1), (1, -1)
        // };
        
        
        
        // public void USpatialLookup(Vector2[] points, float radius)
        // {
        //     this.points = points;
        //     this.radius = radius;
        //
        //     // create Spatial Lookup
        //     Parallel.For(0, points.Length, i =>
        //     {
        //         (int cellX, int cellY) = Pos2CellCord(points[i], radius);
        //         uint cellKey = GetKeyFromHash(HashCell(cellX, cellY));
        //         // spatialLookup[i] = new SpatialHash.Entry(i, cellKey);
        //         spatialLookup[i] = new Entry(i, cellKey);
        //         startIndices[i] = int.MaxValue; // Reset start index
        //     });
        //
        //     // sort
        //     Array.Sort(spatialLookup);
        //
        //     // calculater start index
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

