using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Unity.Mathematics;
using UnityEngine.EventSystems;
using UnityEngine.UI;

using PBA.Fluid2D.Helper;

using static UnityEngine.Mathf;
using Random = System.Random;

namespace PBA.Fluid2D.Main
{
    public class Fluid2D_4 : MonoBehaviour
    {
        [SerializeField] Transform pointPrefab;
        Transform[] myPartical;
        SpriteRenderer[] r;

        [SerializeField] float mass = 1;
        [SerializeField] float smoothRadius = 2;
        [SerializeField] float collisionDamping;
        [SerializeField] float particleSize;
        [SerializeField] float gravity;
        [SerializeField] int numParticles;
        [SerializeField] float particleSpacing;
        [SerializeField] Vector2 boundSize;

        [SerializeField] float targetDensity;
        [SerializeField] float pressureMultiplier;
        
        Vector2[] position;
        Vector2[] predictPos;
        Vector2[] velocity;
        Vector2[] particleProperty;
        float[] densities;

        // test for spatial hash //////////////////////////////////////////
        //public SpatialHash.Entry[] spatialLookup;
        public Entry[] spatialLookup;
        public int[] startIndices;
        Vector2[] points;
        float radius;
        
        // end test ///////////////////////////////////////////////////////

        Color skyBlue = new Color(135f / 255f, 206f / 255f, 235f / 255f);

        void Start()
        {
            position = new Vector2[numParticles];
            predictPos = new Vector2[numParticles];
            
            velocity = new Vector2[numParticles];
            particleProperty = new Vector2[numParticles];
            densities = new float[numParticles];

            // Test spatial hash
            spatialLookup = new Entry[numParticles];
            startIndices = new int[numParticles];
            points = new Vector2[numParticles];

            myPartical = new Transform[numParticles];
            r = new SpriteRenderer[numParticles];

            int partPerRow = (int)math.sqrt(numParticles);
            int partPerCol = (numParticles - 1) / partPerRow + 1;
            float spacing = particleSize * 2 + particleSpacing;
            float d = particleSize * 2f;

            for (int i = 0; i < numParticles; i++)
            {
                myPartical[i] = Instantiate(pointPrefab, this.transform);
                myPartical[i].transform.localScale = new Vector2(d, d);

                r[i] = myPartical[i].GetComponent<SpriteRenderer>();
                r[i].color = skyBlue;

                float x = (i % partPerRow - partPerRow / 2f + 0.5f) * spacing;
                float y = (i / partPerRow - partPerCol / 2f + 0.5f) * spacing;
                position[i] = new Vector2(x, y);
                predictPos[i] = position[i];
                
                particleProperty[i] = Vector2.zero;
                densities[i] = 0f;

                // Test spatial hash
                spatialLookup[i].index = 0;
                spatialLookup[i].cellKey = 0;
                startIndices[i] = 0;
                points[i] = Vector2.zero;

            }
        }

        void Update()
        {
            SimStep(Time.deltaTime);
            DrawPatricles();
        }

        void OnDrawGizmos()
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireCube(Vector2.zero, boundSize);
        }

        void SimStep(float deltaTime)
        {
            Parallel.For(0, numParticles, i =>
            {
                velocity[i] += Vector2.down * gravity * deltaTime;
                predictPos[i] = position[i] + velocity[i] * deltaTime;
            });
            
            // USpatialLookup(predictPos, smoothRadius);
            
            Parallel.For(0, numParticles, i =>
            {
                // Using predictPos to calculate density
                densities[i] = CDensity(predictPos[i]);
            });
            
            Parallel.For(0, numParticles, i =>
            {
                Vector2 pressureForce = CPressureForce(i);
                Vector2 pressureAcc = pressureForce / densities[i];
                velocity[i] += pressureAcc * deltaTime; 
            });
            
            Parallel.For(0, numParticles, i =>
            {
                position[i] += velocity[i] * deltaTime;
                ResolveCollisions(ref position[i], ref velocity[i]); 
            });
        }

        void DrawPatricles()
        {
            for (int i = 0; i < numParticles; i++)
                myPartical[i].position = position[i];
        }

        void ResolveCollisions(ref Vector2 position, ref Vector2 velocity)
        {
            Vector2 halfBoundSize = boundSize / 2 - Vector2.one * particleSize;

            if (Abs(position.x) > halfBoundSize.x)
            {
                position.x = halfBoundSize.x * Sign(position.x);
                velocity.x *= -1 * collisionDamping;
            }

            if (Abs(position.y) > halfBoundSize.y)
            {
                position.y = halfBoundSize.y * Sign(position.y);
                velocity.y *= -1 * collisionDamping;
            }
        }

        static float SmoothingKernel(float radius, float dst)
        {
            if (dst > radius) return 0;
            float ConstVolume = (PI * Pow(radius, 4)) / 6;
            return (radius - dst) * (radius - dst) / ConstVolume;
        }

        static float SmoothingKernelDericatve(float radius, float dst)
        {
            if (dst > radius) return 0;
            float scale = 12 / (Pow(radius, 4) * PI);
            return (dst - radius) * scale;
        }

        float CDensity(Vector2 samplePoint)
        {
            float density = 0;
            // foreach (Vector2 pos in position)
            foreach (Vector2 pos in predictPos)
            {
                float dst = (pos - samplePoint).magnitude;
                float influence = SmoothingKernel(smoothRadius, dst);
                density += mass * influence;
            }
            return density;
        }

        Vector2 CPressureForce(int PIndex)
        {
            Vector2 PressureForce = Vector2.zero;
            for (int OIndex = 0; OIndex < numParticles; OIndex++)
            {
                if (PIndex == OIndex) continue;
                // using predictPos (implicit method)
                Vector2 offset = predictPos[OIndex] - predictPos[PIndex];

                float dst = offset.magnitude;
                Vector2 dir = dst == 0 ? GetRandomDir(PIndex) : offset.normalized; // / dst;
                float slope = SmoothingKernelDericatve(dst, smoothRadius);
                float density = densities[OIndex];
                float sharePressure = CSharePressure(density, densities[PIndex]);
                PressureForce += sharePressure * dir * slope * mass / density;
            }
            return PressureForce;
        }

        Vector2 GetRandomDir(int seed)
        {
            System.Random rng = new(seed);
            float angle = (float)rng.NextDouble() * 2.0f * PI;
            float x = Cos(angle);
            float y = Sin(angle);
            return new Vector2(x, y);
        }

        float Density2Pressure(float density)
        {
            float densityError = density - targetDensity;
            float pressure = densityError * pressureMultiplier;
            return pressure;
        }

        float CSharePressure(float DensityA, float DensityB)
        {
            float PA = Density2Pressure(DensityA);
            float PB = Density2Pressure(DensityB);
            return (PA + PB) / 2;
        }

        public void USpatialLookup(Vector2[] points, float radius)
        {
            this.points = points;
            this.radius = radius;

            // create Spatial Lookup
            Parallel.For(0, points.Length, i =>
            {
                (int cellX, int cellY) = Pos2CellCord(points[i], radius);
                uint cellKey = GetKeyFromHash(HashCell(cellX, cellY));
                // spatialLookup[i] = new SpatialHash.Entry(i, cellKey);
                spatialLookup[i] = new Entry(i, cellKey);
                startIndices[i] = int.MaxValue; // Reset start index
            });

            // sort
            Array.Sort(spatialLookup);

            // calculater start index
            Parallel.For(0, points.Length, i =>
            {
                uint key = spatialLookup[i].cellKey;
                uint keyPrev = i == 0 ? uint.MaxValue : spatialLookup[i - 1].cellKey;
                if (key != keyPrev) startIndices[key] = i;
            });
        }

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
            return hash % (uint)spatialLookup.Length;
        }

        public void ForeachPointInRadius(Vector2 samplPpoint)
        {
            (int centreX, int centreY) = Pos2CellCord(samplPpoint, radius);
            float sqrRadius = radius * radius;
    
            // test 
            (int, int)[] cellOffsets = new (int, int)[numParticles];
            
            foreach ((int offsetX, int offsetY) in cellOffsets)
            {
                uint key = GetKeyFromHash(HashCell(centreX + offsetX, centreY + offsetY));
                int cellStartIndex = startIndices[key];
                for (int i = cellStartIndex; i < spatialLookup.Length; i++)
                {
                    if (spatialLookup[i].cellKey == key) break;
                    int PIndex = spatialLookup[i].index;
                    float sqrDst = (points[PIndex] - samplPpoint).sqrMagnitude;
                    if (sqrDst <= sqrRadius)
                    {
                        // spatialLookup[i].cellKey = key;
                        // if in the smoothing radius, do something
                    }
                }
                
            }
        }
        
    }

}