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
    public class Fluid2D_7 : MonoBehaviour
    {
        [SerializeField] Transform pointPrefab;
        Transform[] myPartical;
        SpriteRenderer[] r;

        [SerializeField] int numParticles;
        [SerializeField] Vector2 boundSize;
            
        [SerializeField][Range(0.01f, 0.25f)] 
        float particleSize;
        [SerializeField][Range(0.01f, 0.25f)] 
        float particleSpacing;
        [SerializeField][Range(0.005f, 1.25f)] 
        float smoothRadius;
        [SerializeField] [Range(0.125f, 1.25f)]
        private float viscosityStrength;
        [SerializeField] [Range(1f, 3f)]
        float IRadius = 2f;
        [SerializeField] [Range(5f, 15f)]
        float IStrength = 2f;
        
        [SerializeField] float mass = 1;
        [SerializeField] float collisionDamping;
        [SerializeField] float gravity;

        [SerializeField] float targetDensity;
        [SerializeField] float pressureMultiplier;
        [SerializeField] float NPMultiplier;

        
        Vector2[] position;
        Vector2[] predictPos;
        Vector2[] velocity;
        Vector2[] particleProperty;
        float[] densities;

        bool isMouseHitLeft;
        bool isMouseHitRight;

        // test for spatial hash //////////////////////////////////////////
        //public SpatialHash.Entry[] spatialLookup;
        (int, int)[] SHOffsets;
        Entry[]      spatialLookup;
        int[]        startIndices;
        // Vector2[]    points;        // ?
        // float        radius;
        float        timeStep = 1f / 90f;
        // end test ///////////////////////////////////////////////////////

        static Color skyBlue = new Color(135f / 255f, 206f / 255f, 235f / 255f);
        static Color Tomato = new Color(1f, 99f / 255f, 71f / 255f);

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
            // points = new Vector2[numParticles];     //?? 

            // particles property
            myPartical = new Transform[numParticles];
            r = new SpriteRenderer[numParticles];

            int partPerRow = (int)math.sqrt(numParticles);
            int partPerCol = (numParticles - 1) / partPerRow + 1;
            float spacing = Min(particleSize,0.05f) * 2 + particleSpacing;
            float d = particleSize * 2f;
            
            SHOffsets = new (int, int)[9]
            {
                (-1, 1),  (0, 1),  (1, 1), 
                (-1, 0),  (0, 0),  (1, 0),
                (-1, -1), (0, -1), (1, -1)
            };

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
                spatialLookup[i].cellKey = 0;
                spatialLookup[i].index = 0;
                startIndices[i] = 0;
                // points[i] = Vector2.zero;
                
            }
        }

        void Update()
        {
            SimStep();
            DrawPatricles();
        }

        void OnDrawGizmos()
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireCube(Vector2.zero, boundSize);
            // Gizmos.DrawSphere(Input.mousePosition, IRadius);
        }

        void SimStep()
        {
            Vector2 Fource = Vector2.down * gravity;
            Vector3 mousePos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            Vector2 MPos = new Vector2(mousePos.x, mousePos.y);
            // isMouseHitLeft = isMouseHitRight = false;
            isMouseHitLeft  = Input.GetMouseButton(0);
            isMouseHitRight = Input.GetMouseButton(1);
            
            Parallel.For(0, numParticles, i =>
            {
                // velocity[i] += Vector2.down * gravity * timeStep;
                if (isMouseHitRight)
                {
                    Fource += InteractionFource(MPos, IRadius, IStrength, i);
                }
                else if (isMouseHitLeft)
                {
                    Fource += InteractionFource(MPos, IRadius, -IStrength, i);
                }
                
                velocity[i] += Fource * timeStep;
                predictPos[i] = position[i] + velocity[i] * timeStep;
            });
            
            USpatialLookup(predictPos, smoothRadius);
            
            Parallel.For(0, numParticles, i =>
            {
                densities[i] = CDensityCB(predictPos[i]);
            });
            
            Parallel.For(0, numParticles, i =>
            {
                Vector2 pressureForce = CPressureForceCB(i);
                if (densities[i] > 0.00001f)
                {
                    Vector2 pressureAcc = pressureForce / densities[i];
                    velocity[i] += pressureAcc * timeStep;
                }
            });

            Parallel.For(0, numParticles, i =>
            {
                Vector2 viscosityForce = CViscosityForce(i);
                if (densities[i] > 0.00001f)
                {
                    Vector2 Acc = viscosityForce / densities[i];
                    velocity[i] += Acc * timeStep;
                }
            });
            
            Parallel.For(0, numParticles, i =>
            {
                position[i] += velocity[i] * timeStep;
                ResolveCollisions(ref position[i], ref velocity[i]); 
            });
        }

        void DrawPatricles()
        {
            for (int i = 0; i < numParticles; i++)
            {
                myPartical[i].position = position[i];
                float interpolatePara = InverseLerp(0.25f, 1.25f, velocity[i].magnitude);
                r[i].color = Color.Lerp(skyBlue, Tomato, interpolatePara);
            }
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

        static float SmoothingKernel(float dst, float radius)
        {
            if (dst >= radius) return 0;
            float ConstVolume = (PI * Pow(radius, 4)) / 6;
            return (radius - dst) * (radius - dst) / ConstVolume;
        }

        static float VisSmoothKernel(float radius, float dst)
        {
            if (dst >= radius) return 0;
        
            float ConstVolume = PI * Pow(radius, 8) / 4;
            float v = Max(0, radius * radius - dst * dst);
            return  v * v * v / ConstVolume;
        }

        static float SmoothingKernelDericatve(float dst, float radius)
        {
            if (dst >= radius) return 0;
            float scale = 12 / (Pow(radius, 4) * PI);
            return (dst - radius) * scale;
        }
        
        public void USpatialLookup(Vector2[] points, float radius)
        {
            // !! not right yet.
            // 

            // create Spatial Lookup
            Parallel.For(0, points.Length, i =>
            {
                (int cellX, int cellY) = Pos2CellCord(points[i], radius);
                uint cellKey = GetKeyFromHash(HashCell(cellX, cellY));
                spatialLookup[i] = new Entry(i, cellKey);
                startIndices[i] = int.MaxValue; // Reset start index
            });

            // sort by cellKey
            Array.Sort(spatialLookup);

            // calculater start index
            Parallel.For(0, points.Length, i =>
            {
                uint key = spatialLookup[i].cellKey;
                uint keyPrev = i == 0 ? uint.MaxValue : spatialLookup[i - 1].cellKey;
                if (key != keyPrev) startIndices[key] = i;
            });
        }
        
        public void getNeighbor(Vector2 samplPpoint, System.Action<int> callback)
        {   
            (int centreX, int centreY) = Pos2CellCord(samplPpoint, smoothRadius);
            float sqrRadius = smoothRadius * smoothRadius;
            
            foreach ((int offsetX, int offsetY) in SHOffsets)
            {
                // For all particles in 9 cells 
                uint key = GetKeyFromHash(HashCell(centreX + offsetX, centreY + offsetY));
                int cellStartIndex = startIndices[key];
                for (int i = cellStartIndex; i < spatialLookup.Length; i++)
                {   
                    // For all particles in a cell
                    if (spatialLookup[i].cellKey != key) break;    
                    // out of current cell, break
                        
                    int PIndex = spatialLookup[i].index;
                    float sqrDst = (predictPos[PIndex] - samplPpoint).sqrMagnitude;
                    
                    if (sqrDst <= sqrRadius) 
                        callback(PIndex);
                }
            }
        }
        
        float CDensityCB(Vector2 samplePoint)
        {
            float density = 0;
            getNeighbor(samplePoint, (int otherPIndex) =>
            {
                density += DInfluence(samplePoint, otherPIndex);
            });
            return density;
        }
        
        Vector2 CPressureForceCB(int PIndex)
        {
            Vector2 PressureForce = Vector2.zero;
            getNeighbor(predictPos[PIndex], (int OIndex) =>
            {
                PressureForce += PInfluence(PIndex, OIndex);
            });
        
            return PressureForce;
        }
        
        Vector2 CViscosityForce(int PIndex)
        {
            Vector2 viscosityForce = Vector2.zero;
            Vector2 pos = position[PIndex];
            // foreach (int OIndex in getNeighbor())
            // {
            //     float dst = (pos - position[OIndex]).magnitude;
            //     float influence = VisSmoothKernel(dst, smoothRadius);
            //     viscosityForce += (velocity[OIndex] - velocity[PIndex]) * influence;
            // }
            
            getNeighbor(pos, (int OIndex) =>
            {
                viscosityForce += VInfluence(PIndex, OIndex);
            });
            
            return viscosityForce * viscosityStrength;
        }
        
        Vector2 VInfluence(int PIndex, int OIndex)
        {
            float dst = (position[PIndex] - position[OIndex]).magnitude;
            float influence = VisSmoothKernel(dst, smoothRadius);
            return (velocity[OIndex] - velocity[PIndex]) * influence;
        }
        
        float DInfluence(Vector2 samplePoint, int neighborIndex)
        {
            float dst = (predictPos[neighborIndex] - samplePoint).magnitude;
            float influence = SmoothingKernel(dst, smoothRadius);
            return mass * influence;
        }

        Vector2 PInfluence(int PIndex, int OIndex)
        {
            if (PIndex == OIndex) return Vector2.zero;
            Vector2 offset = predictPos[OIndex] - predictPos[PIndex];
            float dst = offset.magnitude;
            Vector2 dir = dst == 0 ? GetRandomDir(PIndex) : offset.normalized; 
            
            float slope = SmoothingKernelDericatve(dst,smoothRadius);
        
            float density_O = densities[OIndex]; 
            float density_P = densities[PIndex]; 

            if (density_O == 0) return Vector2.zero;

            float sharePressure = CSharePressure(density_O, density_P);
            Vector2 PInf = dir * sharePressure * slope * mass / density_O;
        
            return PInf;
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
        
        (float, float) CD2NP(float density, float nearDensity)
        {
            float densityError = density - targetDensity;
            float pressure = densityError * pressureMultiplier;
            float nearPressure = nearDensity * NPMultiplier;
            return (pressure, nearPressure);
        }

        float CSharePressure(float DensityA, float DensityB)
        {
            float PA = Density2Pressure(DensityA);
            float PB = Density2Pressure(DensityB);
            return (PA + PB) / 2;
        }
        
        public (int x, int y) Pos2CellCord(Vector2 point, float radius)
        {
            // int cellX = (int)(point.x / radius);
            // int cellY = (int)(point.y / radius);
            int cellX = FloorToInt(point.x / radius);
            int cellY = FloorToInt(point.y / radius);
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
        
        Vector2 InteractionFource(Vector2 inputPos, float radius, float strength, int PIndex)
        {
            Vector2 IForce = Vector2.zero;
            Vector2 offset = inputPos - position[PIndex];
            float sqrDst = Vector2.Dot(offset, offset);

            if (sqrDst < radius * radius)
            {
                float dst = Sqrt(sqrDst);
                Vector2 dir2InputPoint 
                    = dst <= float.Epsilon ? Vector2.zero : offset / dst;
                float centreT = 1 - dst / radius;
                IForce += (dir2InputPoint * strength - velocity[PIndex]) * centreT;
                
            }
            return IForce;
        }

    }
}