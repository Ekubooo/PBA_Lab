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
    public class Fluid2D_5 : MonoBehaviour
    {
        [SerializeField] Transform pointPrefab;
        Transform[] myPartical;
        SpriteRenderer[] r;

        [SerializeField] int numParticles;
        [SerializeField] Vector2 boundSize;
            
        [SerializeField][Range(0.01f, 0.10f)] 
        float particleSize;
        [SerializeField][Range(0.01f, 0.25f)] 
        float particleSpacing;
        [SerializeField][Range(0.005f, 0.25f)] 
        float smoothRadius;
        
        [SerializeField] float mass = 1;
        [SerializeField] float collisionDamping;
        [SerializeField] float gravity;

        [SerializeField] float targetDensity;
        [SerializeField] float pressureMultiplier;
        
        Vector2[] position;
        Vector2[] predictPos;
        Vector2[] velocity;
        Vector2[] particleProperty;
        float[] densities;
        
        float timeStep = 1f / 60f;
        
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
                
            }
        }

        void Update()
        {
            // SimStep(Time.deltaTime);
            SimStep();
            DrawPatricles();
        }

        void OnDrawGizmos()
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireCube(Vector2.zero, boundSize);
        }

        void SimStep()
        {
            Parallel.For(0, numParticles, i =>
            {
                velocity[i] += Vector2.down * gravity * timeStep;
                predictPos[i] = position[i] + velocity[i] * timeStep;
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
                velocity[i] += pressureAcc * timeStep; // = or +=
            });
            
            Parallel.For(0, numParticles, i =>
            {
                position[i] += velocity[i] * timeStep;
                ResolveCollisions(ref position[i], ref velocity[i]); // override
            });
        }

        void DrawPatricles()
        {
            for (int i = 0; i < numParticles; i++)
            {
                myPartical[i].position = position[i];
                // 10 can set as paramart of SerializeField
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

        Vector2 Interaction(Vector2 inputPos, float radius, float strength, int PIndex)
        {
            Vector2 IForce = Vector2.zero;
            Vector2 offset = inputPos - position[PIndex];
            float sqrDst = Vector2.Dot(offset, offset);

            if (sqrDst < radius * radius)
            {
                float dst = Sqrt(sqrDst);
                Vector2 dir2InputPoint = dst 
                    <= float.Epsilon ? Vector2.zero : offset / dst;
                float centreT = 1 - dst / radius;
                IForce += (dir2InputPoint * strength - velocity[PIndex]) * centreT;
                
            }
            return IForce;
        }
        
    }
}