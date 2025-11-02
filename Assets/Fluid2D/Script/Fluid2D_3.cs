using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Unity.Mathematics;
using UnityEngine.UI;

using static UnityEngine.Mathf;
using Random = UnityEngine.Random;

public class Fluid2D_3 : MonoBehaviour
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
    Vector2[] velocity;
    Vector2[] particleProperty;
    float[] densities;
    
    Color skyBlue = new Color(135f / 255f, 206f / 255f, 235f / 255f);
    void Start()
    {
        position = new Vector2[numParticles];
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
            particleProperty[i] = Vector2.zero;
            densities[i] = 0f;
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
            densities[i] = CDensity(position[i]);
        });
        Parallel.For(0, numParticles, i =>
        {
            Vector2 pressureForce = CPressureForce(i);
            Vector2 pressureAcc = pressureForce / densities[i];
            velocity[i] += pressureAcc * deltaTime;     // = or +=
        });
        Parallel.For(0, numParticles, i =>
        {
            position[i] += velocity[i] * deltaTime;
            ResolveCollisions(ref position[i], ref velocity[i]);    // override
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
        if (dst >= radius) return 0;
        
        float ConstVolume = (PI * Pow(radius, 4)) / 6;
        return (radius - dst) * (radius - dst) / ConstVolume;
    }
    
    static float SmoothingKernelDericatve(float radius, float dst)
    {
        if (dst >= radius) return 0;
        float scale = 12 / (PI * Pow(radius, 4));
        return (dst - radius) * scale;
    }

    float CDensity(Vector2 samplePoint)
    {
        float density = 0;
        // const float mass = 1;
        foreach (Vector2 pos in position)
        {
            float dst = (pos - samplePoint).magnitude;
            float influence = SmoothingKernel(smoothRadius, dst);
            density += mass * influence;
        }
        return density;
    }

    float calculateProperty(Vector2 samplePoint)
    {   // SPH core
        // it can be density()
        float property = 0;
        for (int i = 0; i < numParticles; i++)
        {
            float dst = (position[i] - samplePoint).magnitude;
            float influence = SmoothingKernel(dst, smoothRadius);
            float density = CDensity(position[i]);
        }
        return property;
    }
    
    Vector2 CPressureForce(int PIndex)
    {
        Vector2 PressureForce = Vector2.zero;
        for (int OIndex = 0; OIndex < numParticles; OIndex++)
        {
            if (PIndex == OIndex) continue;
            Vector2 offset = position[OIndex] -  position[PIndex];
            
            float dst = offset.magnitude;
            Vector2 dir = dst == 0 ? GetRandomDir() : offset / dst;
            // Vector2 refDir = velocity[PIndex] - velocity[OIndex];
            // Vector2 dir = dst == 0 ? refDir : offset / dst;
            float slope = SmoothingKernelDericatve(dst, smoothRadius);
            float density = densities[OIndex];
            float sharePressure = CSharePressure(density, densities[PIndex]);
            // PressureForce += Density2Pressure(density) * dir * slop * mass / density; 
            PressureForce += sharePressure * dir * slope * mass / density; 
        }
        return PressureForce;
    }

    Vector2 GetRandomDir()
    {
        float angleInRadians = Random.Range(0f, 2f * Mathf.PI);

        float x = Cos(angleInRadians);
        float y = Sin(angleInRadians);
        return new Vector2(x, y);
    }
    
    float Density2Pressure(float density)
    {
        float densityError = density - targetDensity;
        // verify and not good. not the error one. 
        // float densityError = targetDensity - density;
        float pressure = densityError * pressureMultiplier;
        return pressure;
    } 
    
    float CSharePressure(float DensityA, float DensityB)
    {
        float PA = Density2Pressure(DensityA);
        float PB = Density2Pressure(DensityB);
        return (PA + PB) / 2;
    }
    
}
