using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Unity.Mathematics;
using UnityEngine.UI;
using static UnityEngine.Mathf;
using Random = System.Random;

public class Fluid2D_2 : MonoBehaviour
{
    // [SerializeField] GameObject go;
    // GameObject[] myCircle;
    
    [SerializeField] Transform pointPrefab;
    Transform[] myPartical;
    SpriteRenderer[] r;
    
    public float mass = 1;
    public float smoothRadius = 2;
    public float collisionDamping;
    public float particleSize;
    public float gravity;
    public int numParticles;
    public float particleSpacing;
    public Vector2 boundSize;

    public float targetDensity;
    public float pressureMultiplier;
    
    Vector2[] position;
    Vector2[] velocity;
    float[] densities;
    Vector2[] particleProperty;
    
    Color skyBlue = new Color(135f / 255f, 206f / 255f, 235f / 255f);
    void Start()
    {
        // point = Instantiate(pointPrefab);
        // point.localPosition = 2f * Vector2.right;
        
        // init
        position = new Vector2[numParticles];
        velocity = new Vector2[numParticles];
        particleProperty = new Vector2[numParticles];
        densities = new float[numParticles];
        
        // myCircle = new GameObject[numParticles];
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
        // for (int i = 0; i < position.Length; i++)
        // {
        //     velocity[i] += Vector2.down * gravity * Time.deltaTime;
        //     position[i] += velocity[i] * Time.deltaTime;
        //     ResolveCollisions(i);
        //     DrawCircle(position[i], particleSize, skyBlue, i);   
        // }
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
            velocity[i] = Vector2.down * gravity * deltaTime;
            densities[i] = CDensity(position[i]);
        });
        Parallel.For(0, numParticles, i =>
        {
            Vector2 pressureForce = CPressureForce2(i);
            Vector2 pressureAcc = pressureForce / densities[i];
            velocity[i] += pressureAcc * deltaTime;     // = or +=
        });
        Parallel.For(0, numParticles, i =>
        {
            position[i] += velocity[i] * deltaTime;
            ResolveCollisions(ref position[i], ref velocity[i]);    // override
            // mark if thread call not allow
            // myPartical[i].position = position[i];
        });
    }
    
    void DrawCircle(Vector2 pos, float radius, Color color, int i)
    {
        float d = radius * 2f;
        // myCircle[i].transform.position = pos;
        // myCircle[i].transform.localScale = new Vector2(d, d);
        myPartical[i].transform.position = pos;
        myPartical[i].transform.localScale = new Vector2(d, d);
        r[i].color = color;
    }

    void DrawPatricles()
    {
        for (int i = 0; i < numParticles; i++)
            myPartical[i].position = position[i];
    }
    
    void ResolveCollisions(int i)
    {
        Vector2 halfBoundSize = boundSize / 2 - Vector2.one * particleSize;
        Vector2 curPos = position[i];
        Vector2 curVel = velocity[i];
        if (math.abs(curPos.x) > halfBoundSize.x)
        {
            // position[i].x = halfBoundSize.x * math.sign(curPos.x);
            // velocity[i].x *= -1 * collisionDamping;
            curPos.x = halfBoundSize.x * Sign(curPos.x);
            curVel.x *= -1 * collisionDamping;
        }

        if (math.abs(curPos.y) > halfBoundSize.y)
        {
            // position[i].y = halfBoundSize.y * math.sign(curPos.y);
            // velocity[i].y *= -1 * collisionDamping;
            curPos.y = halfBoundSize.y * Sign(curPos.y);
            curVel.y *= -1 * collisionDamping;
        }
        position[i] = curPos;
        velocity[i] = curVel;
    } 
    
    void ResolveCollisions(ref Vector2 position, ref Vector2 velocity)
    {
        Vector2 halfBoundSize = boundSize / 2 - Vector2.one * particleSize;

        if (math.abs(position.x) > halfBoundSize.x)
        {
            // position[i].x = halfBoundSize.x * math.sign(curPos.x);
            // velocity[i].x *= -1 * collisionDamping;
            position.x = halfBoundSize.x * Sign(position.x);
            velocity.x *= -1 * collisionDamping;
        }

        if (math.abs(position.y) > halfBoundSize.y)
        {
            // position[i].y = halfBoundSize.y * math.sign(curPos.y);
            // velocity[i].y *= -1 * collisionDamping;
            position.y = halfBoundSize.y * Sign(position.y);
            velocity.y *= -1 * collisionDamping;
        }
    }

    static float SmoothingKernel(float radius, float dst)
    {
        if (dst >= radius) return 0;
        
        float ConstVolume = PI * Pow(radius, 8) / 4;
        float v = Max(0, radius * radius - dst * dst);
        return v * v * v / ConstVolume;
        // return (radius - dst) * (radius - dst) / ConstVolume;
    }
    
    static float SmoothingKernelDericatve(float radius, float dst)
    {
        if (dst >= radius) return 0;
        float f = radius * radius - dst * dst;
        float scale = -24 / (PI * Pow(radius, 8));
        return scale * dst * f * f;
        // return (dst - radius) * scale;
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
            // property += particleProperty[i] * influence * mass / density;
        }
        return property;
    }

    Vector2 PGradient(Vector2 samplePoint)
    {   // slow one, discard.
        const float stepSize = 0.001f;
        float deltaX = - calculateProperty(samplePoint)
            + calculateProperty(samplePoint + Vector2.right * stepSize);
        float deltaY = -calculateProperty(samplePoint)
            + calculateProperty(samplePoint + Vector2.up * stepSize);
        
        Vector2 gradient = new Vector2(deltaX, deltaY) / stepSize;
        return gradient;
    }

    void UpdateDensities()
    {
        Parallel.For(0, numParticles, i =>
        {
            densities[i] = CDensity(position[i]);
        });
    }
    
    Vector2 CPGradient(Vector2 samplePoint)
    {
        // discard, using pressure versiuon
        Vector2 popGradient = Vector2.zero;
        for (int i = 0; i < numParticles; i++)
        {
            float dst = (position[i] - samplePoint).magnitude;
            Vector2 dir = (position[i] - samplePoint) / dst;
            float slop = SmoothingKernelDericatve(dst, smoothRadius);
            float density = densities[i];
            
            // ??
            popGradient += -particleProperty[i] * dir * slop * mass / density; 
        }
        return popGradient;
    }

    Vector2 CPressureForce(Vector2 samplePoint)
    {   // discard
        Vector2 pressureForce = Vector2.zero;

        for (int i = 0; i < numParticles; i++)
        {
            float dst =  (position[i] - samplePoint).magnitude;
            Vector2 dir = (position[i] - samplePoint) / dst;
            float slope = SmoothingKernelDericatve(dst, smoothRadius);
            float density = densities[i];
            pressureForce 
                += -Density2Pressure(density) * dir * slope * mass / density;
        }
        return pressureForce;
    }
    
    Vector2 CPressureForce2(int PIndex)
    {
        Vector2 PressureForce = Vector2.zero;
        for (int OIndex = 0; OIndex < numParticles; OIndex++)
        {
            if (PIndex == OIndex) continue;
            Vector2 offset = position[OIndex] -  position[PIndex];
            
            float dst = offset.magnitude;
            // Vector2 dir = dst == 0 ? GetRandomDir() : offset / dst;
            Vector2 refDir = velocity[OIndex] - velocity[PIndex];
            Vector2 dir = dst == 0 ? refDir : offset / dst;
            float slop = SmoothingKernelDericatve(dst, smoothRadius);
            float density = densities[OIndex];
            // float sharePressure = CSharePressure(density, densities[PIndex]);
            PressureForce 
                += -Density2Pressure(density) * dir * slop * mass / density; 
        }
        return PressureForce;
    }

    Vector2 GetRandomDir()
    {
        float angleInRadians = UnityEngine.Random.Range(0f, 2f * Mathf.PI);

        float x = Mathf.Cos(angleInRadians);
        float y = Mathf.Sin(angleInRadians);

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
    
}
