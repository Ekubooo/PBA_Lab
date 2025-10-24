using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

using static UnityEngine.Mathf;

public class PreFluid : MonoBehaviour
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
    
    Vector2[] position;
    Vector2[] velocity;
    private Vector2[] particleProperty;
    
    Color skyBlue = new Color(135f / 255f, 206f / 255f, 235f / 255f);
    void Start()
    {
        // point = Instantiate(pointPrefab);
        // point.localPosition = 2f * Vector2.right;
        
        // init
        position = new Vector2[numParticles];
        velocity = new Vector2[numParticles];
        particleProperty = new Vector2[numParticles];
        
        // myCircle = new GameObject[numParticles];
        myPartical = new Transform[numParticles];
        r = new SpriteRenderer[numParticles];
        
        int partPerRow = (int)math.sqrt(numParticles);
        int partPerCol = (numParticles - 1) / partPerRow + 1;
        float spacing = particleSize * 2 + particleSpacing;

        for (int i = 0; i < numParticles; i++)
        {
            myPartical[i] = Instantiate(pointPrefab, this.transform);
            r[i] = myPartical[i].GetComponent<SpriteRenderer>();
            
            float x = (i % partPerRow - partPerRow / 2f + 0.5f) * spacing;
            float y = (i / partPerRow - partPerCol / 2f + 0.5f) * spacing;
            position[i] = new Vector2(x, y);
            particleProperty[i] = Vector2.zero;
        }
    }
    
    void Update()
    {
        for (int i = 0; i < position.Length; i++)
        {
            velocity[i] += Vector2.down * gravity * Time.deltaTime;
            position[i] += velocity[i] * Time.deltaTime;
            ResolveCollisions(i);
            DrawCircle(position[i], particleSize, skyBlue, i);   
        }
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawWireCube(Vector2.zero, boundSize);
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

    static float SmoothingKernel(float radius, float dst)
    {
        float ConstVolume = PI * Pow(radius, 8) / 4;
        float bas = Max(0, radius * radius - dst * dst);    
        return bas * bas * bas / ConstVolume;
    }
    
    static float SmoothingKernelDericatve(float radius, float dst)
    {
        if (dst >= radius) return 0;
        float f = radius * radius -  dst * dst;
        float scale = -24 / (PI * Pow(radius, 8));
        return scale * dst * f * f;
    }

    float Density(Vector2 samplePoint)
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
            float dst =(position[i] - samplePoint).magnitude;
            float influence = SmoothingKernel(dst, smoothRadius);
            float density = Density(position[i]);
            // property += particleProperty[i] * influence * mass / density;
        }
        return property;
    }

    Vector2 PGradient(Vector2 samplePoint)
    {
        const float stepSize = 0.001f;
        float deltaX = - calculateProperty(samplePoint)
            + calculateProperty(samplePoint + Vector2.right * stepSize);
        float deltaY = -calculateProperty(samplePoint)
            + calculateProperty(samplePoint + Vector2.up * stepSize);
        
        Vector2 gradient = new Vector2(deltaX, deltaY) /  stepSize;
        return gradient;
    }

    Vector2 CPGradient(Vector2 samplePoint)
    {
        Vector2 popGradient = Vector2.zero;
        for (int i = 0; i < numParticles; i++)
        {
            float dst = (position[i] - samplePoint).magnitude;
            Vector2 dir = (position[i] - samplePoint) / dst;
            float slop = SmoothingKernelDericatve(dst, smoothRadius);
            float density = Density(position[i]);
            popGradient += - particleProperty[i] * dir * slop * mass / density; 
        }
        return popGradient;
    }
    
}
