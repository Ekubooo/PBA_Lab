using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public class PreFluid : MonoBehaviour
{
    // [SerializeField] Transform pointPrefab;
    // Transform point;
    [SerializeField] GameObject go;
    GameObject myCircle;
    SpriteRenderer r;
    
    public float collisionDamping;
    public float particleSize;
    public float gravity;
    public Vector2 boundSize;
    
    Vector2 position = Vector2.zero;
    Vector2 velocity = Vector2.zero;
    void Start()
    {
        // point = Instantiate(pointPrefab);
        // point.localPosition = 2f * Vector2.right;
        myCircle = Instantiate(go);
        r = myCircle.GetComponent<SpriteRenderer>();
    }
    
    void Update()
    {
        velocity += Vector2.down * gravity * Time.deltaTime;
        position += velocity * Time.deltaTime;
        ResolveCollisions();
        DrawCircle(position, particleSize, new Color(135f/255f,206f/255f,235f/255f));
    }

    void DrawCircle(Vector2 pos, float radius, Color color)
    {
        myCircle.transform.position = pos;
        myCircle.transform.localScale = new Vector2(radius, radius);
        r.color = color;
    }

    void ResolveCollisions()
    {
        Vector2 halfBoundSize = boundSize / 2 - Vector2.one * particleSize;
        if (math.abs(position.x) > halfBoundSize.x)
        {
            position.x = halfBoundSize.x * math.sign(position.x);
            velocity.x *= -1 * collisionDamping;
        }

        if (math.abs(position.y) > halfBoundSize.y)
        {
            position.y = halfBoundSize.y * math.sign(position.y);
            velocity.y *= -1 * collisionDamping;
        }
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawWireCube(Vector2.zero, boundSize);
    }
}
