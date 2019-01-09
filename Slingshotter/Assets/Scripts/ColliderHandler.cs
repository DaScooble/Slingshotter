using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public interface IColliderListener
{
    void CollisionEnter(Collision collision);
}

[RequireComponent(typeof(Collider))]
public class ColliderHandler : MonoBehaviour
{
    GameObject gameObj;
    public GameObject GameObject => gameObj;
    new SphereCollider collider;
    public SphereCollider Collider => collider;
    new private Rigidbody rigidbody;
    public Rigidbody RigidBody => rigidbody;
    IColliderListener listener;
    LayerMask collisionMask;
    List<Rigidbody> touching = new List<Rigidbody>();

    public ColliderHandler Initialize(LayerMask mask, IColliderListener listener = null)
    {
        collisionMask = mask;
        this.listener = listener;
        collider = GetComponent<SphereCollider>();
        rigidbody = GetComponent<Rigidbody>();
        gameObj = gameObject;
        return this;
    }

    public Rigidbody GetOne()
    {
        return (touching.Count > 0) ? touching[touching.Count - 1] : null;
    }

    public List<Rigidbody> GetAll()
    {
        return new List<Rigidbody>(touching);
    }

    void OnCollisionEnter(Collision collision)
    {
        GameObject other = collision.gameObject;
        Rigidbody otherBody = collision.rigidbody;
        if (collisionMask.Contains(other.layer) && otherBody != null)
        {
            touching.Add(otherBody);
        }

        Debug.Log("Collision! Arm velocity was " + RigidBody.velocity);
        if (listener != null)
            listener.CollisionEnter(collision);
    }

    void OnCollisionStay(Collision collision)
    {
        if (listener != null)
            listener.CollisionEnter(collision);
    }

    void OnCollisionExit(Collision collision)
    {
        GameObject other = collision.gameObject;
        Rigidbody otherBody = collision.rigidbody;
        if (collisionMask.Contains(other.layer) && otherBody != null)
        {
            touching.Remove(otherBody);
        }
    }
}
