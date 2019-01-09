using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class PhysicalObject : MonoBehaviour
{
    [SerializeField] float density;
    [SerializeField] bool useDensity;
    Rigidbody rb;
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        UpdateMass();
    }

    public void UpdateMass()
    {
        if (rb == null)
            return;

        if (useDensity)
        {
            rb.SetDensity(density);
            rb.mass = rb.mass;
            // Debug.Log("Set mass to " + rb.mass);
        }
    }
}
