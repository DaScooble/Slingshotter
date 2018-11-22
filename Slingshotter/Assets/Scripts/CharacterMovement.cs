using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class CharacterMovement : MonoBehaviour
{
    [SerializeField] Rigidbody physicalBody;
    [SerializeField] BoxCollider groundCollider;
    [SerializeField] LayerMask groundLayer;
    [SerializeField] float speed;
    [SerializeField] float jumpHeight;
    [SerializeField] float jumpWait;
    bool canJump = true;
    Vector3 inputs;
    bool isGrounded;

    void Start()
    {
        physicalBody = GetComponent(typeof(Rigidbody)) as Rigidbody;
    }

    void Update()
    {
        inputs = Vector3.zero;
        inputs.x = Input.GetAxisRaw("Horizontal");
        inputs.y = Input.GetAxisRaw("Vertical");

        bool prevIsGrounded = isGrounded;
        Vector3 boxOffset = groundCollider.transform.position;
        Vector3 boxCenter = groundCollider.center;
        Vector3 boxHalfExtents = groundCollider.center + groundCollider.bounds.extents;
        Quaternion boxOrientation = Quaternion.identity;
        isGrounded = Physics.CheckBox(boxOffset + boxCenter, boxHalfExtents, boxOrientation, groundLayer, QueryTriggerInteraction.Ignore);
        // isGrounded = Physics.CheckBox(groundCollider.center, groundCollider.center + (Vector3.right * groundCollider.height / 2f), Quaternion.identity, groundLayer, QueryTriggerInteraction.Ignore);
        // Debug.DrawLine(groundCollider.bounds.min, groundCollider.bounds.max, Color.red, Time.deltaTime);
        // isGrounded = Physics.CheckCapsule(groundCollider.bounds.min, groundCollider.bounds.max, groundCollider.radius, groundLayer, QueryTriggerInteraction.Ignore);

        Debug.DrawLine(boxOffset + boxCenter - boxHalfExtents, boxOffset + boxCenter + boxHalfExtents, Color.red, Time.deltaTime);

        if (!prevIsGrounded && isGrounded)
        {
            canJump = false;
            StartCoroutine(Run.Delayed(jumpWait, () => canJump = true));
        }

        if (inputs.y > 0f && isGrounded && canJump)
        {
            canJump = false;
            // Calculate the correct force for a given height.
            Vector3 jumpForce = Vector3.up * Mathf.Sqrt(jumpHeight * Physics.gravity.y * -2f);

            // Set the y velocity to be 0. Needed since y velocity may still be < 0.
            // physicalBody.velocity = new Vector3(physicalBody.velocity.x, 0f, physicalBody.velocity.z);

            // Add the jump force as an impulse.
            physicalBody.AddForce(jumpForce, ForceMode.Impulse);
            Debug.Log("Applying " + jumpForce + " force as jump. Velocity is now " + physicalBody.velocity);

            // Start the timer to be able to jump again.
            // StartCoroutine(Run.Delayed(jumpWait, () => canJump = true));
        }
    }

    void FixedUpdate()
    {
        Move(new Vector3(inputs.x, 0f, 0f) * Time.fixedDeltaTime);
        // physicalBody.MovePosition(physicalBody.position + new Vector3(inputs.x, 0f, 0f) * speed * Time.fixedDeltaTime);
    }

    public void Move(Vector3 movement)
    {
        physicalBody.MovePosition(physicalBody.position + movement * speed);
    }

    public void Move(Vector3 movement, float speed)
    {
        physicalBody.MovePosition(physicalBody.position + movement * speed);
    }

    public void Shove(Vector3 movement)
    {
        physicalBody.AddForce(movement, ForceMode.Impulse);
    }
}
