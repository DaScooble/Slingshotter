using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class CharacterMovement : MonoBehaviour
{
    public enum JumpState
    {
        False = 0,
        Cooldown = 1,
        Cooling = 2,
        True = 3
    }
    [SerializeField] Rigidbody physicalBody;
    [SerializeField] Animator characterAnimator;
    [SerializeField] BoxCollider groundCollider;
    [SerializeField] LayerMask groundLayer;
    [SerializeField] float speed;
    [SerializeField] float jumpHeight;
    [SerializeField] float jumpWait;
    JumpState isJumping = JumpState.False;
    bool canJump = true;
    Vector3 inputs;
    bool isGrounded;
    bool prevGrounded;

    int jumpCount = 0;

    void Start()
    {
        physicalBody = GetComponent(typeof(Rigidbody)) as Rigidbody;
    }

    void Update()
    {
        inputs = Vector3.zero;
        inputs.x = Input.GetAxisRaw("Horizontal");
        inputs.y = Input.GetAxisRaw("Vertical");

        bool prevGrounded = isGrounded;
        UpdateGroundStatus();
        CheckJumpCooldown();
    }

    void FixedUpdate()
    {
        Vector3 movement = inputs.x * Vector3.right;
        HandleJump();

        UpdateAnimator(Mathf.Abs(inputs.x));

        Move(movement * Time.fixedDeltaTime);

        if (isJumping == JumpState.True)
            isJumping = JumpState.Cooldown;
    }

    void CheckJumpCooldown()
    {
        if (!prevGrounded && isGrounded && isJumping == JumpState.Cooldown)
        {
            isJumping = JumpState.Cooling;
            StartCoroutine(Run.Delayed(jumpWait, () =>
            {
                isJumping = JumpState.False;
                Debug.Log("Setting isJumping to False.");
            }));
        }
    }

    void UpdateGroundStatus()
    {
        Vector3 boxOffset = groundCollider.transform.position;
        Vector3 boxCenter = groundCollider.center;
        Vector3 boxHalfExtents = groundCollider.center + groundCollider.bounds.extents;
        Quaternion boxOrientation = Quaternion.identity;
        isGrounded = Physics.CheckBox(boxOffset + boxCenter, boxHalfExtents, boxOrientation, groundLayer, QueryTriggerInteraction.Ignore);
        Debug.DrawLine(boxOffset + boxCenter - boxHalfExtents, boxOffset + boxCenter + boxHalfExtents, Color.red, Time.deltaTime);
    }

    void HandleJump()
    {
        if (inputs.y > 0f && isGrounded && isJumping == JumpState.False)
        {
            isJumping = JumpState.True;
            // canJump = false;

            // Calculate the correct force for a given height.
            Vector3 jumpForce = Vector3.up * Mathf.Sqrt(jumpHeight * Physics.gravity.y * -2f);

            // Add the jump force as an impulse.
            physicalBody.AddForce(jumpForce, ForceMode.Impulse);
            Debug.Log("Applying " + jumpForce + " force as jump " + ++jumpCount + ". Velocity is now " + physicalBody.velocity + " at time " + Time.time);
        }
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

    void UpdateAnimator(float xMovement)
    {
        characterAnimator.SetFloat("Forward", xMovement, 0.1f, Time.deltaTime);
        characterAnimator.SetBool("IsGrounded", isGrounded);
        characterAnimator.SetBool("IsJumping", isJumping == JumpState.True);
    }
}
