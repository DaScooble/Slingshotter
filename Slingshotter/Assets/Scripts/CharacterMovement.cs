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

    [Header("Physics Settings")]
    [SerializeField] Rigidbody physicalBody;
    [SerializeField] BoxCollider groundCollider;
    [SerializeField] LayerMask groundLayer;

    [Header("Animation Settings")]
    [SerializeField] Animator characterAnimator;
    [SerializeField] Transform armController;
    [SerializeField] float armStretchDistance;
    [SerializeField] float armFollowSpeed;
    Transform armGoal;
    GameObject armColliderObject;
    Rigidbody armRB;
    SpringJoint armSpring;
    [SerializeField] GameObject armColliderPrefab;
    [SerializeField] GameObject goalDebugPrefab;
    [Header("Movement Settings")]
    [SerializeField] float speed;
    [SerializeField] float turnSpeed;
    [SerializeField] float jumpHeight;
    [SerializeField] float jumpWait;
    JumpState isJumping = JumpState.False;
    bool canJump = true;
    Vector3 inputs;
    float turnDir = 1f;
    float turnProg;
    const float turnOffset = 90f;
    const float turnFrom = 0f;
    const float turnTo = 180f;

    bool isGrounded;
    bool prevGrounded;

    /* Debug stuff */
    int jumpCount = 0;
    float currArmRotation = 0f;
    float armRotationSpeed = 30f;

    void Start()
    {
        physicalBody = GetComponent(typeof(Rigidbody)) as Rigidbody;
        armGoal = GameObject.Instantiate(goalDebugPrefab).transform;
        armColliderObject = GameObject.Instantiate(armColliderPrefab);
        armRB = armColliderObject.GetComponent<Rigidbody>();
        armSpring = armColliderObject.GetComponent<SpringJoint>();
        armSpring.connectedBody = physicalBody;
    }

    void Update()
    {
        inputs = Vector3.zero;
        inputs.x = Input.GetAxisRaw("Horizontal");
        inputs.y = Input.GetAxisRaw("Vertical");

        Vector3 mousePos = Input.mousePosition;
        mousePos.z = 10;
        Vector3 mouseWorldPos = Camera.main.ScreenToWorldPoint(mousePos);

        if (armGoal)
        {
            armGoal.position = Vector3.Lerp(armGoal.position, mouseWorldPos, 0.25f);
        }

        prevGrounded = isGrounded;
        UpdateGroundStatus();
        CheckJumpCooldown();

        UpdateCharacterDirection();
    }

    void FixedUpdate()
    {
        Vector3 movement = inputs.x * Vector3.right;
        HandleJump();

        UpdateAnimator(Mathf.Abs(inputs.x));

        Move(movement * Time.fixedDeltaTime);
        HandleArm();

        if (isJumping == JumpState.True)
            isJumping = JumpState.Cooldown;
    }

    void LateUpdate()
    {
        if (armRB != null)
        {
            armController.position = armRB.position;
        }
    }

    void OnAnimatorIK()
    {
        // Debug.Log("Executing OnAnimatorIK!");
        if (characterAnimator != null)
        {
            if (armGoal != null)
            {
                // characterAnimator.SetLookAtWeight(1);
                // characterAnimator.SetLookAtPosition(armGoal.position);

                AvatarIKGoal targetIKGoal = AvatarIKGoal.RightHand;

                // Debug.Log("Setting IKPosition for " + targetIKGoal + " to " + armGoal.position);
                characterAnimator.SetIKPositionWeight(targetIKGoal, 1);
                characterAnimator.SetIKRotationWeight(targetIKGoal, 1);
                characterAnimator.SetIKPosition(targetIKGoal, armGoal.position);
                characterAnimator.SetIKRotation(targetIKGoal, armGoal.rotation);
            }
        }
    }

    void CheckJumpCooldown()
    {
        if (!prevGrounded && isGrounded && isJumping != JumpState.Cooling)
        {
            Debug.Log("Running jump cooldown, " + prevGrounded + ", " + isGrounded + ", " + isJumping.ToString());
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

    void HandleArm()
    {
        if (armRB != null && armSpring != null)
        {
            float mouseDistance = Vector3.Distance(armGoal.position, physicalBody.position);

            Vector3 direction = (mouseDistance < armStretchDistance) ? (armGoal.position - physicalBody.position)
                                                                     : (armGoal.position - physicalBody.position).normalized * armStretchDistance;

            Vector3 rbGoal = physicalBody.position + direction;
            Vector3 rbDirection = (rbGoal - armRB.position);

            armRB.MovePosition(armRB.position + rbDirection * armFollowSpeed * Time.deltaTime);

            float armDistance = Vector3.Distance(armRB.position, physicalBody.position);
            armSpring.minDistance = armDistance;
            armSpring.maxDistance = armDistance;

            Debug.Log("Moving armRB to " + armRB.position);
        }
    }

    void UpdateCharacterDirection()
    {
        float turnAmount = inputs.x;
        if (turnAmount > 0)
        {
            turnDir = -turnSpeed;
        }
        else if (turnAmount < 0)
        {
            turnDir = turnSpeed;
        }

        turnProg += Time.deltaTime * turnDir;
        turnProg = Mathf.Clamp(turnProg, 0f, 1f);

        float newTurn = Mathf.Lerp(turnFrom, turnTo, turnProg);
        physicalBody.rotation = Quaternion.Euler(0, newTurn + turnOffset, 0);
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
