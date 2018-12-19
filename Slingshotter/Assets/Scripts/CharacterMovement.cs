using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class CharacterMovement : MonoBehaviour, IColliderListener
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
    [Header("Arm Settings")]
    [SerializeField] Transform armController;
    [SerializeField] float armStretchDistance;
    [SerializeField] float armFollowSpeed;
    [SerializeField] LayerMask grabbableLayer;
    [Header("Grab Settings")]
    [SerializeField] float maxArmStretch;
    [SerializeField] float maxArmTension;
    [SerializeField] float armReleaseForceMultiplier;
    [SerializeField] float armStrength;
    [SerializeField] float armDampening;

    Transform armGoal;
    ColliderHandler armColliderHandler;
    Rigidbody armRB;
    Rigidbody grabbed;
    Vector3 currentTension;
    // SpringJoint armJoint;
    [SerializeField] GameObject armColliderPrefab;
    [SerializeField] GameObject goalDebugPrefab;

    [Header("Launch Settings")]
    [SerializeField] float launchArmVelocityThreshold;
    [SerializeField] float launchMinForce;
    [SerializeField] float launchMaxForce;

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
    int armLaunchCount = 0;

    void Start()
    {
        physicalBody = GetComponent(typeof(Rigidbody)) as Rigidbody;
        armGoal = GameObject.Instantiate(goalDebugPrefab).transform;
        armColliderHandler = GameObject.Instantiate(armColliderPrefab).GetComponent<ColliderHandler>().Initialize(grabbableLayer, this);
        armRB = armColliderHandler.RigidBody;
    }

    void Update()
    {
        HandleInput();

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

        // Only be able to control the character while grounded?
        // if (isGrounded)
        Move(movement * Time.fixedDeltaTime);
        HandleArm();
        HandleGrab();

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

    void HandleInput()
    {
        inputs = Vector3.zero;
        inputs.x = Input.GetAxisRaw("Horizontal");
        inputs.y = Input.GetAxisRaw("Vertical");

        Vector3 mousePos = Input.mousePosition;
        mousePos.z = 10;
        Vector3 mouseWorldPos = Camera.main.ScreenToWorldPoint(mousePos);
        mouseWorldPos.z = 0;
        if (armGoal)
            armGoal.position = Vector3.Lerp(armGoal.position, mouseWorldPos, 0.25f);

        // If left clicking,
        //  Check if arm is colliding with a proper grab target.
        //  If true, lock the arm to that position on the target.
        //      (Best method? Save point on target's collider and just keep track of it and move the arm as needed?)
        //      (Or maybe add a fixedJoint component to the arm and just stop moving it towards the mouse?)
        if (Input.GetMouseButton(0))
        {
            // TODO: Add a fixed joint to the armCollider gameObject. This allows it to pull or be pulled depending on physics.
            // TODO: Set the "grabbed" object to the grabbable object.
            if (grabbed == null)
            {
                Rigidbody grabbable = armColliderHandler.GetOne();
                if (grabbable != null)
                {
                    grabbed = grabbable;
                    Debug.Log("Grab grabbable object!");
                    FixedJoint joint = armColliderHandler.GameObject.AddComponent<FixedJoint>();
                    joint.connectedBody = grabbed;
                }
            }
        }
        else
        {
            // TODO: Remove fixed joint from armCollider if there is one.
            // TODO: Set "grabbed" object to null.
            if (grabbed != null)
            {
                Destroy(armColliderHandler.GetComponent<FixedJoint>());
                grabbed = null;
            }
        }
    }

    // TODO: Add HandleGrab method
    // Use this for spring behaviour.
    // https://forum.unity.com/threads/spring-simulation.245592/

    // Formula is  f = -kx - bv  where

    // f = overall force of the spring
    // k = direction of the force (which way to send the object)
    // x = strength of the force (displacement, how far the spring is stretched)
    // b = dampening amount (dampening constant)
    // v = velocity (current velocity of the object)

    // TODO: Edit to "build up" tension and release most of it when releasing the object.
    // Also, calculate manually with masses. Gives more control over the scaling of it. Doesn't have to be linear that way.

    void HandleGrab()
    {
        if (grabbed != null)
        {
            Vector3 bodyPos = physicalBody.position;
            Vector3 armPos = armColliderHandler.RigidBody.position;

            float force = Vector3.Distance(bodyPos, armPos) * armStrength;
            Vector3 direction = (bodyPos - armPos).normalized;
            Vector3 velocity = armColliderHandler.RigidBody.velocity;

            physicalBody.AddForce(-force * direction - velocity * armDampening);
            armColliderHandler.RigidBody.AddForce(force * direction + velocity * armDampening);


        }
    }

    void CheckJumpCooldown()
    {
        if (!prevGrounded && isGrounded && isJumping != JumpState.Cooling)
        {
            // Debug.Log("Running jump cooldown, " + prevGrounded + ", " + isGrounded + ", " + isJumping.ToString());
            isJumping = JumpState.Cooling;
            StartCoroutine(Run.Delayed(jumpWait, () =>
            {
                isJumping = JumpState.False;
                // Debug.Log("Setting isJumping to False.");
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
            // Debug.Log("Applying " + jumpForce + " force as jump " + ++jumpCount + ". Velocity is now " + physicalBody.velocity + " at time " + Time.time);
        }
    }

    void HandleArm()
    {
        if (armRB != null)
        {
            float mouseDistance = Vector3.Distance(armGoal.position, physicalBody.position);

            Vector3 direction = (mouseDistance < armStretchDistance) ? (armGoal.position - physicalBody.position)
                                                                     : (armGoal.position - physicalBody.position).normalized * armStretchDistance;

            Vector3 rbGoal = physicalBody.position + direction;
            Vector3 rbDirection = (rbGoal - armRB.position);
            // rbDirection = (rbDirection.magnitude < 0.1f) ? Vector3.zero : rbDirection;

            armRB.velocity = rbDirection * armFollowSpeed;
            // armRB.AddForce(rbDirection * armFollowSpeed * Time.deltaTime, ForceMode.VelocityChange);
            // armRB.MovePosition(armRB.position + rbDirection * armFollowSpeed * Time.deltaTime);


            float armDistance = Vector3.Distance(armRB.position, physicalBody.position);
            float armLimit = (armDistance < armStretchDistance) ? armDistance : armStretchDistance;

            // Debug.Log("armDistance is " + armDistance);
            // armJoint.minDistance = armLimit;
            // armJoint.maxDistance = armLimit;

            // Debug.Log("Moving armRB to " + armRB.position);
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

    public void Move_Force(Vector3 movement)
    {
        physicalBody.AddForce(movement * speed, ForceMode.Impulse);
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

    public void CollisionEnter(Collision collision)
    {
        // When the arm collides with something, check the velocity.
        // If the velocity magnitude is greater than the threshold, launch the player in the opposite direction.
        // Vector3 armVelocity = armColliderHandler.RigidBody.velocity;
        // float armVelocityMag = armVelocity.magnitude;
        float armVelocityMag = collision.relativeVelocity.magnitude;
        Debug.Log("Checking collision with relative velocity of " + collision.relativeVelocity + " and magnitude of " + collision.relativeVelocity.magnitude + " against threshold of " + launchArmVelocityThreshold);
        Debug.Log("Arm's velocity was " + armColliderHandler.RigidBody.velocity + " with magnitude of " + armColliderHandler.RigidBody.velocity.magnitude);
        if (armVelocityMag > launchArmVelocityThreshold)
        {
            // Vector3 direction = armVelocity.normalized;
            // Vector3 direction = collision.impulse.normalized;
            Vector3 direction = collision.relativeVelocity.normalized;
            Vector3 force = (armVelocityMag > launchMinForce) ? direction * armVelocityMag : direction * launchMinForce;
            force = Vector3.ClampMagnitude(force, launchMaxForce);
            armColliderHandler.RigidBody.velocity = Vector3.zero;

            physicalBody.AddForce(force, ForceMode.Impulse);
            if (collision.rigidbody)
                collision.rigidbody.AddForceAtPosition(-force, collision.GetContact(0).point, ForceMode.Impulse);
            Debug.Log("Added force in direction of " + (direction) + " resulting in " + (direction * launchMaxForce));
            Debug.Log("Count: " + ++armLaunchCount);
        }
    }
}
