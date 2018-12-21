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

    public enum ArmState
    {
        Default = 0,
        Launching = 1,
        Grabbing = 2,
        Cooling = 3
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
    [SerializeField] float maxArmStretchDistance;
    [SerializeField] float armFollowSpeed;
    [SerializeField] float armLaunchDistance;
    [SerializeField] float armLaunchSpeed;
    [SerializeField] float armLaunchCooldown;
    [SerializeField] float armLaunchTimeout;
    [SerializeField] LayerMask grabbableLayer;
    ArmState armState = ArmState.Default;
    // Relative position to physicalBody.
    Vector3 currentLaunchTarget = Vector3.zero;

    [Header("Grab Settings")]
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
    int armCollisionCount = 0;

    void Start()
    {
        physicalBody = GetComponent(typeof(Rigidbody)) as Rigidbody;
        armGoal = GameObject.Instantiate(goalDebugPrefab).transform;
        armColliderHandler = GameObject.Instantiate(armColliderPrefab).GetComponent<ColliderHandler>().Initialize(grabbableLayer, this);
        armColliderHandler.RigidBody.position = physicalBody.position;
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
        if (grabbed != null)
            return;

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

        // If left mouse button is clicked and the arm is in the default state.
        // 
        if (Input.GetMouseButton(0) && armState == ArmState.Default)
        {
            armState = ArmState.Launching;
            // If after a certain amount of time, the arm has not reached its target, it will automatically go on cooldown.
            StartCoroutine(Run.Delayed(armLaunchTimeout, () =>
            {
                if (armState == ArmState.Launching)
                {
                    StartArmLaunchCooldown();
                }
            }));
        }
        if (Input.GetMouseButton(1))
        {
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
            if (grabbed != null)
            {
                Destroy(armColliderHandler.GetComponent<FixedJoint>());
                grabbed = null;
            }
        }
    }

    void CheckJumpCooldown()
    {
        if (!prevGrounded && isGrounded && isJumping != JumpState.Cooling)
        {
            isJumping = JumpState.Cooling;
            StartCoroutine(Run.Delayed(jumpWait, () => { isJumping = JumpState.False; }));
        }
    }

    void StartArmLaunchCooldown()
    {
        armState = ArmState.Cooling;
        StartCoroutine(Run.Delayed(armLaunchCooldown, () => { armState = ArmState.Default; currentLaunchTarget = Vector3.zero; }));
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

    // TODO: Change to active "click to launch arm". Arm should point in direction of mouse but not extend until the "launch arm" action is executed (left click).
    //       Should then grab the first grabbable object it collides with. If it collides with a non-grabbable object instead, it should launch the player character in the opposite direction.
    //       If the arm does not collide with anything or after it collides with a non-grabbable object, it should then retract to its original length.
    void HandleArm()
    {
        if (grabbed != null || armRB == null)
            return;

        if (armState == ArmState.Default || armState == ArmState.Cooling)
        {
            float mouseDistance = Vector3.Distance(armGoal.position, physicalBody.position);

            Vector3 direction = (mouseDistance < armStretchDistance) ? (armGoal.position - physicalBody.position)
                                                                     : (armGoal.position - physicalBody.position).normalized * armStretchDistance;

            Vector3 rbGoal = physicalBody.position + direction;
            // Vector3 rbAdjustedGoal = Vector3.MoveTowards(armRB.position, rbGoal, armFollowSpeed);
            // Vector3 rbDirection = (rbAdjustedGoal - armRB.position).normalized;

            Vector3 rbDirection = (rbGoal - armRB.position);

            armRB.velocity = rbDirection * armFollowSpeed;
            // armRB.AddForce(rbDirection * armFollowSpeed * Time.deltaTime, ForceMode.VelocityChange);
            // armRB.MovePosition(armRB.position + rbDirection * armFollowSpeed * Time.deltaTime);
        }
        else if (armState == ArmState.Launching)
        {
            if (currentLaunchTarget == Vector3.zero)
            {
                float mouseDistance = Vector3.Distance(armGoal.position, physicalBody.position);
                currentLaunchTarget = (armGoal.position - physicalBody.position).normalized * armLaunchDistance;
            }

            Vector3 rbGoal = physicalBody.position + currentLaunchTarget;
            Vector3 rbDirection = (rbGoal - armRB.position);

            armRB.velocity = rbDirection * armLaunchSpeed;

            float armDistance = Vector3.Distance(armRB.position, rbGoal);
            if (armDistance <= 0.05f)
            {
                armState = ArmState.Cooling;
                StartCoroutine(Run.Delayed(armLaunchCooldown, () => { armState = ArmState.Default; currentLaunchTarget = Vector3.zero; }));
            }
        }

        // if (armRB != null)
        // {
        //     float mouseDistance = Vector3.Distance(armGoal.position, physicalBody.position);

        //     Vector3 direction = (mouseDistance < armStretchDistance) ? (armGoal.position - physicalBody.position)
        //                                                              : (armGoal.position - physicalBody.position).normalized * armStretchDistance;

        //     Vector3 rbGoal = physicalBody.position + direction;
        //     Vector3 rbDirection = (rbGoal - armRB.position);
        //     // rbDirection = (rbDirection.magnitude < 0.1f) ? Vector3.zero : rbDirection;

        //     armRB.velocity = rbDirection * armFollowSpeed;
        //     // armRB.AddForce(rbDirection * armFollowSpeed * Time.deltaTime, ForceMode.VelocityChange);
        //     // armRB.MovePosition(armRB.position + rbDirection * armFollowSpeed * Time.deltaTime);

        //     float armDistance = Vector3.Distance(armRB.position, physicalBody.position);
        //     float armLimit = (armDistance < armStretchDistance) ? armDistance : armStretchDistance;
        // }
    }

    // Use this for spring behaviour.
    // https://forum.unity.com/threads/spring-simulation.245592/
    // https://gafferongames.com/post/spring_physics/

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
        // Between 0 and armStretchDistance, apply no extra forces. Between armStretchDistance and maxArmStretch, apply increasing force.
        // Above maxArmStretch, manually move (lighter?) objects within range of each other.
        if (grabbed != null)
        {
            Vector3 bodyPos = physicalBody.position;
            Vector3 armPos = armColliderHandler.RigidBody.position;

            // Gets the displacement.
            float distance = Vector3.Distance(bodyPos, armPos);

            // TODO: If distance > maxArmStretch, manually move it into acceptable range somehow.

            distance = Mathf.Clamp(distance, armStretchDistance, maxArmStretchDistance);
            // Remaps to 0.0 - 1.0 range.
            // distance = distance.Map(armStretchDistance, maxArmStretch, 0f, 1f);

            // Maps the distance value to a 0.0 - maxArmTension range.
            float force = distance.Map(armStretchDistance, maxArmStretchDistance, 0f, maxArmTension);

            Vector3 direction = (bodyPos - armPos).normalized;
            Vector3 velocity = physicalBody.velocity - armColliderHandler.RigidBody.velocity;
            // Vector3 velocity2 = armColliderHandler.RigidBody.velocity - physicalBody.velocity;

            physicalBody.AddForce(-force * direction - velocity * armDampening);
            armColliderHandler.RigidBody.AddForce(force * direction + velocity * armDampening);


            // distance = Mathf.Clamp(distance, armStretchDistance, maxArmStretch);
        }

        // if (grabbed != null)
        // {
        //     Vector3 bodyPos = physicalBody.position;
        //     Vector3 armPos = armColliderHandler.RigidBody.position;

        //     float force = Vector3.Distance(bodyPos, armPos) * armStrength;
        //     Vector3 direction = (bodyPos - armPos).normalized;
        //     Vector3 velocity = armColliderHandler.RigidBody.velocity;

        //     physicalBody.AddForce(-force * direction - velocity * armDampening);
        //     armColliderHandler.RigidBody.AddForce(force * direction + velocity * armDampening);


        // }
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
        physicalBody.AddForce(movement * speed, ForceMode.VelocityChange);
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
        Debug.Log("Collision count: " + ++armCollisionCount);
        // Ignore this stuff if grabbing onto something.
        if (grabbed != null || armState != ArmState.Launching)
            return;

        StartArmLaunchCooldown();

        // TODO: Also call this method for OnCollisionStay, so that if the arm is already colliding with an object when the arm is launched, it still launches the player.
        // TODO: Rewrite to work better with the "arm launch" system. Instead probably based off of distance traveled? Probably inverse, actually.
        //       Yeah. Probably more force the further the arm is from its intended destination.

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
