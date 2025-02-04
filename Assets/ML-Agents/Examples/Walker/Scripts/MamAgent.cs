using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using BodyPart = Unity.MLAgentsExamples.BodyPart;
using Random = UnityEngine.Random;

public class MamAgent : Agent
{
    [Header("Walk Speed")]
    [Range(0.1f, 10)]
    [SerializeField]
    //The walking speed to try and achieve
    private float m_TargetWalkingSpeed = 10;

    public float MTargetWalkingSpeed // property
    {
        get { return m_TargetWalkingSpeed; }
        set { m_TargetWalkingSpeed = Mathf.Clamp(value, .1f, m_maxWalkingSpeed); }
    }

    const float m_maxWalkingSpeed = 10; //The max walking speed

    //Should the agent sample a new goal velocity each episode?
    //If true, walkSpeed will be randomly set between zero and m_maxWalkingSpeed in OnEpisodeBegin()
    //If false, the goal velocity will be walkingSpeed
    public bool randomizeWalkSpeedEachEpisode;

    //The direction an agent will walk during training.
    public Vector3 m_WorldDirToWalk = Vector3.right;

    [Header("Target To Walk Towards")] public Transform target; //Target the agent will walk towards during training.

    [Header("Body Parts")]
    public Transform body;
    public Transform neck;
    public Transform head;
    public Transform leg1_1;
    public Transform leg1_2;
    public Transform leg1_3;
    public Transform leg2_1;
    public Transform leg2_2;
    public Transform leg2_3;
    public Transform leg3_1;
    public Transform leg3_2;
    public Transform leg3_3;
    public Transform leg4_1;
    public Transform leg4_2;
    public Transform leg4_3;
    public Transform mouthPosition;

    // [HideInInspector]
    public bool runningToItem;
    // [HideInInspector]
    public bool returningItem;

    public bool trotLeft = false;
    public bool trotRight = false;

    public const int timerRimit = 60;

    public float trotTimerLeft = 0;
    public float trotTimerRight = 0;

    //This will be used as a stabilized model space reference point for observations
    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    OrientationCubeController m_OrientationCube;

    //The indicator graphic gameobject that points towards the target
    DirectionIndicator m_DirectionIndicator;
    JointDriveController jdController;
    EnvironmentParameters m_ResetParams;

    public override void Initialize()
    {
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();

        //Setup each body part
        jdController = GetComponent<JointDriveController>();
        jdController.SetupBodyPart(body);
        jdController.SetupBodyPart(neck);
        jdController.SetupBodyPart(head);
        jdController.SetupBodyPart(leg1_1);
        jdController.SetupBodyPart(leg1_2);
        jdController.SetupBodyPart(leg1_3);
        jdController.SetupBodyPart(leg2_1);
        jdController.SetupBodyPart(leg2_2);
        jdController.SetupBodyPart(leg2_3);
        jdController.SetupBodyPart(leg3_1);
        jdController.SetupBodyPart(leg3_2);
        jdController.SetupBodyPart(leg3_3);
        jdController.SetupBodyPart(leg4_1);
        jdController.SetupBodyPart(leg4_2);
        jdController.SetupBodyPart(leg4_3);

        m_ResetParams = Academy.Instance.EnvironmentParameters;

        SetResetParameters();
    }

    /// <summary>
    /// Loop over body parts and reset them to initial conditions.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        //Reset all of the body parts
        foreach (var bodyPart in jdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }

        //Random start rotation to help generalize
        body.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);

        UpdateOrientationObjects();

        //Set our goal walking speed
        MTargetWalkingSpeed =
            randomizeWalkSpeedEachEpisode ? Random.Range(0.1f, m_maxWalkingSpeed) : MTargetWalkingSpeed;

        SetResetParameters();
    }

    /// <summary>
    /// Add relevant information on each body part to observations.
    /// </summary>
    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        //GROUND CHECK
        sensor.AddObservation(bp.groundContact.touchingGround); // Is this bp touching the ground

        //Get velocities in the context of our orientation cube's space
        //Note: You can get these velocities in world space as well but it may not train as well.
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.velocity));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.angularVelocity));

        //Get position relative to body in the context of our orientation cube's space
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.position - body.position));

        if (bp.rb.transform != body) //&& bp.rb.transform != handL && bp.rb.transform != handR)
        {
            sensor.AddObservation(bp.rb.transform.localRotation);
            sensor.AddObservation(bp.currentStrength / jdController.maxJointForceLimit);
        }
    }

    /// <summary>
    /// Loop over body parts to add them to observation.
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        var cubeForward = m_OrientationCube.transform.forward;

        //velocity we want to match
        var velGoal = cubeForward * MTargetWalkingSpeed;
        //ragdoll's avg vel
        var avgVel = GetAvgVelocity();

        //current ragdoll velocity. normalized
        sensor.AddObservation(Vector3.Distance(velGoal, avgVel));
        //avg body vel relative to cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(avgVel));
        //vel goal relative to cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(velGoal));

        //rotation deltas
        sensor.AddObservation(Quaternion.FromToRotation(body.forward, cubeForward));
        sensor.AddObservation(Quaternion.FromToRotation(head.forward, cubeForward));

        //Position of target position relative to cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(target.transform.position));

        foreach (var bodyPart in jdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)

    {
        var bpDict = jdController.bodyPartsDict;
        var i = -1;

        var vectorAction = actionBuffers.ContinuousActions;
        bpDict[neck].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
        bpDict[head].SetJointTargetRotation(0, 0, vectorAction[++i]);
        bpDict[leg1_1].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
        bpDict[leg1_2].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
        bpDict[leg1_3].SetJointTargetRotation(vectorAction[++i], 0, 0);
        bpDict[leg2_1].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
        bpDict[leg2_2].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
        bpDict[leg2_3].SetJointTargetRotation(vectorAction[++i], 0, 0);
        bpDict[leg3_1].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
        bpDict[leg3_2].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
        bpDict[leg3_3].SetJointTargetRotation(vectorAction[++i], 0, 0);
        bpDict[leg4_1].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
        bpDict[leg4_2].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
        bpDict[leg4_3].SetJointTargetRotation(vectorAction[++i], 0, 0);
        bpDict[neck].SetJointStrength(vectorAction[++i]);
        bpDict[head].SetJointStrength(vectorAction[++i]);
        bpDict[leg1_1].SetJointStrength(vectorAction[++i]);
        bpDict[leg1_2].SetJointStrength(vectorAction[++i]);
        bpDict[leg1_3].SetJointStrength(vectorAction[++i]);
        bpDict[leg2_1].SetJointStrength(vectorAction[++i]);
        bpDict[leg2_2].SetJointStrength(vectorAction[++i]);
        bpDict[leg2_3].SetJointStrength(vectorAction[++i]);
        bpDict[leg3_1].SetJointStrength(vectorAction[++i]);
        bpDict[leg3_2].SetJointStrength(vectorAction[++i]);
        bpDict[leg3_3].SetJointStrength(vectorAction[++i]);
        bpDict[leg4_1].SetJointStrength(vectorAction[++i]);
        bpDict[leg4_2].SetJointStrength(vectorAction[++i]);
        bpDict[leg4_3].SetJointStrength(vectorAction[++i]);
    }

    //Update OrientationCube and DirectionIndicator
    public void UpdateOrientationObjects()
    {
        m_WorldDirToWalk = target.position - body.position;
        m_OrientationCube.UpdateOrientation(body, target);
        if (m_DirectionIndicator)
        {
            m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
        }
    }

    void FixedUpdate()
    {
        UpdateOrientationObjects();

        var cubeForward = m_OrientationCube.transform.forward;

        // Set reward for this step according to mixture of the following elements.
        // a. Match target speed
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        var matchSpeedReward = GetMatchingVelocityReward(cubeForward * MTargetWalkingSpeed, GetAvgVelocity());

        //Check for NaNs
        if (float.IsNaN(matchSpeedReward))
        {
            throw new ArgumentException(
                "NaN in moveTowardsTargetReward.\n" +
                $" cubeForward: {cubeForward}\n" +
                $" body.velocity: {jdController.bodyPartsDict[body].rb.velocity}\n" +
                $" maximumWalkingSpeed: {m_maxWalkingSpeed}"
            );
        }

        // b. Rotation alignment with target direction.
        //This reward will approach 1 if it faces the target direction perfectly and approach zero as it deviates
        var lookAtTargetReward = (Vector3.Dot(cubeForward, head.forward) + 1) * .5F;

        //Check for NaNs
        if (float.IsNaN(lookAtTargetReward))
        {
            throw new ArgumentException(
                "NaN in lookAtTargetReward.\n" +
                $" cubeForward: {cubeForward}\n" +
                $" head.forward: {head.forward}"
            );
        }

        AddReward(matchSpeedReward * lookAtTargetReward);


        // trot
        if(jdController.bodyPartsDict[leg1_3].groundContact.touchingGround && jdController.bodyPartsDict[leg4_3].groundContact.touchingGround && !jdController.bodyPartsDict[leg2_3].groundContact.touchingGround && !jdController.bodyPartsDict[leg3_3].groundContact.touchingGround){
            trotLeft = true;
            trotRight = false;
        }
        else if(jdController.bodyPartsDict[leg2_3].groundContact.touchingGround && jdController.bodyPartsDict[leg3_3].groundContact.touchingGround && !jdController.bodyPartsDict[leg1_3].groundContact.touchingGround && !jdController.bodyPartsDict[leg4_3].groundContact.touchingGround){
            trotRight = true;
            trotLeft = false;
        }
        else{
            trotRight = false;
            trotLeft = false;
        }

        if(trotLeft){
            trotTimerRight = 0;
            trotTimerLeft++;
            if(trotTimerLeft < timerRimit){
                AddReward(0.005f * trotTimerLeft);
            }
            else{
                AddReward(-0.01f * (trotTimerLeft - timerRimit));
            }
        }
        else if(trotRight){
            trotTimerLeft = 0;
            trotTimerRight++;
            if(trotTimerRight < timerRimit){
                AddReward(0.005f * trotTimerRight);
            }
            else{
                AddReward(-0.01f * (trotTimerRight - timerRimit));
            }
        }
        else{
            AddReward(-0.1f);
        }

        // float bodyRotationX = Math.Abs(jdController.bodyPartsDict[body].rb.transform.localRotation.x);
        // float bodyRotationZ = Math.Abs(jdController.bodyPartsDict[body].rb.transform.localRotation.z);
        // if(bodyRotationX < 10 && bodyRotationZ < 10){
        //    AddReward(0.05f); 
        // }
        // else{
        //     AddReward(-0.3f); 
        // }

        // // float legAngVel_1 = Math.Abs(jdController.bodyPartsDict[leg1_1].rb.angularVelocity.x);
        // // float legAngVel_2 = Math.Abs(jdController.bodyPartsDict[leg2_1].rb.angularVelocity.x);
        // // float legAngVel_3 = Math.Abs(jdController.bodyPartsDict[leg3_1].rb.angularVelocity.x);
        // // float legAngVel_4 = Math.Abs(jdController.bodyPartsDict[leg4_1].rb.angularVelocity.x);

        // // AddReward(legAngVel_1 * legAngVel_4 * 0.002f);
        // // AddReward(legAngVel_3 * legAngVel_2 * 0.002f);

        // float legAng_1 = Math.Abs(jdController.bodyPartsDict[leg1_1].rb.transform.localRotation.x);
        // float legAng_2 = Math.Abs(jdController.bodyPartsDict[leg2_1].rb.transform.localRotation.x);
        // float legAng_3 = Math.Abs(jdController.bodyPartsDict[leg3_1].rb.transform.localRotation.x);
        // float legAng_4 = Math.Abs(jdController.bodyPartsDict[leg4_1].rb.transform.localRotation.x);

        // float rightLegDif = Math.Abs(legAng_1 - legAng_2);
        // float leftLegDif = Math.Abs(legAng_3 - legAng_4);

        // AddReward(rightLegDif * 0.002f);
        // AddReward(leftLegDif * 0.002f);



    }

    //Returns the average velocity of all of the body parts
    //Using the velocity of the body only has shown to result in more erratic movement from the limbs, so...
    //...using the average helps prevent this erratic movement
    Vector3 GetAvgVelocity()
    {
        Vector3 velSum = Vector3.zero;

        //ALL RBS
        int numOfRb = 0;
        foreach (var item in jdController.bodyPartsList)
        {
            numOfRb++;
            velSum += item.rb.velocity;
        }

        var avgVel = velSum / numOfRb;
        return avgVel;
    }

    //normalized value of the difference in avg speed vs goal walking speed.
    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
    {
        //distance between our actual velocity and goal velocity
        var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, MTargetWalkingSpeed);

        //return the value on a declining sigmoid shaped curve that decays from 1 to 0
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / MTargetWalkingSpeed, 2), 2);
    }

    /// <summary>
    /// Agent touched the target
    /// </summary>
    public void TouchedTarget()
    {
        AddReward(1f);
    }

    public void SetTorsoMass()
    {
        jdController.bodyPartsDict[body].rb.mass = m_ResetParams.GetWithDefault("spine_mass", 5);
    }

    public void SetResetParameters()
    {
        SetTorsoMass();
        trotTimerRight = 0;
        trotTimerLeft = 0;
    }
}
