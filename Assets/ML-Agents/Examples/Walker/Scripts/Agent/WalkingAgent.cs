using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using Random = UnityEngine.Random;

[RequireComponent(typeof(JointDriveController))] // Required to set joint forces
public class WalkingAgent : Agent
{

    [Header("Walk Speed")]
    [Range(0.1f, m_maxWalkingSpeed)]
    [SerializeField]
    [Tooltip(
        "The speed the agent will try to match.\n\n" +
        "TRAINING:\n" +
        "For VariableSpeed envs, this value will randomize at the start of each training episode.\n" +
        "Otherwise the agent will try to match the speed set here.\n\n" +
        "INFERENCE:\n" +
        "During inference, VariableSpeed agents will modify their behavior based on this value " +
        "whereas the CrawlerDynamic & CrawlerStatic agents will run at the speed specified during training "
    )]
    //The walking speed to try and achieve
    private float m_TargetWalkingSpeed = m_maxWalkingSpeed;

    const float m_maxWalkingSpeed = 15; //The max walking speed

    //The current target walking speed. Clamped because a value of zero will cause NaNs
    public float TargetWalkingSpeed
    {
        get { return m_TargetWalkingSpeed; }
        set { m_TargetWalkingSpeed = Mathf.Clamp(value, .1f, m_maxWalkingSpeed); }
    }

    //The direction an agent will walk during training.
    [Header("Target To Walk Towards")]
    public Transform TargetPrefab; //Target prefab to use in Dynamic envs
    private Transform m_Target; //Target the agent will walk towards during training.

    [Header("Body Parts")]
    public Transform hip;
    public Transform body;
    public Transform head;

    public Transform frontLeftLegUpper, frontLeftLegLower, frontLeftFoot;
    public Transform frontRightLegUpper, frontRightLegLower, frontRightFoot;
    public Transform backLeftLegUpper, backLeftLegLower, backLeftFoot;
    public Transform backRightLegUpper, backRightLegLower, backRightFoot;

    //This will be used as a stabilized model space reference point for observations
    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    OrientationCubeController m_OrientationCube;

    //The indicator graphic gameobject that points towards the target
    DirectionIndicator m_DirectionIndicator;
    JointDriveController m_JdController;

    [Header("Foot Grounded Visualization")]
    [Space(10)]
    public bool useFootGroundedVisualization;

    public MeshRenderer foot0;
    public MeshRenderer foot1;
    public MeshRenderer foot2;
    public MeshRenderer foot3;
    public Material groundedMaterial;
    public Material unGroundedMaterial;

    public override void Initialize()
    {
        SpawnTarget(TargetPrefab, transform.position); //spawn target

        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();
        m_JdController = GetComponent<JointDriveController>();

        //Setup each body part
        m_JdController.SetupBodyPart(hip);
        m_JdController.SetupBodyPart(body);
        m_JdController.SetupBodyPart(head);

        m_JdController.SetupBodyPart(frontLeftLegUpper);
        m_JdController.SetupBodyPart(frontLeftLegLower);
        m_JdController.SetupBodyPart(frontLeftFoot);

        m_JdController.SetupBodyPart(frontRightLegUpper);
        m_JdController.SetupBodyPart(frontRightLegLower);
        m_JdController.SetupBodyPart(frontRightFoot);

        m_JdController.SetupBodyPart(backLeftLegUpper);
        m_JdController.SetupBodyPart(backLeftLegLower);
        m_JdController.SetupBodyPart(backLeftFoot);

        m_JdController.SetupBodyPart(backRightLegUpper);
        m_JdController.SetupBodyPart(backRightLegLower);
        m_JdController.SetupBodyPart(backRightFoot);
    }

    /// <summary>
    /// Spawns a target prefab at pos
    /// </summary>
    /// <param name="prefab"></param>
    /// <param name="pos"></param>
    void SpawnTarget(Transform prefab, Vector3 pos)
    {
        m_Target = Instantiate(prefab, pos, Quaternion.identity, transform.parent);
    }

    /// <summary>
    /// Loop over body parts and reset them to initial conditions.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }

        //Random start rotation to help generalize
        //body.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);

        UpdateOrientationObjects();

        //Set our goal walking speed
        TargetWalkingSpeed = Random.Range(0.1f, m_maxWalkingSpeed);
    }

    /// <summary>
    /// Add relevant information on each body part to observations.
    /// </summary>
    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        //GROUND CHECK
        sensor.AddObservation(bp.groundContact.touchingGround); // Is this bp touching the ground

        if (bp.rb.transform != body)
        {
            sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
        }
    }

    /// <summary>
    /// Loop over body parts to add them to observation.
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        var cubeForward = m_OrientationCube.transform.forward;

        //velocity we want to match
        var velGoal = cubeForward * TargetWalkingSpeed;
        //ragdoll's avg vel
        var avgVel = GetAvgVelocity();

        //current ragdoll velocity. normalized
        sensor.AddObservation(Vector3.Distance(velGoal, avgVel));
        //avg body vel relative to cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(avgVel));
        //vel goal relative to cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(velGoal));
        //rotation delta
        sensor.AddObservation(Quaternion.FromToRotation(body.forward, cubeForward));

        //Add pos of target relative to orientation cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(m_Target.transform.position));

        RaycastHit hit;
        float maxRaycastDist = 10;
        if (Physics.Raycast(body.position, Vector3.down, out hit, maxRaycastDist))
        {
            sensor.AddObservation(hit.distance / maxRaycastDist);
        }
        else
            sensor.AddObservation(1);

        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // The dictionary with all the body parts in it are in the jdController
        var bpDict = m_JdController.bodyPartsDict;

        var continuousActions = actionBuffers.ContinuousActions;
        var i = -1;
        
        bpDict[body].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[head].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);

        bpDict[frontLeftLegUpper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[frontRightLegUpper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[backLeftLegUpper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[backRightLegUpper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);

        bpDict[frontLeftLegLower].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[frontRightLegLower].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[backLeftLegLower].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[backRightLegLower].SetJointTargetRotation(continuousActions[++i], 0, 0);

        bpDict[frontLeftFoot].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[frontRightFoot].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[backLeftFoot].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[backRightFoot].SetJointTargetRotation(continuousActions[++i], 0, 0);

        // Update joint strength
        bpDict[body].SetJointStrength(continuousActions[++i]);
        bpDict[head].SetJointStrength(continuousActions[++i]);

        bpDict[frontLeftLegUpper].SetJointStrength(continuousActions[++i]);
        bpDict[frontRightLegUpper].SetJointStrength(continuousActions[++i]);
        bpDict[backLeftLegUpper].SetJointStrength(continuousActions[++i]);
        bpDict[backRightLegUpper].SetJointStrength(continuousActions[++i]);

        bpDict[frontLeftLegLower].SetJointStrength(continuousActions[++i]);
        bpDict[frontRightLegLower].SetJointStrength(continuousActions[++i]);
        bpDict[backLeftLegLower].SetJointStrength(continuousActions[++i]);
        bpDict[backRightLegLower].SetJointStrength(continuousActions[++i]);

        bpDict[frontLeftFoot].SetJointStrength(continuousActions[++i]);
        bpDict[frontRightFoot].SetJointStrength(continuousActions[++i]);
        bpDict[backLeftFoot].SetJointStrength(continuousActions[++i]);
        bpDict[backRightFoot].SetJointStrength(continuousActions[++i]);
    }

    void FixedUpdate()
    {
        UpdateOrientationObjects();

        // If enabled the feet will light up green when the foot is grounded.
        // This is just a visualization and isn't necessary for function
        if (useFootGroundedVisualization)
        {
            foot0.material = m_JdController.bodyPartsDict[frontLeftFoot].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
            foot1.material = m_JdController.bodyPartsDict[frontRightFoot].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
            foot2.material = m_JdController.bodyPartsDict[backLeftFoot].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
            foot3.material = m_JdController.bodyPartsDict[backRightFoot].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
        }

        var cubeForward = m_OrientationCube.transform.forward;

        // Set reward for this step according to mixture of the following elements.
        // a. Match target speed
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        var matchSpeedReward = GetMatchingVelocityReward(cubeForward * TargetWalkingSpeed, GetAvgVelocity());

        // b. Rotation alignment with target direction.
        //This reward will approach 1 if it faces the target direction perfectly and approach zero as it deviates
        var lookAtTargetReward = (Vector3.Dot(cubeForward, body.forward) + 1) * .5F;

        AddReward(matchSpeedReward * lookAtTargetReward);

        AddTrotGaitReward();
        AddBalanceReward();
        
    }

    /// <summary>
    /// Update OrientationCube and DirectionIndicator
    /// </summary>
    void UpdateOrientationObjects()
    {
        m_OrientationCube.UpdateOrientation(body, m_Target);
        if (m_DirectionIndicator)
        {
            m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
        }
    }

    /// <summary>
    ///Returns the average velocity of all of the body parts
    ///Using the velocity of the body only has shown to result in more erratic movement from the limbs
    ///Using the average helps prevent this erratic movement
    /// </summary>
    Vector3 GetAvgVelocity()
    {
        Vector3 velSum = Vector3.zero;
        Vector3 avgVel = Vector3.zero;

        //ALL RBS
        int numOfRb = 0;
        foreach (var item in m_JdController.bodyPartsList)
        {
            numOfRb++;
            velSum += item.rb.velocity;
        }

        avgVel = velSum / numOfRb;
        return avgVel;
    }

    /// <summary>
    /// Normalized value of the difference in actual speed vs goal walking speed.
    /// </summary>
    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
    {
        //distance between our actual velocity and goal velocity
        var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, TargetWalkingSpeed);

        //return the value on a declining sigmoid shaped curve that decays from 1 to 0
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / TargetWalkingSpeed, 2), 2);
    }

    private void AddTrotGaitReward()
    {
        var bpDict = m_JdController.bodyPartsDict;

        // 脚の角速度を取得（X軸周りの角速度を使用）
        float frontLeftLegSwing = bpDict[frontLeftLegUpper].rb.angularVelocity.x;
        float backRightLegSwing = bpDict[backRightLegUpper].rb.angularVelocity.x;
        float frontRightLegSwing = bpDict[frontRightLegUpper].rb.angularVelocity.x;
        float backLeftLegSwing = bpDict[backLeftLegUpper].rb.angularVelocity.x;

        // 対角線上の脚が同一方向にスイングしているかを評価
        bool isDiagonalSwing1 = frontLeftLegSwing * backRightLegSwing > 0;
        bool isDiagonalSwing2 = frontRightLegSwing * backLeftLegSwing > 0;

        // 横の脚のペアが反対方向にスイングしているかを評価
        bool isFrontPairSwing = frontLeftLegSwing * frontRightLegSwing < 0;
        bool isBackPairSwing = backLeftLegSwing * backRightLegSwing < 0;

        // 足の接地状態を取得（足の下部パーツから）
        bool isFrontLeftFootGrounded = frontLeftFoot.GetComponent<GroundContact>().touchingGround;
        bool isBackRightFootGrounded = backRightFoot.GetComponent<GroundContact>().touchingGround;
        bool isFrontRightFootGrounded = frontRightFoot.GetComponent<GroundContact>().touchingGround;
        bool isBackLeftFootGrounded = backLeftFoot.GetComponent<GroundContact>().touchingGround;

        // 対角線上の足が接地しているかを評価
        bool isDiagonalPair1Grounded = isFrontLeftFootGrounded && isBackRightFootGrounded;
        bool isDiagonalPair2Grounded = isFrontRightFootGrounded && isBackLeftFootGrounded;

        // トロットの判定
        bool Trot1 = isDiagonalSwing1 && isDiagonalPair2Grounded && !isDiagonalPair1Grounded && isFrontPairSwing && isBackPairSwing;
        bool Trot2 = isDiagonalSwing2 && isDiagonalPair1Grounded && !isDiagonalPair2Grounded && isFrontPairSwing && isBackPairSwing;

        float reward = 0f;

        // 対角線上の脚が反対方向にスイングし、その他のペアの足が接地している場合に報酬を加算
        if (Trot1)
        {
            reward += 1f; // スコア基準を1に設定
        }

        if (Trot2)
        {
            reward += 1f; // スコア基準を1に設定
        }

        // 報酬を0〜1にスケール
        float normalizedReward = reward > 0 ? reward / 2f : 0f; // 最大報酬が2なので正規化
        // スケーリング
        normalizedReward *= 0.03f;

        AddReward(normalizedReward); // スケールは調整可能

    }

    private void AddBalanceReward()
    {
        // HIPのUP方向とワールドUP方向の一致度を計算
        float uprightAlignment = Vector3.Dot(hip.up, Vector3.up);

        // 直立に近いほど高い報酬を与える
        float balanceReward = Mathf.Clamp01(uprightAlignment) * 0.01f;
        AddReward(balanceReward);

        // 傾きすぎた場合のペナルティ
        if (uprightAlignment < 0.8f) // 閾値調整可能
        {
            AddReward(-0.02f);
        }
    }

    /// <summary>
    /// Agent touched the target
    /// </summary>
    public void TouchedTarget()
    {
        AddReward(1f);
    }
}
