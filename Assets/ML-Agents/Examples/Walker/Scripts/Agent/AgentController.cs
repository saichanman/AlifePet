using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using Random = UnityEngine.Random;
using UnityEngine.Events;

[RequireComponent(typeof(JointDriveController))]
public class AgentController : Agent
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

    const float m_maxWalkingSpeed = 10; //The max walking speed

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
    public Transform FoodPrefab;
    public Transform ToyPrefab;
    public Transform TargetPrefabEmpty;

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

    // ドラッグ操作に必要な変数
    private Camera mainCamera;         // シーン内のカメラを参照
    private bool isDragging = false;   // ドラッグ中かどうかを追跡
    private Vector3 offset;            // マウスとHIPの相対位置
    private Rigidbody rb_hip;              // HIPのRigidbody
    private Rigidbody rb_head;
    private Rigidbody rb_body;
    private float dragHeight = 3.0f;   // ドラッグ時にHIPが保持する高さ

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

        // メインカメラを取得
        mainCamera = Camera.main;

        // HIPのRigidbodyを取得
        rb_hip = hip.GetComponent<Rigidbody>();
        rb_head = head.GetComponent<Rigidbody>();
        rb_body = body.GetComponent<Rigidbody>();
    }

    /// <summary>
    /// Spawns a target prefab at pos
    /// </summary>
    /// <param name="prefab"></param>
    /// <param name="pos"></param>
    void SpawnTarget(Transform prefab, Vector3 pos)
    {
        // ターゲットの生成
        if (prefab != null)
        {
            Vector3 spawnPosition = transform.position + new Vector3(Random.Range(-10f, 10f), 0, Random.Range(-10f, 10f));
            m_Target = Instantiate(prefab, spawnPosition, Quaternion.identity, transform.parent);
            m_Target.tag = "target"; // タグを設定
        }
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
        //TargetWalkingSpeed = Random.Range(0.1f, m_maxWalkingSpeed);
        TargetWalkingSpeed = 3.0f;
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

        // ターゲットが存在する場合は位置を観測
        if (m_Target != null)
        {
            sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(m_Target.position));
        }
        else
        {
            // ターゲットがない場合、デフォルト値を使用
            sensor.AddObservation(Vector3.zero);
        }
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

    void Update()
    {
        // マウスクリックしてドラッグ開始
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit) && (hit.transform == hip || hit.transform == head || hit.transform == body))
            {
                isDragging = true;
                offset = hip.position - GetMouseWorldPosition();
                rb_hip.isKinematic = true; // ドラッグ中は物理挙動を停止
            }
        }

        // マウスクリックを離してドラッグ終了
        if (Input.GetMouseButtonUp(0) && isDragging)
        {
            isDragging = false;
            rb_hip.isKinematic = false; // ドラッグ終了で物理挙動を再開

            // HIPの角度を調整（X,Z軸をゼロに戻す）
            Quaternion currentRotation = hip.rotation;
            hip.rotation = Quaternion.Euler(0, currentRotation.eulerAngles.y, 0);
        }
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

        if (isDragging)
        {
            // HIPの角度を調整してX,Z軸回転をゼロにする
            Quaternion currentRotation = hip.rotation;
            hip.rotation = Quaternion.Euler(0, currentRotation.eulerAngles.y, 0);

            // HIPの位置をドラッグに合わせる（ドラッグ時の高さを固定）
            Vector3 targetPosition = GetMouseWorldPosition() + offset;
            targetPosition.y = dragHeight; // Y座標を固定
            hip.position = targetPosition;
        }
        else
        {
            UpdateOrientationObjects();
        }
        
    }

    private Vector3 GetMouseWorldPosition()
    {
        Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
        if (Physics.Raycast(ray, out RaycastHit hit))
        {
            return hit.point; // マウスが指しているワールド座標を返す
        }
        return Vector3.zero; // 何もヒットしなかった場合
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
        normalizedReward *= 0.02f;

        AddReward(normalizedReward); // スケールは調整可能

    }

    /// <summary>
    /// Agent touched the target
    /// </summary>
    public void TouchedTarget()
    {
        AddReward(1f);
    }

    public void SwitchTargetPrefab(Transform newPrefab)
    {
        // 新しいプレハブを設定
        TargetPrefab = newPrefab;

        // 現在のターゲットをクリア
        ClearTarget();

        // 新しいターゲットをスポーン
        SpawnTarget(TargetPrefab, transform.position + new Vector3(Random.Range(-20, 20), 0, Random.Range(-20, 20)));

        // ログで確認
        Debug.Log($"Target prefab switched to: {newPrefab.name}");
    }

    public void SetFoodPrefab()
    {
        SwitchTargetPrefab(FoodPrefab);
    }

    public void SetToyPrefab()
    {
        SwitchTargetPrefab(ToyPrefab);
    }

    public void SetDefaultTargetPrefab()
    {
        // デフォルトのターゲットプレハブを設定
        if (TargetPrefab != null)
        {
            SwitchTargetPrefab(TargetPrefabEmpty);
        }
        else
        {
            Debug.LogWarning("TargetPrefab が設定されていません。Inspector を確認してください。");
        }
    }

    private void ClearTarget()
    {
        // 現在のターゲットが存在すれば削除
        if (m_Target != null)
        {
            Destroy(m_Target.gameObject);
            m_Target = null; // 参照をクリア
        }

        // シーン内のターゲットプレハブをすべて探して削除
        var targets = GameObject.FindGameObjectsWithTag("target");
        foreach (var target in targets)
        {
            Destroy(target);
        }
    }    
}