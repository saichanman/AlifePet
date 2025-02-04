using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using System.Linq;

public class QuadrupedAgent : Agent
{
    [Header("Agent Settings")]
    [Range(0.1f, 10)]
    public float maxWalkingSpeed = 5f;

    [Header("Energy Settings")]
    public float maxEnergy = 100f;
    public float energyConsumptionRate = 1f;
    public float energyRecoveryRate = 5f;
    private float energy;

    [Header("Hunger Settings")]
    public float maxHunger = 100f;
    public float hungerIncreaseRate = 0.5f;
    public float hungerDecreaseAmount = 50f;
    private float hunger;

    [Header("Stress Settings")]
    public float maxStress = 100f;
    public float stressIncreaseRate = 0.3f;
    public float stressDecreaseAmount = 30f;
    private float stress;

    [Header("State Durations")]
    public float idleDuration = 2f;
    public float restDuration = 5f;

    [Header("Targets")]
    public Transform foodObject;
    public Transform toyObject;

    [Header("Body Parts")]
    public Transform hip;
    public Transform spine1;
    public Transform spine2;
    public Transform neck;
    public Transform head;
    public Transform tail1;
    public Transform tail2;
    public Transform frontLeftLeg;
    public Transform frontRightLeg;
    public Transform backLeftLeg;
    public Transform backRightLeg;

    [Header("Spawner Reference")]
    public ItemSpawner itemSpawner;

    [Header("Look At Camera Settings")]
    public float lookAtCameraChance = 0.01f; // 状態遷移の確率（0.0〜1.0）
    public float lookAtCameraDuration = 2f;  // 状態の継続時間（秒）
    public float headTurnSpeed = 2f;         // 頭部の回転速度

    // カメラの参照
    [HideInInspector]
    public Transform cameraTransform;

    private OrientationCubeController m_OrientationCube;
    private DirectionIndicator m_DirectionIndicator;
    private JointDriveController m_JdController;
    private EnvironmentParameters m_ResetParams;

    private State currentState;
    public float stateStartTime;
    private Vector3 randomWalkDirection;
    private bool isFallen = false;

    public override void Initialize()
    {
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();

        m_JdController = GetComponent<JointDriveController>();
        m_JdController.SetupBodyPart(hip);
        m_JdController.SetupBodyPart(spine1);
        m_JdController.SetupBodyPart(spine2);
        m_JdController.SetupBodyPart(neck);
        m_JdController.SetupBodyPart(head);
        m_JdController.SetupBodyPart(tail1);
        m_JdController.SetupBodyPart(tail2);
        m_JdController.SetupBodyPart(frontLeftLeg);
        m_JdController.SetupBodyPart(frontRightLeg);
        m_JdController.SetupBodyPart(backLeftLeg);
        m_JdController.SetupBodyPart(backRightLeg);

        m_ResetParams = Academy.Instance.EnvironmentParameters;

        energy = maxEnergy;
        hunger = 0f;
        stress = 0f;

        // カメラの参照を取得
        cameraTransform = Camera.main?.transform;
        if (cameraTransform == null)
        {
            Debug.LogWarning("Main Camera がシーンに存在しません。LookAtCameraState は動作しません。");
        }

        ChangeState(new IdleState(this));

        // itemSpawnerの自動取得
        if (itemSpawner == null)
        {
            itemSpawner = FindObjectOfType<ItemSpawner>();
            if (itemSpawner == null)
            {
                Debug.LogError("ItemSpawnerがシーンに存在しません。");
            }
        }
    }

    public override void OnEpisodeBegin()
    {
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }

        hip.rotation = Quaternion.Euler(0, 0, 0);

        SetRandomWalkDirection();

        energy = maxEnergy;
        hunger = 0f;
        stress = 0f;
        isFallen = false;

        ChangeState(new IdleState(this));
    }

    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        sensor.AddObservation(bp.groundContact.touchingGround ? 1f : 0f);
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.velocity));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.angularVelocity));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(bp.rb.position));
        sensor.AddObservation(bp.rb.transform.localRotation);
        sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        var cubeForward = m_OrientationCube.transform.forward;
        var velGoal = cubeForward * maxWalkingSpeed;
        var avgVel = GetAvgVelocity();

        sensor.AddObservation(Vector3.Distance(velGoal, avgVel));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(avgVel));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(velGoal));
        sensor.AddObservation(Quaternion.FromToRotation(hip.forward, cubeForward));
        sensor.AddObservation(Quaternion.FromToRotation(head.forward, cubeForward));

        // 餌とおもちゃの位置を観察
        if (foodObject != null)
        {
            sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(foodObject.position));
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
        }

        if (toyObject != null)
        {
            sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(toyObject.position));
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
        }

        // カメラの位置を観察
        if (cameraTransform != null)
        {
            sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(cameraTransform.position));
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
        }

        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }

        sensor.AddObservation(energy / maxEnergy);
        sensor.AddObservation(hunger / maxHunger);
        sensor.AddObservation(stress / maxStress);
        sensor.AddObservation(isFallen ? 1f : 0f);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var bpDict = m_JdController.bodyPartsDict;
        var i = -1;
        var continuousActions = actionBuffers.ContinuousActions;

        // 背骨、首、尻尾の回転制御（3軸）
        bpDict[spine1].SetJointTargetRotation(
            continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[spine2].SetJointTargetRotation(
            continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[neck].SetJointTargetRotation(
            continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[tail1].SetJointTargetRotation(
            continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[tail2].SetJointTargetRotation(
            continuousActions[++i], continuousActions[++i], continuousActions[++i]);

        // 足の回転制御（3軸）
        bpDict[frontLeftLeg].SetJointTargetRotation(
            continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[frontRightLeg].SetJointTargetRotation(
            continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[backLeftLeg].SetJointTargetRotation(
            continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[backRightLeg].SetJointTargetRotation(
            continuousActions[++i], continuousActions[++i], continuousActions[++i]);

        // 頭の回転制御（3軸）
        bpDict[head].SetJointTargetRotation(
            continuousActions[++i], continuousActions[++i], continuousActions[++i]);

        // ジョイントの強度設定
        bpDict[spine1].SetJointStrength(continuousActions[++i]);
        bpDict[spine2].SetJointStrength(continuousActions[++i]);
        bpDict[neck].SetJointStrength(continuousActions[++i]);
        bpDict[tail1].SetJointStrength(continuousActions[++i]);
        bpDict[tail2].SetJointStrength(continuousActions[++i]);
        bpDict[frontLeftLeg].SetJointStrength(continuousActions[++i]);
        bpDict[frontRightLeg].SetJointStrength(continuousActions[++i]);
        bpDict[backLeftLeg].SetJointStrength(continuousActions[++i]);
        bpDict[backRightLeg].SetJointStrength(continuousActions[++i]);
        bpDict[head].SetJointStrength(continuousActions[++i]);

        // エネルギーの消費
        float energyPenalty = energyConsumptionRate * Time.fixedDeltaTime;
        energy -= energyPenalty;
        AddReward(-energyPenalty * 0.001f);

        // 空腹度とストレスの増加
        hunger = Mathf.Min(hunger + hungerIncreaseRate * Time.fixedDeltaTime, maxHunger);
        stress = Mathf.Min(stress + stressIncreaseRate * Time.fixedDeltaTime, maxStress);

        // エネルギーが尽きたら休憩状態へ遷移
        if (energy <= 0)
        {
            energy = 0;
            ChangeState(new RestState(this));
        }
    }

    private void UpdateOrientationObjects()
    {
        if (currentState is ForageState)
        {
            // 最も近い餌をターゲットに設定
            foodObject = GetClosestFoodObject();

            if (foodObject != null)
            {
                m_OrientationCube.UpdateOrientation(hip, foodObject);
                m_DirectionIndicator?.MatchOrientation(m_OrientationCube.transform);
            }
        }
        else if (currentState is PlayState)
        {
            // 最も近いおもちゃをターゲットに設定
            toyObject = GetClosestToyObject();

            if (toyObject != null)
            {
                m_OrientationCube.UpdateOrientation(hip, toyObject);
                m_DirectionIndicator?.MatchOrientation(m_OrientationCube.transform);
            }
        }
        else if (currentState is LookAtCameraState)
        {
            // カメラの方向を向く
            if (cameraTransform != null)
            {
                m_OrientationCube.UpdateOrientation(hip, cameraTransform);
                m_DirectionIndicator?.MatchOrientation(m_OrientationCube.transform);
            }
        }
        else
        {
            m_OrientationCube.transform.rotation = Quaternion.LookRotation(randomWalkDirection);
            m_DirectionIndicator?.MatchOrientation(m_OrientationCube.transform);
        }
    }

    private void FixedUpdate()
    {
        UpdateOrientationObjects();
        currentState?.Execute();
        CheckIfFallen();
        CalculateRewards();
    }

    private void CheckIfFallen()
    {
        float hipRotationZ = hip.eulerAngles.z;
        if (hipRotationZ > 180f) hipRotationZ -= 360f;

        if (Mathf.Abs(hipRotationZ) > 45f)
        {
            if (!isFallen)
            {
                isFallen = true;
                AddReward(-1f);
            }
        }
        else
        {
            if (isFallen)
            {
                isFallen = false;
                AddReward(0.5f);
            }
        }
    }

    private Vector3 GetAvgVelocity()
    {
        Vector3 velSum = Vector3.zero;
        foreach (var item in m_JdController.bodyPartsList)
        {
            velSum += item.rb.velocity;
        }
        return velSum / m_JdController.bodyPartsList.Count;
    }

    private void CalculateRewards()
    {
        if (!isFallen)
        {
            var cubeForward = m_OrientationCube.transform.forward;
            var avgVel = GetAvgVelocity();
            float speedReward = Vector3.Dot(cubeForward, avgVel);
            AddReward(speedReward * 0.001f);
        }
        else
        {
            AddReward(-0.001f);
        }
    }

    public void ChangeState(State newState)
    {
        currentState?.Exit();
        currentState = newState;
        stateStartTime = Time.time;
        currentState?.Enter();
    }

    public float TimeInState()
    {
        return Time.time - stateStartTime;
    }

    public void SetRandomWalkDirection()
    {
        randomWalkDirection = new Vector3(
            Random.Range(-1f, 1f),
            0,
            Random.Range(-1f, 1f)
        ).normalized;
    }

    public Transform GetClosestFoodObject()
    {
        if (itemSpawner == null || itemSpawner.SpawnedFoodItems == null)
            return null;

        Transform closestFood = null;
        float minDistance = Mathf.Infinity;

        foreach (var food in itemSpawner.SpawnedFoodItems)
        {
            if (food == null) continue;

            float distance = Vector3.Distance(hip.position, food.transform.position);
            if (distance < minDistance)
            {
                minDistance = distance;
                closestFood = food.transform;
            }
        }
        return closestFood;
    }

    public Transform GetClosestToyObject()
    {
        if (itemSpawner == null || itemSpawner.SpawnedToyItems == null)
            return null;

        Transform closestToy = null;
        float minDistance = Mathf.Infinity;

        foreach (var toy in itemSpawner.SpawnedToyItems)
        {
            if (toy == null) continue;

            float distance = Vector3.Distance(hip.position, toy.transform.position);
            if (distance < minDistance)
            {
                minDistance = distance;
                closestToy = toy.transform;
            }
        }
        return closestToy;
    }

    public void HandleCollision(Collision collision, GameObject bodyPart)
    {
        if (collision.gameObject.CompareTag("Food"))
        {
            Debug.Log("エージェントが餌に接触しました。");  // ログ出力

            if (itemSpawner != null)
            {
                itemSpawner.RemoveFood(collision.gameObject);
            }
            else
            {
                Debug.LogError("ItemSpawnerが設定されていないため、餌を削除できません。");
            }

            AddReward(1.0f);
            energy = Mathf.Min(energy + 20f, maxEnergy);
            hunger = Mathf.Max(hunger - hungerDecreaseAmount, 0f);
            foodObject = GetClosestFoodObject();

            if (stress >= maxStress * 0.5f)
            {
                ChangeState(new PlayState(this));
            }
            else
            {
                ChangeState(new IdleState(this));
            }
        }
        else if (collision.gameObject.CompareTag("Toy"))
        {
            Debug.Log("エージェントがおもちゃに接触しました。");  // ログ出力
            AddReward(0.5f);
            stress = Mathf.Max(stress - stressDecreaseAmount, 0f);
            toyObject = GetClosestToyObject();
            ChangeState(new IdleState(this));
        }
    }

    // 状態クラスの定義
    public abstract class State
    {
        protected QuadrupedAgent agent;

        protected State(QuadrupedAgent agent)
        {
            this.agent = agent;
        }

        public abstract void Enter();
        public abstract void Execute();
        public abstract void Exit();
    }

    // IdleStateの実装
    public class IdleState : State
    {
        public IdleState(QuadrupedAgent agent) : base(agent) { }

        public override void Enter() { }

        public override void Execute()
        {
            // ランダムに LookAtCameraState に遷移
            if (Random.value < agent.lookAtCameraChance)
            {
                agent.ChangeState(new LookAtCameraState(agent));
                return;
            }

            if (agent.hunger >= agent.maxHunger * 0.7f)
            {
                agent.ChangeState(new ForageState(agent));
            }
            else if (agent.stress >= agent.maxStress * 0.5f)
            {
                agent.ChangeState(new PlayState(agent));
            }
            else if (agent.TimeInState() > agent.idleDuration)
            {
                agent.ChangeState(new WalkState(agent));
            }
        }

        public override void Exit() { }
    }

    // WalkStateの実装
    public class WalkState : State
    {
        public WalkState(QuadrupedAgent agent) : base(agent) { }

        public override void Enter()
        {
            agent.SetRandomWalkDirection();
        }

        public override void Execute()
        {
            // ランダムに LookAtCameraState に遷移
            if (Random.value < agent.lookAtCameraChance)
            {
                agent.ChangeState(new LookAtCameraState(agent));
                return;
            }

            if (agent.hunger >= agent.maxHunger * 0.7f)
            {
                agent.ChangeState(new ForageState(agent));
            }
            else if (agent.stress >= agent.maxStress * 0.5f)
            {
                agent.ChangeState(new PlayState(agent));
            }
            else if (agent.TimeInState() > 10f)
            {
                agent.ChangeState(new IdleState(agent));
            }
        }

        public override void Exit() { }
    }

    // ForageStateの実装
    public class ForageState : State
    {
        public ForageState(QuadrupedAgent agent) : base(agent) { }

        public override void Enter()
        {
            // 最も近い餌をターゲットに設定
            agent.foodObject = agent.GetClosestFoodObject();

            if (agent.foodObject == null)
            {
                // 餌が存在しない場合、IdleStateに遷移
                agent.ChangeState(new IdleState(agent));
            }
        }

        public override void Execute()
        {
            if (agent.hunger < agent.maxHunger * 0.7f)
            {
                agent.ChangeState(new IdleState(agent));
            }
        }

        public override void Exit() { }
    }

    // PlayStateの実装
    public class PlayState : State
    {
        public PlayState(QuadrupedAgent agent) : base(agent) { }

        public override void Enter()
        {
            // 最も近いおもちゃをターゲットに設定
            agent.toyObject = agent.GetClosestToyObject();

            if (agent.toyObject == null)
            {
                // おもちゃが存在しない場合、IdleStateに遷移
                agent.ChangeState(new IdleState(agent));
            }
        }

        public override void Execute() { }

        public override void Exit() { }
    }

    // RestStateの実装
    public class RestState : State
    {
        public RestState(QuadrupedAgent agent) : base(agent) { }

        public override void Enter()
        {
            agent.stateStartTime = Time.time;
        }

        public override void Execute()
        {
            agent.energy = Mathf.Min(agent.energy + agent.energyRecoveryRate * Time.fixedDeltaTime, agent.maxEnergy);
            agent.stress = Mathf.Max(agent.stress - agent.stressDecreaseAmount * Time.fixedDeltaTime, 0f);

            if (agent.energy >= agent.maxEnergy || agent.TimeInState() > agent.restDuration)
            {
                agent.ChangeState(new IdleState(agent));
            }
        }

        public override void Exit() { }
    }

    // LookAtCameraStateの実装
    public class LookAtCameraState : State
    {
        public LookAtCameraState(QuadrupedAgent agent) : base(agent) { }

        public override void Enter()
        {
            agent.stateStartTime = Time.time;
            agent.cameraTransform = Camera.main?.transform;
        }

        public override void Execute()
        {
            if (agent.cameraTransform != null)
            {
                Vector3 directionToCamera = agent.cameraTransform.position - agent.head.position;
                Quaternion targetRotation = Quaternion.LookRotation(directionToCamera, Vector3.up);
                agent.head.rotation = Quaternion.Slerp(
                    agent.head.rotation, targetRotation, Time.deltaTime * agent.headTurnSpeed);
            }

            if (agent.TimeInState() > agent.lookAtCameraDuration)
            {
                agent.ChangeState(new IdleState(agent));
            }
        }

        public override void Exit() { }
    }
}