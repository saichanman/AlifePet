using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;

[RequireComponent(typeof(JointDriveController))]
public class RecoveryAgent : Agent
{
    [Header("Body Parts")]
    public Transform hip;
    public Transform body;
    public Transform head;

    public Transform frontLeftLegUpper, frontLeftLegLower, frontLeftFoot;
    public Transform frontRightLegUpper, frontRightLegLower, frontRightFoot;
    public Transform backLeftLegUpper, backLeftLegLower, backLeftFoot;
    public Transform backRightLegUpper, backRightLegLower, backRightFoot;

    private JointDriveController m_JdController;
    private Vector3 initialPosition;
    private Quaternion initialRotation;

    //The indicator graphic gameobject that points towards the target
    DirectionIndicator m_DirectionIndicator;
    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    OrientationCubeController m_OrientationCube;

    [Header("Recovery Settings")]
    public float uprightThreshold = 0.8f; // HIPのUP方向の一致度しきい値（1.0に近いほど直立）
    public int requiredFeetOnGround = 4; // 接地しているFOOTの必要数
    public float maxRecoveryTime = 10f; // 起き上がりの制限時間（秒）

    private float recoveryTimer = 0f; // 現在の復帰時間

    private float allFeetGroundedTime = 0f; // 全ての足が同時に接地している時間
    private float groundedTimeThreshold = 2.0f; // 起立と判定するための接地時間（秒）

    public override void Initialize()
    {
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();
        m_JdController = GetComponent<JointDriveController>();
        SetupBodyParts();

        // 初期位置と回転を保存
        initialPosition = transform.position;
        initialRotation = transform.rotation;
    }

    private void SetupBodyParts()
    {
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

    public override void OnEpisodeBegin()
    {
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }

        // ランダムな初期状態にエージェントを配置
        hip.position = initialPosition + new Vector3(0, 2.0f, 0);
        // HIPのランダムな姿勢設定
        float randomX = Random.value > 0.5f ? Random.Range(90f, 180f) : Random.Range(-180f, -90f); // X軸は絶対値90度以上
        float randomZ = Random.value > 0.5f ? Random.Range(90f, 180f) : Random.Range(-180f, -90f); // Z軸も同様
        float randomY = Random.Range(0, 360f); // Y軸は自由に回転

        hip.rotation = Quaternion.Euler(0, randomY, randomZ);

        // 初期化処理
        allFeetGroundedTime = 0f;

        // その他のリセット処理
        recoveryTimer = 0f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // HIPのUP方向を観測
        Vector3 hipUp = hip.up; // HIPの上方向ベクトル
        sensor.AddObservation(hipUp);

        // 各ボディパーツの観測データを追加
        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            sensor.AddObservation(bodyPart.groundContact.touchingGround ? 1.0f : 0.0f); // 地面接触状態
            sensor.AddObservation(bodyPart.rb.velocity); // 各パーツの速度
        }
    }

    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        //GROUND CHECK
        sensor.AddObservation(bp.groundContact.touchingGround); // Is this bp touching the ground

        if (bp.rb.transform != body)
        {
            sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
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

    public override void OnActionReceived(ActionBuffers actions)
    {
        var bpDict = m_JdController.bodyPartsDict;
        var continuousActions = actions.ContinuousActions;
        int i = -1;

        // 各関節のターゲット回転を設定
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

        // 各関節の強度を更新
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

        var cubeForward = m_OrientationCube.transform.forward;

        EvaluateRecovery();
    }

    private void EvaluateRecovery()
    {
        // HIPのUP方向とワールドのUP方向の一致度
        float uprightReward = Vector3.Dot(hip.up, Vector3.up); // 1に近いほど直立
        AddReward(Mathf.Clamp01(uprightReward * 0.01f)); // スケーリングした報酬を加算

        // FOOTパーツの接地状態を確認
        bool allFeetGrounded = frontLeftFoot.GetComponent<GroundContact>().touchingGround &&
                            frontRightFoot.GetComponent<GroundContact>().touchingGround &&
                            backLeftFoot.GetComponent<GroundContact>().touchingGround &&
                            backRightFoot.GetComponent<GroundContact>().touchingGround;

        // 全ての足が接地している場合にタイマーを増加、そうでなければリセット
        if (allFeetGrounded)
        {
            allFeetGroundedTime += Time.fixedDeltaTime;
        }
        else
        {
            allFeetGroundedTime = 0f;
        }

        // 全ての足が一定時間以上接地している場合に起立成功と判定
        if (allFeetGroundedTime >= groundedTimeThreshold && uprightReward > uprightThreshold)
        {
            AddReward(1.0f); // 起立成功時の報酬
            EndEpisode();
        }

        // 時間経過による失敗判定
        recoveryTimer += Time.fixedDeltaTime;
        if (recoveryTimer > maxRecoveryTime)
        {
            AddReward(-1.0f); // 時間切れのペナルティ
            EndEpisode();
        }
    }


}