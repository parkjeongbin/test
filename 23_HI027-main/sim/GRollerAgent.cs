using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
//mlAgent 사용시 포함해야 됨

public class gRollerAgent : Agent
{
    Rigidbody rBody;
    void Start()
    {
        rBody = GetComponent<Rigidbody>();
    }

    public Transform Target;

    public GameObject viewModel = null;

    public override void OnEpisodeBegin()
    {
        //새로운 애피소드 시작시, 다시 에이전트의 포지션의 초기화
        // If the Agent fell, zero its momentum
       

        //타겟의 위치는 에피소드 시작시 랜덤하게 변경된다.
        // Move the target to a new spot
        float rx = 0;
        float rz = 0;

        float maxX = 50f; // X축 최대 값 (맵 가로 크기의 절반)
	float maxZ = 75f; // Z축 최대 값 (맵 세로 크기의 절반)

	rx = Random.Range(-maxX, maxX);
	rz = Random.Range(-maxZ, maxZ);

        Target.localPosition = new Vector3(rx,
                                           5f,
                                           rz);
    }


    /// <summary>
    /// 강화학습을 위한, 강화학습을 통한 행동이 결정되는 곳
    /// </summary>
    public float forceMultiplier = 10;
    //float m_LateralSpeed = 1.0f;
    float m_ForwardSpeed = 1.0f;

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // Rewards
        float distanceToTarget = Vector3.Distance(this.transform.localPosition, Target.localPosition);

        //타겟을 찻을시 리워드점수를 주고, 에피소드를 종료시킨다.
        // Reached target
        if (distanceToTarget < 1.42f)
        {
            SetReward(1.0f);
            EndEpisode();
        }

        MoveAgent(actionBuffers.DiscreteActions);
    }
        public void MoveAgent(ActionSegment<int> act)
    {
        var dirToGo = Vector3.zero;
        var rotateDir = Vector3.zero;

        var forwardAxis = act[0];
        var rotateAxis = act[1];

        switch (forwardAxis)
        {
            case 1:
                dirToGo = transform.forward * m_ForwardSpeed;
                break;

        }

        switch (rotateAxis)
        {
            case 1:
                rotateDir = transform.up * -1f;
                break;
            case 2:
                rotateDir = transform.up * 1f;
                break;
        }

        transform.Rotate(rotateDir, Time.deltaTime * 100f);
        rBody.AddForce(dirToGo * forceMultiplier, ForceMode.VelocityChange);
    }

    /// <summary>
    /// 해당 함수는 직접조작 혹은 규칙성있는 코딩으로 조작시키기 위한 함수
    /// </summary>
    /// <param name="actionsOut"></param>

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActionsOut = actionsOut.DiscreteActions;
        discreteActionsOut.Clear();
        //forward
        if (Input.GetKey(KeyCode.W))
        {
            discreteActionsOut[0] = 1;
        }

        //rotate
        if (Input.GetKey(KeyCode.A))
        {
            discreteActionsOut[1] = 1;
        }
        if (Input.GetKey(KeyCode.D))
        {
            discreteActionsOut[1] = 2;
        }
    }
}
