using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class JumpingAgent : Agent
{
    public void Awake()
    {
        Academy.Instance.OnEnvironmentReset += EnvironmentReset;
    }

    Rigidbody leg_rigidbody;
    public Rigidbody base_rigidbody;
    
    public Transform agent_transform;
    public Transform goal_transform;
    public Transform rotor_holder;  // Set to one of the 4 rotor holders

    // Rotors are seen from the perspective of the jumping agent (it has eyes)
    public Transform rotor1; // Right
    public Transform rotor2; // Back
    public Transform rotor3; // Left
    public Transform rotor4; // Front

    public ConfigurableJoint base_configurable_joint;

    float gravity = -9.82f;
    float[] turn_rate = { 0.0f, 10.0f };
    float[] rotation_speed = { 500.0f, 2000.0f };
    //List<float> spring_compress = new List<float>() { -1.38f, -1.05f 
    float[] spring_compress = { -1.50f, -1.05f };
    float[] goal_Z_limits = { -9.0f, 9.0f };
    float[] goal_Y_limits = { 3.0f, 6.3f };

    // Start Positions
    Vector3 default_leg_pos;
    Vector3 default_leg_rot;
    Vector3 default_base_pos;
    Vector3 default_base_rot;

    // Start is called before the first frame update
    void Start()
    {
        Physics.gravity = new Vector3(0, gravity, 0);   // Set gravity for the environment
        //Fetch the Rigidbody from the GameObject with this script attached
        leg_rigidbody = GetComponent<Rigidbody>();
        default_leg_pos = transform.position;
        default_leg_rot = transform.eulerAngles;
        default_base_pos = base_rigidbody.transform.position;
        default_base_rot = base_rigidbody.transform.eulerAngles;
    }

    void apply_rotor_force(Rigidbody agent_body, Transform rotor_transform, float multiplyer)
    {
        Vector3 pos = transform.TransformPoint(new Vector3(0.0f, rotor_holder.position.y, 0.0f));
        Vector3 force_pos = rotor_transform.TransformDirection(transform.up);
        agent_body.AddForceAtPosition(force_pos * multiplyer, pos);
    }

    void update_rotor(Transform rotor, float turn_rate_motor)
    {
        rotor.Rotate(Vector3.up * (rotation_speed[0] + turn_rate_motor / turn_rate[1] * rotation_speed[1] - rotation_speed[0]) * Time.deltaTime, Space.Self);
    }

    void set_random_goal_position(Transform goal_transform)
    {
        Vector3 position = new Vector3(0.0f, Random.Range(goal_Y_limits[0], goal_Y_limits[1]), Random.Range(goal_Z_limits[0], goal_Z_limits[1]));
    }

    void EnvironmentReset()
    {
        // Set agent to defaultposition

        // If the Agent fell, zero its momentum for all rigidbodies (aka base and leg)
        leg_rigidbody.angularVelocity = Vector3.zero;
        leg_rigidbody.velocity = Vector3.zero;
        base_rigidbody.angularVelocity = Vector3.zero;
        base_rigidbody.velocity = Vector3.zero;

        // Reset the default position and rotation
        transform.localPosition.Set(default_leg_pos.x, default_leg_pos.y, default_leg_pos.z);
        transform.localEulerAngles.Set(default_leg_rot.x, default_leg_rot.y, default_leg_rot.z);
        base_rigidbody.transform.localPosition.Set(default_base_pos.x, default_base_pos.y, default_base_pos.z);
        base_rigidbody.transform.localEulerAngles.Set(default_base_rot.x, default_base_rot.y, default_base_rot.z);

        // Set a new goal position
        set_random_goal_position(goal_transform);
    }

    public override void OnEpisodeBegin()
    {
        EnvironmentReset();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Observations, size = 8
        // [0] -> Agent y position
        // [1] -> Agent z position
        // [2] -> Agent x euler angle (aka slope
        // [3] -> Agent y velocity
        // [4] -> Agent z veloity
        // [5] -> Goal y position
        // [6] -> Goal z position
        // [7] -> Spring contraction distance

        // Agent positions and rotation
        sensor.AddObservation(transform.localPosition.y);
        sensor.AddObservation(transform.localPosition.z);
        sensor.AddObservation(transform.localEulerAngles.x);

        // Agent velocity
        sensor.AddObservation(leg_rigidbody.velocity.y);
        sensor.AddObservation(leg_rigidbody.velocity.z);

        // Goal position
        sensor.AddObservation(goal_transform.position.y);
        sensor.AddObservation(goal_transform.position.z);

        // Spring Contraction Distance
        float spring_contraction = Vector3.Distance(transform.position, base_rigidbody.transform.position);
        sensor.AddObservation(spring_contraction);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // Actions, size = 3
        // [0] -> Right rotor aka rotor1
        // [1] -> Left rotor aka rotor3
        // [2] -> Spring contraction
        float turn_rate_rotor1 = actionBuffers.ContinuousActions[0];
        float turn_rate_rotor3 = actionBuffers.ContinuousActions[1];
        float spring_compress_action = actionBuffers.ContinuousActions[2];

        // Clamp actions to the values defined in the beginning (user should handle this on their own in python but this is a safety)

        // Clamp Motor Right and Left
        if (turn_rate_rotor1 < turn_rate[0])
        {
            turn_rate_rotor1 = turn_rate[0];
        }
        if (turn_rate_rotor1 > turn_rate[1])
        {
            turn_rate_rotor1 = turn_rate[1];
        }
        if (turn_rate_rotor3 < turn_rate[0])
        {
            turn_rate_rotor3 = turn_rate[0];
        }
        if (turn_rate_rotor3 > turn_rate[1])
        {
            turn_rate_rotor3 = turn_rate[1];
        }
        // Clamp Spring Contraction
        if (spring_compress_action < spring_compress[0])
        {
            spring_compress_action = spring_compress[0];
        }
        if (spring_compress_action > spring_compress[1])
        {
            spring_compress_action = spring_compress[1];
        }

        // Apply forces to both rotors
        apply_rotor_force(leg_rigidbody, rotor1, turn_rate_rotor1);
        apply_rotor_force(leg_rigidbody, rotor3, turn_rate_rotor3);

        // Update propeller animation
        update_rotor(rotor1, turn_rate_rotor1);
        update_rotor(rotor3, turn_rate_rotor3);

        // Compress spring
        base_configurable_joint.connectedAnchor = new Vector3(0.0f, spring_compress_action, 0.0f);

        // Reward is set in python based on observation.
        //SetReward(1.0f);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        //var continuousActionsOut = actionsOut.ContinuousActions;
        ActionSegment<float> continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = 0.0f;
        continuousActionsOut[1] = 0.0f;
        continuousActionsOut[2] = 0.0f;

        if (Input.GetKey(KeyCode.RightArrow))
        {
            continuousActionsOut[0] = turn_rate[1];
        }
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            continuousActionsOut[1] = turn_rate[1];
        }

        // Jumping motion
        //Apply a force to this Rigidbody in direction of this GameObjects up axis
        if (Input.GetKey(KeyCode.Space))
        {
            continuousActionsOut[2] = spring_compress[0];
        }
        if (Input.GetKey(KeyCode.R))
        {
            continuousActionsOut[2] = spring_compress[1];
        }
    }
}


