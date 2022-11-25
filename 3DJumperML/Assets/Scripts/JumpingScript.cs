using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using Unity.VisualScripting;
using Unity.VisualScripting.Dependencies.Sqlite;
using UnityEngine;
using static Unity.VisualScripting.Member;

public class JumpingScript : MonoBehaviour
{
    Rigidbody leg_rigidbody;
    Rigidbody rotor_binder;
    public Transform agent_transform;
    float turn_rate_low = 0.0f;
    float turn_rate_high = 10.0f;
    float turn_rate = 10.0f;
    //public float turn_rate = 0.0f;

    float turn_rate1 = 0.0f;
    float turn_rate2 = 0.0f;
    float turn_rate3 = 0.0f;
    float turn_rate4 = 0.0f;

    public Transform rotor1;
    public Transform rotor2;
    public Transform rotor3;
    public Transform rotor4;

    public Transform rotor_holder;

    public Transform base_transform;
    public Rigidbody base_rigidbody;
    public ConfigurableJoint base_configurable_joint;

    public float gravity = -9.82f;

    float rotation_speed_low = 500;
    float rotation_speed_high = 2000;

    //List<float> spring_compress = new List<float>() { -1.38f, -1.05f };

    public float[] spring_compress = {-1.50f, -1.05f};



    void apply_rotor_force(Rigidbody agent_body, Transform rotor_transform, float multiplyer)
    {

        // Position is in world space, not local space -> transform local position into world space
        //Vector3 pos = rotor_transform.TransformPoint(new Vector3(agent_body.position.x, agent_body.position.y, rotor_transform.position.z));
        Vector3 pos = transform.TransformPoint(new Vector3(0.0f, rotor_holder.position.y, 0.0f));
        Vector3 force_pos = rotor_transform.TransformDirection(transform.up);
        agent_body.AddForceAtPosition(force_pos * multiplyer, pos);
        //sphere.transform.position = pos;
    }

    // Start is called before the first frame update
    void Start()
    {
        Physics.gravity = new Vector3(0, gravity, 0);
        //Fetch the Rigidbody from the GameObject with this script attached
        leg_rigidbody = GetComponent<Rigidbody>();
    }

    private void Update()
    {
        rotor1.Rotate(Vector3.up * (500 + turn_rate1 / turn_rate_high * 2000 - 500) * Time.deltaTime, Space.Self);
        rotor2.Rotate(Vector3.up * (500 + turn_rate2 / turn_rate_high * 2000 - 500) * Time.deltaTime, Space.Self);
        rotor3.Rotate(Vector3.up * (500 + turn_rate3 / turn_rate_high * 2000 - 500) * Time.deltaTime, Space.Self);
        rotor4.Rotate(Vector3.up * (500 + turn_rate4 / turn_rate_high * 2000 - 500) * Time.deltaTime, Space.Self);
    }

    // FixedUpdate is called once per frame, FixedUpdate is 50fps like the physics simulation 
    // Use Time.fixedDeltaTime to access this value.
    void FixedUpdate()
    {
        if (Input.GetKey(KeyCode.RightArrow))
        {
            turn_rate1 = turn_rate;
            //leg_rigidbody.AddForceAtPosition(leg_rigidbody.rotation * Vector3.back * turn_rate1, )
            apply_rotor_force(leg_rigidbody, rotor1, turn_rate1);
        }
        else
        {
            turn_rate1 = 0.0f;
        }
        if (Input.GetKey(KeyCode.UpArrow))
        {
            turn_rate2 = turn_rate;
            apply_rotor_force(leg_rigidbody, rotor2, turn_rate2);
        }
        else
        {
            turn_rate2 = 0.0f;
        }
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            turn_rate3 = turn_rate;
            apply_rotor_force(leg_rigidbody, rotor3, turn_rate3);
        }
        else
        {
            turn_rate3 = 0.0f;
        }
        if (Input.GetKey(KeyCode.DownArrow))
        {
            turn_rate4 = turn_rate;
            apply_rotor_force(leg_rigidbody, rotor4, turn_rate4);
        }
        else
        {
            turn_rate4 = 0.0f;
        }

        // Jumping motion
        //Apply a force to this Rigidbody in direction of this GameObjects up axis
        if (Input.GetKey(KeyCode.Space))
        {
            base_configurable_joint.connectedAnchor = new Vector3(0.0f, spring_compress[0], 0.0f);
        }

        if (Input.GetKey(KeyCode.R))
        {
            base_configurable_joint.connectedAnchor = new Vector3(0.0f, spring_compress[1], 0.0f);
        }


        //leg_rigidbody.AddForce(-transform.up * compress_thrust);


        //int leftright = 0;
        //int updown = 0;


    }
}
