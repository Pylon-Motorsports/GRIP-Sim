extends CharacterBody3D

var rpm = 0.0
const max_speed = 13.0
const min_rpm = 0.0
const max_rpm = 5000.0
const max_brake_force_per_second = 2000.0
const min_gas_force_per_second = 300.0
const max_gas_force_per_second = 1000.0
const max_engine_brake_force_per_second = 500.0
const max_rotate_per_second = 0.02

func _physics_process(delta):
    var forward_velocity = velocity.z
    
    var steer = Input.get_axis("steer_left", "steer_right")
    var brake = Input.get_action_strength("pedal_brake")
    var gas = Input.get_action_strength("pedal_gas")
    
    if (0 != forward_velocity):
        rotate_object_local(Vector3(0, 1, 0), -steer * max_rotate_per_second)
    
    var brake_force = brake * max_brake_force_per_second * delta
    var gas_force = (min_gas_force_per_second + ((max_gas_force_per_second - min_gas_force_per_second) * delta * (rpm / max_rpm))) * gas
    var engine_brake_force = rpm * (1.0 - gas) * max_engine_brake_force_per_second * delta
    
    var forward_force = gas_force - brake_force - engine_brake_force
    
    rpm += forward_force
    if (rpm < min_rpm):
        rpm = min_rpm
    if (rpm > max_rpm):
        rpm = max_rpm
    
    velocity = (rpm * (max_speed / max_rpm)) * Vector3.BACK.rotated(Vector3(0, 1, 0), rotation.y)
    #velocity = -10 * Vector3.FORWARD.rotated(Vector3(0, 1, 0), rotation.y)
    
    move_and_slide()
