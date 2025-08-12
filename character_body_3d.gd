extends CharacterBody3D

var speed = 0.5
# var velocity = Vector3.ZERO

func _physics_process(_delta):
    velocity = Vector3.ZERO
    #var input_dir = Vector3.ZERO
    var input_vector = Input.get_vector("steer_left", "steer_right", "pedal_brake", "pedal_gas")
    #velocity = input_vector * speedf
    #velocity = Vector3(0, 0, input_vector.y * speed)
    translate_object_local(Vector3(0, 0, input_vector.y * speed))
    rotate_object_local(Vector3(0,1, 0), -input_vector.x / 36.0)
    #rotate_y(-input_vector.x / 360.0)
    #rotation = Vector3(0, input_vector.x, 0)
    #if Input.is_action_pressed("move_forward"):
        #input_dir.z -= 1
    #if Input.is_action_pressed("move_back"):
       #input_dir.z += 1
    #if Input.is_action_pressed("move_left"):
        #input_dir.x -= 1
    #if Input.is_action_pressed("move_right"):
        #input_dir.x += 1
#
    ## Normalize input_dir if using multiple directions for consistent speed
    #input_dir = input_dir.normalized()

    #velocity.x = velocity.x * speed
    #velocity.z = velocity.z * speed

    # Apply gravity if needed
    # velocity.y -= gravity * delta

    move_and_slide()
