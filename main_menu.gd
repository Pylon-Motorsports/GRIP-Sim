extends Node


const timeoutTime = 10
var settingGas = false
var settingBrake = false
var settingLeft = false
var settingRight = false
var joy_names = [
    "joy_0_minus",
    "joy_1_minus",
    "joy_2_minus",
    "joy_3_minus",
    "joy_4_minus",
    "joy_5_minus",
    "joy_6_minus",
    "joy_7_minus",
    "joy_8_minus",
    "joy_9_minus",
    "joy_0_plus",
    "joy_1_plus",
    "joy_2_plus",
    "joy_3_plus",
    "joy_4_plus",
    "joy_5_plus",
    "joy_6_plus",
    "joy_7_plus",
    "joy_8_plus",
    "joy_9_plus"]


func _ready():
    var eventGas = InputMap.action_get_events("pedal_gas").get(0)
    var eventBrake = InputMap.action_get_events("pedal_brake").get(0)
    var eventLeft = InputMap.action_get_events("steer_left").get(0)
    
    $ButtonMapGas.pressed.connect(_button_gas)
    $ButtonMapBrake.pressed.connect(_button_brake)
    $ButtonMapLeft.pressed.connect(_button_left)
    $ButtonMapRight.pressed.connect(_button_right)
    
    _set_control_labels()


func _set_control_labels():
    var eventGas = InputMap.action_get_events("pedal_gas").get(0)
    var eventBrake = InputMap.action_get_events("pedal_brake").get(0)
    var eventLeft = InputMap.action_get_events("steer_left").get(0)
    var eventRight = InputMap.action_get_events("steer_right").get(0)
    
    get_tree().root.get_node("Control/ButtonMapGas/RichTextLabel").text = Input.get_joy_name(eventGas.device) + " " + str(eventGas.axis)
    get_tree().root.get_node("Control/ButtonMapBrake/RichTextLabel").text = Input.get_joy_name(eventBrake.device) + " " + str(eventBrake.axis)
    get_tree().root.get_node("Control/ButtonMapLeft/RichTextLabel").text = Input.get_joy_name(eventLeft.device) + " " + str(eventLeft.axis)
    get_tree().root.get_node("Control/ButtonMapRight/RichTextLabel").text = Input.get_joy_name(eventRight.device) + " " + str(eventRight.axis)


func _input(event):
    var max_strength = 0;
    var max_strength_index = -1;
    for i in joy_names.size():
        var strength = Input.get_action_strength(joy_names[i])
        if strength > max_strength:
            max_strength = strength
            max_strength_index = i
    if max_strength_index >= 0:
        var new_event = InputMap.action_get_events(joy_names[max_strength_index])[0]
        # TODO: remove duplicate code
        if settingGas:
            InputMap.action_erase_events("pedal_gas")
            InputMap.action_add_event("pedal_gas", new_event)
            settingGas = false
        if settingBrake:
            InputMap.action_erase_events("pedal_brake")
            InputMap.action_add_event("pedal_brake", new_event)
            settingBrake = false
        if settingLeft:
            InputMap.action_erase_events("steer_left")
            InputMap.action_add_event("steer_left", new_event)
            settingLeft = false
        if settingRight:
            InputMap.action_erase_events("steer_right")
            InputMap.action_add_event("steer_right", new_event)
            settingRight = false
        _set_control_labels()


# TODO: GUI indication of setting

func _button_gas():
    settingGas = true
    await get_tree().create_timer(timeoutTime).timeout
    settingGas = false


func _button_brake():
    settingBrake = true
    await get_tree().create_timer(timeoutTime).timeout
    settingBrake = false


func _button_left():
    settingLeft = true
    await get_tree().create_timer(timeoutTime).timeout
    settingLeft = false


func _button_right():
    settingRight = true
    await get_tree().create_timer(timeoutTime).timeout
    settingRight = false
