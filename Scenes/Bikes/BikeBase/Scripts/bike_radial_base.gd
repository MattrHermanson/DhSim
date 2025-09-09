extends RigidBody3D
class_name BikeRadialBase

enum WheelType {
	FRONT,
	REAR,
}

#region Wheel Settings and Settings Dictionaries
@export_category("Front Wheel")
@export var front_wheel : BikeRadialWheelBase
@export_group("Front Wheel Settings")
@export var f_spring_strength := 6000.0
@export var f_spring_damping := 350.0
@export var f_wheel_radius := 0.36
@export var f_brake_force := 500.0
@export var f_head_tube_angle := 63.5
@export var f_max_steering_angle := 45.0

@onready var f_settings_dict = {
	"spring_strength" : f_spring_strength,
	"spring_damping" : f_spring_damping,
	"wheel_radius" : f_wheel_radius,
	"pedal_force" : 0.0,
	"brake_force" : f_brake_force,
	"head_tube_angle" : f_head_tube_angle,
	"max_steering_angle" : f_max_steering_angle,
	"wheel_type" : WheelType.FRONT,
}

@export_category("Rear Wheel")
@export var rear_wheel : BikeRadialWheelBase
@export_group("Rear Wheel Settings")
@export var r_spring_strength := 6000.0
@export var r_spring_damping := 350.0
@export var r_wheel_radius := 0.36
@export var r_pedal_force := 200.0
@export var r_brake_force := 500.0

@onready var r_settings_dict = {
	"spring_strength" : r_spring_strength,
	"spring_damping" : r_spring_damping,
	"wheel_radius" : r_wheel_radius,
	"pedal_force" : r_pedal_force,
	"brake_force" : r_brake_force,
	"head_tube_angle" : 0.0,
	"max_steering_angle" : 0.0,
	"wheel_type" : WheelType.REAR,
}
#endregion

var wheels: Array[BikeRadialWheelBase]

var pedal_input := 0.0
var steering_input := 0.0
var lean_input := 0.0
var front_brake_input := 0.0
var rear_brake_input := 0.0

var reset_position : Array[Vector3] = []
var reset_basis : Array[Basis] = []

func _ready() -> void:
		assert(front_wheel != null, "ERROR: 'front_wheel' must not be null!")
		assert(rear_wheel != null, "ERROR: 'rear_wheel' must not be null!")
		
		wheels = [front_wheel, rear_wheel]
		wheels[0].setup_wheel(f_settings_dict)
		wheels[1].setup_wheel(r_settings_dict)
		
		reset_position.append(global_position)
		reset_basis.append(global_basis)

func _process(delta: float) -> void:
	DebugDraw3D.draw_box(to_global(center_of_mass), global_basis, Vector3(0.1, 0.1, 0.1), Color.GREEN_YELLOW, true) # draw center of mass
	steering_input = Input.get_axis("SteerLeft", "SteerRight")
	lean_input = Input.get_axis("LeanLeft", "LeanRight")
	pedal_input = Input.get_action_strength("Pedal")
	front_brake_input = Input.get_action_strength("FrontBrake")
	rear_brake_input = Input.get_action_strength("RearBrake")
	if Input.is_action_just_pressed("Reset"):
		reset_bike()
	
	$Camera3D/Control/Label.text = "Speed: " + str(snapped(linear_velocity.length(), 0.1)) + " F: " + str(wheels[0].is_sliding) + " R: " + str(wheels[1].is_sliding) + "\nLean: " + str(snapped(rad_to_deg(-global_rotation.z), 1))

func _physics_process(delta: float) -> void:
	for wheel in wheels:
		if wheel.is_colliding():
			var velocity_at_contact = _get_point_velocity(wheel.get_collision_point())
			var force_vector = wheel.get_forces(pedal_input, interpolate_steering(steering_input, delta), front_brake_input, rear_brake_input, velocity_at_contact)
			var force_pos_offset := wheel.get_collision_point() - global_position
			apply_force(force_vector, force_pos_offset)


# Helper function to get velocity at point
func _get_point_velocity(point: Vector3) -> Vector3:
	return linear_velocity + angular_velocity.cross(point - global_position)


# Steering Interpolation Variables
var max_lean_angle := 25.0 # degrees
var integration_stored := 0.0
var P := 2.5
var I := 0.0 # Not using I
var D := 2.0

# auto steers towards a lean angle
func interpolate_steering(steering_input: float, delta: float) -> float:
	var lean_angle := rad_to_deg(-global_rotation.z) # lean angle in degrees
	if is_zero_approx(lean_angle):
		lean_angle = 0.0
	
	# get target lean angle and current error
	var target_lean_angle := steering_input * max_lean_angle
	var error := target_lean_angle - lean_angle
	
	#integration_stored = integration_stored + (error * delta) #I term not being used
	
	var p_term := error * P
	var i_term := integration_stored * I
	var d_term := rad_to_deg(-angular_velocity.z) * D
	
	var correction_steering_angle := (p_term + i_term + d_term) * -1
	
	correction_steering_angle /= 45.0 # maps steering degrees to steering input range (-1,1) TODO get max steering angle, don't hard code!!!
	correction_steering_angle = clamp(correction_steering_angle, -1, 1)
	
	return correction_steering_angle


func reset_bike() -> void:
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	
	global_position = reset_position[0]
	global_basis = reset_basis[0]
