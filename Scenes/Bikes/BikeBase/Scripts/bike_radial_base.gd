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

func _ready() -> void:
		assert(front_wheel != null, "ERROR: 'front_wheel' must not be null!")
		assert(rear_wheel != null, "ERROR: 'rear_wheel' must not be null!")
		
		wheels = [front_wheel, rear_wheel]
		wheels[0].setup_wheel(f_settings_dict)
		wheels[1].setup_wheel(r_settings_dict)


func _process(delta: float) -> void:
	DebugDraw3D.draw_box(to_global(center_of_mass), global_basis, Vector3(0.1, 0.1, 0.1), Color.GREEN_YELLOW, true) # draw center of mass
	steering_input = Input.get_axis("SteerLeft", "SteerRight")
	lean_input = Input.get_axis("LeanLeft", "LeanRight")
	pedal_input = Input.get_action_strength("Pedal")
	front_brake_input = Input.get_action_strength("FrontBrake")
	rear_brake_input = Input.get_action_strength("RearBrake")
	
	if Input.is_action_just_pressed("AddTorque"):
		apply_torque(Vector3(0, 0, 100.0))
	
	$CameraPivot/Camera3D/Control/Label.text = "Speed: " + str(snapped(linear_velocity.length(), 0.1)) + "\nLean: " + str(snapped(rad_to_deg(-global_rotation.z), 1))


func _physics_process(delta: float) -> void:
	var steering_output : float
	
	# get the average normal vector
	var total_normal_vector := Vector3.ZERO
	if wheels[0].is_colliding(): total_normal_vector += wheels[0].get_collision_normal()
	if wheels[1].is_colliding(): total_normal_vector += wheels[1].get_collision_normal()
	
	# don't add roll torque if not on ground
	if total_normal_vector.is_zero_approx():
		steering_output = 0.0 # TODO blend in front wheel input in air
	else:
		var average_normal_vector := total_normal_vector.normalized()
		steering_output = roll_pid(steering_input, average_normal_vector)
	
	
	for wheel in wheels:
		if wheel.is_colliding():
			var velocity_at_contact = _get_point_velocity(wheel.get_collision_point())
			var impulse_vector = wheel.get_impulse(pedal_input, steering_output, front_brake_input, rear_brake_input, velocity_at_contact, delta)
			var force_pos_offset := wheel.get_collision_point() - global_position
			apply_impulse(impulse_vector, force_pos_offset)


# Helper function to get velocity at point
func _get_point_velocity(point: Vector3) -> Vector3:
	return linear_velocity + angular_velocity.cross(point - global_position)


#region Steering System
# Takes maps input as a target lean angle between 0 to +/- max_lean_angle
# Uses a PID feedback controller to add roll torque | TODO use a more realistic control vector e.g. moving rider weight
# Calculates steering angle based on lean angle, speed, desired turn radius

# takes steering input and uses a PID control to add torque to reach target lean angle
# TODO consider "If you want faster convergence, add anticipated steady-state roll requirement when δ changes quickly; often not needed initially."
func roll_pid(steering_input: float, normal_vector: Vector3) -> float:
	
	# PID Constants
	var Kp := 0.0
	var Ki := 0.0
	var Kd := 0.0
	
	# Calculate target lean angle and lean angle error
	var max_lean_angle := 25.0
	var target_lean_angle := steering_input * max_lean_angle
	
	var lean_axis = -global_basis.z
	var current_lean_angle := rad_to_deg(normal_vector.signed_angle_to(global_basis.y, lean_axis))
	var lean_angle_error := target_lean_angle - current_lean_angle
	
	var torque = (Kp * lean_angle_error) - (Kd * angular_velocity.z) # Not including I right now
	
	shift_center_of_mass(steering_input, 0.2)
	
	return 0.0 #get_steering_angle(target_lean_angle)


# Calculates a steering input [-1, 1] based on desired turn radius.
func get_steering_angle(lean_target: float) -> float:
	
	if abs(linear_velocity.z) < 0.5:
		return (lean_target / 25.0) * 0.6
	else:
		var velocity := -linear_velocity.z
		var caster_cos := wheels[0].steering_axis.dot(global_basis.y)
		var denominator := (velocity ** 2) * caster_cos
		var wheelbase := wheels[0].global_position.distance_to(wheels[1].global_position)
	
		var computed_steering_angle := (wheelbase * tan(deg_to_rad(lean_target)) * 9.8) / denominator
		computed_steering_angle = asin(clamp(computed_steering_angle, -1, 1))
		computed_steering_angle = rad_to_deg(computed_steering_angle)
	
		var steering_output = clamp(computed_steering_angle/wheels[0].max_steering_angle, -1, 1) # map to steering_input range [-1, 1]
	
		return steering_output


# Shifts the center of mass between +/- max_range on the x axis using an input between [-1, 1]
func shift_center_of_mass(input: float, max_range: float) -> void:
	const com_origin := Vector3(0, 0.55, 0.0)
	center_of_mass = com_origin + Vector3(input * max_range, 0, 0)
	
#endregion
