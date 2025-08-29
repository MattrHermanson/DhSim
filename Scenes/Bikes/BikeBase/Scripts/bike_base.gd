extends RigidBody3D
class_name BikeBase

enum WheelType {
	FRONT,
	REAR,
}

#region Wheel Settings and Settings Dictionaries
@export_category("Front Wheel")
@export var front_wheel : BikeWheelBase
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
@export var rear_wheel : BikeWheelBase
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

var wheels: Array[BikeWheelBase]

var pedal_input := 0.0
var steering_input := 0.0
var front_brake_input := 0.0
var rear_brake_input := 0.0


func _ready() -> void:
		assert(front_wheel != null, "ERROR: 'front_wheel' must not be null!")
		assert(rear_wheel != null, "ERROR: 'rear_wheel' must not be null!")
		
		wheels = [front_wheel, rear_wheel]
		wheels[0].setup_wheel(f_settings_dict)
		wheels[1].setup_wheel(r_settings_dict)


func _process(delta: float) -> void:
	DebugDraw3D.draw_box((global_position + center_of_mass + Vector3(0, 0, 0)), global_transform.basis, Vector3(0.1, 0.1, 0.1), Color.GREEN_YELLOW, true) # draw center of mass
	steering_input = Input.get_axis("SteerLeft", "SteerRight")
	pedal_input = Input.get_action_strength("Pedal")
	front_brake_input = Input.get_action_strength("FrontBrake")
	rear_brake_input = Input.get_action_strength("RearBrake")


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
var max_lean_angle := 25.0
var last_error := 0.0
var integration_stored := 0.0
var P := 5.0
var I := 1.0
var D := 1.0

# takes steering_input as a "lean input" and converts that to a steering input for the front wheel while applying torque to lean
func interpolate_steering(steering_input: float, delta: float) -> float:
	var roll_angle := rad_to_deg(global_rotation.z)
	if is_zero_approx(roll_angle):
		roll_angle = 0.0
	
	var target_lean_angle := steering_input * max_lean_angle
	var error := -target_lean_angle - roll_angle
	
	integration_stored = integration_stored + (-error * delta)
	
	var p_term := -error * P
	var i_term := integration_stored * I
	var d_term := ((-error - last_error) * delta) * D
	
	var lean_torque := p_term + i_term + d_term
	last_error = error
	
	#print("lean t ", lean_torque, " p_term ", p_term, " d_term ", d_term)
	
	apply_torque(Vector3(0, 0, lean_torque))
	
	# TODO think of a smarter way to add steering, maybe make it scale with speed
	return steering_input * 0.5
