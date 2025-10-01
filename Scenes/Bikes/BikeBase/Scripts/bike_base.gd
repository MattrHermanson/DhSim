extends RigidBody3D
class_name BikeBase

enum WheelType {
	FRONT,
	REAR,
}

#region Wheel Settings and Settings Dictionaries
@export_category("Front Wheel")
@export var front_suspension_bar : RigidBody3D
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
@export var rear_suspension_bar : RigidBody3D
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

var suspension_bars: Array[RigidBody3D]
var wheels: Array[BikeRadialWheelBase]

var pedal_input := 0.0
var steering_input := 0.0
var lean_input := 0.0
var front_brake_input := 0.0
var rear_brake_input := 0.0

func _ready() -> void:
		assert(front_wheel != null, "ERROR: 'front_wheel' must not be null!")
		assert(rear_wheel != null, "ERROR: 'rear_wheel' must not be null!")
		
		suspension_bars = [front_suspension_bar, rear_suspension_bar]
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
	
	$Camera3D/Control/Label.text = "Speed: " + str(snapped(linear_velocity.length(), 0.1)) + " F: " + str(wheels[0].is_sliding) + " R: " + str(wheels[1].is_sliding) + "\nLean: " + str(snapped(rad_to_deg(-global_rotation.z), 1))

func _physics_process(delta: float) -> void:
	shift_cog(lean_input)
	stiffen_lean()
	for i in range(2):
		if wheels[i].is_colliding():
			var velocity_at_contact := _get_point_velocity(suspension_bars[i], wheels[0].get_collision_point())
			var force_vector = wheels[i].get_forces(pedal_input, interpolate_steering(steering_input, delta), front_brake_input, rear_brake_input, velocity_at_contact)
			var force_pos_offset := wheels[i].get_collision_point() - global_position
			suspension_bars[i].apply_force(force_vector, force_pos_offset)


# Helper function to get velocity at point
func _get_point_velocity(body: RigidBody3D, point: Vector3) -> Vector3:
	return body.linear_velocity + angular_velocity.cross(point - global_position)


# Steering Interpolation Variables
var max_lean_angle := 25.0 # degrees
var integration_stored := 0.0
var P := 0.0 #2.5
var I := 0.0 # Not using I
var D := 0.0 #2.0 

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
	correction_steering_angle = tanh(correction_steering_angle)#clamp(correction_steering_angle, -1, 1)
	
	#print("T:", target_lean_angle, " L:", lean_angle, " E:", error, " P:", p_term, " D:", d_term, " Correct Steer:", correction_steering_angle)
	
	#return correction_steering_angle
	return steering_input


# TODO quick implementation
var max_shift := 0.05 # meters

func shift_cog(lean_input: float) -> void:
	var x_shift = max_shift * lean_input
	var current_center_of_mass := get_center_of_mass()
	set_center_of_mass(Vector3(x_shift, current_center_of_mass.y, current_center_of_mass.z))


var lean_stiffness := 100.0

func stiffen_lean() -> void:
	apply_torque(Vector3(0.0, 0.0, -(angular_velocity.z * lean_stiffness)))
