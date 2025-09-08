extends ShapeCast3D
class_name BikeShapeWheelBase

enum WheelType {
	FRONT,
	REAR,
}

# Wheel Settings
var spring_strength : float
var spring_damping : float
var wheel_radius : float
var pedal_force : float
var brake_force : float
var head_tube_angle : float
var max_steering_angle : float
var wheel_type : WheelType
@onready var wheel: Node3D = get_child(0) # reference to wheel mesh

# Internal Variables
var current_steering_angle := 0.0
var steering_axis : Vector3
var is_sliding := false
var is_setup := false

func _ready() -> void:
	# set target position
	target_position = Vector3.ZERO


func setup_wheel(settings_dict: Dictionary) -> void:
	spring_strength = settings_dict["spring_strength"]
	spring_damping = settings_dict["spring_damping"]
	wheel_radius = settings_dict["wheel_radius"]
	pedal_force = settings_dict["pedal_force"]
	brake_force = settings_dict["brake_force"]
	head_tube_angle = settings_dict["head_tube_angle"]
	max_steering_angle = settings_dict["max_steering_angle"]
	wheel_type = settings_dict["wheel_type"]
	set_steering_axis(deg_to_rad(head_tube_angle))
	
	# set shape
	var sphere = SphereShape3D.new()
	sphere.radius = wheel_radius
	sphere.margin = 0.04 # meters
	set_shape(sphere)
	
	is_setup = true


# Returns all forces that are generated from the wheel
func get_forces(closest_collision_index: int, pedal_input: float, steering_input: float, front_brake_input: float, rear_brake_input: float, velocity: Vector3) -> Vector3:
	if not is_setup:
		push_error("Wheel is not setup")
	
	var total_force_vector: Vector3
	var longitudinal_force_vector: Vector3
	var lateral_force_vector: Vector3
	
	# TEMP - TODO figure out getting friction from raycast
	var static_friction = 1.0
	var kinetic_friction = 0.7
	
	# Add all wheel forces
	total_force_vector += get_normal_force(closest_collision_index, velocity)
	
	# Add rear-only forces
	if wheel_type == WheelType.REAR:
		update_animation(velocity, 0.0)
		longitudinal_force_vector += get_pedal_force(closest_collision_index, pedal_input)
		longitudinal_force_vector += get_brake_force(closest_collision_index, rear_brake_input, velocity)
		lateral_force_vector += get_steering_force(closest_collision_index, velocity)
	
	# Add front-only forces
	if wheel_type == WheelType.FRONT:
		update_animation(velocity, steering_input)
		longitudinal_force_vector += get_brake_force(closest_collision_index, front_brake_input, velocity)
		lateral_force_vector += get_steering_force(closest_collision_index, velocity)
	
	#limit total force
	total_force_vector += apply_friction_circle(longitudinal_force_vector, lateral_force_vector, total_force_vector.length(), static_friction, kinetic_friction)
	
	if is_sliding:
		print("sliding")
	
	return total_force_vector


# TODO can't steering while in air because this only gets called when contacting the ground
func update_animation(velocity: Vector3, steering_input: float) -> void:
	# rotate wheel to velocity
	var forward_direction := -global_basis.z
	var forward_velocity := forward_direction.dot(velocity)
	wheel.rotate_x((-forward_velocity * get_process_delta_time()) / wheel_radius)
	
	# turn the wheel to steering_input
	var degrees_to_rotate := (max_steering_angle * steering_input) - current_steering_angle
	current_steering_angle += degrees_to_rotate
	#global_basis = global_basis.rotated(steering_axis, deg_to_rad(-degrees_to_rotate))
	rotate_y(deg_to_rad(-degrees_to_rotate))


func get_normal_force(collision_index: int, velocity: Vector3) -> Vector3:
	var contact := get_collision_point(collision_index)
	var spring_up_direction := global_basis.y
	var penetration := wheel_radius - global_position.distance_to(contact)
	var spring_force := spring_strength * penetration
	var relative_vel := spring_up_direction.dot(velocity)
	var spring_damp_force := spring_damping * relative_vel
	var force_vector := (spring_force - spring_damp_force) * get_collision_normal(collision_index)
	return force_vector


func get_pedal_force(collision_index: int, pedal_input: float) -> Vector3:
	if pedal_input > 0.0:
		var foward_direction := -global_basis.z
		var contact := get_collision_point(collision_index)
		var force_vector := foward_direction * pedal_force * pedal_input
		return force_vector
	else: return Vector3(0, 0, 0)


# Works now, might be a problem later with all the easing to a stop
func get_brake_force(collision_index: int, brake_input: float, velocity: Vector3) -> Vector3:
	if brake_input > 0.0:
		var forward_direction := -global_basis.z
		var relative_velocity := forward_direction.dot(velocity)
		
		# No velocity = No brake force
		if is_zero_approx(relative_velocity):
			return Vector3(0, 0, 0)
		
		# get direction of brake force
		var force_direction := forward_direction
		force_direction *= -sign(relative_velocity)
		
		# eases into a stop
		if absf(relative_velocity) < 0.5:
			return force_direction * relative_velocity
		
		var force_vector: Vector3
		
		# if velocity is low enough apply brake as damping
		if absf(relative_velocity) < 3:
			force_vector = force_direction * relative_velocity * (brake_force * brake_input)
		
		# else apply brake force normally
		force_vector = force_direction * brake_force * brake_input
		return force_vector
	
	else: return Vector3(0, 0, 0)


func get_steering_force(collision_index: int, velocity: Vector3) -> Vector3:
	var cornering_stiffness := 750.0 # magic cornering coefficient - N per rad slip angle
	var camber_stiffness := 100.0
	
	# get forward and side vectors to calculate slip angle
	var forward_direction := -global_basis.z
	var side_direction := global_basis.x
	var ground_normal := get_collision_normal(collision_index)
	
	# project the forward and side vectors onto the ground normal
	forward_direction = (forward_direction - forward_direction.project(ground_normal)).normalized()
	side_direction = (side_direction - side_direction.project(ground_normal)).normalized()
	
	var slip_angle := calculate_slip_angle(velocity, forward_direction, side_direction)
	
	var roll_angle = -global_rotation.z
	if abs(roll_angle) < 0.01:
		roll_angle = 0.0
		
	var camber_thrust := side_direction * camber_stiffness * tan(roll_angle)
	var steering_force := side_direction * cornering_stiffness * slip_angle

	steering_force += camber_thrust
	#DebugDraw3D.draw_arrow_ray(global_position, forward_direction, 1.0, Color.BLUE, 0.1)
	#DebugDraw3D.draw_arrow_ray(global_position, side_direction, 1.0, Color.GREEN, 0.1)
	DebugDraw3D.draw_arrow_ray(global_position, steering_force.normalized(), 0.5, Color.RED, 0.1)
	DebugDraw3D.draw_arrow_ray(global_position, camber_thrust.normalized(), 0.5, Color.GREEN, 0.1)
	#DebugDraw3D.draw_arrow_ray(global_position, velocity.normalized(), 1.5, Color.PURPLE, 0.1)
	#print("slip_angle ", slip_angle, " steering ", steering_force)
	
	return steering_force


func calculate_slip_angle(velocity: Vector3, forward_direction: Vector3, side_direction: Vector3) -> float:
	# get forward and side velocity components
	var lateral_velocity := side_direction.dot(velocity)
	var longitudinal_velocity := forward_direction.dot(velocity)
	
	# zero floats if approx zero to remove force error
	if abs(longitudinal_velocity) < 0.1:
		return 0.0
	
	var slip_angle := -atan2(lateral_velocity, longitudinal_velocity)
	return slip_angle


func set_steering_axis(angle: float) -> void:
	steering_axis = Vector3(0.0, sin(angle), -cos(angle)).normalized()


# takes long and lat force vectors and limits them to the friction circle
func apply_friction_circle(longitudinal_force: Vector3, lateral_force: Vector3, normal_force: float, static_friction: float, kinetic_friction: float) -> Vector3:
	
	var max_grip := 0.0
	var forward_direction := -global_basis.z
	var side_direction := global_basis.x
	var up_direction := global_basis.y
	
	# calculate the correct max friction limits
	if not is_sliding:
		max_grip = normal_force * static_friction
	elif is_sliding:
		max_grip = normal_force * kinetic_friction
	
	var total_desired := sqrt((longitudinal_force.length() ** 2) + (lateral_force.length() ** 2))
	
	if total_desired > max_grip:
		
		# start sliding
		max_grip = normal_force * kinetic_friction
		is_sliding = true
		
		# scale forces
		var scale_factor := max_grip / total_desired
		longitudinal_force *= scale_factor
		lateral_force *= scale_factor
		
		var limited_force_vector := longitudinal_force + lateral_force
		return limited_force_vector
		
	else:
		is_sliding = false # reset sliding flag
		return longitudinal_force + lateral_force
