extends RayCast3D
class_name BikeWheelBase

enum WheelType {
	FRONT,
	REAR,
}

# Wheel Settings - TODO make conditional http://kehomsforge.com/tutorials/single/conditionally-export-properties-godot/
@export var spring_strength := 100.0
@export var spring_damping := 2.0
@export var wheel_radius := 0.4
@export var pedal_force := 200
@export var brake_force := 500
@export var head_tube_angle := 63.5
@export var max_steering_angle := 45.0
@export var wheel_type : WheelType
@onready var wheel: Node3D = get_child(0) # reference to wheel mesh

# Internal Variables
var current_steering_angle := 0.0
var steering_axis : Vector3
var is_sliding := false

func _ready() -> void:
	if not target_position.y == -(wheel_radius + 0.05):
		push_warning("Target position not set correctly and won't reflect actual raycast")
	
	set_steering_axis(deg_to_rad(head_tube_angle))

# Returns all forces that are generated from the wheel
func get_forces(pedal_input: float, steering_input: float, front_brake_input: float, rear_brake_input: float, velocity: Vector3) -> Vector3:
	var total_force_vector: Vector3
	var longitudinal_force_vector: Vector3
	var lateral_force_vector: Vector3
	
	# TEMP - TODO figure out getting friction from raycast
	var static_friction = 1.0
	var kinetic_friction = 0.7
	
	# Add all wheel forces
	total_force_vector += get_normal_force(velocity)
	
	# Add rear-only forces
	if wheel_type == WheelType.REAR:
		update_animation(velocity, 0.0)
		longitudinal_force_vector += get_pedal_force(pedal_input)
		longitudinal_force_vector += get_brake_force(rear_brake_input, velocity)
		lateral_force_vector += get_steering_force(velocity)
	
	# Add front-only forces
	if wheel_type == WheelType.FRONT:
		update_animation(velocity, steering_input)
		longitudinal_force_vector += get_brake_force(front_brake_input, velocity)
		lateral_force_vector += get_steering_force(velocity)
	
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
	global_basis = global_basis.rotated(steering_axis, deg_to_rad(-degrees_to_rotate))


func get_normal_force(velocity: Vector3) -> Vector3:
	target_position.y = -(wheel_radius + 0.05) # wheel_radius + magic offset, maybe remove
	var contact := get_collision_point()
	var spring_up_direction := global_basis.y
	var penetration := wheel_radius - global_position.distance_to(contact)
	var spring_force := spring_strength * penetration
	var relative_vel := spring_up_direction.dot(velocity)
	var spring_damp_force := spring_damping * relative_vel
	var force_vector := (spring_force - spring_damp_force) * get_collision_normal()
	return force_vector


func get_pedal_force(pedal_input: float) -> Vector3:
	if pedal_input > 0.0:
		var foward_direction := -global_basis.z
		var contact := get_collision_point()
		var force_vector := foward_direction * pedal_force * pedal_input
		return force_vector
	else: return Vector3(0, 0, 0)


# Works now, might be a problem later with all the easing to a stop
func get_brake_force(brake_input: float, velocity: Vector3) -> Vector3:
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


func get_steering_force(velocity: Vector3) -> Vector3:
	var cornering_stiffness := 100.0 # magic cornering coefficient - N per rad slip angle
	
	# get forward and side vectors to calculate slip angle
	var forward_direction := -global_basis.z
	var side_direction := global_basis.x
	var ground_normal := get_collision_normal()
	
	# project the forward and side vectors onto the ground normal
	forward_direction = (forward_direction - forward_direction.project(ground_normal)).normalized()
	side_direction = (side_direction - side_direction.project(ground_normal)).normalized()
	
	var slip_angle := calculate_slip_angle(velocity, forward_direction, side_direction)
	
	var steering_force := side_direction * cornering_stiffness * slip_angle
	
	#DebugDraw3D.draw_arrow_ray(global_position, forward_direction, 1.0, Color.BLUE, 0.1)
	#DebugDraw3D.draw_arrow_ray(global_position, side_direction, 1.0, Color.GREEN, 0.1)
	DebugDraw3D.draw_arrow_ray(global_position, steering_force, 1.0, Color.RED, 0.1)
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
