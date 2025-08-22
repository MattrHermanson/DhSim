extends RigidBody3D
class_name BikeBase

@export var front_wheel : BikeWheelBase
@export var rear_wheel : BikeWheelBase
var wheels: Array[BikeWheelBase]

var pedal_input = 0
var steering_input = 0
var front_brake_input = 0
var rear_brake_input = 0

func _ready() -> void:
		assert(front_wheel != null, "ERROR: 'front_wheel' must not be null!")
		assert(rear_wheel != null, "ERROR: 'rear_wheel' must not be null!")
		
		wheels = [front_wheel, rear_wheel]

func _unhandled_input(event: InputEvent) -> void:
	pass
	

func _process(delta: float) -> void:
	DebugDraw3D.draw_box((global_position + center_of_mass + Vector3(0, 0, 0)), global_transform.basis, Vector3(0.25, 0.25, 0.25), Color.GREEN_YELLOW, true) # draw center of mass
	pass
	
		
func _physics_process(delta: float) -> void:
	for wheel in wheels:
		if wheel.is_colliding():
			var velocity_at_contact = _get_point_velocity(wheel.get_collision_point())
			var force_vector = wheel.get_forces(velocity_at_contact)
			var force_pos_offset := wheel.get_collision_point() - global_position
			apply_force(force_vector, force_pos_offset)
			DebugDraw3D.draw_arrow_ray(wheel.get_collision_point(), force_vector, force_vector.length()*0.0005, Color.SKY_BLUE, 0.1)

# Helper function to get velocity at point
func _get_point_velocity(point: Vector3) -> Vector3:
	return linear_velocity + angular_velocity.cross(point - global_position)
