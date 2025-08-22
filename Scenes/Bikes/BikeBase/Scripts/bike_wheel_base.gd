extends RayCast3D
class_name BikeWheelBase

enum WheelType {
	FRONT,
	REAR,
}

# Wheel Settings
@export var spring_strength := 100.0
@export var spring_damping := 2.0
@export var wheel_radius := 0.4
@export var wheel_type : WheelType
@onready var wheel: Node3D = get_child(0) # reference to wheel mesh

func _ready() -> void:
	if not target_position.y == -(wheel_radius + 0.05):
		push_warning("Target position not set correctly and won't reflect actual raycast")

func get_forces(velocity: Vector3) -> Vector3:
	var total_force_vector: Vector3
	total_force_vector += get_normal_force(velocity)
	return total_force_vector


func get_normal_force(velocity: Vector3) -> Vector3:
	target_position.y = -(wheel_radius + 0.05) # wheel_radius + magic offset, maybe remove
	var contact := get_collision_point()
	var spring_up_dir := global_transform.basis.y
	var penetration := wheel_radius - global_position.distance_to(contact)
	#wheel.position.y = -spring_len
	var spring_force := spring_strength * penetration
	var relative_vel := spring_up_dir.dot(velocity)
	var spring_damp_force := spring_damping * relative_vel
	return (spring_force - spring_damp_force) * get_collision_normal()
