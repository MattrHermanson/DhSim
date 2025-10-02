extends Node3D
class_name CameraPivot

func _process(delta: float) -> void:
	rotate_z(-global_rotation.z)
