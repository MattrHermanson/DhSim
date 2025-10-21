extends RigidBody3D
class_name BikeRigidWheelBase

enum WheelType {
	FRONT,
	REAR,
}

@export var wheel_type : WheelType

var pedal_input := 0.0

func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	
	pedal_input = Input.get_action_strength("Pedal")
	
	if wheel_type == WheelType.FRONT:
		pass
	elif wheel_type == WheelType.REAR:
		spin(pedal_input, state)


func spin(input: float, state: PhysicsDirectBodyState3D) -> void:
	var torque = Vector3(input * 10, 0, 0)
	var angular_impulse = torque * state.step
	state.apply_torque_impulse(angular_impulse)
