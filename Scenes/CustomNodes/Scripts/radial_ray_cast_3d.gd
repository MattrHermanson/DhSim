@tool
@icon("res://icons/RadialRayCast3D.svg")
extends Node3D
class_name RadialRayCast3D

# Raycast3D Properties
var collide_with_areas := false
var collide_with_bodies := true
var collision_mask := 1
var debug_shape_custom_color := Color(0, 0, 0, 1)
var debug_shape_thickness := 2
var enabled := true
var exclude_parent := true
var hit_back_faces := true
var hit_from_inside := false

# Custom Properties
@export_range(3, 360, 1, "suffix:rays") var raycast_resolution := 8:
	set(value): # select a factor of 360
		
		if 360 % value == 0:
			raycast_resolution = value
			
		elif not 360 % value == 0 and value < raycast_resolution:
			while (not 360 % value == 0):
				value -= 1
			
			raycast_resolution = value
			
		elif not 360 % value == 0 and value > raycast_resolution:
			while (not 360 % value == 0):
				value += 1
			
			raycast_resolution = value
		
		if is_inside_tree(): # Prevent errors during initialization
			_update_raycasts()

@export_range(0.001, 10.000, 0.001, "or_greater", "suffix:m") var raycast_length := 1.0: # meters
	set(value):
		raycast_length = value
		
		if is_inside_tree(): # Prevent errors during initialization
			_update_raycasts()

var raycast_array : Array[RayCast3D] = []

func _update_raycasts() -> void:
	for child in get_children(true):
		if child is RayCast3D:
			remove_child(child)
			child.free()
	raycast_array.clear()
	
	var degrees_between := 360 / raycast_resolution
	
	for i in range(raycast_resolution):
		var new_raycast := RayCast3D.new()
		var current_rotation = deg_to_rad(i * degrees_between)
		var target_vector := Vector3(0, cos(current_rotation), sin(current_rotation))
		target_vector *= raycast_length
		
		# Pass raycast settings to each raycast
		new_raycast.target_position = target_vector
		new_raycast.collide_with_areas = collide_with_areas
		new_raycast.collide_with_bodies = collide_with_bodies
		new_raycast.collision_mask = collision_mask
		new_raycast.debug_shape_custom_color = debug_shape_custom_color
		new_raycast.debug_shape_thickness = debug_shape_thickness
		new_raycast.enabled = enabled
		new_raycast.exclude_parent = exclude_parent
		new_raycast.hit_back_faces = hit_back_faces
		new_raycast.hit_from_inside = hit_from_inside
		
		raycast_array.append(new_raycast)
		add_child(new_raycast, false, Node.INTERNAL_MODE_BACK)
		new_raycast.set_meta("_edit_lock_", true)
		
		if Engine.is_editor_hint():
			new_raycast.owner = get_tree().edited_scene_root
			new_raycast.set_display_folded(true)


func _ready() -> void:
	_update_raycasts()

# returns the index of the ray with the nearest collision, returns -1 if no ray is colliding
func _get_nearest_collision_index() -> int:
	var closest_distance = INF
	var closest_ray_index = null
	
	for i in range(raycast_array.size()):
		if raycast_array[i].is_colliding():
			var distance := raycast_array[i].get_collision_point().distance_to(global_position)
			if distance < closest_distance:
				closest_distance = distance
				closest_ray_index = i
	
	if closest_ray_index == null:
		return -1
	else:
		return closest_ray_index


# TODO consider moving _get_nearest_collision_index here because it should always be called before using other methods
func is_colliding() -> bool:
	for ray in raycast_array:
		if ray.is_colliding():
			return true
	
	return false


func get_collision_normal() -> Vector3:
	var index = (_get_nearest_collision_index())
	if index == -1:
		push_error("No ray is colliding, ensure is_colliding() is being called before this method")
	
	return raycast_array[index].get_collision_normal()


func get_collision_point() -> Vector3:
	var index = (_get_nearest_collision_index())
	if index == -1:
		push_error("No ray is colliding, ensure is_colliding() is being called before this method")
	
	return raycast_array[index].get_collision_point()


func get_collider() -> Object:
	var index = (_get_nearest_collision_index())
	if index == -1:
		push_error("No ray is colliding, ensure is_colliding() is being called before this method")
	
	return raycast_array[index].get_collider()
