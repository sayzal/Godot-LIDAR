extends CharacterBody3D

@onready var Head = $Head
@onready var Hand = $Head/Hand
@onready var Camera = $Head/Camera3D
@onready var Lines = $Head/Hand/Scanner/Lines

var move_speed = 7
var acceleration
var gravity = 17.6
var jump_height = 5.75
var air_acceleration = 1
var floor_acceleration = 5

var mouse_sensitivity = 0.2
var joystick_sensitivity = 3
var stop_momentum = true
var MaxSlopeAngle = 20
var ground_contact = false
var ViewmodelSway = 2
var in_air = false

var direction
var gravity_vector
var movement

var max_random_rotation = 10
var scanning = false
var lidar_size = 10000
var max_points = lidar_size * 100
@export var LIDARMeshScene : PackedScene
@onready var lidar_container = $LIDARContainer
var current_point = 0
var PointPerSecond = 2000
@onready var lidar_ray = $Head/LidarRay
@export var enemy_color : Color
@export var point_color : Color
var full_scan_size = 1000
var full_scan_progress = 0

func _ready():
	Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
	
func _process(delta):
	print(Engine.get_frames_per_second())
	var h = Input.get_action_strength("look_r") - Input.get_action_strength("look_l")
	var v = Input.get_action_strength("look_up") - Input.get_action_strength("look_dw")
	rotate_camera(h * joystick_sensitivity, v * -joystick_sensitivity)

	var angle_step = 2
	if (Input.is_action_just_released("scan_size_up")):
		max_random_rotation -= angle_step
	if (Input.is_action_just_released("scan_size_down")):
		max_random_rotation += angle_step

	max_random_rotation = clamp(max_random_rotation, 2, 90)

	Hand.rotation = Vector3(
		lerp_angle(Hand.rotation.x, 0, ViewmodelSway * delta * 10),
		lerp_angle(Hand.rotation.y, 0, ViewmodelSway * delta * 10),
		0)

	Lines.rotation = -Hand.rotation

	if (Input.is_action_just_pressed("restart")):
		remove_lidar_meshes()
		pass

	if (Input.is_action_just_pressed("toggle_camera")):
		if (Camera.get_cull_mask() == 1):
			Camera.set_cull_mask(2)
		else:
			Camera.set_cull_mask(1)
		pass
		
func _physics_process(delta):
	physics_lidar(delta);

	direction = Vector3.ZERO

	in_air = !is_on_floor()

	if (is_on_floor()):
		acceleration = floor_acceleration
	else:
		acceleration = air_acceleration

	if (Input.is_action_just_pressed("jump") and is_on_floor()):
		jump()
	else:
		in_air = true
		acceleration = floor_acceleration
		gravity_vector = velocity

	var _direction_not_rotated = Vector3(Input.get_action_strength("move_r") - Input.get_action_strength("move_l"),
	0, Input.get_action_strength("move_fw") - Input.get_action_strength("move_bw"))

	var dir_node = self
	direction -= Input.get_action_strength("move_fw") * dir_node.global_transform.basis.z
	direction += Input.get_action_strength("move_bw") * dir_node.global_transform.basis.z
	direction -= Input.get_action_strength("move_l") * dir_node.global_transform.basis.x
	direction += Input.get_action_strength("move_r") * dir_node.global_transform.basis.x
	if (direction.length_squared() > 1):
		direction = direction.normalized();

	if (stop_momentum):
		var v = velocity
		v.y = 0
		v = v.normalized()

		if (direction.dot(v) <= -0.8):
			velocity = Vector3.ZERO

	var acutalSepeed
	if (scanning):
		acutalSepeed = move_speed / 3.0
	else:
		acutalSepeed = move_speed

	velocity = velocity.lerp(direction * acutalSepeed, acceleration * delta)

	if ((is_on_floor() && !in_air) or (is_on_ceiling() && in_air)):
		gravity_vector = Vector3.ZERO
	else:
		gravity_vector.y -= gravity * delta

	movement = velocity + gravity_vector
	move_and_slide()

func jump():
	in_air = true
	gravity_vector = Vector3.UP * jump_height
	ground_contact = false
		
func _input(event):
	if event is InputEventMouseMotion:
		rotate_camera(event.relative.x * mouse_sensitivity, event.relative.y * mouse_sensitivity)

func rotate_camera(h, v):
	rotate_y(deg_to_rad(-h))

	Hand.rotation = Vector3(
		clamp(Hand.rotation.x + deg_to_rad(v / 5) * ViewmodelSway, deg_to_rad(-15), deg_to_rad(15)),
		clamp(Hand.rotation.y + deg_to_rad(h / 5) * ViewmodelSway, deg_to_rad(-15), deg_to_rad(15)), 0);

	Lines.rotation = -Hand.rotation;

	Head.rotation = Vector3(clamp(Head.rotation.x - deg_to_rad(v), deg_to_rad(-89), deg_to_rad(89)),
		Head.rotation.y, Head.rotation.z);

#LIDAR Part (Player.LIDAR.cs)

func create_lidar_mesh() -> MultiMeshInstance3D:
	var mesh = LIDARMeshScene.instantiate() as MultiMeshInstance3D

	mesh.multimesh = mesh.multimesh.duplicate() as MultiMesh
	mesh.multimesh.instance_count = lidar_size
	mesh.multimesh.visible_instance_count = 0

	lidar_container.add_child(mesh)
	mesh.set_as_top_level(true)
	mesh.global_transform = Transform3D()

	return mesh
	
func remove_lidar_meshes():
	current_point = 0
	for m in lidar_container.get_children():
		m.queue_free()
		
func circle_scan(delta):
	if (scanning): return

	var ps = int(ceil(PointPerSecond * delta))

	for i in ps:
		var rv = Vector2(remap(randf_range(0, 1.0) , 0, 1, -1, 1), remap(randf_range(0, 1.0) , 0, 1, -1, 1))

		if (rv.length() > 1): rv = rv.normalized()

		rv *= max_random_rotation;

		lidar_ray.rotation_degrees = Vector3(rv.x, rv.y, 0);

		var start = lidar_ray.global_transform.origin;
		var end = start + (-lidar_ray.global_transform.basis.z * 200);

		PutPoint(start, end);
		
func PutPoint(start, end):
	var space_state = get_world_3d().direct_space_state
	var params = PhysicsRayQueryParameters3D.new()
	params.from = start
	params.to = end
	params.exclude = []
	params.collision_mask = 1
	var col = space_state.intersect_ray(params)

	if (col != null and col.size() > 0):
		var hit = Vector3(col.position)
		var body = (col.collider)

		#if (body is IScannable sc): sc.OnScan(start, hit);

		#Lines.get_mesh().clear_surfaces()
		Lines.get_mesh().surface_begin(Mesh.PRIMITIVE_LINES)
		Lines.get_mesh().surface_add_vertex(Vector3.ZERO)
		Lines.get_mesh().surface_add_vertex(Lines.to_local(hit))
		Lines.get_mesh().surface_end()

		var trans = Transform3D(Basis.IDENTITY, hit)

		var is_enemy = body.is_in_group("Enemy")

		var clr
		if (is_enemy):
			clr = enemy_color
		else:
			clr = point_color
		var max_offset = 0.2
			
		clr += Color( 
			float(remap(randf_range(0, 1), 0, 1, -max_offset, max_offset)),
			float(remap(randf_range(0, 1), 0, 1, -max_offset, max_offset)),
			float(remap(randf_range(0, 1), 0, 1, -max_offset, max_offset))
			)

		set_point(current_point, trans, clr);

		current_point = current_point + 1
		if (current_point == max_points): current_point = 0
		
func full_scan():
	if (scanning): return
	scanning = true

func set_point(idx, trans, col):
	var meshid = idx / lidar_size;

	var childcount = lidar_container.get_child_count();
	if (meshid >= childcount):
		create_lidar_mesh();

	var mesh = (lidar_container.get_child(meshid) as MultiMeshInstance3D).multimesh;

	var pointid = idx % lidar_size;
	mesh.visible_instance_count = max(mesh.visible_instance_count, pointid + 1);
	mesh.set_instance_transform(pointid, trans);
	mesh.set_instance_color(pointid, col);

func physics_lidar(delta):
	Lines.get_mesh().clear_surfaces()
	#Lines.get_mesh().surface_begin(Mesh.PRIMITIVE_LINES)
	if (!scanning):
		if (Input.is_action_pressed("attack1")):
			circle_scan(delta)
		if (Input.is_action_just_pressed("attack2")):
			full_scan()
	else:
		var pointcount = int(ceil(PointPerSecond * delta))
		pointcount *= 4

		for i in pointcount:
			var y = (full_scan_progress + i) / float(full_scan_size)
			var x = (full_scan_progress + i) % full_scan_size

			var screenpos = Vector2(
				x / float(full_scan_size),
				y / float(full_scan_size))
			screenpos *= Vector2(get_tree().root.size)
			screenpos += Vector2(remap(randf_range(0, 1.0) , 0, 1, -2, 2), remap(randf_range(0, 1.0) , 0, 1, -2, 2))

			var start = Camera.project_ray_origin(screenpos)
			var end = start + (Camera.project_ray_normal(screenpos) * 2000)

			PutPoint(start, end)

			full_scan_progress += pointcount
			if (full_scan_progress >= full_scan_size * full_scan_size):
				full_scan_progress = 0
				scanning = false
		#Lines.get_mesh().surface_end()
