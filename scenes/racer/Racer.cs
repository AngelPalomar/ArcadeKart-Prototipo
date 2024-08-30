using Godot;
using System;

public partial class Racer : RigidBody3D
{
	public Node3D Car, CameraController, CameraPivot;
	public RayCast3D RayCastCar;
	public MeshInstance3D CarBody, RightWheel, LeftWheel;

	[Export] public Vector3 SphereOffset = Vector3.Down;
	[Export] public float Speed = 1.5f;
	[Export] public float Acceleration = 35f;
	[Export] public float Steering = 25f;
	[Export] public float TurnSpeed = 8f;
	[Export] public float TurnStopLimit = 0.75f;
	public float SpeedInput, TurnInput = 0f, CameraControllerInput = 0f, CameraPivotInput = 0f;
	public float BodyTilt = 35f;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		Car = GetNode<Node3D>("Car");
		CarBody = GetNode<MeshInstance3D>("Car/body");
		RightWheel = GetNode<MeshInstance3D>("Car/wheel-front-right");
		LeftWheel = GetNode<MeshInstance3D>("Car/wheel-front-left");
		CameraController = GetNode<Node3D>("Car/CameraController");
		CameraPivot = GetNode<Node3D>("Car/CameraController/CameraPivot");
		RayCastCar = GetNode<RayCast3D>("Car/RayCastCar");
	}

	public override void _PhysicsProcess(double delta)
	{
		Car.Position = Position + SphereOffset;
		if (RayCastCar.IsColliding())
			ApplyCentralForce(CameraController.Basis * -Car.Transform.Basis.Z * SpeedInput * Speed);
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		if (!RayCastCar.IsColliding())
			return;
		else if (RayCastCar.IsColliding())
		{
			//Alineacion con el suelo
			var collisionNormal = RayCastCar.GetCollisionNormal();
			Transform3D xForm = AlignWithY(Car.GlobalTransform, collisionNormal);
			Car.GlobalTransform = Car.GlobalTransform.InterpolateWith(xForm, (float)(10f * delta));
		}

		//Rotación visual de las llantas
		RightWheel.Rotation = new Vector3(RightWheel.Rotation.X, TurnInput, RightWheel.Rotation.Z);
		LeftWheel.Rotation = new Vector3(LeftWheel.Rotation.X, TurnInput, LeftWheel.Rotation.Z);

		//Dirección y giro
		SpeedInput = Input.GetAxis("brake", "accelerate") * Acceleration;
		TurnInput = Input.GetAxis("steer_right", "steer_left") * Mathf.DegToRad(Steering);

		//Rotación del Auto
		if (LinearVelocity.Length() > TurnStopLimit)
		{
			Basis newBasis = Car.GlobalTransform.Basis.Rotated(Car.GlobalTransform.Basis.Y, TurnInput);
			Car.GlobalTransform = new Transform3D(
					Car.GlobalTransform.Basis.Slerp(newBasis, (float)(TurnSpeed * delta)),
					Car.GlobalTransform.Origin
				).Orthonormalized();

			//Rotación visual del modelo del auto
			var tilt = -TurnInput * LinearVelocity.Length() / BodyTilt;
			Vector3 rotation = CarBody.Rotation;
			rotation.Y = Mathf.Lerp(CarBody.Rotation.Y, tilt, (float)(10 * delta));
			CarBody.Rotation = rotation;
		}

		//Control de la camara
		CameraController.RotateY(CameraControllerInput);
		CameraPivot.RotateY(CameraPivotInput);

		float clampedRotationX = Mathf.Clamp(CameraPivot.Rotation.X, Mathf.DegToRad(-90.0f), Mathf.DegToRad(30.0f));
		CameraPivot.Rotation = new Vector3(clampedRotationX, CameraPivot.Rotation.Y, CameraPivot.Rotation.Z);

		CameraControllerInput = 0f;
		CameraPivotInput = 0f;
	}

	private Transform3D AlignWithY(Transform3D xForm, Vector3 collisionNormal)
	{
		xForm.Basis.Y = collisionNormal;
		xForm.Basis.X = -xForm.Basis.Z.Cross(collisionNormal);
		xForm.Basis = xForm.Basis.Orthonormalized();
		return xForm.Orthonormalized();
	}

}
