#include "Assignment.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"


using namespace BGE;

Assignment::Assignment(void)
{
}

Assignment::~Assignment(void)
{
}

std::shared_ptr<GameComponent> station2;

bool Assignment::Initialise()
{
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();

	setGravity(glm::vec3(0, -9, 0));

	size = 8;
	height = 8;
	width = 8;
	

	CreateWall(size, width, height);


	if (!Game::Initialise()) {
		return false;
	}



	return true;
}

void BGE::Assignment::CreateWall(int size, int width, int height)
{
	for (int i = 0; i < height; i++)
	{
		for (int y = 0; y < width; y++)
		{
			shared_ptr<PhysicsController> box1 = physicsFactory->CreateBox(size, size, size, glm::vec3(30 + (y * size), 1 + (i * size), 0), glm::quat());
		}
	}
}

void BGE::Assignment::Update()
{

	Game::Update();
}

void BGE::Assignment::Cleanup()
{
	Game::Cleanup();
}
